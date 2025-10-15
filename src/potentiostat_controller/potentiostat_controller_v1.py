#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This Python program allows control over the USB potentiostat/galvanostat using a graphical user interface. It supports real-time data acquisition and plotting, manual control and
# calibration, and three pre-programmed measurement methods geared towards battery research (staircase cyclic voltammetry, constant-current charge/discharge, and rate testing).
# It is cross-platform, requiring a working installation of Python 3.x together with the PyUSB, Scipy, Numpy, PyQt5, PyQtGraph, and Matplotlib packages.
# Adapted from code originally written by Thomas Dobbelaere.

# Author: Alexander Bell
# License: GNU General Public License v3.0

import sys
import platform
import pyqtgraph
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5 import QtWidgets
import time
import datetime
import timeit
import usb.core
import usb.util
import os.path
import collections
from collections import defaultdict
import numpy
import scipy.integrate
from datetime import datetime
from itertools import zip_longest
import re

import matplotlib
from matplotlib.figure import Figure
from matplotlib.ticker import MaxNLocator
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

usb_vid = "0xa0a0"  # Default USB vendor ID, can also be adjusted in the GUI
usb_pid = "0x0002"  # Default USB product ID, can also be adjusted in the GUI
current_range_list = ["20 mA", u"200 µA", u"2 µA"]
shunt_calibration = [1., 1., 1.]  # Fine adjustment for shunt resistors, containing values of R1/10ohm, R2/1kohm, R3/100kohm (can also be adjusted in the GUI)
currentrange = 0  # Default current range (expressed as index in current_range_list)
units_list = ["Potential (V)", "Current (mA)", "DAC Code"]
dev = None  # Global object which is reserved for the USB device
current_offset = 0.  # Current offset in DAC counts
potential_offset = 0.  # Potential offset in DAC counts
potential = 0.  # Measured potential in V
current = 0.  # Measured current in mA
last_potential_values = collections.deque(maxlen=200)
last_current_values = collections.deque(maxlen=200)
raw_potential = 0  # Measured potential in ADC counts
raw_current = 0  # Measured current in ADC counts
last_raw_potential_values = collections.deque(maxlen=200)
last_raw_current_values = collections.deque(maxlen=200)

# Colour blind-friendly plot colours
CB_color_cycle = [
	'#e41a1c',  # Red
	'#00BFC4',  # Teal
	'#4daf4a',  # Green
	'#f781bf',  # Pink
	'#FFA07A',  # Light Salmon
	'#984ea3',  # Purple
]



"""_____INITIALISE GLOBAL PARAMETERS_____"""

"""CORE PROGRAM GLOBALS"""

overcounter, undercounter, skipcounter = 0, 0, 0  # Global counters used for automatic current ranging
time_of_last_adcread = 0.
adcread_interval = 0.09  # ADC sampling interval (in seconds)
logging_enabled = False  # Enable logging of potential and current in idle mode (can be adjusted in the GUI)
legend_in_use = None  # Store legend currently in use

"""USER-DEFINED SOFTWARE SETTINGS"""

global_software_settings = {
	'OCP_eq_tolerance': 1.0,  # Tolerance (mV) for OCP equilibration
	'OCP_eq_timescale': 60.0,  # Timescale (s) over which OCP stability is measured
	'OCP_eq_timeout': 3600.0,  # Maximum duration to wait for OCP equilibration
	'OCP_eq_num_samples': 3,  # Number of samples to average during OCP equilibration
	'cv_reverse_current_lockout': 2.0,  # Lockout (s) to prevent re-triggering reverse current limits
	'gcd_nth_cycles': 100,  # GCD experiments store every nth cycle for plotting
	'tab_frame_width': 125,  # Tab frame width (in spaces)
}

"""CV GLOBALS"""

cv_current_exp_index = None

cv_total_time = None
cv_remaining_time = None

cv_parameters_checked = False
cv_filenames_checked = False

cv_parameters = {'type': 'cv'}


"""LSV GLOBALS"""

lsv_current_exp_index = None

lsv_total_time = None
lsv_remaining_time = None

lsv_parameters_checked = False
lsv_filenames_checked = False

lsv_parameters = {'type': 'lsv'}


"""GCD GLOBALS"""

gcd_current_exp_index = None
gcd_current_halfcyclenum = None
gcd_current_cyclenum = None

gcd_total_halfcycles = None
gcd_cumulative_halfcyclenum = None

gcd_parameters_checked = False
gcd_filenames_checked = False

gcd_parameters = {'type': 'gcd'}


"""CA GLOBALS"""

ca_total_segments = None
ca_current_segment_index = None

ca_parameters_checked = False
ca_filenames_checked = False

ca_parameters = {'type': 'ca'}


"""CP GLOBALS"""

cp_total_segments = None
cp_current_segment_index = None

cp_parameters_checked = False
cp_filenames_checked = False

cp_parameters = {'type': 'cp'}


"""SD GLOBALS"""

sd_total_segments = None
sd_current_segment_index = None

sd_parameters_checked = False
sd_filenames_checked = False

sd_parameters = {'type': 'sd'}


"""RATE-TESTING GLOBALS"""

rate_current_exp_index = None
rate_current_c_rate_index = None
rate_waiting_for_next_c_rate = False

rate_total_c_rates = None
rate_cumulative_c_rate = None

rate_parameters_checked = False
rate_filenames_checked = False

rate_parameters = {'type': 'rate'}



"""_____CORE FUNCTIONS FOR PROGRAM_____"""

if platform.system() != "Windows":
	# On Linux/OSX, use the Qt timer
	busyloop_interval = 0
	qt_timer_period = 1e3 * adcread_interval  # convert to ms
else:
	# On MS Windows, system timing is inaccurate, so use a busy loop instead
	busyloop_interval = adcread_interval
	qt_timer_period = 0

class AverageBuffer:
	"""Collect samples and compute an average as soon as a sufficient number of samples is added."""
	def __init__(self, number_of_samples_to_average):
		self.number_of_samples_to_average = number_of_samples_to_average
		self.samples = []
		self.averagebuffer = []

	def add_sample(self, sample):
		self.samples.append(sample)
		if len(self.samples) >= self.number_of_samples_to_average:
			self.averagebuffer.append(sum(self.samples)/len(self.samples))
			self.samples = []

	def clear(self):
		self.samples = []
		self.averagebuffer = []

class States:
	"""Expose a named list of states to be used as a simple state machine."""
	NotConnected = 0
	Idle_Init = 1
	Idle = 2
	Measuring_Offset = 3
	Stationary_Graph = 4
	Measuring_CV = 5
	Measuring_CV_OCP_eq = 6
	Measuring_CV_Delay = 7
	Measuring_LSV = 8
	Measuring_LSV_OCP_eq = 9
	Measuring_LSV_Delay = 10
	Measuring_GCD = 11
	Measuring_GCD_OCP_eq = 12
	Measuring_GCD_Delay = 13
	Measuring_CA = 14
	Measuring_CA_OCP_eq = 15
	Measuring_CA_Delay = 16
	Measuring_CP = 17
	Measuring_CP_OCP_eq = 18
	Measuring_CP_Delay = 19
	Measuring_SD = 20
	Measuring_SD_OCP_eq = 21
	Measuring_SD_Delay = 22
	Measuring_Rate = 23
	Measuring_Rate_One_C_Calc = 24
	Measuring_Rate_OCP_eq = 25
	Measuring_Rate_Delay = 26

class Legends:
	"""Manages plot legend across experiment types and previews."""
	legends = {
		'live_graph': None,
		'cv': None,
		'lsv': None,
		'lsv_preview': None,
		'gcd': None,
		'ca': None,
		'ca_preview': None,
		'cp': None,
		'cp_preview': None,
		'sd': None,
		'rate': None,
		'rate_one_c_calc': None,
		'OCP_eq': None
	}

	@classmethod
	def remove_all_legends(cls):
		"""Remove all legends from their scenes if they exist."""
		for key, legend in cls.legends.items():
			if legend is not None and hasattr(legend, 'scene'):
				try:
					legend.scene().removeItem(legend)
				except AttributeError:
					pass  # Legend might have already been removed


state = States.NotConnected  # Initial state

legend = None  # Initial legend

def current_to_string(currentrange, current_in_mA):
	"""Format the measured current into a string with appropriate units and number of significant digits."""
	abs_value = abs(current_in_mA)
	if currentrange == 0:
		if abs_value <= 9.9995:
			return u"%+6.3f mA" % current_in_mA
		else:
			return u"%+6.2f mA" % current_in_mA
	elif currentrange == 1:
		if abs_value < 9.9995e-2:
			return u"%+06.2f µA" % (current_in_mA * 1e3)
		else:
			return u"%+6.1f µA" % (current_in_mA * 1e3)
	elif currentrange == 2:
		return u"%+6.3f µA" % (current_in_mA * 1e3)

def potential_to_string(potential_in_V):
	"""Format the measured potential into a string with appropriate units and number of significant digits."""
	return u"%+6.3f V" % potential_in_V

def twocomplement_to_decimal(msb, middlebyte, lsb):
	"""Convert a 22-bit two-complement ADC value consisting of three bytes to a signed integer (see MCP3550 datasheet for details)."""
	ovh = (msb > 63) and (msb < 128)  # Check for overflow high (B22 set)
	ovl = (msb > 127)  # Check for overflow low (B23 set)
	combined_value = (msb % 64)*2**16 + middlebyte*2**8 + lsb  # Get rid of overflow bits
	if not ovh and not ovl:
		if msb > 31:  # B21 set -> negative number
			answer = combined_value - 2**22
		else:
			answer = combined_value
	else:  # overflow
		if msb > 127:  # B23 set -> negative number
			answer = combined_value - 2**22
		else:
			answer = combined_value
	return answer

def decimal_to_dac_bytes(value):
	"""Convert a floating-point number, ranging from -2**19 to 2**19-1, to three data bytes in the proper format for the DAC1220."""
	code = 2**19 + int(round(value))  # Convert the (signed) input value to an unsigned 20-bit integer with zero at midway
	code = numpy.clip(code, 0, 2**20 - 1)  # If the input exceeds the boundaries of the 20-bit integer, clip it
	byte1 = code // 2**12
	byte2 = (code % 2**12) // 2**4
	byte3 = (code - byte1*2**12 - byte2*2**4)*2**4
	return bytes([byte1, byte2, byte3])

def dac_bytes_to_decimal(dac_bytes):
	"""Convert three data bytes in the DAC1220 format to a 20-bit number ranging from -2**19 to 2**19-1."""
	code = 2**12*dac_bytes[0] + 2**4*dac_bytes[1] + dac_bytes[2]/2**4
	return code - 2**19

def make_groupbox_indicator(title_name, default_text):
	"""Make a GUI box (used for the potential, current, and status indicators)."""
	label = QtWidgets.QLabel(text=default_text, alignment=QtCore.Qt.AlignCenter)
	box = QtWidgets.QGroupBox(title=title_name, flat=False)
	format_box_for_display(box)
	layout = QtWidgets.QVBoxLayout()
	layout.addWidget(label, 0, alignment=QtCore.Qt.AlignCenter)
	layout.setSpacing(0)
	layout.setContentsMargins(30, 3, 30, 0)
	box.setLayout(layout)
	return label, box

def add_my_tab(tab_frame, tab_name):
	"""Add a tab to the tab view."""
	widget = QtWidgets.QWidget()
	tab_frame.addTab(widget, tab_name)
	return widget

def format_box_for_display(box):
	"""Adjust the appearance of a groupbox border for the status display."""
	color = box.palette().color(QtGui.QPalette.Background)  # Get the background color
	r, g, b = int(0.9*color.red()), int(0.9*color.green()), int(0.9*color.blue())  # Make background 10% darker to make the border color
	box.setStyleSheet("""
		QGroupBox {
			border: 1px solid rgb(%d,%d,%d);
			border-radius: 4px;
			margin-top: 0.5em;
			font-weight: normal;
			color: gray;
		}
		GroupBox::title {
			subcontrol-origin:
			margin; left: 10px;
			padding: 0 3px 0 3px;
		}
	""" % (r, g, b))

def format_box_for_parameter(box):
	"""Adjust the appearance of a groupbox border for parameter input."""
	color = box.palette().color(QtGui.QPalette.Background)  # Get the background color
	r, g, b = int(0.7*color.red()), int(0.7*color.green()), int(0.7*color.blue())  # Make background 30% darker to make the border color
	box.setStyleSheet("""
		QGroupBox {
			border: 1px solid rgb(%d,%d,%d);
			border-radius: 4px;
			margin-top: 0.5em;
			font-weight: bold
		}
		QGroupBox::title {
			subcontrol-origin: margin;
			left: 10px;
			padding: 0 3px 0 3px;
		}
	""" % (r, g, b))

def format_box_for_parameter_centered_title(box):
	"""Adjust the appearance of a groupbox border for parameter input."""
	color = box.palette().color(QtGui.QPalette.Background)  # Get the background color
	r, g, b = int(0.7*color.red()), int(0.7*color.green()), int(0.7*color.blue())  # Make background 30% darker to make the border color
	box.setStyleSheet("""
		QGroupBox {
			border: 1px solid rgb(%d,%d,%d);
			border-radius: 4px;
			margin-top: 0.5em;
			font-weight: bold;
		}
		QGroupBox::title {
			subcontrol-origin: margin;
			subcontrol-position: top center;
			padding: 0 3px;
		}
	""" % (r, g, b))

def make_label_entry(parent, labelname):
	"""Make a labelled input field for parameter input."""
	hbox = QtWidgets.QHBoxLayout()
	label = QtWidgets.QLabel(text=labelname)
	entry = QtWidgets.QLineEdit()
	hbox.addWidget(label)
	hbox.addWidget(entry)
	parent.addLayout(hbox)
	return entry

def custom_size_font(fontsize):
	"""Return the default Qt font with a custom point size."""
	myfont = QtGui.QFont()
	myfont.setPointSize(fontsize)
	return myfont

def log_message(message):
	"""Log a string to the message log."""
	statustext.appendPlainText(datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ") + message)
	statustext.ensureCursorVisible()

def safe_usb_string(dev, attr_name):
	"""Safely read a USB string descriptor, return fallback if not accessible."""
	try:
		return getattr(dev, attr_name)
	except usb.core.USBError as e:
		log_message(f"USBError reading {attr_name}: {e}")
		return "Unknown"
	except ValueError as e:
		log_message(f"ValueError reading {attr_name}: {e}")
		return "Unknown"

def connect_disconnect_usb():
	"""Toggle the USB device between connected and disconnected states."""
	global dev, state
	# If the device is already connected, then this function should disconnect it
	if dev is not None:
		usb.util.dispose_resources(dev)
		dev = None
		state = States.NotConnected
		hardware_usb_connectButton.setText("Connect")
		log_message("USB Interface disconnected.")
		return

	# Otherwise, try to connect
	usb_vid_string = str(hardware_usb_vid.text())
	usb_pid_string = str(hardware_usb_pid.text())
	dev = usb.core.find(idVendor=int(usb_vid_string, 0), idProduct=int(usb_pid_string, 0))
	if dev is None:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"USB Device Not Found",
			"No USB device was found with VID %s and PID %s. Verify the vendor/product ID and check the USB connection."%(usb_vid_string,usb_pid_string)
		)
	else:
		hardware_usb_connectButton.setText("Disconnect")
		log_message("USB Interface connected.")

		# Read descriptors safely
		manufacturer = safe_usb_string(dev, "manufacturer")
		product = safe_usb_string(dev, "product")
		serial_number = safe_usb_string(dev, "serial_number")
		hardware_device_info_text.setText(
			f"Manufacturer: {manufacturer}\nProduct: {product}\nSerial #: {serial_number}"
		)

		# Warn the user if USB device descriptors cannot be read (permissions issue)
		if "Unknown" in (manufacturer, product, serial_number):
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"USB Device Warning",
				"USB device found but cannot read manufacturer/product/serial number.\n\n"
				"If the potential/current is not displayed in the plot window, you may need to run as sudo or create a udev rule for your user (please see installation instructions in the GitHub repository)."
			)

		try:
			get_calibration()
			set_cell_status(False)  # Cell off
			set_control_mode(False)  # Potentiostatic control
			set_current_range()  # Read current range from GUI
			state = States.Idle_Init  # Start idle mode
		except ValueError:
			pass  # In case the device is not yet calibrated
		except usb.core.USBError as e:
			log_message(f"USB communication error: {e}")
			if e.errno == 13:  # Access denied
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"USB Access Denied",
					"Permission denied when communicating with the USB device.\n\n"
					"Run the program with sudo or create a udev rule to grant access (please see installation instructions in the GitHub repository)."
				)
			else:
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"USB Communication Error",
					f"An unexpected USB error occurred:\n\n{e}"
				)

		# Reset check states if checked as the measured cell potential and current will have changed
		checkstate_reset(mode="USB_connected")

def not_connected_errormessage():
	"""Generate an error message stating that the device is not connected."""
	QtWidgets.QMessageBox.critical(
		mainwidget,
		"Not connected",
		"This command cannot be executed because the USB device is not connected. Press the \"Connect\" button and try again."
	)

def check_state(desired_states):
	"""Check if the current state is in a given list. If so, return True; otherwise, show an error message and return False."""
	if state not in desired_states:
		if state == 0:
			not_connected_errormessage()
		else:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error",
				"This command cannot be executed in the current state."
			)
		return False
	else:
		return True

def send_command(command_string, expected_response, log_msg=None):
	"""Send a command string to the USB device and check the response; optionally logs a message to the message log."""
	if dev is not None:  # Make sure it's connected
		dev.write(0x01, command_string)  # 0x01 = write address of EP1
		response = bytes(dev.read(0x81, 64))  # 0x81 = read address of EP1
		if response != expected_response:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Unexpected Response",
				"The command \"%s\" resulted in an unexpected response. The expected response was \"%s\"; the actual response was \"%s\"" % (command_string,expected_response.decode("ascii"), response.decode("ascii"))
			)
		else:
			if log_msg != None:
				log_message(log_msg)
		return True
	else:
		not_connected_errormessage()
		return False

def set_cell_status(cell_on_boolean):
	"""Switch the cell connection (True = cell on, False = cell off)."""
	if cell_on_boolean:
		if send_command(b'CELL ON', b'OK'):
			cell_status_monitor.setText("CELL ON")
	else:
		if send_command(b'CELL OFF', b'OK'):
			cell_status_monitor.setText("CELL OFF")

def set_control_mode(galvanostatic_boolean):
	"""Switch the control mode (True = galvanostatic, False = potentiostatic)."""
	if galvanostatic_boolean:
		if send_command(b'GALVANOSTATIC', b'OK'):
			control_mode_monitor.setText("GALVANOSTATIC")
	else:
		if send_command(b'POTENTIOSTATIC', b'OK'):
			control_mode_monitor.setText("POTENTIOSTATIC")

def set_current_range():
	"""Switch the current range based on the GUI dropdown selection."""
	global currentrange
	index = hardware_manual_control_range_dropdown.currentIndex()
	commandstring = [b'RANGE 1', b'RANGE 2', b'RANGE 3'][index]
	if send_command(commandstring, b'OK'):
		current_range_monitor.setText(current_range_list[index])
		currentrange = index

def auto_current_range(experiment):
	"""Automatically switch the current range based on the measured current; returns a number of measurements to skip (to suppress artifacts)."""
	global currentrange, overcounter, undercounter
	relativecurrent = abs(current/(20./100.**currentrange))
	if experiment == "CV":
		range_checkboxes = cv_range_checkboxes
	elif experiment == "LSV":
		range_checkboxes = lsv_range_checkboxes
	elif experiment == "CA":
		range_checkboxes = ca_range_checkboxes

	if relativecurrent > 1.05 and currentrange != 0 and range_checkboxes[currentrange-1].isChecked():  # Switch to higher current range (if possible) after three detections
		overcounter += 1
	else:
		overcounter = 0
	if relativecurrent < 0.0095 and currentrange != 2 and range_checkboxes[currentrange+1].isChecked():  # Switch to lower current range (if possible) after three detections
		undercounter += 1
	else:
		undercounter = 0
	if overcounter > 3:
		currentrange -= 1
		hardware_manual_control_range_dropdown.setCurrentIndex(currentrange)
		set_current_range()
		overcounter = 0
		return 2  # Skip next two measurements to suppress artifacts
	elif undercounter > 3:
		currentrange += 1
		hardware_manual_control_range_dropdown.setCurrentIndex(currentrange)
		set_current_range()
		undercounter = 0
		return 2  # Skip next two measurements to suppress artifacts
	else:
		return 0

def current_range_from_current(current):
	"""Return the current range that best corresponds to a given current."""
	current = abs(current)
	if current <= 0.002:
		return 2  # Lowest current range (2 µA)
	elif current <= 0.2:
		return 1  # Intermediate current range (200 µA)
	else:
		return 0  # Highest current range (20 mA)

def get_next_enabled_current_range(desired_currentrange, experiment):
	"""Return an enabled current range that best corresponds to a desired current range."""
	range_found = False
	found_currentrange = desired_currentrange

	if experiment == "CV":
		range_checkboxes = cv_range_checkboxes
	elif experiment == "LSV":
		range_checkboxes = lsv_range_checkboxes
	elif experiment == "CA":
		range_checkboxes = ca_range_checkboxes

	for i in range(desired_currentrange, -1, -1):  # Look for an enabled current range, going up in current range
		if range_checkboxes[i].isChecked():
			found_currentrange = i
			range_found = True
			break
	if not range_found:
		for i in range(desired_currentrange, 3):  # Look for an enabled current range, going down in current range
			if range_checkboxes[i].isChecked():
				found_currentrange = i
				break
	return found_currentrange

def set_offset():
	"""Save offset values to the device's flash memory."""
	send_command(b'OFFSETSAVE '+decimal_to_dac_bytes(potential_offset)+decimal_to_dac_bytes(current_offset), b'OK', "Offset values saved to flash memory.")

def get_offset():
	"""Retrieve offset values from the device's flash memory."""
	global potential_offset, current_offset
	if dev is not None:  # Make sure it's connected
		dev.write(0x01, b'OFFSETREAD')  # 0x01 = write address of EP1
		response = bytes(dev.read(0x81, 64))  # 0x81 = read address of EP1
		if response != bytes([255, 255, 255, 255, 255, 255]):  # If no offset value has been stored, all bits will be set
			potential_offset = dac_bytes_to_decimal(response[0:3])
			current_offset = dac_bytes_to_decimal(response[3:6])
			hardware_calibration_potential_offset.setText("%d" % potential_offset)
			hardware_calibration_current_offset.setText("%d" % current_offset)
			log_message("Offset values read from flash memory.")
		else:
			log_message("No offset values were found in flash memory.")
	else:
		not_connected_errormessage()

def float_to_twobytes(value):
	"""Convert a floating-point number ranging from -2^15 to 2^15-1 to a 16-bit representation stored in two bytes."""
	code = 2**15 + int(round(value))
	code = numpy.clip(code, 0, 2**16 - 1)  # If the code exceeds the boundaries of a 16-bit integer, clip it
	byte1 = code // 2**8
	byte2 = code % 2**8
	return bytes([byte1, byte2])

def twobytes_to_float(bytes_in):
	"""Convert two bytes to a number ranging from -2^15 to 2^15-1."""
	code = 2**8*bytes_in[0]+bytes_in[1]
	return float(code - 2**15)

def set_shunt_calibration():
	"""Save shunt calibration values to the device's flash memory."""
	send_command(b'SHUNTCALSAVE '+float_to_twobytes((shunt_calibration[0]-1.)*1e6)+float_to_twobytes((shunt_calibration[1]-1.)*1e6)+float_to_twobytes((shunt_calibration[2]-1.)*1e6), b'OK', "Shunt calibration values saved to flash memory.")

def get_shunt_calibration():
	"""Retrieve shunt calibration values from the device's flash memory."""
	if dev is not None:  # Make sure it's connected
		dev.write(0x01, b'SHUNTCALREAD')  # 0x01 = write address of EP1
		response = bytes(dev.read(0x81, 64))  # 0x81 = read address of EP1
		if response != bytes([255, 255, 255, 255, 255, 255]):  # If no calibration value has been stored, all bits are set
			for i in range(0, 3):
				shunt_calibration[i] = 1.+twobytes_to_float(response[2*i:2*i+2])/1e6  # Yields an adjustment range from 0.967 to 1.033 in steps of 1 ppm
				hardware_calibration_shuntvalues[i].setText("%.4f" % shunt_calibration[i])
			log_message("Shunt calibration values read from flash memory.")
		else:
			log_message("No shunt calibration values were found in flash memory.")
	else:
		not_connected_errormessage()

def zero_offset():
	"""Calculate offset values in order to zero the potential and current."""
	if not check_state([States.Idle]):
		return  # Device needs to be in the idle state for this
	pot_offs = int(round(numpy.average(list(last_raw_potential_values))))  # Average potential offset
	cur_offs = int(round(numpy.average(list(last_raw_current_values))))  # Average current offset
	hardware_calibration_potential_offset.setText("%d" % pot_offs)
	hardware_calibration_current_offset.setText("%d" % cur_offs)
	offset_changed_callback()

def offset_changed_callback():
	"""Set the potential and current offset from the input fields."""
	global potential_offset, current_offset
	try:
		potential_offset = int(hardware_calibration_potential_offset.text())
		hardware_calibration_potential_offset.setStyleSheet("")
	except ValueError:  # If the input field cannot be interpreted as a number, color it red
		hardware_calibration_potential_offset.setStyleSheet("QLineEdit { background: red; }")
	try:
		current_offset = int(hardware_calibration_current_offset.text())
		hardware_calibration_current_offset.setStyleSheet("")
	except ValueError:  # If the input field cannot be interpreted as a number, color it red
		hardware_calibration_current_offset.setStyleSheet("QLineEdit { background: red; }")

def shunt_calibration_changed_callback():
	"""Set the shunt calibration values from the input fields."""
	for i in range(0, 3):
		try:
			shunt_calibration[i] = float(hardware_calibration_shuntvalues[i].text())
			hardware_calibration_shuntvalues[i].setStyleSheet("")
		except ValueError:  # If the input field cannot be interpreted as a number, color it red
			hardware_calibration_shuntvalues[i].setStyleSheet("QLineEdit { background: red; }")

def set_dac_calibration():
	"""Save DAC calibration values to the DAC and the device's flash memory."""
	try:
		dac_offset = int(hardware_calibration_dac_offset.text())
		hardware_calibration_dac_offset.setStyleSheet("")
	except ValueError:  # If the input field cannot be interpreted as a number, color it red
		hardware_calibration_dac_offset.setStyleSheet("QLineEdit { background: red; }")
		return
	try:
		dac_gain = int(hardware_calibration_dac_gain.text())
		hardware_calibration_dac_gain.setStyleSheet("")
	except ValueError:  # If the input field cannot be interpreted as a number, color it red
		hardware_calibration_dac_gain.setStyleSheet("QLineEdit { background: red; }")
		return
	send_command(b'DACCALSET '+decimal_to_dac_bytes(dac_offset)+decimal_to_dac_bytes(dac_gain-2**19), b'OK', "DAC calibration saved to flash memory.")

def get_dac_calibration():
	"""Retrieve DAC calibration values from the device's flash memory."""
	if dev is not None:  # Make sure it's connected
		dev.write(0x01, b'DACCALGET')  # 0x01 = write address of EP1
		response = bytes(dev.read(0x81, 64))  # 0x81 = write address of EP1
		if response != bytes([255, 255, 255, 255, 255, 255]):  # If no calibration value has been stored, all bits are set
			dac_offset = dac_bytes_to_decimal(response[0:3])
			dac_gain = dac_bytes_to_decimal(response[3:6])+2**19
			hardware_calibration_dac_offset.setText("%d" % dac_offset)
			hardware_calibration_dac_gain.setText("%d" % dac_gain)
			log_message("DAC calibration read from flash memory.")
		else:
			log_message("No DAC calibration values were found in flash memory.")
	else:
		not_connected_errormessage()

def set_calibration():
	"""Save all calibration values to the device's flash memory."""
	set_dac_calibration()
	set_offset()
	set_shunt_calibration()

def get_calibration():
	"""Retrieve all calibration values from the device's flash memory."""
	get_dac_calibration()
	get_offset()
	get_shunt_calibration()

def dac_calibrate():
	"""Activate the automatic DAC1220 calibration function and retrieve the results."""
	send_command(b'DACCAL', b'OK', "DAC calibration performed.")
	get_dac_calibration()

def set_output(value_units_index, value):
	"""Output data to the DAC; units can be either V (index 0), mA (index 1), or raw counts (index 2)."""
	global last_potentiostatic_command, last_galvanostatic_command
	if value_units_index == 0:
		send_command(b'DACSET '+decimal_to_dac_bytes(value/8.*2.**19+int(round(potential_offset/4.))), b'OK')
	elif value_units_index == 1:
		send_command(b'DACSET '+decimal_to_dac_bytes(value/(25./(shunt_calibration[currentrange]*100.**currentrange))*2.**19+int(round(current_offset/4.))), b'OK')
	elif value_units_index == 2:
		send_command(b'DACSET '+decimal_to_dac_bytes(value), b'OK')

def set_output_from_gui():
	"""Output data to the DAC from the GUI input field (hardware tab, manual control)."""
	value_units_index = hardware_manual_control_output_dropdown.currentIndex()
	if value_units_index == 0:  # Potential (V)
		try:
			value = float(hardware_manual_control_output_entry.text())
		except ValueError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Not a number",
				"The value you have entered is not a floating-point number."
			)
			return
	elif value_units_index == 1:  # Current (mA)
		try:
			value = float(hardware_manual_control_output_entry.text())
		except ValueError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Not a number",
				"The value you have entered is not a floating-point number."
			)
			return
	elif value_units_index == 2:  # DAC Code
		try:
			value = int(hardware_manual_control_output_entry.text())
		except ValueError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Not a number",
				"The value you have entered is not an integer number."
			)
			return
	else:
		return
	set_output(value_units_index, value)

def wait_for_adcread():
	"""Wait for the duration specified in the busyloop_interval."""
	if busyloop_interval == 0:
		return  # On Linux/Mac, system timing is used instead of the busyloop
	else:
		time.sleep(busyloop_interval/2.)  # Sleep for some time to prevent wasting too many CPU cycles
		app.processEvents()  # Update the GUI
		while timeit.default_timer() < time_of_last_adcread + busyloop_interval:
			pass  # Busy loop (this is the only way to get accurate timing on MS Windows)

def read_potential_current():
	"""Read the most recent potential and current values from the device's ADC."""
	global potential, current, raw_potential, raw_current, time_of_last_adcread
	wait_for_adcread()
	time_of_last_adcread = timeit.default_timer()
	dev.write(0x01, b'ADCREAD')  # 0x01 = write address of EP1
	response = bytes(dev.read(0x81, 64))  # 0x81 = read address of EP1
	if response != b'WAIT':  # 'WAIT' is received if a conversion has not yet finished
		raw_potential = twocomplement_to_decimal(response[0], response[1], response[2])
		raw_current = twocomplement_to_decimal(response[3], response[4], response[5])
		potential = (raw_potential-potential_offset)/2097152.*8.  # Calculate potential in V, compensating for offset
		current = (raw_current-current_offset)/2097152.*25./(shunt_calibration[currentrange]*100.**currentrange)  # Calculate current in mA, taking current range into account and compensating for offset
		potential_monitor.setText(potential_to_string(potential))
		current_monitor.setText(current_to_string(currentrange, current))
		if logging_enabled:  # If enabled, all measurements are appended to an output file (even in idle mode)
			try:
				print("%.2f\t%e\t%e" % (time_of_last_adcread, potential, current*1e-3), file=open(hardware_log_filename.text(), 'a', 1))  # Output tab-separated data containing time (in s), potential (in V), and current (in A)
			except:
				QtWidgets.QMessageBox.critical(mainwidget, "Logging error!", "Logging error!")
				hardware_log_checkbox.setChecked(False)  # Disable logging in case of file errors

def idle_init():
	"""Perform some necessary initialization before entering the Idle state."""
	global potential_plot_curve, current_plot_curve, legend, legend_in_use, state
	plot_frame.clear()

	Legends.remove_all_legends()
	plot_frame.setLabel('bottom', 'Sample #', units="")
	plot_frame.setLabel('left', 'Value', units="")
	legend = pyqtgraph.LegendItem(offset=(60, 10))
	legend.setParentItem(plot_frame.plotItem)
	plot_frame.enableAutoRange()
	plot_frame.getAxis('bottom').setTicks(None)
	plot_frame.setXRange(0, 200, update=True)
	potential_plot_curve = plot_frame.plot(pen='g', name='Potential (V)')
	legend.addItem(potential_plot_curve, "Potential (V)")
	current_plot_curve = plot_frame.plot(pen='r', name='Current (mA)')
	legend.addItem(current_plot_curve, "Current (mA)")

	# Update the active legend in the Legends dictionary
	Legends.legends['live_graph'] = legend
	legend_in_use = 'live_graph'

	state = States.Idle  # Proceed to the Idle state

def update_live_graph():
	"""Add newly measured potential and current values to their respective buffers and update the plot curves."""
	last_potential_values.append(potential)
	last_current_values.append(current)
	last_raw_potential_values.append(raw_potential)
	last_raw_current_values.append(raw_current)
	xvalues = range(last_potential_values.maxlen-len(last_potential_values), last_potential_values.maxlen)
	potential_plot_curve.setData(xvalues, list(last_potential_values))
	current_plot_curve.setData(xvalues, list(last_current_values))

def choose_file(file_entry_field, questionstring):
	"""Open a file dialog and write the path of the selected file to a given entry field."""
	file_path, _ = QtWidgets.QFileDialog.getSaveFileName(mainwidget, questionstring, "", "ASCII data (*.txt)",options=QtWidgets.QFileDialog.DontConfirmOverwrite)
	file_entry_field.setText(file_path)

def toggle_logging(checkbox_state):
	"""Enable or disable logging of measurements to a file based on the state of a checkbox (2 means checked)."""
	global logging_enabled
	logging_enabled = (checkbox_state == 2)

def validate_file(filename):
	"""Check if a filename can be written to. If so, return True."""
	if os.path.isfile(filename):
		if QtWidgets.QMessageBox.question(
			mainwidget,
			"File exists",
			"The specified output file already exists. Do you want to overwrite it?",
			QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
			QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
			return False
	try:
		tryfile = open(filename, 'w', 1)
		tryfile.close()
		return True
	except IOError:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"File error",
			"The specified output file path is not valid."
		)
		return False

def preview_cancel():
	"""Cancel the preview (stationary graph) state and go back to the idle state."""
	global state
	Legends.remove_all_legends()
	state = States.Idle_Init
	preview_cancel_button.hide()

def checkstate_reset(mode):
	"""Reset check states for all parameters which are checked."""

	param_map = [
		(cv_parameters_checked, cv_parameters, cv_reset_experiment_controller),
		(lsv_parameters_checked, lsv_parameters, lsv_reset_experiment_controller),
		(gcd_parameters_checked, gcd_parameters, gcd_reset_experiment_controller),
		(ca_parameters_checked, ca_parameters, ca_reset_experiment_controller),
		(cp_parameters_checked, cp_parameters, cp_reset_experiment_controller),
		(sd_parameters_checked, sd_parameters, sd_reset_experiment_controller),
		(rate_parameters_checked, rate_parameters, rate_reset_experiment_controller),
	]
	for checked, parameters, reset_controller in param_map:
		if not checked:
			continue

		if mode == "OCP_eq_timescale_changed":
			if parameters.get("OCP_bool", False):
				reset_controller(mode="input_changed")
		else:
			reset_controller(mode="input_changed")


"""_____CYCLIC VOLTAMMETRY FUNCTIONS_____"""

"""CV PARAMETER FUNCTIONS"""

def cv_checkbutton_callback():
	"""Function to control the data-validation process, called when "CHECK" button pressed."""
	global cv_parameters_checked, cv_filenames_checked

	# Initialise with parameters_checked = False, filenames_checked = False, and a check button style reset
	cv_parameters_checked = False
	cv_filenames_checked = False
	cv_variables_checkbutton.setStyleSheet("")

	# Remove any previous program state entry
	cv_info_program_state_entry.setText("No experiments running")

	# Check input parameter formats
	if cv_validate_inputs():
		pass
	else:
		cv_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
		log_message("CV check button failed: Inputs are not in the correct format.")
		return False

	# Write input parameters to global dictionary
	if cv_get_parameters():
		pass
	else:
		cv_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
		log_message("CV check button failed: Could not write experiment parameters to a global dictionary.")
		return False

	# If filename provided, check if filenames are acceptable
	if cv_file_entry.text().strip() != "":
		if cv_get_filenames():
			cv_filenames_checked = True
		else:
			cv_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
			log_message("CV check button failed: Experiment filenames could not be constructed.")
			return False

	# Give any parameter warnings
	cv_parameter_warnings()

	# Set global parameters_checked state as True
	cv_parameters_checked = True

	# Make check button green
	cv_variables_checkbutton.setStyleSheet("background-color: green; color: white;")

	# Calculate the time this experiment will take if init_OCP_valid
	if not cv_parameters['OCP_bool'] or (cv_parameters['OCP_bool'] and cv_parameters['future_OCP_valid_bool']):
		cv_calculate_experiment_time(cv_current_exp_index, initial=True)
		log_message(f"Check button successful! Experiments are ready to run. Estimated time to complete: {cv_remaining_time:.1f} s")
	else:
		QtWidgets.QMessageBox.warning(
			mainwidget,
			"Warning: Initial OCP invalid",
			"Experiment time cannot be calculated as the pre-equilibration estimate of OCP (the current cell potential) is invalid with experiment parameters.\n\n"
			"You may still progress to your experiments, but be aware that they may be interrupted if the equilibrated OCP is still invalid with your experiment parameters.\n\n"
			"If the equilibrated OCP is valid, the progress bar will be updated to reflect the time remaining."
		)
		log_message("Check button successful! Be aware - the estimated OCP is invalid with experiment parameters, and estimated time remaining cannot be calculated.")

	# Update progress bar and give green border
	cv_update_progress_bar()
	cv_progress_bar.set_solid_green_style()

	return True

def cv_validate_inputs():
	"""Ensure inputs are of the correct format."""

	# Potential limits
	try:
		lbounds_str = cv_params_lbound_entry.text().strip()
		ubounds_str = cv_params_ubound_entry.text().strip()
		lbounds_list = [lbound.strip() for lbound in lbounds_str.split(",")]
		ubounds_list = [ubound.strip() for ubound in ubounds_str.split(",")]
		for i, lbound in enumerate(lbounds_list):
			if lbound.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				lbounds_list[i] = "OCP"
			else:
				lbounds_list[i] = float(lbound)
		for i, ubound in enumerate(ubounds_list):
			if ubound.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				ubounds_list[i] = "OCP"
			else:
				ubounds_list[i] = float(ubound)

		# Potential limits are the same length
		if len(lbounds_list) != len(ubounds_list):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Potential limits input",
				"Lower and upper potential limits must have the same number of inputs."
			)
			return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Potential limits input",
			"Lower and upper potential limits must be numeric values or 'OCP'."
		)
		return False

	# Upper potential limit is greater than the lower limit
	for lbound, ubound in zip(lbounds_list, ubounds_list):
		if lbound != "OCP" and ubound != "OCP":
			if lbound >= ubound:
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Potential limits input",
					"Upper potential limit must be greater than the lower potential limit for a given experiment."
				)
				return False
		elif lbound == "OCP" and ubound == "OCP":
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Potential limits input",
				"Lower and upper potential limits cannot both be set to 'OCP' for a given experiment."
			)
			return False

	# Start potentials
	try:
		startpots_str = cv_params_startpot_entry.text().strip()
		startpots_list = [startpot.strip() for startpot in startpots_str.split(",")]
		for i, startpot in enumerate(startpots_list):
			if startpot.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				startpots_list[i] = "OCP"
			else:
				startpots_list[i] = float(startpot)

		# Start potentials are same length as potential limits
		if len(startpots_list) != len(lbounds_list):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Start potentials input",
				"Start potentials must be the same length as potential limits."
			)
			return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Start potentials input",
			"Start potentials must be numeric values or 'OCP'."
		)
		return False

	# Stop potentials
	try:
		stoppots_str = cv_params_stoppot_entry.text().strip()
		stoppots_list = [stoppot.strip() for stoppot in stoppots_str.split(",")]
		for i, stoppot in enumerate(stoppots_list):
			if stoppot.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				stoppots_list[i] = "OCP"
			else:
				stoppots_list[i] = float(stoppot)

		# Stop potentials are same length as potential limits
		if len(stoppots_list) != len(lbounds_list):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Stop potentials input",
				"Stop potentials must be the same length as potential limits."
			)
			return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Stop potentials input",
			"Stop potentials must be a numeric values or 'OCP'."
		)
		return False

	# Scan rates
	try:
		scanrates_str = cv_params_scanrate_entry.text().strip()
		scanrates_list = [float(scanrate.strip()) for scanrate in scanrates_str.split(",")]
		if any(scanrate == 0 for scanrate in scanrates_list):
			raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Scan rates input",
			"Scan rates must be non-zero numeric values."
		)
		return False

	# Determine scanrate / lbound / ubound / startpot compatibility
	for lbound, ubound, startpot in zip(lbounds_list, ubounds_list, startpots_list):
		for scanrate in scanrates_list:
			if scanrate > 0:
				if ubound != "OCP" and startpot != "OCP" and ubound < startpot:
					QtWidgets.QMessageBox.critical(
						mainwidget,
					"Error: Upper potential limit/starting potential/scan rate",
					"For all experiments with a positive scan rate, the start potential must be lower than or equal to the upper potential limit."
					)
					return False

			elif scanrate < 0:
				if lbound != "OCP" and startpot != "OCP" and lbound > startpot:
					QtWidgets.QMessageBox.critical(
						mainwidget,
					"Error: Lower potential limit/starting potential/scan rate",
					"For all experiments with a negative scan rate, the start potential must be greater than or equal to the lower potential limit."
					)
					return False

	# Current limits per scan rate
	try:
		neg_currents_str = cv_params_reverse_current_negative_entry.text().strip()
		pos_currents_str = cv_params_reverse_current_positive_entry.text().strip()

		if neg_currents_str == "":
			neg_currents_list = []
		else:
			neg_currents_list = [neg_current.strip() for neg_current in neg_currents_str.split(",")]
			for i, neg_current in enumerate(neg_currents_list):
				if neg_current.lower() == "none":  # Remove case-sensitivity for "None"
					neg_currents_list[i] = None
				else:
					neg_currents_list[i] = float(neg_current)

			# Negative reverse currents same length as scan rates
			if len(neg_currents_list) != len(scanrates_list):
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Negative reverse currents input",
					"If negative reverse currents are given, there must be one per scan rate."
				)
				return False

		if pos_currents_str == "":
			pos_currents_list = []
		else:
			pos_currents_list = [pos_current.strip() for pos_current in pos_currents_str.split(",")]
			for i, pos_current in enumerate(pos_currents_list):
				if pos_current.lower() == "none":  # Remove case-sensitivity for "None"
					pos_currents_list[i] = None
				else:
					pos_currents_list[i] = float(pos_current)

			# Positive reverse currents same length as scan rates
			if len(pos_currents_list) != len(scanrates_list):
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Positive reverse currents input",
					"If positive reverse currents are given, there must be one per scan rate."
				)
				return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Reverse currents input",
			"Negative and positive reverse currents must be numeric values or 'None'."
		)
		return False

	# Negative currents are negative
	if any(neg_current is not None and neg_current > 0 for neg_current in neg_currents_list):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Negative reverse currents input",
			"Negative reverse currents must be negative or zero."
		)
		return False

	# Positive currents are positive
	if any(pos_current is not None and pos_current < 0 for pos_current in pos_currents_list):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Positive reverse currents input",
			"Positive reverse currents must be positive or zero."
		)
		return False

	# Current limits are present if expected
	if cv_params_reverse_current_negative_checkbox.isChecked() and neg_currents_list == []:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Negative reverse currents input",
			"Negative reverse currents expected but not given."
		)
		return False
	if cv_params_reverse_current_positive_checkbox.isChecked() and pos_currents_list == []:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Positive reverse currents input",
			"Positive reverse current input expected but not given."
		)
		return False

	# Number of cycles per scan rate
	try:
		num_cycles = int(cv_params_num_cycles_entry.text().strip())
		if num_cycles <= 0:
			raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Number of cycles input",
			"Number of cycles input must be a positive integer value."
		)
		return False

	# Samples to average
	try:
		num_samples_str = cv_params_num_samples_entry.text().strip()
		num_samples_list = [int(num_sample.strip()) for num_sample in num_samples_str.split(",")]
		if any(num_sample <= 0 for num_sample in num_samples_list):
			raise ValueError

		if len(num_samples_list) != 1 and len(num_samples_list) != len(scanrates_list):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Samples to average input",
				"Samples to average input must be of either length 1 (to auto-calculate for each scan rate) or the same length as the number of scan rates."
			)
			return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Samples to average input",
			"Samples to average must be a positive integer value or a csv list of positive integer values."
		)
		return False

	# Auto-scale num_samples for each scan rate if only 1 given
	if len(num_samples_list) == 1 and len(scanrates_list) > 1:
		num_sample = num_samples_list[0]
		lowest_scanrate = min(scanrates_list, key=abs)
		lowest_abs_scanrate = abs(lowest_scanrate)
		scaled_num_samples = []
		for scanrate in scanrates_list:
			# Ensure scaled num_samples are always at least 1
			scaled_num_sample = max(1, round((lowest_abs_scanrate/abs(scanrate)) * num_sample))
			scaled_num_samples.append(scaled_num_sample)
		cv_params_num_samples_entry.setText(f"{', '.join(map(str, scaled_num_samples))}")

	# Pre-scan rate delay
	try:
		scanrate_delay = float(cv_params_scanrate_delay_entry.text().strip())
		if scanrate_delay < 0:
			raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Pre-scan rate delay input",
			"Pre-scan rate delay must be a non-negative numeric value."
		)
		return False

	# Determine if all potential windows require OCP equilibration
	if cv_params_pot_window_delay_OCP_checkbox.isChecked():
		OCP_eq = True
	else:
		OCP_eq = False
		for lbound, ubound, startpot, stoppot in zip(lbounds_list, ubounds_list, startpots_list, stoppots_list):
			if "OCP" in (lbound, ubound, startpot, stoppot):
				OCP_eq = True

	# Pre-potential window delay if required
	if not OCP_eq:
		try:
			pot_window_delay = float(cv_params_pot_window_delay_entry.text().strip())
			if pot_window_delay < 0:
				raise ValueError

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Pre-potential window delay input",
				"Pre-potential window delay must be a non-negative numeric value and given if not waiting for OCP equilibration."
			)
			return False

	else:  # Remove the delay
		cv_params_pot_window_delay_OCP_checkbox.setChecked(True)
		cv_params_pot_window_delay_entry.setText("")

	return True

def cv_get_parameters():
	"""Write experiment parameters to a global dictionary."""
	global cv_parameters

	# Initialise parameter dictionary
	cv_parameters = {'type': 'cv'}

	try:
		# Track if OCP to be equilibrated before each potential window
		OCP_bool = False

		# Potential limits
		lbounds = cv_params_lbound_entry.text().strip()
		ubounds = cv_params_ubound_entry.text().strip()
		lbounds_list = [lbound.strip() for lbound in lbounds.split(",")]
		ubounds_list = [ubound.strip() for ubound in ubounds.split(",")]
		for i, lbound in enumerate(lbounds_list):
			if lbound.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				lbounds_list[i] = "OCP"
				OCP_bool = True
			else:
				lbounds_list[i] = float(lbound)
		for i, ubound in enumerate(ubounds_list):
			if ubound.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				ubounds_list[i] = "OCP"
				OCP_bool = True
			else:
				ubounds_list[i] = float(ubound)

		cv_parameters['unique_lbounds'] = lbounds_list
		cv_parameters['unique_ubounds'] = ubounds_list

		# Start and stop potentials
		startpots = cv_params_startpot_entry.text().strip()
		stoppots = cv_params_stoppot_entry.text().strip()
		startpots_list = [startpot.strip() for startpot in startpots.split(",")]
		stoppots_list = [stoppot.strip() for stoppot in stoppots.split(",")]
		for i, startpot in enumerate(startpots_list):
			if startpot.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				startpots_list[i] = "OCP"
				OCP_bool = True
			else:
				startpots_list[i] = float(startpot)
		for i, stoppot in enumerate(stoppots_list):
			if stoppot.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				stoppots_list[i] = "OCP"
				OCP_bool = True
			else:
				stoppots_list[i] = float(stoppot)

		cv_parameters['unique_startpots'] = startpots_list
		cv_parameters['unique_stoppots'] = stoppots_list

		# Scan rates
		scanrate_mV_list = [float(scanrate_mV.strip()) for scanrate_mV in cv_params_scanrate_entry.text().strip().split(",")]
		scanrate_V_list = [scanrate_mV * 1e-3 for scanrate_mV in scanrate_mV_list]  # Convert mV/s to V/s
		cv_parameters['unique_scanrates_mV/s'] = scanrate_mV_list

		# Negative reverse currents per scan rate
		if cv_params_reverse_current_negative_checkbox.isChecked():
			neg_reverse_currents_list = [neg_current.strip() for neg_current in cv_params_reverse_current_negative_entry.text().strip().split(",")]
			for i, neg_current in enumerate(neg_reverse_currents_list):
				if neg_current.lower() == "none":  # Remove case-sensitivity for "None"
					neg_reverse_currents_list[i] = None
				else:
					neg_reverse_currents_list[i] = float(neg_current)
		else:
			neg_reverse_currents_list = [None] * len(scanrate_mV_list)

		# Positive reverse currents per scan rate
		if cv_params_reverse_current_positive_checkbox.isChecked():
			pos_reverse_currents_list = [pos_current.strip() for pos_current in cv_params_reverse_current_positive_entry.text().strip().split(",")]
			for i, pos_current in enumerate(pos_reverse_currents_list):
				if pos_current.lower() == "none":  # Remove case-sensitivity for "None"
					pos_reverse_currents_list[i] = None
				else:
					pos_reverse_currents_list[i] = float(pos_current)
		else:
			pos_reverse_currents_list = [None] * len(scanrate_mV_list)

		# Number of samples to average
		num_samples_list = [int(num_sample.strip()) for num_sample in cv_params_num_samples_entry.text().strip().split(",")]

		# Make lists to loop through all experiments
		exp_lbounds = []
		exp_ubounds = []
		exp_startpots = []
		exp_stoppots = []
		exp_scanrates_V = []
		exp_scanrates_mV = []
		exp_neg_reverse_currents = []
		exp_pos_reverse_currents = []
		exp_num_samples = []

		# Loop through each experiment and store parameters for that index
		for lbound, ubound, startpot, stoppot in zip(lbounds_list, ubounds_list, startpots_list, stoppots_list):
			for i, scanrate in enumerate(scanrate_V_list):
				exp_lbounds.append(lbound)
				exp_ubounds.append(ubound)
				exp_startpots.append(startpot)
				exp_stoppots.append(stoppot)
				exp_scanrates_V.append(scanrate)
				exp_scanrates_mV.append(scanrate_mV_list[i])
				exp_neg_reverse_currents.append(neg_reverse_currents_list[i])
				exp_pos_reverse_currents.append(pos_reverse_currents_list[i])
				exp_num_samples.append(num_samples_list[i])
		cv_parameters['lbound'] = exp_lbounds
		cv_parameters['ubound'] = exp_ubounds
		cv_parameters['startpot'] = exp_startpots
		cv_parameters['stoppot'] = exp_stoppots
		cv_parameters['scanrate'] = exp_scanrates_V
		cv_parameters['scanrate_mV/s'] = exp_scanrates_mV
		cv_parameters['neg_reverse_current'] = exp_neg_reverse_currents
		cv_parameters['pos_reverse_current'] = exp_pos_reverse_currents
		cv_parameters['num_samples'] = exp_num_samples

		# Number of experiments
		cv_parameters['num_experiments'] = len(cv_parameters['lbound'])

		# Number of cycles
		cv_parameters['num_cycles'] = int(cv_params_num_cycles_entry.text().strip())

		# Pre-scan rate delay
		cv_parameters['scanrate_delay'] = float(cv_params_scanrate_delay_entry.text().strip())

		# Pre-potential window delay
		if cv_params_pot_window_delay_OCP_checkbox.isChecked():
			OCP_bool = True
		else:
			cv_parameters['pot_window_delay'] = float(cv_params_pot_window_delay_entry.text().strip())

		# OCP equilibration pre-potential window
		cv_parameters['OCP_bool'] = OCP_bool

		if OCP_bool:
			cv_parameters['current_OCP'] = potential  # Store current measured potential as the current OCP for calculating experiment time
			cv_parameters['OCP_warnings'] = defaultdict(list)  # Initialise list to store OCP compatibility warnings
			cv_parameters['OCP_invalid_strings'] = []
			cv_parameters['future_OCP_valid_bool'] = True  # Initialise validity as True

			for exp_idx in range(cv_parameters['num_experiments']):
				if exp_idx % len(cv_parameters['unique_scanrates_mV/s']) == 0:
					OCP_valid_bool, _ = cv_OCP_valid_bool(exp_idx, write_errors=False, write_warnings=False)
					if not OCP_valid_bool:
						cv_parameters['future_OCP_valid_bool'] = False

		# Experiment notes
		experiment_notes = cv_file_notes_entry.toPlainText()
		if not experiment_notes:
			experiment_notes = "No notes provided."
		cv_parameters['experiment_notes'] = experiment_notes

		# Plot pen colours
		cv_parameters['plot_pen_color'] = defaultdict(lambda: defaultdict(list))
		i = 0
		for lbound, ubound in zip(cv_parameters['unique_lbounds'], cv_parameters['unique_ubounds']):
			potential_window = f"{lbound}/{ubound}"
			for scanrate in cv_parameters['unique_scanrates_mV/s']:
				cv_parameters['plot_pen_color'][potential_window][scanrate] = CB_color_cycle[i % len(CB_color_cycle)]
				i += 1

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Input error",
			"One or more parameters are in the wrong format and cannot be written to a global dictionary."
		)
		return False

	return True

def cv_parameter_warnings():
	"""Give GUI warnings for unused parameters or OCP values which may become invalid."""

	# Unused reverse currents warning
	unused_neg_currents = cv_params_reverse_current_negative_entry.text().strip() != "" and not cv_params_reverse_current_negative_checkbox.isChecked()
	unused_pos_currents = cv_params_reverse_current_positive_entry.text().strip() != "" and not cv_params_reverse_current_positive_checkbox.isChecked()
	if unused_neg_currents or unused_pos_currents:
		QtWidgets.QMessageBox.warning(
			mainwidget,
			"Warning: Reverse currents input",
			"Reverse currents provided as an input but not used.\n\n"
			"Please ensure that this is deliberate!\n\n"
			"Tick the checkbox if you wish to use these values."
		)

	# OCP / 0th cycle warnings
	pot_window_OCP = "OCP" in cv_parameters['lbound'] or "OCP" in cv_parameters['ubound']
	startpot_OCP = "OCP" in cv_parameters['startpot']

	if pot_window_OCP:
		QtWidgets.QMessageBox.warning(
			mainwidget,
			"Warning: Possible OCP complications",
			"One or more experiments use 'OCP' as a potential limit.\n\n"
			"The open-circuit potential (OCP) can vary between experiments, which "
			"may cause your experiments to stop or perform a 0th cycle to reach the potential window "
			"depending on your lower and upper potential limits, start potential, and scan rate direction.\n\n"
			"Please ensure that this is intentional!"
		)

	elif startpot_OCP:
		QtWidgets.QMessageBox.warning(
			mainwidget,
			"Warning: Possible OCP complications",
			"Experiments use 'OCP' as a start potential but not as potential limits.\n\n"
			"The open-circuit potential (OCP) can vary between experiments, which "
			"may cause your experiments to stop or perform a 0th cycle to reach the potential window "
			"depending on your lower and upper potential limits, start potential, and scan rate direction.\n\n"
			"Please ensure that this is intentional!"
		)

	# Warn if any experiment with current limits will require a 0th cycle due to start potential outside of the the sweep window
	if not pot_window_OCP and not startpot_OCP:
		warning_str = ""
		num_experiments = cv_parameters['num_experiments']
		for idx, (startpot, lbound, ubound, scanrate_mV) in enumerate(zip(cv_parameters['startpot'], cv_parameters['lbound'], cv_parameters['ubound'], cv_parameters['scanrate_mV/s'])):
			if cv_parameters['neg_reverse_current'][idx] or cv_parameters['pos_reverse_current'][idx]:
				if (scanrate_mV > 0) and (startpot < lbound) or (scanrate_mV < 0 and startpot > ubound):
					warning_str += f"- Experiment {idx+1}/{num_experiments}: {lbound}/{ubound} V; {scanrate_mV} mV/s\n"

		if warning_str:
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"Warning: Start potential outside of limits",
				"The following experiments will perform a 0th cycle to reach the potential window from the start potential:\n"
				f"{warning_str}\n"
				"Note: If a reverse current limit is reached during the 0th cycle, experiments will be stopped.\n\n"
				"Please ensure that this is intentional!"
			)

	return True

def cv_get_filenames():
	"""Construct filenames for the experiments."""
	global cv_parameters

	try:
		filename_entry = str(cv_file_entry.text().strip())
		if filename_entry == "":
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: No filename provided",
				"Please provide a base filename for the experiments."
			)
			return False

		directory_path = os.path.dirname(filename_entry)
		if directory_path == "":
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: No parent directory specified",
				f"The output files will be stored in the current working directory. Is this okay?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False
			directory_path = os.getcwd()
		elif not os.path.isdir(directory_path):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Directory does not exist",
				f"The directory {directory_path} does not exist."
			)
			return False

		if "_CV_{experiment_info_here}" in filename_entry:
			filename_entry = filename_entry.split("_CV_{experiment_info_here}")[0]
		filename = os.path.basename(filename_entry)
		if filename == "":  # If no filename given, only path
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: No filename provided",
				"Please provide a base filename for the experiments in addition to the path."
			)
			return False

		cv_parameters['directory_path'] = directory_path
		cv_parameters['base_filename'], _ = os.path.splitext(filename)

		exp_filenames = []
		for i in range(cv_parameters['num_experiments']):
			lbound = cv_parameters['lbound'][i]
			ubound = cv_parameters['ubound'][i]
			scanrate_mV = cv_parameters['scanrate_mV/s'][i]
			exp_filename = f"{cv_parameters['base_filename']}_CV_exp{i+1}_{lbound}_{ubound}_V_{scanrate_mV}_mV_s"
			exp_filenames.append(exp_filename)

		cv_parameters['filenames'] = [name + ".dat" for name in exp_filenames]
		cv_parameters['path_filenames'] = [os.path.join(directory_path, name) for name in cv_parameters['filenames']]

		cv_parameters['experiment_info_filename'] = cv_parameters['base_filename'] + "_CV_experiment_info.txt"
		cv_parameters['experiment_info_path_filename'] = os.path.join(directory_path, cv_parameters['experiment_info_filename'])

		for file in cv_parameters['path_filenames']:
			if os.path.isfile(file):
				if QtWidgets.QMessageBox.question(
					mainwidget,
					"Warning: File already exists",
					f"The output file {file} already exists. Do you want to overwrite it?",
					QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
					QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
					return False

		info_file = cv_parameters['experiment_info_path_filename']
		if os.path.isfile(info_file):
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: Results file already exists",
				f"The experiment info output file {info_file} already exists. Do you want to overwrite it?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False

	except Exception as e:
		print(e)
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: File error",
			f"One or more output filepaths are not valid."
		)
		return False

	cv_file_entry.setText(os.path.join(cv_parameters['directory_path'], f"{cv_parameters['base_filename']}_CV_{{experiment_info_here}}"))

	return True

def cv_validate_filenames():
	"""Check validity of files by creating and attempting to open them."""

	for file in cv_parameters.get('path_filenames', []):
		try:
			with open(file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {file} is not valid."
			)
			return False

	info_file = cv_parameters.get('experiment_info_path_filename')
	if info_file:
		try:
			with open(info_file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {info_file} is not valid."
			)
			return False

	return True

def cv_freeze_inputs(freeze):
	"""Function to freeze and unfreeze GUI inputs when experiments are running."""

	if freeze:
		cv_params_lbound_entry.setEnabled(False)
		cv_params_ubound_entry.setEnabled(False)
		cv_params_startpot_entry.setEnabled(False)
		cv_params_stoppot_entry.setEnabled(False)
		cv_params_scanrate_entry.setEnabled(False)
		cv_params_reverse_current_negative_entry.setEnabled(False)
		cv_params_reverse_current_negative_checkbox.setEnabled(False)
		cv_params_reverse_current_positive_entry.setEnabled(False)
		cv_params_reverse_current_positive_checkbox.setEnabled(False)
		cv_params_num_cycles_entry.setEnabled(False)
		cv_params_num_samples_entry.setEnabled(False)
		cv_params_scanrate_delay_entry.setEnabled(False)
		cv_params_pot_window_delay_entry.setEnabled(False)
		cv_params_pot_window_delay_OCP_checkbox.setEnabled(False)
		cv_file_entry.setEnabled(False)
		cv_file_notes_entry.setEnabled(False)
		for i in range(len(current_range_list)):
			cv_range_checkboxes[i].setEnabled(False)
		cv_variables_checkbutton.setEnabled(False)
		software_globals_menu_button.setEnabled(False)

	elif not freeze:
		cv_params_lbound_entry.setEnabled(True)
		cv_params_ubound_entry.setEnabled(True)
		cv_params_startpot_entry.setEnabled(True)
		cv_params_stoppot_entry.setEnabled(True)
		cv_params_scanrate_entry.setEnabled(True)
		cv_params_reverse_current_negative_entry.setEnabled(True)
		cv_params_reverse_current_negative_checkbox.setEnabled(True)
		cv_params_reverse_current_positive_entry.setEnabled(True)
		cv_params_reverse_current_positive_checkbox.setEnabled(True)
		cv_params_num_cycles_entry.setEnabled(True)
		cv_params_num_samples_entry.setEnabled(True)
		cv_params_scanrate_delay_entry.setEnabled(True)
		cv_params_pot_window_delay_entry.setEnabled(True)
		cv_params_pot_window_delay_OCP_checkbox.setEnabled(True)
		cv_file_entry.setEnabled(True)
		cv_file_notes_entry.setEnabled(True)
		for i in range(len(current_range_list)):
			cv_range_checkboxes[i].setEnabled(True)
		cv_variables_checkbutton.setEnabled(True)
		software_globals_menu_button.setEnabled(True)


"""CV CORE FUNCTIONS"""

def cv_initialise():
	"""Initialise CV experiments."""
	global state, cv_data, cv_current_exp_index

	# Ensure parameters have been verified using the "CHECK" button
	if not cv_parameters_checked:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Parameters not checked",
			"Click the 'CHECK' button to ensure parameters are appropriate before starting your experiments."
		)
		return False

	# Ensure filenames have been checked
	if not cv_filenames_checked:
		if cv_get_filenames():
			pass
		else:
			return False

	if check_state([States.Idle, States.Stationary_Graph]):

		# Validate filenames before experiments begin
		if cv_validate_filenames():
			pass
		else:
			cv_reset_experiment_controller(mode="interrupted")
			log_message("CV experiments could not initialise due to invalid output filename.")
			return False

		# Turn cell off if under manual control
		set_cell_status(False)

		# Freeze input fields and hide return to live graph button
		cv_freeze_inputs(freeze=True)
		preview_cancel_button.hide()

		# Initialise experiment index
		cv_current_exp_index = 0

		# Write experiment info to summary file
		cv_write_summary_file(cv_current_exp_index, section="initial")

		# Update GUI
		cv_info_expnum_entry.setText(f"{cv_current_exp_index + 1}/{cv_parameters['num_experiments']}")
		cv_info_cyclenum_entry.setText(f"-/{cv_parameters['num_cycles']}")
		log_message("Starting CV experiments...")

		# Initialise cv_data directory
		cv_data = {
			'starttime': defaultdict(float),
			'starttime_readable': defaultdict(str),
			'finishtime_readable': defaultdict(str),
			'experiments_starttime': timeit.default_timer(),

			'neg_current_reversed_cyclenums': defaultdict(list),
			'pos_current_reversed_cyclenums': defaultdict(list),

			'potential_data_finalcycle': defaultdict(lambda: defaultdict(list)),
			'current_data_finalcycle': defaultdict(lambda: defaultdict(list)),
			'potential_data_finalcycle_buffer': [],
			'current_data_finalcycle_buffer': [],

			'potential_data_allcycles': defaultdict(list),
			'current_data_allcycles': defaultdict(list),

			'charge_arr': defaultdict(list),

			'potential_data_currentcycle': [],
			'current_data_currentcycle': [],
		}

		# Re-calculate experiment time
		if not cv_parameters['OCP_bool'] or (cv_parameters['OCP_bool'] and cv_parameters['future_OCP_valid_bool']):
			cv_calculate_experiment_time(cv_current_exp_index, initial=True)

		# Start experiment progress timer to update progress bar
		cv_experiment_progress_timer.start(int(cv_experiment_progress_update_interval))

		# Pass to OCP equilibration controller if required
		if cv_parameters['OCP_bool']:
			OCP_initialise_data_entries(cv_data)
			OCP_equilibration_controller(cv_parameters, cv_data, cv_current_exp_index, equilibrated=False)
		else:
			# Write pre-potential window delay to summary file and update GUI
			cv_write_summary_file(cv_current_exp_index, section="pot_window_delay")
			cv_info_program_state_entry.setText(f"Delay of {cv_parameters['pot_window_delay']} s")

			# Update progress bar style to solid yellow border
			cv_progress_bar.set_solid_yellow_style()

			# Update state
			state = States.Measuring_CV_Delay

			# Launch the delay timer
			cv_delay_timer.start(int(cv_parameters['pot_window_delay'] * 1000))  # Delay input in ms

def cv_start(experiment_index):
	"""Begin the CV experiment."""
	global state, skipcounter
	global cv_data, cv_current_cyclenum, cv_time_data, cv_potential_data, cv_current_data, cv_output_file
	global cv_plot_curve, legend, legend_in_use
	global cv_reverse_current_timeshift, cv_negative_reverse_current_locked_until, cv_positive_reverse_current_locked_until

	if experiment_index is None:
		state = States.Stationary_Graph
		preview_cancel_button.show()
		cv_write_summary_file(experiment_index, section="error")
		cv_reset_experiment_controller(mode="interrupted")
		log_message("Experiments could not start due to experiment_index initialisation error.")
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Initialisation",
			"The experiments could not initialise experiment_index correctly."
		)
		return

	# Write experiment information to summary file
	cv_data['starttime_readable'][experiment_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]
	cv_write_summary_file(experiment_index, section="experiment_start")

	# Open output file and write header
	try:
		cv_output_file = open(cv_parameters['path_filenames'][experiment_index], 'w', 1)
		cv_output_file.write("Cycle number\tElapsed time (s)\tPotential (V)\tCurrent (A)\n")
	except Exception as e:
		log_message(f"Write to file failed: {e}")
		cv_stop(experiment_index, interrupted=True)
		return

	# Initialise buffers for holding averaged elapsed time, potential, and current data for each cycle
	cv_time_data = AverageBuffer(cv_parameters['num_samples'][experiment_index])
	cv_potential_data = AverageBuffer(cv_parameters['num_samples'][experiment_index])
	cv_current_data = AverageBuffer(cv_parameters['num_samples'][experiment_index])

	# Initialise reverse current timeshift
	cv_reverse_current_timeshift = 0

	# Initialise reverse current time locks at lockout time to avoid early-experiment fluctuations
	cv_negative_reverse_current_locked_until = global_software_settings['cv_reverse_current_lockout']
	cv_positive_reverse_current_locked_until = global_software_settings['cv_reverse_current_lockout']

	# Initialise current cyclenum
	cv_current_cyclenum = 0

	# Construct the potential profile for this experiment
	cv_parameters['potential_profile'] = cv_construct_profile(experiment_index)

	# Initialise the plot area
	Legends.remove_all_legends()
	plot_frame.clear()
	plot_frame.enableAutoRange()
	plot_frame.getAxis('bottom').setTicks(None)
	plot_frame.setLabel('bottom', 'Potential', units="V")
	plot_frame.setLabel('left', 'Current', units="A")
	cv_plot_curve = plot_frame.plot(pen='y')  # Plot CV in yellow
	legend = pyqtgraph.LegendItem(offset=(60, 10))
	legend.setParentItem(plot_frame.plotItem)
	Legends.legends['cv'] = legend
	legend_in_use = 'cv'

	# Re-calculate time remaining and set progress bar style to solid green
	if not cv_parameters['OCP_bool'] or (cv_parameters['OCP_bool'] and cv_parameters['future_OCP_valid_bool']):
		cv_calculate_experiment_time(experiment_index, initial=False)
	cv_progress_bar.set_solid_green_style()

	# Display experiment info in GUI
	cv_info_program_state_entry.setText("Measuring CV")
	cv_info_cyclenum_entry.setText(f"-/{cv_parameters['num_cycles']}")
	log_message(f"Experiment {experiment_index + 1}/{cv_parameters['num_experiments']} started. Saving to: {cv_parameters['filenames'][experiment_index]}")

	# If startpot set to OCP
	startpot = cv_parameters['startpot'][experiment_index]
	if startpot == "OCP":
		startpot = cv_parameters['current_OCP']

	# Initialise DAC and cell for measurements
	set_output(0, startpot)  # Send the starting potential to the DAC
	set_control_mode(False)  # Potentiostatic control
	hardware_manual_control_range_dropdown.setCurrentIndex(0)  # Start at highest current range
	set_current_range()
	time.sleep(.1)  # Allow DAC some time to settle
	set_cell_status(True)  # Cell on
	time.sleep(.1)  # Allow feedback loop some time to settle
	read_potential_current()
	time.sleep(.1)
	read_potential_current()  # Two reads are necessary because each read actually returns the result of the previous conversion
	hardware_manual_control_range_dropdown.setCurrentIndex(
		get_next_enabled_current_range(current_range_from_current(current), experiment="CV"))  # Autorange based on the measured current
	set_current_range()
	time.sleep(.1)
	read_potential_current()
	time.sleep(.1)
	read_potential_current()
	hardware_manual_control_range_dropdown.setCurrentIndex(
		get_next_enabled_current_range(current_range_from_current(current), experiment="CV"))  # Another autorange, just to be sure
	set_current_range()

	# Begin calling cv_update() through periodic_update()
	state = States.Measuring_CV
	skipcounter = 2  # Skip first two data points to suppress artifacts

	# Store experiment start time
	cv_data['starttime'][experiment_index] = timeit.default_timer()

def cv_construct_profile(experiment_index):
	"""Construct the potential profile for the CV sweep in segments with their corresponding cycle numbers."""

	# Cache parameters
	lbound = cv_parameters['lbound'][experiment_index]
	ubound = cv_parameters['ubound'][experiment_index]
	scanrate = cv_parameters['scanrate'][experiment_index]
	startpot = cv_parameters['startpot'][experiment_index]
	stoppot = cv_parameters['stoppot'][experiment_index]
	num_cycles = cv_parameters['num_cycles']

	# Handle OCP as inputs
	if lbound == "OCP":
		lbound = cv_parameters['current_OCP']
	if ubound == "OCP":
		ubound = cv_parameters['current_OCP']
	if startpot == "OCP":
		startpot = cv_parameters['current_OCP']
	if stoppot == "OCP":
		stoppot = cv_parameters['current_OCP']

	segments = []
	running_time = 0.0
	direction = 1 if scanrate > 0 else -1

	def add_segment(start, end, cycle):
		nonlocal running_time
		duration = abs(end - start) / abs(scanrate)
		end_time = running_time + duration
		segments.append((running_time, end_time, start, end, cycle))
		running_time += duration

	# Build the profile
	if startpot < lbound:
		add_segment(startpot, lbound, 0)
		for cycle in range(1, num_cycles + 1):
			add_segment(lbound, ubound, cycle)
			add_segment(ubound, lbound, cycle)
		if stoppot != lbound:
			add_segment(lbound, stoppot, num_cycles + 1)

	elif startpot > ubound:
		add_segment(startpot, ubound, 0)
		for cycle in range(1, num_cycles + 1):
			add_segment(ubound, lbound, cycle)
			add_segment(lbound, ubound, cycle)
		if stoppot != ubound:
			add_segment(ubound, stoppot, num_cycles + 1)

	elif startpot == lbound:
		for cycle in range(1, num_cycles + 1):
			add_segment(lbound, ubound, cycle)
			add_segment(ubound, lbound, cycle)
		if stoppot != lbound:
			add_segment(lbound, stoppot, num_cycles + 1)

	elif startpot == ubound:
		for cycle in range(1, num_cycles + 1):
			add_segment(ubound, lbound, cycle)
			add_segment(lbound, ubound, cycle)
		if stoppot != ubound:
			add_segment(ubound, stoppot, num_cycles + 1)

	elif lbound < startpot < ubound:
		for cycle in range(1, num_cycles + 1):
			if direction > 0:
				add_segment(startpot, ubound, cycle)
				add_segment(ubound, lbound, cycle)
				add_segment(lbound, startpot, cycle)
			else:
				add_segment(startpot, lbound, cycle)
				add_segment(lbound, ubound, cycle)
				add_segment(ubound, startpot, cycle)
		if stoppot != startpot:
			if (direction > 0 and stoppot > startpot) or (direction < 0 and stoppot < startpot):
				add_segment(startpot, stoppot, num_cycles + 1)
			else:
				if direction > 0:
					add_segment(startpot, ubound, num_cycles + 1)
					add_segment(ubound, stoppot, num_cycles + 1)
				else:
					add_segment(startpot, lbound, num_cycles + 1)
					add_segment(lbound, stoppot, num_cycles + 1)

	return segments

def cv_sweep(time_elapsed, profile_segments):
	"""Use the constructed potential profile and elapsed time to return potential and cycle number."""

	for start_time, end_time, startpot, endpot, cycle_num in profile_segments:
		if start_time <= time_elapsed < end_time:
			fraction = (time_elapsed - start_time) / (end_time - start_time)
			potential = startpot + fraction * (endpot - startpot)
			return potential, cycle_num

	return None, None  # Elapsed time beyond final segment, CV finished

def cv_update(experiment_index):
	"""Add new data to the CV measurement."""
	global skipcounter
	global cv_current_cyclenum, cv_reverse_current_timeshift
	global cv_negative_reverse_current_locked_until, cv_positive_reverse_current_locked_until
	global cv_remaining_time

	elapsed_time = timeit.default_timer() - cv_data['starttime'][experiment_index]
	adjusted_time = elapsed_time + cv_reverse_current_timeshift

	lbound = cv_parameters['lbound'][experiment_index]
	ubound = cv_parameters['ubound'][experiment_index]
	potential_window = f"{lbound}/{ubound}"
	scanrate = cv_parameters['scanrate'][experiment_index]
	scanrate_mV = cv_parameters['scanrate_mV/s'][experiment_index]
	previous_cyclenum = cv_current_cyclenum

	cv_output, cv_current_cyclenum = cv_sweep(adjusted_time, cv_parameters['potential_profile'])

	if cv_output is None:  # End of CV scan

		# Store final cycle data
		cv_data['potential_data_finalcycle'][potential_window][scanrate_mV] = cv_data['potential_data_finalcycle_buffer'][:]
		cv_data['current_data_finalcycle'][potential_window][scanrate_mV] = cv_data['current_data_finalcycle_buffer'][:]

		# Stop the experiment
		cv_stop(experiment_index, interrupted=False)
		return

	else:  # Continue CV cycling
		set_output(0, cv_output)  # Output a new potential value
		read_potential_current()  # Read new potential and current

		# If cycle_num has changed, update GUI Cycle number box and initialise data lists for this cycle
		if cv_current_cyclenum != previous_cyclenum:
			cv_info_cyclenum_entry.setText(f"{cv_current_cyclenum}/{cv_parameters['num_cycles']}")
			potential_data = cv_data['potential_data_currentcycle'][:]
			current_data = cv_data['current_data_currentcycle'][:]
			if len(potential_data) > 0 and len(current_data) > 0:
				cv_data['potential_data_allcycles'][previous_cyclenum] = potential_data
				cv_data['current_data_allcycles'][previous_cyclenum] = current_data

			# Reset lists for the next cycle
			cv_data['potential_data_currentcycle'] = []
			cv_data['current_data_currentcycle'] = []

		if skipcounter == 0:  # Process new measurements
			cv_time_data.add_sample(elapsed_time)
			cv_potential_data.add_sample(potential)
			cv_current_data.add_sample(1e-3 * current)  # Convert from mA to A
			if len(cv_time_data.samples) == 0 and len(cv_time_data.averagebuffer) > 0:  # Check if a new average was just calculated

				# Write new data to output file
				try:
					cv_output_file.write("%d\t%e\t%e\t%e\n" % (
						cv_current_cyclenum,
						cv_time_data.averagebuffer[-1],
						cv_potential_data.averagebuffer[-1],
						cv_current_data.averagebuffer[-1]
					))
				except Exception as e:
					log_message(f"Write to file failed: {e}")
					cv_stop(experiment_index, interrupted=True)
					return

				# Append data to list for this cycle
				cv_data['potential_data_currentcycle'].append(cv_potential_data.averagebuffer[-1])
				cv_data['current_data_currentcycle'].append(cv_current_data.averagebuffer[-1])

				if cv_current_cyclenum == cv_parameters['num_cycles']:
					cv_data['potential_data_finalcycle_buffer'].append(cv_potential_data.averagebuffer[-1])
					cv_data['current_data_finalcycle_buffer'].append(cv_current_data.averagebuffer[-1])

				# Update plot
				cv_update_plot(experiment_index)

				# If negative current reached
				neg_reverse_current = cv_parameters['neg_reverse_current'][experiment_index]
				pos_reverse_current = cv_parameters['pos_reverse_current'][experiment_index]
				if neg_reverse_current and cv_current_data.averagebuffer[-1] <= neg_reverse_current * 1e-6:  # Convert µA to A
					if elapsed_time > cv_negative_reverse_current_locked_until:

						# Interrupt experiments if current limits surpassed during 0th cycle
						if cv_current_cyclenum == 0:
							cv_data['neg_current_reversed_cyclenums'][experiment_index].append(cv_current_cyclenum)
							log_message("Negative reverse current limit hit during 0th cycle; aborting experiment...")
							cv_write_summary_file(experiment_index, section="neg_reverse_current_0th_cycle")
							cv_stop(experiment_index, interrupted=True)
							return

						# Stop experiment if current limits surpassed during final sweep to stoppot
						final_segment_starttime, final_segment_finishtime, *_ = cv_parameters['potential_profile'][-1]
						if elapsed_time >= final_segment_starttime:
							cv_data['neg_current_reversed_cyclenums'][experiment_index].append(cv_current_cyclenum)
							time_to_skip = final_segment_finishtime - elapsed_time
							cv_reverse_current_timeshift += time_to_skip + 1  # Shift time to just beyond final segment
							if cv_remaining_time is not None:
								cv_remaining_time -= time_to_skip
							return

						# Calculate time to lower bound and shift experiment time
						if elapsed_time > cv_parameters['potential_profile'][1][0]:
							cv_data['neg_current_reversed_cyclenums'][experiment_index].append(cv_current_cyclenum)
							if lbound == "OCP":
								lbound = cv_parameters['current_OCP']
							time_to_lbound = abs(cv_potential_data.averagebuffer[-1] - lbound) / abs(scanrate)  # Use most recent raw potential value
							cv_reverse_current_timeshift += (time_to_lbound * 2)  # Shift experiment time to opposite scan rate direction
							if cv_remaining_time is not None:
								cv_remaining_time -= (time_to_lbound * 2)  # Update CV remaining time

							# Update the lockout time to prevent re-triggering current limit
							cv_negative_reverse_current_locked_until = elapsed_time + global_software_settings['cv_reverse_current_lockout']

				if pos_reverse_current and cv_current_data.averagebuffer[-1] >= pos_reverse_current * 1e-6:  # Convert µA to A
					if elapsed_time > cv_positive_reverse_current_locked_until:

						# Interrupt experiments if current limits surpassed during 0th cycle
						if cv_current_cyclenum == 0:
							cv_data['pos_current_reversed_cyclenums'][experiment_index].append(cv_current_cyclenum)
							log_message("Positive reverse current limit hit during 0th cycle; aborting experiment...")
							cv_write_summary_file(experiment_index, section="pos_reverse_current_0th_cycle")
							cv_stop(experiment_index, interrupted=True)
							return

						# Stop experiment if current limits surpassed during final sweep to stoppot
						final_segment_starttime, final_segment_finishtime, *_ = cv_parameters['potential_profile'][-1]
						if elapsed_time >= final_segment_starttime:
							cv_data['pos_current_reversed_cyclenums'][experiment_index].append(cv_current_cyclenum)
							time_to_skip = final_segment_finishtime - elapsed_time
							cv_reverse_current_timeshift += time_to_skip + 1  # Shift time to just beyond final segment
							if cv_remaining_time is not None:
								cv_remaining_time -= time_to_skip
							return

						# Calculate time to upper bound and shift experiment time
						if elapsed_time > cv_parameters['potential_profile'][1][0]:
							cv_data['pos_current_reversed_cyclenums'][experiment_index].append(cv_current_cyclenum)
							if ubound == "OCP":
								ubound = cv_parameters['current_OCP']
							time_to_ubound = abs(cv_potential_data.averagebuffer[-1] - ubound) / abs(scanrate)  # Use most recent raw potential value
							cv_reverse_current_timeshift += (time_to_ubound * 2)  # Shift experiment time to opposite scan rate direction
							if cv_remaining_time is not None:
								cv_remaining_time -= (time_to_ubound * 2)  # Update CV remaining time

							# Update the lockout time to prevent re-triggering current limit
							cv_positive_reverse_current_locked_until = elapsed_time + global_software_settings['cv_reverse_current_lockout']

			skipcounter = auto_current_range(experiment="CV")
		else:  # Wait until the required number of data points are skipped
			skipcounter -= 1

def cv_stop(experiment_index, interrupted=True):
	"""Finish the CV experiment."""
	global state, cv_current_exp_index

	if check_state([States.Measuring_CV, States.Measuring_CV_OCP_eq, States.Measuring_CV_Delay]):

		set_cell_status(False)  # Cell off
		state = States.Stationary_Graph

		# Close output file
		try:
			cv_output_file.close()
		except:
			pass

		# Save experiment finish time
		cv_data['finishtime_readable'][experiment_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]

		# Integrate current between zero crossings to produce list of inserted/extracted charges
		try:
			cv_data['charge_arr'][experiment_index] = charge_from_cv(cv_time_data.averagebuffer, cv_current_data.averagebuffer)
		except NameError:
			cv_data['charge_arr'][experiment_index] = []

		if interrupted:

			# Write to summary file and close
			cv_write_summary_file(experiment_index, section="interrupted")

			# Reset experiment
			cv_reset_experiment_controller(mode="interrupted")

			# Update GUI
			log_message("*** EXPERIMENTS INTERRUPTED ***")
			QtWidgets.QMessageBox.information(
				mainwidget,
				"CV experiments interrupted",
				"Oh no! CV experiments have been interrupted.\n\nGlobal experiment parameters have been reset."
			)
			preview_cancel_button.show()

		elif not interrupted:

			# Write to summary file
			cv_write_summary_file(experiment_index, section="experiment_end")
			log_message(f"*** Experiment {experiment_index + 1}/{cv_parameters['num_experiments']} completed ***")

			# If not final experiment
			if experiment_index + 1 != cv_parameters['num_experiments']:

				# Re-initialise data for next experiment
				cv_data['potential_data_allcycles'] = defaultdict(list)
				cv_data['current_data_allcycles'] = defaultdict(list)
				cv_data['potential_data_currentcycle'] = []
				cv_data['current_data_currentcycle'] = []
				cv_data['potential_data_finalcycle_buffer'] = []
				cv_data['current_data_finalcycle_buffer'] = []

				# Increment the current experiment index and update GUI
				cv_current_exp_index += 1
				cv_info_expnum_entry.setText(f"{cv_current_exp_index + 1} / {cv_parameters['num_experiments']}")
				cv_info_cyclenum_entry.setText(f"-/{cv_parameters['num_cycles']}")

				# Determine if moving to the next potential window
				next_scanrate_idx = cv_current_exp_index % len(cv_parameters['unique_scanrates_mV/s'])

				# If moving to next potential window
				if next_scanrate_idx == 0:
					if cv_parameters['OCP_bool']:
						OCP_equilibration_controller(cv_parameters, cv_data, cv_current_exp_index, equilibrated=False)
					else:
						# Write pre-potential window delay to summary file and update GUI
						cv_write_summary_file(cv_current_exp_index, section="pot_window_delay")
						cv_info_program_state_entry.setText(f"Delay of {cv_parameters['pot_window_delay']} s")

						# Update progress bar style to solid yellow border
						cv_progress_bar.set_solid_yellow_style()

						# Update state
						state = States.Measuring_CV_Delay

						# Launch the pre-experiment delay timer
						cv_delay_timer.start(int(cv_parameters['pot_window_delay'] * 1000))  # Delay input in ms

				# Moving to next scan rate
				else:
					# Write pre-scan rate delay to summary file and update GUI
					cv_write_summary_file(cv_current_exp_index, section="scanrate_delay")
					cv_info_program_state_entry.setText(f"Delay of {cv_parameters['scanrate_delay']} s")

					# Update progress bar style to solid yellow border
					cv_progress_bar.set_solid_yellow_style()

					# Update state
					state = States.Measuring_CV_Delay

					# Launch the pre-experiment delay timer
					cv_delay_timer.start(int(cv_parameters['scanrate_delay'] * 1000))  # Delay input in ms

			# If final experiment completed
			elif experiment_index+1 == cv_parameters['num_experiments']:

				# Write to summary file and close
				cv_write_summary_file(experiment_index, section="all_experiments_completed")

				# Reset experiment
				cv_reset_experiment_controller(mode="all_experiments_completed")

				# Update GUI
				cv_info_program_state_entry.setText("All experiments completed")
				QtWidgets.QMessageBox.information(
					mainwidget,
					"CV experiments completed",
					"CONGRATULATIONS! All CV experiments have completed successfully.\n\nGlobal experiment parameters have been reset."
				)
				preview_cancel_button.show()

def cv_reset_experiment_controller(mode):
	"""Controller for resetting global experiment parameters."""
	global cv_parameters, cv_data
	global cv_parameters_checked, cv_filenames_checked
	global cv_current_exp_index, cv_current_cyclenum
	global cv_total_time, cv_remaining_time
	global cv_reverse_current_timeshift, cv_negative_reverse_current_locked_until, cv_positive_reverse_current_locked_until

	# Stop timers
	cv_experiment_progress_timer.stop()
	cv_delay_timer.stop()

	if mode == "input_changed":
		if cv_parameters_checked:

			# Reset globals
			cv_parameters_checked = False
			cv_filenames_checked = False
			cv_variables_checkbutton.setStyleSheet("")

			# Reset progress bar
			cv_total_time = None
			cv_remaining_time = None
			cv_update_progress_bar()

			log_message("CV input parameters or program state has changed since the last successful check - check-state has been reset.")

		return

	elif mode == "checkbutton_failed":

		# Reset progress bar
		cv_total_time = None
		cv_remaining_time = None
		cv_update_progress_bar()
		return

	# Ensure output files are closed
	for file in ('cv_output_file', 'cv_summary_file'):
		try:
			if file in globals():
				f = globals()[file]
				if f and hasattr(f, 'close'):
					f.close()
					globals()[file] = None  # Clear reference to file
		except Exception as e:
			log_message(f"Error closing {file}: {e}")

	# Reset globals
	cv_parameters_checked = False
	cv_filenames_checked = False
	cv_variables_checkbutton.setStyleSheet("")
	cv_parameters = {'type': 'cv'}
	cv_data = {}
	cv_current_exp_index = None
	cv_current_cyclenum = None
	cv_total_time = None
	cv_remaining_time = None
	cv_reverse_current_timeshift = 0
	cv_negative_reverse_current_locked_until = -1
	cv_positive_reverse_current_locked_until = -1

	# Reset GUI
	cv_info_expnum_entry.setText("-/-")
	cv_info_cyclenum_entry.setText("-/-")

	# Unfreeze input fields
	cv_freeze_inputs(freeze=False)

	if mode == "all_experiments_completed":
		cv_progress_bar.set_completed_state()

	elif mode == "interrupted":
		cv_info_program_state_entry.setText(f"Experiments interrupted")
		cv_progress_bar.set_interrupted_state()

	elif mode == "OCP_interrupted":
		cv_progress_bar.set_OCP_interrupted_state()


"""CV ACCESSORY FUNCTIONS"""

def cv_write_summary_file(experiment_index, section):
	"""Write summary file for the experiment."""
	global cv_summary_file

	try:
		if section == "initial":
			cv_summary_file = open(cv_parameters['experiment_info_path_filename'], 'w', 1)
			cv_summary_file.write("CV EXPERIMENTS INFORMATION FILE\n*******************************\n")

			cv_summary_file.write(f"\nExperiment notes: {cv_parameters['experiment_notes']}\n\n")

			cv_summary_file.write("\nExperiment information file for the experiments stored in:\n")
			for file in cv_parameters['path_filenames']:
				cv_summary_file.write(f"{file}\n")
			cv_summary_file.write("\n")

			potential_windows = []
			for lbound, ubound in zip(cv_parameters['unique_lbounds'], cv_parameters['unique_ubounds']):
				potential_windows.append((lbound, ubound))
			potential_windows_str = ', '.join(f"[{lbound}, {ubound}]" for lbound, ubound in potential_windows)
			num_scanrates = len(cv_parameters['unique_scanrates_mV/s'])

			current_ranges = []
			for i in range(len(current_range_list)):
				if cv_range_checkboxes[i].isChecked():
					current_ranges.append(current_range_list[i])
			current_range_str = ', '.join(current_ranges)

			if cv_parameters['OCP_bool']:
				pot_window_delay_str = "Wait for OCP equilibration"
			else:
				pot_window_delay_str = f"{cv_parameters['pot_window_delay']}"

			cv_summary_file.write(f"Potential windows cycled through (V): {potential_windows_str}\n")
			cv_summary_file.write(f"Scan rates cycled through (mV/s): {cv_parameters['unique_scanrates_mV/s']}\n")
			cv_summary_file.write(f"Number of experiments: {cv_parameters['num_experiments']}\n")
			cv_summary_file.write(f"Start potentials (V): {cv_parameters['unique_startpots']}\n")
			cv_summary_file.write(f"Stop potentials (V): {cv_parameters['unique_stoppots']}\n")
			cv_summary_file.write(f"Negative reverse current limits per scan rate (µA): {cv_parameters['neg_reverse_current'][:num_scanrates]}\n")
			cv_summary_file.write(f"Positive reverse current limits per scan rate (µA): {cv_parameters['pos_reverse_current'][:num_scanrates]}\n")
			cv_summary_file.write(f"Number of cycles per scan rate: {cv_parameters['num_cycles']}\n")
			cv_summary_file.write(f"Samples to average per scan rate: {cv_parameters['num_samples'][:num_scanrates]}\n")
			cv_summary_file.write(f"Autoranging: {current_range_str}\n")
			cv_summary_file.write(f"Pre-scan rate delay (s): {cv_parameters['scanrate_delay']}\n")
			cv_summary_file.write(f"Pre-potential window delay (s): {pot_window_delay_str}\n")

		elif section == "pot_window_delay":
			cv_summary_file.write(f"\n*** Pre-potential window delay of {cv_parameters['pot_window_delay']} seconds for experiment: {experiment_index + 1}/{cv_parameters['num_experiments']} ***\n")

		elif section == "scanrate_delay":
			cv_summary_file.write(f"\n*** Pre-scan rate delay of {cv_parameters['scanrate_delay']} seconds for experiment: {experiment_index + 1}/{cv_parameters['num_experiments']} ***\n")

		elif section == "experiment_start":
			cv_summary_file.write("\n**************************************\n\tCV MEASUREMENT STARTED\n**************************************\n")
			cv_summary_file.write(f"Filepath: {cv_parameters['path_filenames'][experiment_index]}\n")
			cv_summary_file.write(f"Experiment number: {experiment_index + 1}/{cv_parameters['num_experiments']}")
			cv_summary_file.write("\n")

			cv_summary_file.write(f"Lower/upper potential limits (V): {cv_parameters['lbound'][experiment_index]}, {cv_parameters['ubound'][experiment_index]}\n")
			cv_summary_file.write(f"Starting potential (V): {cv_parameters['startpot'][experiment_index]}\n")
			cv_summary_file.write(f"Stop potential (V): {cv_parameters['stoppot'][experiment_index]}\n")
			cv_summary_file.write(f"Scan rate (mV/s): {cv_parameters['scanrate_mV/s'][experiment_index]}\n")
			cv_summary_file.write(f"Negative/positive reverse current limits (µA): {cv_parameters['neg_reverse_current'][experiment_index]}, {cv_parameters['pos_reverse_current'][experiment_index]}\n")
			cv_summary_file.write(f"Number of cycles: {cv_parameters['num_cycles']}\n")
			cv_summary_file.write(f"Samples to average: {cv_parameters['num_samples'][experiment_index]}\n")
			cv_summary_file.write("\n")

			cv_summary_file.write(f"Experiment start time:  {cv_data['starttime_readable'][experiment_index]}\n")

		elif section == "experiment_end":
			charges_str = ', '.join(f"{value:.2f}" for value in cv_data['charge_arr'][experiment_index])

			cv_summary_file.write(f"Experiment finish time: {cv_data['finishtime_readable'][experiment_index]}\n")
			cv_summary_file.write("\n")

			cv_summary_file.write("****************************************\n\tCV MEASUREMENT COMPLETED\n****************************************\n")
			cv_summary_file.write(f"Calculated charges (µAh): [{charges_str}]\n")
			if cv_parameters['neg_reverse_current'][experiment_index] is not None:
				neg_reverse_cycles = cv_data[f'neg_current_reversed_cyclenums'][experiment_index]
				cv_summary_file.write(f"Negative reverse current reached in cycles: {neg_reverse_cycles}\n")
			if cv_parameters['pos_reverse_current'][experiment_index] is not None:
				pos_reverse_cycles = cv_data[f'pos_current_reversed_cyclenums'][experiment_index]
				cv_summary_file.write(f"Positive reverse current reached in cycles: {pos_reverse_cycles}\n")

		elif section == "all_experiments_completed":
			cv_summary_file.write("\n******************************************************\n\tALL EXPERIMENTS COMPLETED SUCCESSFULLY\n******************************************************\n")
			if cv_parameters['OCP_bool']:
				OCP_values_str = ', '.join(f"{value:.6f}" for value in cv_data['OCP_values'])
				OCP_times_str = ', '.join(f"{value:.3f}" for value in cv_data['OCP_eq_elapsed_times'])

				cv_summary_file.write("\n")
				cv_summary_file.write("*** OCP drift information ***\n")
				cv_summary_file.write(f"OCP value across potential windows (V): {OCP_values_str}\n")
				cv_summary_file.write(f"OCP equilibration times (s): {OCP_times_str}\n")

			cv_summary_file.write("\n")
			cv_summary_file.write("********************\n")
			cv_summary_file.write("EXPERIMENTS COMPLETE\n")
			cv_summary_file.close()

		elif section == "OCP_eq_start":
			pot_window_exp_indexes = list(range(experiment_index, experiment_index + len(cv_parameters['unique_scanrates_mV/s'])))

			cv_summary_file.write("\n*********************************\n\tOCP EQUILIBRATING\n*********************************\n")
			cv_summary_file.write(f"OCP equilibrating for experiments: [{', '.join(str(idx + 1) for idx in pot_window_exp_indexes)}]/{cv_parameters['num_experiments']}\n")
			cv_summary_file.write(f"Equilibration tolerance (mV): {global_software_settings['OCP_eq_tolerance']}\n")
			cv_summary_file.write(f"Equilibration timescale (s): {global_software_settings['OCP_eq_timescale']}\n")
			cv_summary_file.write(f"Equilibration timeout (s): {global_software_settings['OCP_eq_timeout']}\n")
			cv_summary_file.write("\n")

			cv_summary_file.write(f"OCP equilibration start time: {cv_data['OCP_starttime_readable'][experiment_index]}\n")

		elif section == "OCP_equilibrated":
			cv_summary_file.write(f"OCP equilibration finish time: {cv_data['OCP_eq_finishtime_readable'][experiment_index]}\n")
			cv_summary_file.write("\n")

			cv_summary_file.write(f"Elapsed time (s): {cv_data['OCP_eq_elapsed_times'][-1]:.3f}\n")
			cv_summary_file.write(f"Starting potential (V): {cv_data['OCP_eq_startpot'][experiment_index]:.5g}\n")
			cv_summary_file.write(f"Final equilibrated OCP (V): {cv_data['OCP_eq_stoppot'][experiment_index]:.5g}\n")
			cv_summary_file.write(f"Total potential difference (V): {cv_data['OCP_eq_total_pot_diff'][experiment_index]:.5g}\n")
			cv_summary_file.write(f"Final ({global_software_settings['OCP_eq_timescale']}) s potential difference (V): {cv_data['OCP_eq_timescale_pot_diff'][experiment_index]:.5g}\n")
			cv_summary_file.write("\n")

			cv_summary_file.write("***** OCP successfully equilibrated *****\n")

		elif section == "OCP_valid":
			pot_window_exp_indexes = list(range(experiment_index, experiment_index + len(cv_parameters['unique_scanrates_mV/s'])))

			if cv_parameters['OCP_parameters'] == []:
				cv_summary_file.write(f"\nParameters set as OCP for experiments [{', '.join(str(idx + 1) for idx in pot_window_exp_indexes)}]/{cv_parameters['num_experiments']}: None\n")
			else:
				cv_summary_file.write(f"\nParameters set as OCP for experiments [{', '.join(str(idx + 1) for idx in pot_window_exp_indexes)}]/{cv_parameters['num_experiments']}: {', '.join(cv_parameters['OCP_parameters'])}\n")

			cv_summary_file.write(f"{cv_parameters['OCP_valid_text']}\n")
			if cv_parameters['OCP_warnings'][experiment_index]:
				cv_summary_file.write("Warnings:\n")
				for warning in cv_parameters['OCP_warnings'][experiment_index]:
					cv_summary_file.write(f"{warning}\n")

		elif section == "OCP_invalid":
			pot_window_exp_indexes = list(range(experiment_index, experiment_index + len(cv_parameters['unique_scanrates_mV/s'])))
			scanrates_str = ', '.join(str(cv_parameters['scanrate_mV/s'][exp_idx]) for exp_idx in pot_window_exp_indexes)

			cv_summary_file.write("\n*******************************************\n\tCV MEASUREMENTS INTERRUPTED\n*******************************************\n")
			cv_summary_file.write("CV experiments stopped due to OCP becoming incompatible with experiment parameters.\n")
			cv_summary_file.write(f"Parameters set as OCP for experiments [{', '.join(str(idx + 1) for idx in pot_window_exp_indexes)}]/{cv_parameters['num_experiments']}: {', '.join(cv_parameters['OCP_parameters'])}\n")
			cv_summary_file.write(f"OCP (V): {cv_parameters['current_OCP']:.5g}\n")
			cv_summary_file.write(f"\n")

			cv_summary_file.write(f"Experiment parameters for experiments [{', '.join(str(idx + 1) for idx in pot_window_exp_indexes)}]/{cv_parameters['num_experiments']}:\n")
			cv_summary_file.write(f"Lower potential limit (V): {cv_parameters['lbound'][experiment_index]}\n")
			cv_summary_file.write(f"Upper potential limit (V): {cv_parameters['ubound'][experiment_index]}\n")
			cv_summary_file.write(f"Start potential (V): {cv_parameters['startpot'][experiment_index]}\n")
			cv_summary_file.write(f"Scan rates (mV/s): {scanrates_str}\n")
			cv_summary_file.write("\n")

			cv_summary_file.write(f"{cv_parameters['OCP_valid_text']}\n")
			for error_msg in cv_parameters['OCP_invalid_strings']:
				cv_summary_file.write(f"{error_msg}\n")

			cv_summary_file.write("\n")
			cv_summary_file.write("**********************\n")
			cv_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			cv_summary_file.close()

		elif section == "OCP_timeout":
			cv_summary_file.write(f"OCP equilibration timeout time: {cv_data['OCP_timeout_finishtime_readable'][experiment_index]}\n")

			cv_summary_file.write("\n*******************************************\n\tCV MEASUREMENTS INTERRUPTED\n*******************************************\n")
			cv_summary_file.write(f"CV experiments stopped due to OCP equilibration timeout.\n")
			cv_summary_file.write(f"OCP did not equilibrate to within tolerance by the timeout threshold.\n")
			cv_summary_file.write("\n")

			cv_summary_file.write(f"OCP timeout threshold (s): {global_software_settings['OCP_eq_timeout']}\n")
			cv_summary_file.write(f"Elapsed time (s): {cv_data['OCP_timeout_time_elapsed'][experiment_index]:.3g}\n")
			cv_summary_file.write(f"Starting potential (V): {cv_data['OCP_timeout_startpot'][experiment_index]:.5g}\n")
			cv_summary_file.write(f"Final potential (V): {cv_data['OCP_timeout_stoppot'][experiment_index]:.5g}\n")
			cv_summary_file.write(f"Total potential difference (V): {cv_data['OCP_timeout_total_pot_diff'][experiment_index]:.5g}\n")
			cv_summary_file.write(f"Final ({global_software_settings['OCP_eq_timescale']}) s potential difference (V): {cv_data['OCP_timeout_timescale_pot_diff'][experiment_index]:.5g}\n")

			cv_summary_file.write("\n")
			cv_summary_file.write("**********************\n")
			cv_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			cv_summary_file.close()

		elif section == "interrupted":
			charges_str = ', '.join(f"{value:.2f}" for value in cv_data['charge_arr'][experiment_index])

			cv_summary_file.write("\n*******************************************\n\tCV MEASUREMENTS INTERRUPTED\n*******************************************\n")
			cv_summary_file.write(f"Experiment interrupted: {experiment_index + 1}/{cv_parameters['num_experiments']}\n")
			cv_summary_file.write(f"Experiment interruption time: {cv_data['finishtime_readable'][experiment_index]}\n")
			cv_summary_file.write(f"CV measurement interrupted: {cv_parameters['lbound'][experiment_index]}/{cv_parameters['ubound'][experiment_index]} V; {cv_parameters['scanrate_mV/s'][experiment_index]} mV/s\n")
			cv_summary_file.write(f"Interrupted measurement data saved to: {cv_parameters['path_filenames'][experiment_index]}\n")
			cv_summary_file.write("\n")

			cv_summary_file.write(f"Calculated charges (in µAh): [{charges_str}]\n")
			if cv_parameters['neg_reverse_current'][experiment_index] is not None:
				neg_reverse_cycles = cv_data[f'neg_current_reversed_cyclenums'][experiment_index]
				cv_summary_file.write(f"Negative reverse current reached in cycles: {neg_reverse_cycles}\n")
			if cv_parameters['pos_reverse_current'][experiment_index] is not None:
				pos_reverse_cycles = cv_data[f'pos_current_reversed_cyclenums'][experiment_index]
				cv_summary_file.write(f"Positive reverse current reached in cycles: {pos_reverse_cycles}\n")

			cv_summary_file.write("\n")
			cv_summary_file.write("**********************\n")
			cv_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			cv_summary_file.close()

		elif section == "neg_reverse_current_0th_cycle":
			cv_summary_file.write("***** NEGATIVE REVERSE CURRENT REACHED DURING 0th CYCLE *****\n")
			cv_summary_file.write("***** INTERRUPTING EXPERIMENTS *****\n")

		elif section == "pos_reverse_current_0th_cycle":
			cv_summary_file.write("***** POSITIVE REVERSE CURRENT REACHED DURING 0th CYCLE *****\n")
			cv_summary_file.write("***** INTERRUPTING EXPERIMENTS *****\n")

		elif section == "error":
			cv_summary_file.write("\n**********************************************\n\tERROR INITIALISING EXPERIMENTS\n**********************************************\n")

			cv_summary_file.write("\n")
			cv_summary_file.write("**********************\n")
			cv_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			cv_summary_file.close()

	except Exception as e:
		log_message(f"Write to summary file failed: {e}")

def cv_calculate_experiment_time(experiment_index, initial):
	"""Calculate the total and remaining time for the experiments."""
	global cv_total_time, cv_remaining_time

	if initial:
		experiment_index = 0

	experiment_time_remaining = 0
	experiment_indexes = [exp for exp in range(cv_parameters['num_experiments'])]
	experiment_indexes_remaining = experiment_indexes[experiment_index:]

	for i in experiment_indexes_remaining:

		# Skip the delay for the current experiment if not initial
		if initial or i != experiment_index:
			scanrate_idx = i % len(cv_parameters['unique_scanrates_mV/s'])

			# If a new potential window
			if scanrate_idx == 0:
				if cv_parameters['OCP_bool']:
					# If no OCP equilibrations have taken place
					if 'cv_data' not in globals() or not cv_data.get('OCP_eq_elapsed_times', None):
						experiment_time_remaining += global_software_settings['OCP_eq_timescale']

					# If OCP equilibrations have happened, calculate the average time taken for them to complete
					else:
						average_eq_time = sum(cv_data['OCP_eq_elapsed_times']) / len(cv_data['OCP_eq_elapsed_times'])
						experiment_time_remaining += average_eq_time
				else:
					experiment_time_remaining += cv_parameters['pot_window_delay']

			# Moving to next scan rate
			else:
				experiment_time_remaining += cv_parameters['scanrate_delay']

		potential_profile = cv_construct_profile(i)
		profile_time = potential_profile[-1][1]

		experiment_time_remaining += profile_time

	cv_remaining_time = experiment_time_remaining

	if initial:
		cv_total_time = cv_remaining_time

	elif not initial:
		total_time_elapsed = timeit.default_timer() - cv_data['experiments_starttime']
		cv_total_time = total_time_elapsed + cv_remaining_time

	return cv_total_time, cv_remaining_time

def cv_OCP_valid_bool(experiment_index, write_errors=True, write_warnings=True):
	"""Determine whether the equilibrated OCP value is valid for the experiments at this potential window."""

	lbound = cv_parameters['lbound'][experiment_index]
	ubound = cv_parameters['ubound'][experiment_index]
	pot_window_exp_indexes = list(range(experiment_index, experiment_index + len(cv_parameters['unique_scanrates_mV/s'])))
	scanrates = [cv_parameters['scanrate'][idx] for idx in pot_window_exp_indexes]
	startpot = cv_parameters['startpot'][experiment_index]
	num_experiments = cv_parameters['num_experiments']
	OCP_value = cv_parameters['current_OCP']
	OCP_valid_bool = True  # Initialise as True
	warnings_bool = False  # Initialise whether warnings to issue

	if lbound == "OCP":
		if ubound <= OCP_value:
			if write_errors:
				cv_parameters['OCP_invalid_strings'].append("* All experiments: Upper potential limit must be greater than the lower potential limit.")
			OCP_valid_bool = False

	if ubound == "OCP":
		if lbound >= OCP_value:
			if write_errors:
				cv_parameters['OCP_invalid_strings'].append("* All experiments: Lower potential limit must be less than the upper potential limit.")
			OCP_valid_bool = False

	for idx, scanrate in zip(pot_window_exp_indexes, scanrates):
		if startpot == "OCP":
			if scanrate > 0:
				if ubound != "OCP" and ubound < OCP_value:
					if write_errors:
						cv_parameters['OCP_invalid_strings'].append(f"* Experiment {idx + 1}/{num_experiments}: For a positive scan rate, the start potential (in this case, the OCP) must be lower than or equal to the fixed upper potential limit.")
					OCP_valid_bool = False

				elif lbound != "OCP" and lbound > OCP_value:
					if write_warnings:
						cv_parameters['OCP_warnings'][experiment_index].append(f"* Experiment {idx + 1}/{num_experiments} warning: Experiments will run but the starting potential is outside of the potential window.\n\tA 0th cycle will be performed as the sweep from the starting potential (OCP) to the lower potential limit.")
					warnings_bool = True

			elif scanrate < 0:
				if lbound != "OCP" and lbound > OCP_value:
					if write_errors:
						cv_parameters['OCP_invalid_strings'].append(f"* Experiment {idx + 1}/{num_experiments}: For a negative scan rate, the start potential (in this case, the OCP) must be greater than or equal to the fixed lower potential limit.")
					OCP_valid_bool = False

				elif ubound != "OCP" and ubound < OCP_value:
					if write_warnings:
						cv_parameters['OCP_warnings'][experiment_index].append(f"* Experiment {idx + 1}/{num_experiments} warning: Experiments will run but the starting potential is outside of the potential window.\n\tA 0th cycle will be performed as the sweep from the starting potential (OCP) to the upper potential limit.")
					warnings_bool = True

		elif startpot != "OCP":
			if scanrate > 0:
				if ubound == "OCP" and startpot > OCP_value:
					if write_errors:
						cv_parameters['OCP_invalid_strings'].append(f"* Experiment {idx + 1}/{num_experiments}: For a positive scan rate, the upper potential limit (in this case, the OCP) must be greater than or equal to the fixed starting potential.")
					OCP_valid_bool = False

				elif lbound == "OCP" and startpot < OCP_value:
					if write_warnings:
						cv_parameters['OCP_warnings'][experiment_index].append(f"* Experiment {idx + 1}/{num_experiments} warning: Experiments will run but the starting potential is outside of the potential window.\n\tA 0th cycle will be performed as the sweep from the starting potential to the lower potential limit (OCP).")
					warnings_bool = True

			elif scanrate < 0:
				if lbound == "OCP" and startpot < OCP_value:
					if write_errors:
						cv_parameters['OCP_invalid_strings'].append(f"* Experiment {idx + 1}/{num_experiments}: For a negative scan rate, the lower potential limit (in this case, the OCP) must be lower than or equal to the fixed starting potential.")
					OCP_valid_bool = False

				elif ubound == "OCP" and startpot > OCP_value:
					if write_warnings:
						cv_parameters['OCP_warnings'][experiment_index].append(f"* Experiment {idx + 1}/{num_experiments} warning: Experiments will run but the starting potential is outside of the potential window.\n\tA 0th cycle will be performed as the sweep from the starting potential to the upper potential limit (OCP).")
					warnings_bool = True

	if OCP_valid_bool:
		if warnings_bool:
			return True, "All experiment parameters are valid after OCP equilibration for experiments at this potential window (with warnings)."
		else:
			return True, "All experiment parameters are valid after OCP equilibration for experiments at this potential window."
	else:
		return False, "OCP invalid for the following experiments:"

def charge_from_cv(time_arr, current_arr):
	"""Integrate current as a function of time to calculate charge between zero crossings."""

	zero_crossing_indices = []
	charge_arr = []
	running_index = 0
	while running_index < len(current_arr):
		counter = 0
		while running_index < len(current_arr) and current_arr[
			running_index] >= 0.:  # Iterate over a block of positive currents
			running_index += 1
			counter += 1
		if counter >= 10:  # Check if the block holds at least 10 values (this makes the counting immune to noise around zero crossings)
			zero_crossing_indices.append(
				running_index - counter)  # If so, append the index of the start of the block to the list of zero-crossing indices
		counter = 0
		while running_index < len(current_arr) and current_arr[
			running_index] <= 0.:  # Do the same for a block of negative currents
			running_index += 1
			counter += 1
		if counter >= 10:
			zero_crossing_indices.append(running_index - counter)
	for index in range(0, len(zero_crossing_indices) - 1):  # Go over all zero crossings
		zc_index1 = zero_crossing_indices[index]  # Start index
		zc_index2 = zero_crossing_indices[index + 1]  # End index
		charge_arr.append(numpy.trapezoid(current_arr[zc_index1:zc_index2], time_arr[zc_index1:zc_index2]) * 1000. / 3.6)  # Integrate current over time using the trapezoid rule, convert coulomb to µAh

	return charge_arr


"""CV PLOT AND GUI FUNCTIONS"""

def cv_update_plot(experiment_index):
	"""Update the plot with the current CV cycle, and other experiments/cycles depending on GUI inputs."""

	# Clear plot frame and legend
	plot_frame.clear()
	legend.clear()

	# Title for legend
	dummy_item = pyqtgraph.PlotDataItem([], [], pen=None)
	legend.addItem(dummy_item, "Cyclic voltammetry experiments:")

	# Cache current experiment parameters
	lbound = cv_parameters['lbound'][experiment_index]
	ubound = cv_parameters['ubound'][experiment_index]
	current_potential_window = f"{lbound}/{ubound}"
	current_scanrate = cv_parameters['scanrate_mV/s'][experiment_index]

	# Add current cycle to the top of the legend
	legend.addItem(cv_plot_curve, f"Current experiment: {current_potential_window} V; {current_scanrate} mV/s")

	# Plot previous cycles from this experiment
	if cv_plot_options_prev_cycles_checkbox.isChecked():
		cycles = sorted(cv_data['potential_data_allcycles'].keys())
		for i, cycle in enumerate(cycles):
			x = cv_data['potential_data_allcycles'][cycle]
			y = cv_data['current_data_allcycles'][cycle]
			alpha = int(225 * (i + 1) / (len(cycles) + 1))
			pen = pyqtgraph.mkPen(color=(225, 225, 0, alpha))
			plot_frame.plot(x, y, pen=pen)

	# Plot final cycle from previous experiments
	if cv_plot_options_prev_experiments_checkbox.isChecked():
		dropdown_index = cv_plot_options_prev_experiments_dropdown.currentIndex()
		final_data = cv_data.get('potential_data_finalcycle', {})

		for potential_window, scanrate_dict in final_data.items():
			for scanrate, data in scanrate_dict.items():
				if dropdown_index == 1 and potential_window != current_potential_window:
					continue
				if dropdown_index == 2 and scanrate != current_scanrate:
					continue

				color = cv_parameters['plot_pen_color'][potential_window][scanrate]
				pen = pyqtgraph.mkPen(color=color)

				x = cv_data['potential_data_finalcycle'][potential_window][scanrate]
				y = cv_data['current_data_finalcycle'][potential_window][scanrate]
				cv_prev_exp_curve = plot_frame.plot(x, y, pen=pen)
				legend.addItem(cv_prev_exp_curve, f"Previous experiment: {potential_window} V; {scanrate} mV/s")

	# Plot current limits
	if cv_plot_options_reverse_currents_checkbox.isChecked():
		neg_current = cv_parameters['neg_reverse_current'][experiment_index]
		if neg_current:
			line = pyqtgraph.InfiniteLine(
				pos=neg_current * 1e-6,
				angle=0,
				pen=pyqtgraph.mkPen('w', style=QtCore.Qt.DashLine)
			)
			plot_frame.addItem(line)
		pos_current = cv_parameters['pos_reverse_current'][experiment_index]
		if pos_current:
			line = pyqtgraph.InfiniteLine(
				pos=pos_current * 1e-6,
				angle=0,
				pen=pyqtgraph.mkPen('w', style=QtCore.Qt.DashLine)
			)
			plot_frame.addItem(line)

	# Plot current cycle over the top
	x = cv_data['potential_data_currentcycle']
	y = cv_data['current_data_currentcycle']
	cv_plot_curve.setData(x, y)
	plot_frame.addItem(cv_plot_curve)

def cv_experiment_progress_controller():
	"""Called periodically to control updates to the remaining time and progress bar."""
	global cv_remaining_time

	if cv_remaining_time is not None:
		interval = cv_experiment_progress_update_interval / 1000  # Convert interval from ms to s
		if cv_remaining_time >= interval:
			cv_remaining_time -= interval  # Incrementally decrease remaining time
		elif cv_remaining_time < interval:
			cv_remaining_time = 0  # Prevent from decreasing below 0
		cv_update_progress_bar()

def cv_update_progress_bar():
	"""Update the progress bar to reflect percentage of total experiment time elapsed."""

	cv_progress_bar.update_progress_bar(cv_total_time, cv_remaining_time)
	cv_progress_bar.update()



"""_____LINEAR SWEEP VOLTAMMETRY FUNCTIONS_____"""

"""LSV PARAMETER FUNCTIONS"""

def lsv_checkbutton_callback():
	"""Function to control the data-validation process, called when "CHECK" button pressed."""
	global lsv_parameters_checked, lsv_filenames_checked

	# Initialise with parameters_checked = False, filenames_checked = False, and a check button style reset
	lsv_parameters_checked = False
	lsv_filenames_checked = False
	lsv_variables_checkbutton.setStyleSheet("")

	# Remove any previous program state entry
	lsv_info_program_state_entry.setText("No experiments running")

	# Check input parameter formats
	if lsv_validate_inputs():
		pass
	else:
		lsv_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
		log_message("LSV check button failed: Inputs are not in the correct format.")
		return False

	# Write input parameters to global dictionary
	if lsv_get_parameters():
		pass
	else:
		lsv_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
		log_message("LSV check button failed: Could not write experiment parameters to a global dictionary.")
		return False

	# If filename provided, check if filenames are acceptable
	if lsv_file_entry.text().strip() != "":
		if lsv_get_filenames():
			lsv_filenames_checked = True
		else:
			lsv_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
			log_message("LSV check button failed: Experiment filenames could not be constructed.")
			return False

	# Give any parameter warnings
	lsv_parameter_warnings()

	# Set global parameters_checked state as True
	lsv_parameters_checked = True

	# Make check button green
	lsv_variables_checkbutton.setStyleSheet("background-color: green; color: white;")

	# Calculate the time this experiment will take
	lsv_calculate_experiment_time(lsv_current_exp_index, initial=True)

	# Update progress bar and give green border
	lsv_update_progress_bar()
	lsv_progress_bar.set_solid_green_style()

	log_message(f"Check button successful! Experiments are ready to run. Estimated time to complete: {lsv_remaining_time:.1f} s")

	return True

def lsv_validate_inputs():
	"""Ensure inputs are of the correct format."""

	# Start and stop potentials
	try:
		startpots_str = lsv_params_startpot_entry.text().strip()
		stoppots_str = lsv_params_stoppot_entry.text().strip()
		startpots_list = [startpot.strip() for startpot in startpots_str.split(",")]
		stoppots_list = [stoppot.strip() for stoppot in stoppots_str.split(",")]
		for i, startpot in enumerate(startpots_list):
			if startpot.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				startpots_list[i] = "OCP"
			else:
				startpots_list[i] = float(startpot)
		for i, stoppot in enumerate(stoppots_list):
			if stoppot.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				stoppots_list[i] = "OCP"
			else:
				stoppots_list[i] = float(stoppot)

		# Start and stop potentials are the same length
		if len(startpots_list) != len(stoppots_list):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Start/stop potentials input",
				"Start and stop potentials must have the same number of inputs."
			)
			return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Start/stop potentials input",
			"Start and stop potentials must be numeric values or 'OCP'."
		)
		return False

	# Startpot and stoppot must be different values
	for startpot, stoppot in zip(startpots_list, stoppots_list):
		if startpot == stoppot:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Start/stop potentials input",
				"Start and stop potentials cannot be the same value for a given experiment."
			)
			return False

	# Scan rates
	try:
		scanrates_str = lsv_params_scanrate_entry.text().strip()
		scanrates_list = [float(scanrate.strip()) for scanrate in scanrates_str.split(",")]
		if any(scanrate <= 0 for scanrate in scanrates_list):
			raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget, "Error: Scan rates input",
			"Scan rates must be positive numeric values."
		)
		return False

	# Current limits per scan rate
	try:
		neg_currents_str = lsv_params_current_limit_negative_entry.text().strip()
		pos_currents_str = lsv_params_current_limit_positive_entry.text().strip()

		if neg_currents_str == "":
			neg_currents_list = []
		else:
			neg_currents_list = [neg_current.strip() for neg_current in neg_currents_str.split(",")]
			for i, neg_current in enumerate(neg_currents_list):
				if neg_current.lower() == "none":  # Remove case-sensitivity for "None"
					neg_currents_list[i] = None
				else:
					neg_currents_list[i] = float(neg_current)

			# Negative current limits same length as scan rates
			if len(neg_currents_list) != len(scanrates_list):
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Negative current limits input",
					"If negative current limits are given, there must be one per LSV scan rate."
				)
				return False

		if pos_currents_str == "":
			pos_currents_list = []
		else:
			pos_currents_list = [pos_current.strip() for pos_current in pos_currents_str.split(",")]
			for i, pos_current in enumerate(pos_currents_list):
				if pos_current.lower() == "none":  # Remove case-sensitivity for "None"
					pos_currents_list[i] = None
				else:
					pos_currents_list[i] = float(pos_current)

			# Positive current limits same length as scan rates
			if len(pos_currents_list) != len(scanrates_list):
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Positive currents limits input",
					"If positive current limits are given, there must be one per LSV scan rate."
				)
				return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Current limits input",
			"Negative and positive current limits must be numeric values or 'None'."
		)
		return False

	# Negative currents are negative
	if any(neg_current is not None and neg_current > 0 for neg_current in neg_currents_list):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Negative current limits input",
			"Negative current limits must be negative or zero."
		)
		return False

	# Positive currents are positive
	if any(pos_current is not None and pos_current < 0 for pos_current in pos_currents_list):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Positive current limits input",
			"Positive current limits must be positive or zero."
		)
		return False

	# Check current limits are present if expected
	if lsv_params_current_limit_negative_checkbox.isChecked() and neg_currents_list == []:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Negative current limits input",
			"Negative current limits expected but not given."
		)
		return False
	if lsv_params_current_limit_positive_checkbox.isChecked() and pos_currents_list == []:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Positive current limits input",
			"Positive current limits expected but not given."
		)
		return False

	# Initialisation scan rates
	try:
		init_scanrates_str = lsv_params_init_scanrate_entry.text().strip()
		if init_scanrates_str == "":
			init_scanrates_list = []
		else:
			init_scanrates_list = [init_scanrate.strip() for init_scanrate in init_scanrates_str.split(",")]
			for i, init_scanrate in enumerate(init_scanrates_list):
				if init_scanrate.lower() == "step":  # Remove case-sensitivity for "STEP"
					init_scanrates_list[i] = "STEP"
				else:
					init_scanrates_list[i] = float(init_scanrate)

			# Initialisation scan rates same length as LSV scan rates
			if len(init_scanrates_list) != len(scanrates_list):
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Initialisation scan rates input",
					"If initialisation scan rates are given, there must be one per LSV scan rate."
				)
				return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Initialisation scan rates input",
			"Initialisation scan rates must be positive numeric values or 'STEP'."
		)
		return False

	# Initialisations can rates are positive
	if any(init_scanrate != "STEP" and init_scanrate <= 0 for init_scanrate in init_scanrates_list):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Initialisation scan rates input",
			"Initialisation scan rates must be positive numeric values or 'STEP'."
		)
		return False

	# Initialisation scan rates are present if expected
	if lsv_params_init_scanrate_checkbox.isChecked() and init_scanrates_list == []:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Initialisation scan rates input",
			"Initialisation scan rates expected but not given."
		)
		return False

	# Initialisation hold time
	try:
		init_hold_time = float(lsv_params_init_holdtime_entry.text().strip())
		if init_hold_time < 0:
			raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Initialisation hold time input",
			"Initialisation hold time must be a non-negative numeric value."
		)
		return False

	# Samples to average
	try:
		num_samples_str = lsv_params_num_samples_entry.text().strip()
		num_samples_list = [int(num_sample.strip()) for num_sample in num_samples_str.split(",")]
		if any(num_sample <= 0 for num_sample in num_samples_list):
			raise ValueError

		if len(num_samples_list) != 1 and len(num_samples_list) != len(scanrates_list):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Samples to average input",
				"Samples to average input must be of either length 1 (to auto-calculate for each LSV scan rate) or the same length as the number of LSV scan rates."
			)
			return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Samples to average input",
			"Samples to average must be a positive integer value or a csv list of positive integer values."
		)
		return False

	# Auto-scale num_samples for each scan rate if only 1 given
	if len(num_samples_list) == 1 and len(scanrates_list) > 1:
		num_sample = num_samples_list[0]
		lowest_scanrate = min(scanrates_list)
		scaled_num_samples = []
		for scanrate in scanrates_list:
			# Ensure scaled num_samples are always at least 1
			scaled_num_sample = max(1, round((lowest_scanrate/scanrate) * num_sample))
			scaled_num_samples.append(scaled_num_sample)
		lsv_params_num_samples_entry.setText(f"{', '.join(map(str, scaled_num_samples))}")

	# Pre-scan rate delay
	try:
		scanrate_delay = float(lsv_params_scanrate_delay_entry.text().strip())
		if scanrate_delay < 0:
			raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Pre-scan rate delay input",
			"Pre-scan rate delay must be a non-negative numeric value."
		)
		return False

	# Determine if all potential windows require OCP equilibration
	if lsv_params_pot_window_delay_OCP_checkbox.isChecked():
		OCP_eq = True
	else:
		OCP_eq = False
		for startpot, stoppot in zip(startpots_list, stoppots_list):
			if "OCP" in (startpot, stoppot):
				OCP_eq = True

	# Pre-potential window delay if required
	if not OCP_eq:
		try:
			pot_window_delay = float(lsv_params_pot_window_delay_entry.text().strip())
			if pot_window_delay < 0:
				raise ValueError

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Pre-potential window delay input",
				"Pre-potential window delay must be a non-negative numeric value and given if not waiting for OCP equilibration."
			)
			return False

	else:  # Remove the delay
		lsv_params_pot_window_delay_OCP_checkbox.setChecked(True)
		lsv_params_pot_window_delay_entry.setText("")

	return True

def lsv_get_parameters():
	""""Write experiment parameters to a global dictionary."""
	global lsv_parameters

	# Initialise parameter directory
	lsv_parameters = {'type': 'lsv'}

	try:
		# Track if OCP to be equilibrated before each potential window
		OCP_bool = False

		# Start and stop potentials
		startpots = lsv_params_startpot_entry.text().strip()
		stoppots = lsv_params_stoppot_entry.text().strip()
		startpots_list = [startpot.strip() for startpot in startpots.split(",")]
		stoppots_list = [stoppot.strip() for stoppot in stoppots.split(",")]
		for i, startpot in enumerate(startpots_list):
			if startpot.lower() == "ocp":
				startpots_list[i] = "OCP"
				OCP_bool = True
			else:
				startpots_list[i] = float(startpot)
		for i, stoppot in enumerate(stoppots_list):
			if stoppot.lower() == "ocp":
				stoppots_list[i] = "OCP"
				OCP_bool = True
			else:
				stoppots_list[i] = float(stoppot)

		lsv_parameters['unique_startpots'] = startpots_list
		lsv_parameters['unique_stoppots'] = stoppots_list

		# Scan rates
		scanrate_mV_list = [float(scanrate_mV.strip()) for scanrate_mV in lsv_params_scanrate_entry.text().strip().split(",")]
		scanrate_V_list = [scanrate_mV * 1e-3 for scanrate_mV in scanrate_mV_list]  # Convert mV/s to V/s
		lsv_parameters['unique_scanrates_mV/s'] = scanrate_mV_list

		# Negative current limits per scan rate
		if lsv_params_current_limit_negative_checkbox.isChecked():
			neg_current_list = [neg_current.strip() for neg_current in lsv_params_current_limit_negative_entry.text().strip().split(",")]
			for i, neg_current in enumerate(neg_current_list):
				if neg_current.lower() == "none":  # Remove case-sensitivity for "None"
					neg_current_list[i] = None
				else:
					neg_current_list[i] = float(neg_current)
		else:
			neg_current_list = [None] * len(scanrate_mV_list)

		# Positive current limits per scan rate
		if lsv_params_current_limit_positive_checkbox.isChecked():
			pos_current_list = [pos_current.strip() for pos_current in lsv_params_current_limit_positive_entry.text().strip().split(",")]
			for i, pos_current in enumerate(pos_current_list):
				if pos_current.lower() == "none":  # Remove case-sensitivity for "None"
					pos_current_list[i] = None
				else:
					pos_current_list[i] = float(pos_current)
		else:
			pos_current_list = [None] * len(scanrate_mV_list)

		# Initialisation scan rates
		if lsv_params_init_scanrate_checkbox.isChecked():
			init_scanrate_mV_list = [scanrate_mV.strip() for scanrate_mV in lsv_params_init_scanrate_entry.text().strip().split(",")]
			init_scanrate_V_list = [scanrate_V.strip() for scanrate_V in lsv_params_init_scanrate_entry.text().strip().split(",")]
			for i, init_scanrate in enumerate(init_scanrate_mV_list):
				if init_scanrate.lower() == "step":  # Remove case-sensitivity for "STEP"
					init_scanrate_mV_list[i] = "STEP"
					init_scanrate_V_list[i] = "STEP"
				else:
					init_scanrate_mV_list[i] = float(init_scanrate)
					init_scanrate_V_list[i] = float(init_scanrate) * 1e-3  # Convert mV/s to V/s
		else:
			init_scanrate_mV_list = ["STEP"] * len(scanrate_mV_list)
			init_scanrate_V_list = ["STEP"] * len(scanrate_mV_list)

		# num_samples per scan rate
		num_sample_list = [int(num_sample.strip()) for num_sample in lsv_params_num_samples_entry.text().strip().split(",")]

		# Make lists to loop through all experiments
		exp_startpots = []
		exp_stoppots = []
		exp_scanrates_V = []
		exp_scanrates_mV = []
		exp_neg_current_limits = []
		exp_pos_current_limits = []
		exp_init_scanrates_V = []
		exp_init_scanrates_mV = []
		exp_num_samples = []

		# Loop through each experiment and store parameters for that index
		for startpot, stoppot in zip(startpots_list, stoppots_list):
			for i, scanrate_V in enumerate(scanrate_V_list):
				exp_startpots.append(startpot)
				exp_stoppots.append(stoppot)
				exp_scanrates_mV.append(scanrate_mV_list[i])
				exp_scanrates_V.append(scanrate_V)
				exp_neg_current_limits.append(neg_current_list[i])
				exp_pos_current_limits.append(pos_current_list[i])
				exp_init_scanrates_mV.append(init_scanrate_mV_list[i])
				exp_init_scanrates_V.append(init_scanrate_V_list[i])
				exp_num_samples.append(num_sample_list[i])
		lsv_parameters['startpot'] = exp_startpots
		lsv_parameters['stoppot'] = exp_stoppots
		lsv_parameters['scanrate'] = exp_scanrates_V
		lsv_parameters['scanrate_mV/s'] = exp_scanrates_mV
		lsv_parameters['neg_current_limit'] = exp_neg_current_limits
		lsv_parameters['pos_current_limit'] = exp_pos_current_limits
		lsv_parameters['init_scanrate'] = exp_init_scanrates_V
		lsv_parameters['init_scanrate_mV/s'] = exp_init_scanrates_mV
		lsv_parameters['num_samples'] = exp_num_samples

		# Initialisation hold time
		lsv_parameters['init_hold_time'] = float(lsv_params_init_holdtime_entry.text().strip())

		# Number of experiments
		lsv_parameters['num_experiments'] = len(lsv_parameters['startpot'])

		# Pre-scan rate delay
		lsv_parameters['scanrate_delay'] = float(lsv_params_scanrate_delay_entry.text().strip())

		# Pre-potential window delay
		if lsv_params_pot_window_delay_OCP_checkbox.isChecked():
			OCP_bool = True
		else:
			lsv_parameters['pot_window_delay'] = float(lsv_params_pot_window_delay_entry.text().strip())

		# OCP equilibration pre-potential window
		lsv_parameters['OCP_bool'] = OCP_bool

		# Store current measured potential as the current OCP for calculating experiment time
		if OCP_bool:
			lsv_parameters['current_OCP'] = potential

		# Experiment notes
		experiment_notes = lsv_file_notes_entry.toPlainText()
		if not experiment_notes:
			experiment_notes = "No notes provided."
		lsv_parameters['experiment_notes'] = experiment_notes

		# Plot pen colours
		lsv_parameters['plot_pen_color'] = defaultdict(lambda: defaultdict(list))
		i = 0
		for startpot, stoppot in zip(lsv_parameters['unique_startpots'], lsv_parameters['unique_stoppots']):
			potential_window = f"{startpot}/{stoppot}"
			for scanrate in lsv_parameters['unique_scanrates_mV/s']:
				lsv_parameters['plot_pen_color'][potential_window][scanrate] = CB_color_cycle[i % len(CB_color_cycle)]
				i += 1

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Input error",
			"One or more parameters are in the wrong format and cannot be written to a global dictionary."
		)
		return False

	return True

def lsv_parameter_warnings():
	"""Give GUI warnings for any unused parameters."""

	# Unused negative current limits warning
	if lsv_params_current_limit_negative_entry.text().strip() != "" and not lsv_params_current_limit_negative_checkbox.isChecked():
		QtWidgets.QMessageBox.warning(
			mainwidget,
			"Warning: Negative current limits input",
			"Negative current limits provided as an input but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values."
		)

	# Unused positive current limits warning
	if lsv_params_current_limit_positive_entry.text().strip() != "" and not lsv_params_current_limit_positive_checkbox.isChecked():
		QtWidgets.QMessageBox.warning(
			mainwidget,
			"Warning: Positive current limits input",
			"Positive current limits provided as an input but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values."
		)

	# Unused initialisation scan rates warning
	if lsv_params_init_scanrate_entry.text().strip() != "" and not lsv_params_init_scanrate_checkbox.isChecked():
		QtWidgets.QMessageBox.warning(
			mainwidget,
			"Warning: Initialisation ramp rates input",
			"Initialisation ramp rates provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values, otherwise, the potential will be stepped."
		)

	return True

def lsv_get_filenames():
	"""Construct filenames for the experiments."""
	global lsv_parameters

	try:
		filename_entry = str(lsv_file_entry.text().strip())
		if filename_entry == "":
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: No filename provided",
				"Please provide a base filename for the experiments."
			)
			return False

		directory_path = os.path.dirname(filename_entry)
		if directory_path == "":
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: No parent directory specified",
				f"The output files will be stored in the current working directory. Is this okay?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False
			directory_path = os.getcwd()
		elif not os.path.isdir(directory_path):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Directory does not exist",
				f"The directory {directory_path} does not exist."
			)
			return False

		if "_LSV_{experiment_info_here}" in filename_entry:
			filename_entry = filename_entry.split("_LSV_{experiment_info_here}")[0]
		filename = os.path.basename(filename_entry)
		if filename == "":  # If no filename given, only path
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: No filename provided",
				"Please provide a base filename for the experiments in addition to the path."
			)
			return False

		lsv_parameters['directory_path'] = directory_path
		lsv_parameters['base_filename'], _ = os.path.splitext(filename)

		exp_filenames = []
		for i in range(lsv_parameters['num_experiments']):
			startpot = lsv_parameters['startpot'][i]
			stoppot = lsv_parameters['stoppot'][i]
			scanrate = lsv_parameters['scanrate_mV/s'][i]
			exp_filename = f"{lsv_parameters['base_filename']}_LSV_exp{i+1}_{startpot}_{stoppot}_V_{scanrate}_mV_s"
			exp_filenames.append(exp_filename)

		lsv_parameters['filenames'] = [name + ".dat" for name in exp_filenames]
		lsv_parameters['path_filenames'] = [os.path.join(directory_path, name) for name in lsv_parameters['filenames']]

		lsv_parameters['experiment_info_filename'] = lsv_parameters['base_filename'] + "_LSV_experiment_info.txt"
		lsv_parameters['experiment_info_path_filename'] = os.path.join(directory_path, lsv_parameters['experiment_info_filename'])

		for file in lsv_parameters['path_filenames']:
			if os.path.isfile(file):
				if QtWidgets.QMessageBox.question(
					mainwidget,
					"Warning: File already exists",
					f"The output file {file} already exists. Do you want to overwrite it?",
					QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
					QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
					return False

		info_file = lsv_parameters['experiment_info_path_filename']
		if os.path.isfile(info_file):
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: Results file already exists",
				f"The experiment info output file {info_file} already exists. Do you want to overwrite it?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False

	except Exception as e:
		print(e)
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: File error",
			f"One or more output filepaths are not valid."
		)
		return False

	lsv_file_entry.setText(os.path.join(lsv_parameters['directory_path'], f"{lsv_parameters['base_filename']}_LSV_{{experiment_info_here}}"))

	return True

def lsv_validate_filenames():
	"""Check validity of files by creating and attempting to open them."""

	for file in lsv_parameters.get('path_filenames', []):
		try:
			with open(file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {file} is not valid."
			)
			return False

	info_file = lsv_parameters.get('experiment_info_path_filename')
	if info_file:
		try:
			with open(info_file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {info_file} is not valid."
			)
			return False

	return True

def lsv_freeze_inputs(freeze):
	"""Function to freeze and unfreeze GUI inputs when experiments are running."""

	if freeze:
		lsv_params_startpot_entry.setEnabled(False)
		lsv_params_stoppot_entry.setEnabled(False)
		lsv_params_scanrate_entry.setEnabled(False)
		lsv_params_current_limit_negative_entry.setEnabled(False)
		lsv_params_current_limit_negative_checkbox.setEnabled(False)
		lsv_params_current_limit_positive_entry.setEnabled(False)
		lsv_params_current_limit_positive_checkbox.setEnabled(False)
		lsv_params_init_scanrate_entry.setEnabled(False)
		lsv_params_init_scanrate_checkbox.setEnabled(False)
		lsv_params_init_holdtime_entry.setEnabled(False)
		lsv_params_num_samples_entry.setEnabled(False)
		lsv_params_scanrate_delay_entry.setEnabled(False)
		lsv_params_pot_window_delay_entry.setEnabled(False)
		lsv_params_pot_window_delay_OCP_checkbox.setEnabled(False)
		lsv_file_entry.setEnabled(False)
		lsv_file_notes_entry.setEnabled(False)
		for i in range(len(current_range_list)):
			lsv_range_checkboxes[i].setEnabled(False)
		lsv_variables_checkbutton.setEnabled(False)
		software_globals_menu_button.setEnabled(False)

	elif not freeze:
		lsv_params_startpot_entry.setEnabled(True)
		lsv_params_stoppot_entry.setEnabled(True)
		lsv_params_scanrate_entry.setEnabled(True)
		lsv_params_current_limit_negative_entry.setEnabled(True)
		lsv_params_current_limit_negative_checkbox.setEnabled(True)
		lsv_params_current_limit_positive_entry.setEnabled(True)
		lsv_params_current_limit_positive_checkbox.setEnabled(True)
		lsv_params_init_scanrate_entry.setEnabled(True)
		lsv_params_init_scanrate_checkbox.setEnabled(True)
		lsv_params_init_holdtime_entry.setEnabled(True)
		lsv_params_num_samples_entry.setEnabled(True)
		lsv_params_scanrate_delay_entry.setEnabled(True)
		lsv_params_pot_window_delay_entry.setEnabled(True)
		lsv_params_pot_window_delay_OCP_checkbox.setEnabled(True)
		lsv_file_entry.setEnabled(True)
		lsv_file_notes_entry.setEnabled(True)
		for i in range(len(current_range_list)):
			lsv_range_checkboxes[i].setEnabled(True)
		lsv_variables_checkbutton.setEnabled(True)
		software_globals_menu_button.setEnabled(True)


"""LSV CORE FUNCTIONS"""

def lsv_initialise():
	"""Initialise LSV experiments."""
	global state, lsv_data, lsv_current_exp_index

	# Ensure parameters have been verified using the "CHECK" button
	if not lsv_parameters_checked:
		QtWidgets.QMessageBox.critical(
			mainwidget, "Error: Parameters not checked",
			"Click the 'CHECK' button to ensure parameters are appropriate before starting your experiments."
		)
		return False

	# Ensure filenames have been checked
	if not lsv_filenames_checked:
		if lsv_get_filenames():
			pass
		else:
			return False

	if check_state([States.Idle, States.Stationary_Graph]):

		# Validate filenames before experiments begin
		if lsv_validate_filenames():
			pass
		else:
			lsv_reset_experiment_controller(mode="interrupted")
			log_message("LSV experiments could not initialise due to invalid output filename.")
			return False

		# Turn cell off if under manual control
		set_cell_status(False)

		# Freeze input fields and hide return to live graph button
		lsv_freeze_inputs(freeze=True)
		preview_cancel_button.hide()

		# Initialise experiment index
		lsv_current_exp_index = 0

		# Write experiment info to summary file
		lsv_write_summary_file(lsv_current_exp_index, section="initial")

		# Update GUI
		lsv_info_expnum_entry.setText(f"{lsv_current_exp_index + 1}/{lsv_parameters['num_experiments']}")
		log_message("Starting LSV experiments...")

		# Initialise lsv_data dictionary
		lsv_data = {
			'starttime': defaultdict(float),
			'starttime_readable': defaultdict(str),
			'finishtime_readable': defaultdict(str),
			'experiments_starttime': timeit.default_timer(),

			'prev_time': defaultdict(lambda: defaultdict(list)),
			'prev_potential': defaultdict(lambda: defaultdict(list)),
			'prev_current': defaultdict(lambda: defaultdict(list)),
			'prev_segment_type': defaultdict(lambda: defaultdict(list)),

			'time': [],
			'potential': [],
			'current': [],
			'segment_type': [],
		}

		# Store experiments parameters as they're calculated
		lsv_parameters['init_potential'] = defaultdict(float)
		lsv_parameters['init_sweep_time'] = defaultdict(float)
		lsv_parameters['total_init_time'] = defaultdict(float)
		lsv_parameters['LSV_sweep_time'] = defaultdict(float)
		lsv_parameters['total_experiment_time'] = defaultdict(float)

		# Re-calculate experiment time
		lsv_calculate_experiment_time(lsv_current_exp_index, initial=True)

		# Start experiment progress timer to update progress bar
		lsv_experiment_progress_timer.start(int(lsv_experiment_progress_update_interval))

		# Pass to OCP equilibration controller if required
		if lsv_parameters['OCP_bool']:
			OCP_initialise_data_entries(lsv_data)
			OCP_equilibration_controller(lsv_parameters, lsv_data, lsv_current_exp_index, equilibrated=False)
		else:
			# Write pre-potential window delay to summary file and update GUI
			lsv_write_summary_file(lsv_current_exp_index, section="pot_window_delay")
			lsv_info_program_state_entry.setText(f"Delay of {lsv_parameters['pot_window_delay']} s")

			# Update progress bar style to solid yellow border
			lsv_progress_bar.set_solid_yellow_style()

			# Update state
			state = States.Measuring_LSV_Delay

			# Launch the delay timer
			lsv_delay_timer.start(int(lsv_parameters['pot_window_delay'] * 1000))  # Delay input in ms

def lsv_start(experiment_index):

	global state, skipcounter
	global lsv_data, lsv_parameters, lsv_previous_stage
	global lsv_time_data, lsv_potential_data, lsv_current_data, lsv_output_file
	global lsv_plot_curve, legend, legend_in_use

	if experiment_index is None:
		state = States.Stationary_Graph
		preview_cancel_button.show()
		lsv_write_summary_file(experiment_index, section="error")
		lsv_reset_experiment_controller(mode="interrupted")
		log_message("Experiments could not start due to experiment_index initialisation error.")
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Initialisation",
			"The experiments could not initialise experiment_index correctly."
		)
		return

	# Write experiment information to summary file
	lsv_data['starttime_readable'][experiment_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]
	lsv_write_summary_file(experiment_index, section="experiment_start")

	# Open output file and write header
	try:
		lsv_output_file = open(lsv_parameters['path_filenames'][experiment_index], 'w', 1)
		lsv_output_file.write("Experiment stage\tElapsed time (s)\tPotential (V)\tCurrent (A)\n")
	except Exception as e:
		log_message(f"Write to file failed: {e}")
		lsv_stop(experiment_index, interrupted=True)
		return

	# Initialise buffers for holding averaged elapsed time, potential, and current data for the experiment
	lsv_time_data = AverageBuffer(lsv_parameters['num_samples'][experiment_index])
	lsv_potential_data = AverageBuffer(lsv_parameters['num_samples'][experiment_index])
	lsv_current_data = AverageBuffer(lsv_parameters['num_samples'][experiment_index])

	# Handle "OCP" as start and stop potentials
	startpot = lsv_parameters['startpot'][experiment_index]
	stoppot = lsv_parameters['stoppot'][experiment_index]
	if startpot == "OCP":
		startpot = lsv_parameters['current_OCP']
	if stoppot == "OCP":
		stoppot = lsv_parameters['current_OCP']

	# Store initial potential for experiment
	lsv_parameters['init_potential'][experiment_index] = potential

	# Calculate initialisation time
	if lsv_parameters['init_scanrate'][experiment_index] == "STEP":
		lsv_parameters['init_sweep_time'][experiment_index] = 0
	else:
		init_potential_to_sweep = startpot - potential  # Use current measured potential
		lsv_parameters['init_sweep_time'][experiment_index] = abs(init_potential_to_sweep / lsv_parameters['init_scanrate'][experiment_index])
	lsv_parameters['total_init_time'][experiment_index] = lsv_parameters['init_sweep_time'][experiment_index] + lsv_parameters['init_hold_time']

	# Calculate LSV sweep time
	sweep_potential_to_sweep = stoppot - startpot
	lsv_parameters['LSV_sweep_time'][experiment_index] = abs(sweep_potential_to_sweep / lsv_parameters['scanrate'][experiment_index])

	# Re-calculate total experiment time
	lsv_parameters['total_experiment_time'][experiment_index] = lsv_parameters['total_init_time'][experiment_index] + lsv_parameters['LSV_sweep_time'][experiment_index]

	# Initialise lsv_previous_stage to track progress
	lsv_previous_stage = None

	# Initialise the plot area
	Legends.remove_all_legends()
	plot_frame.clear()
	plot_frame.enableAutoRange()
	plot_frame.getAxis('bottom').setTicks(None)
	plot_frame.setLabel('bottom', 'Potential', units="V")
	plot_frame.setLabel('left', 'Current', units="A")
	lsv_plot_curve = plot_frame.plot(pen='y')  # Plot LSV in yellow
	legend = pyqtgraph.LegendItem(offset=(60, 10))
	legend.setParentItem(plot_frame.plotItem)
	Legends.legends['lsv'] = legend
	legend_in_use = 'lsv'

	# Re-calculate time remaining and set progress bar style to solid green
	lsv_calculate_experiment_time(experiment_index, initial=False)
	lsv_progress_bar.set_solid_green_style()

	# Display experiment info in GUI
	lsv_info_program_state_entry.setText("Measuring LSV")
	lsv_info_current_segment_entry.setText("Initialising")
	log_message(f"Experiment {experiment_index + 1}/{lsv_parameters['num_experiments']} started. Saving to: {lsv_parameters['filenames'][experiment_index]}")

	# Determine potential to send to the DAC
	if lsv_parameters['init_scanrate'][experiment_index] == "STEP":
		setpot = startpot
	else:
		setpot = potential

	# Initialise DAC and cell for measurement
	set_output(0, setpot)
	set_control_mode(False)  # Potentiostatic control
	hardware_manual_control_range_dropdown.setCurrentIndex(0)  # Start at highest current range
	set_current_range()
	time.sleep(.1)  # Allow DAC some time to settle
	set_cell_status(True)  # Cell on
	time.sleep(.1)  # Allow feedback loop some time to settle
	read_potential_current()
	time.sleep(.1)
	read_potential_current()  # Two reads are necessary because each read actually returns the result of the previous conversion
	hardware_manual_control_range_dropdown.setCurrentIndex(
		get_next_enabled_current_range(current_range_from_current(current), experiment="LSV"))  # Autorange based on the measured current
	set_current_range()
	time.sleep(.1)
	read_potential_current()
	time.sleep(.1)
	read_potential_current()
	hardware_manual_control_range_dropdown.setCurrentIndex(
		get_next_enabled_current_range(current_range_from_current(current), experiment="LSV"))  # Another autorange, just to be sure
	set_current_range()

	# Begin calling lsv_update() through periodic_update()
	state = States.Measuring_LSV
	skipcounter = 2  # Skip first two data points to suppress artifacts

	# Store experiment start time
	lsv_data['starttime'][experiment_index] = timeit.default_timer()

def lsv_update(experiment_index):
	"""Add new data to the LSV measurement."""
	global skipcounter
	global lsv_previous_stage, lsv_remaining_time

	elapsed_time = timeit.default_timer() - lsv_data['starttime'][experiment_index]
	total_time = lsv_parameters['total_experiment_time'][experiment_index]

	if elapsed_time >= total_time:  # End of LSV sweep

		potential_window = f"{lsv_parameters['startpot'][experiment_index]}/{lsv_parameters['stoppot'][experiment_index]}"
		scanrate_mV = lsv_parameters['scanrate_mV/s'][experiment_index]

		# Store experiment data
		lsv_data['prev_time'][potential_window][scanrate_mV] = lsv_data['time'][:]
		lsv_data['prev_potential'][potential_window][scanrate_mV] = lsv_data['potential'][:]
		lsv_data['prev_current'][potential_window][scanrate_mV] = lsv_data['current'][:]
		lsv_data['prev_segment_type'][potential_window][scanrate_mV] = lsv_data['segment_type'][:]

		# Stop the experiment
		lsv_stop(experiment_index, interrupted=False)
		return

	else:  # Continue LSV sweep
		lsv_output, lsv_stage = lsv_sweep(experiment_index, elapsed_time)

		# Update GUI if stage has changed
		if lsv_stage != lsv_previous_stage:
			lsv_previous_stage = lsv_stage
			lsv_info_current_segment_entry.setText(f"{lsv_stage}")

		set_output(0, lsv_output)  # Send potential to the DAC
		read_potential_current()  # Read new potential and current

		if skipcounter == 0:
			lsv_time_data.add_sample(elapsed_time)
			lsv_potential_data.add_sample(potential)
			lsv_current_data.add_sample(1e-3 * current)  # Convert from mA to A
			if len(lsv_time_data.samples) == 0 and len(lsv_time_data.averagebuffer) > 0:  # Check if a new average was just calculated

				# Write new data to output file
				try:
					lsv_output_file.write("%s\t%e\t%e\t%e\n" % (
						lsv_stage,
						lsv_time_data.averagebuffer[-1],
						lsv_potential_data.averagebuffer[-1],
						lsv_current_data.averagebuffer[-1]
					))
				except Exception as e:
					log_message(f"Write to file failed: {e}")
					lsv_stop(experiment_index, interrupted=True)
					return

				lsv_data['time'].append(lsv_time_data.averagebuffer[-1])
				lsv_data['potential'].append(lsv_potential_data.averagebuffer[-1])
				lsv_data['current'].append(lsv_current_data.averagebuffer[-1])
				lsv_data['segment_type'].append(lsv_stage)

				# Update plot
				lsv_update_plot(experiment_index)

				# If negative current limit reached
				if lsv_parameters['neg_current_limit'][experiment_index] and lsv_stage == "Sweeping":
					if lsv_current_data.averagebuffer[-1] < lsv_parameters['neg_current_limit'][experiment_index] * 1e-6:  # Convert µA to A
						# Write to summary file
						lsv_write_summary_file(experiment_index, section="neg_current_limit_reached")
						# Update remaining time
						lsv_remaining_time -= (total_time - elapsed_time)
						# Stop the experiment
						lsv_stop(experiment_index, interrupted=False)
						return

				if lsv_parameters['pos_current_limit'][experiment_index] and lsv_stage == "Sweeping":
					if lsv_current_data.averagebuffer[-1] > lsv_parameters['pos_current_limit'][experiment_index] * 1e-6:  # Convert µA to A
						# Write to summary file
						lsv_write_summary_file(experiment_index, section="pos_current_limit_reached")
						# Update remaining time
						lsv_remaining_time -= (total_time - elapsed_time)
						# Stop the experiment
						lsv_stop(experiment_index, interrupted=False)
						return

			skipcounter = auto_current_range(experiment="LSV")

		else:  # Wait until the required number of data points are skipped
			skipcounter -= 1

def lsv_sweep(experiment_index, elapsed_time):
	"""Generate the potential profile for a linear sweep."""

	init_potential = lsv_parameters['init_potential'][experiment_index]
	init_sweep_time = lsv_parameters['init_sweep_time'][experiment_index]
	init_hold_time = lsv_parameters['init_hold_time']
	init_total_time = init_sweep_time + init_hold_time
	total_time = lsv_parameters['total_experiment_time'][experiment_index]

	# Handle "OCP" as start and stop potentials
	startpot = lsv_parameters['startpot'][experiment_index]
	stoppot = lsv_parameters['stoppot'][experiment_index]
	if startpot == "OCP":
		startpot = lsv_parameters['current_OCP']
	if stoppot == "OCP":
		stoppot = lsv_parameters['current_OCP']

	# Initialisation sweep to start potential
	if elapsed_time < init_sweep_time:
		return numpy.interp(elapsed_time, [0, init_sweep_time], [init_potential, startpot]), "Initialising"

	# Hold at start potential
	elif elapsed_time < init_total_time:
		return startpot, "Holding"

	# Perform linear sweep voltammetry
	elif elapsed_time <= total_time:
		return numpy.interp(elapsed_time, [init_total_time, total_time], [startpot, stoppot]), "Sweeping"

def lsv_stop(experiment_index, interrupted=True):
	"""Finish the LSV experiment."""
	global state, lsv_current_exp_index

	if check_state([States.Measuring_LSV, States.Measuring_LSV_OCP_eq, States.Measuring_LSV_Delay]):

		set_cell_status(False)  # Cell off
		state = States.Stationary_Graph

		# Close output file
		try:
			lsv_output_file.close()
		except:
			pass

		# Save experiment finish time
		lsv_data['finishtime_readable'][experiment_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]

		if interrupted:

			# Write to summary file and close
			lsv_write_summary_file(experiment_index, section="interrupted")

			# Reset experiment
			lsv_reset_experiment_controller(mode="interrupted")

			# Update GUI
			log_message("*** EXPERIMENTS INTERRUPTED ***")
			QtWidgets.QMessageBox.information(
				mainwidget,
				"LSV experiments interrupted",
				"Oh no! LSV experiments have been interrupted.\n\nGlobal experiment parameters have been reset."
			)
			preview_cancel_button.show()

		elif not interrupted:

			# Write to summary file
			lsv_write_summary_file(experiment_index, section="experiment_end")
			log_message(f"*** Experiment {experiment_index + 1}/{lsv_parameters['num_experiments']} completed ***")

			# If not final experiment
			if experiment_index + 1 != lsv_parameters['num_experiments']:

				# Re-initialise data for next experiment
				lsv_data['time'] = []
				lsv_data['potential'] = []
				lsv_data['current'] = []
				lsv_data['segment_type'] = []

				# Increment the current experiment index and update GUI
				lsv_current_exp_index += 1
				lsv_info_expnum_entry.setText(f"{lsv_current_exp_index} / {lsv_parameters['num_experiments']}")
				lsv_info_current_segment_entry.setText("-")

				# Determine if moving to the next potential window
				next_scanrate_idx = lsv_current_exp_index % len(lsv_parameters['unique_scanrates_mV/s'])

				# If moving to next potential window
				if next_scanrate_idx == 0:
					if lsv_parameters['OCP_bool']:
						OCP_equilibration_controller(lsv_parameters, lsv_data, lsv_current_exp_index, equilibrated=False)
					else:
						# Write pre-potential window delay to summary file and update GUI
						lsv_write_summary_file(lsv_current_exp_index, section="pot_window_delay")
						lsv_info_program_state_entry.setText(f"Delay of {lsv_parameters['pot_window_delay']} s")

						# Update progress bar style to solid yellow border
						lsv_progress_bar.set_solid_yellow_style()

						# Update state
						state = States.Measuring_LSV_Delay

						# Launch the pre-experiment delay timer
						lsv_delay_timer.start(int(lsv_parameters['pot_window_delay'] * 1000))  # Delay input in ms

				# Moving to next scan rate
				else:
					# Write pre-scan rate delay to summary file and update GUI
					lsv_write_summary_file(lsv_current_exp_index, section="scanrate_delay")
					lsv_info_program_state_entry.setText(f"Delay of {lsv_parameters['scanrate_delay']} s")

					# Update progress bar style to solid yellow border
					lsv_progress_bar.set_solid_yellow_style()

					# Update state
					state = States.Measuring_LSV_Delay

					# Launch the pre-experiment delay timer
					lsv_delay_timer.start(int(lsv_parameters['scanrate_delay'] * 1000))  # Delay input in ms

			# If final experiment completed
			elif experiment_index+1 == lsv_parameters['num_experiments']:

				# Write to summary file and close
				lsv_write_summary_file(experiment_index, section="all_experiments_completed")

				# Reset experiment
				lsv_reset_experiment_controller(mode="all_experiments_completed")

				# Update GUI
				lsv_info_program_state_entry.setText("All experiments completed")
				QtWidgets.QMessageBox.information(
					mainwidget,
					"LSV experiments completed",
					"CONGRATULATIONS! All LSV experiments have completed successfully.\n\nGlobal experiment parameters have been reset."
				)
				preview_cancel_button.show()

def lsv_reset_experiment_controller(mode):
	"""Controller for resetting global experiment parameters."""
	global lsv_parameters, lsv_data
	global lsv_parameters_checked, lsv_filenames_checked
	global lsv_current_exp_index
	global lsv_total_time, lsv_remaining_time

	# Stop timers
	lsv_experiment_progress_timer.stop()
	lsv_delay_timer.stop()

	if mode == "input_changed":
		if lsv_parameters_checked:  # If inputs have changed since last successful check

			# Reset globals
			lsv_parameters_checked = False
			lsv_filenames_checked = False
			lsv_variables_checkbutton.setStyleSheet("")

			# Reset progress bar
			lsv_total_time = None
			lsv_remaining_time = None
			lsv_update_progress_bar()

			# Remove preview if showing
			if legend == Legends.legends['lsv_preview'] and legend_in_use == 'lsv_preview':
				if state == States.NotConnected:
					plot_frame.clear()
					legend.clear()
				elif state in (States.Idle, States.Stationary_Graph):
					preview_cancel()

			log_message("LSV input parameters or program state has changed since the last successful check - check-state has been reset.")

		return

	elif mode == "checkbutton_failed":

		# Reset progress bar
		lsv_total_time = None
		lsv_remaining_time = None
		lsv_update_progress_bar()
		return

	# Ensure output files are closed
	for file in ('lsv_output_file', 'lsv_summary_file'):
		try:
			if file in globals():
				f = globals()[file]
				if f and hasattr(f, 'close'):
					f.close()
					globals()[file] = None  # Clear reference to file
		except Exception as e:
			log_message(f"Error closing {file}: {e}")

	# Reset globals
	lsv_parameters_checked = False
	lsv_filenames_checked = False
	lsv_variables_checkbutton.setStyleSheet("")
	lsv_parameters = {'type' : 'lsv'}
	lsv_data = {}
	lsv_current_exp_index = None
	lsv_total_time = None
	lsv_remaining_time = None

	# Reset GUI
	lsv_info_expnum_entry.setText("-/-")
	lsv_info_current_segment_entry.setText("-")

	# Unfreeze input fields
	lsv_freeze_inputs(freeze=False)

	if mode == "all_experiments_completed":
		lsv_progress_bar.set_completed_state()

	elif mode == "interrupted":
		lsv_info_program_state_entry.setText(f"Experiments interrupted")
		lsv_progress_bar.set_interrupted_state()

	elif mode == "OCP_interrupted":
		lsv_progress_bar.set_OCP_interrupted_state()


"""LSV ACCESSORY FUNCTIONS"""

def lsv_preview():
	"""Generate a preview of the LSV potential profile in the plot window, based on the LSV parameters currently entered in the GUI."""
	global legend, legend_in_use

	if lsv_parameters_checked is False:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Parameters not checked",
			"Click the 'CHECK' button to ensure parameters are appropriate before constructing the preview."
		)
		return False

	if check_state([States.NotConnected, States.Idle, States.Stationary_Graph]):

		init_potentials, init_times = [], []
		lsv_potentials, lsv_times = [], []
		segment_start_end, segment_labels = [], []

		running_time = 0.0
		OCP_bool = lsv_parameters['OCP_bool']
		OCP_in_use = False  # Track whether OCP used as a potential
		num_scanrates = len(lsv_parameters['unique_scanrates_mV/s'])
		if not OCP_bool:
			pot_window_delay = lsv_parameters['pot_window_delay']
		scanrate_delay = lsv_parameters['scanrate_delay']

		# Construct potential profile
		for exp_idx in range(lsv_parameters['num_experiments']):
			init_startpot = potential if exp_idx == 0 else lsv_parameters['stoppot'][exp_idx - 1]
			init_scanrate = lsv_parameters['init_scanrate'][exp_idx]
			init_hold_time = lsv_parameters['init_hold_time']
			lsv_startpot = lsv_parameters['startpot'][exp_idx]
			lsv_stoppot = lsv_parameters['stoppot'][exp_idx]
			lsv_scanrate = lsv_parameters['scanrate'][exp_idx]

			# Handle OCP as inputs
			if init_startpot == "OCP":
				init_startpot = potential
				OCP_in_use = True
			if lsv_startpot == "OCP":
				lsv_startpot = potential
				OCP_in_use = True
			if lsv_stoppot == "OCP":
				lsv_stoppot = potential
				OCP_in_use = True

			# New potential window
			if exp_idx % num_scanrates == 0:
				delay_duration = global_software_settings['OCP_eq_timescale'] if OCP_bool else pot_window_delay
			else:  # New scanrate
				delay_duration = scanrate_delay

			running_time += delay_duration

			# Initialisation sweep
			init_sweep_time = 0 if init_scanrate == "STEP" else abs((lsv_startpot - init_startpot) / init_scanrate)
			init_potentials.append([init_startpot, lsv_startpot, lsv_startpot])
			init_times.append([running_time, running_time + init_sweep_time, running_time + init_sweep_time + init_hold_time])
			running_time += init_sweep_time + init_hold_time

			# LSV sweep
			lsv_sweep_time = abs((lsv_stoppot - lsv_startpot) / lsv_scanrate)
			lsv_potentials.append([lsv_startpot, lsv_stoppot])
			lsv_times.append([running_time, running_time + lsv_sweep_time])
			segment_start_end.append([running_time, running_time + lsv_sweep_time])
			segment_labels.append(f"{exp_idx + 1}")
			running_time += lsv_sweep_time

		# Set up plot area
		Legends.remove_all_legends()
		plot_frame.clear()
		plot_frame.enableAutoRange()
		plot_frame.getAxis('bottom').setTicks(None)
		plot_frame.setLabel('bottom', 'Time', units='s')
		plot_frame.setLabel('left', 'Potential', units='V')

		legend = pyqtgraph.LegendItem(offset=(60, 10))
		legend.setParentItem(plot_frame.plotItem)
		Legends.legends['lsv_preview'] = legend
		legend_in_use = 'lsv_preview'

		sweep_curve = pyqtgraph.PlotDataItem([], [], pen='g')
		init_curve = pyqtgraph.PlotDataItem([], [], pen='w')
		legend.addItem(sweep_curve, "LSV potential profile")
		legend.addItem(init_curve, "Initialisation")

		if OCP_in_use:
			dummy_item = pyqtgraph.PlotDataItem([], [], pen=None)
			legend.addItem(dummy_item, f"(estimating OCP as measured potential: {potential:.3f} V)")

		# Plot segment lines and labels
		all_potentials = [pot for segment in lsv_potentials for pot in segment]
		min_pot = min(all_potentials)
		max_pot = max(all_potentials)
		pot_range = max_pot - min_pot
		margin = pot_range * 0.1

		y0 = min_pot - margin
		y1 = max_pot + margin

		for i, (segment_start, segment_end) in enumerate(segment_start_end):
			start_line = pyqtgraph.PlotDataItem(
				x=[segment_start, segment_start],
				y=[y0, y1],
				pen=pyqtgraph.mkPen('w', width=1, style=QtCore.Qt.DashLine)
			)
			end_line = pyqtgraph.PlotDataItem(
				x=[segment_end, segment_end],
				y=[y0, y1],
				pen=pyqtgraph.mkPen('w', width=1, style=QtCore.Qt.DashLine)
			)
			plot_frame.addItem(start_line)
			plot_frame.addItem(end_line)
			label = pyqtgraph.TextItem(segment_labels[i], color='w', anchor=(0.5, 1))
			label.setPos((segment_start + segment_end) / 2, max_pot)
			plot_frame.addItem(label)

		# Plot potential profiles
		for x, y in zip(init_times, init_potentials):
			plot_frame.plot(x, y, pen='w')
		for x, y in zip(lsv_times, lsv_potentials):
			plot_frame.plot(x, y, pen='g')

		if state in [States.Idle, States.Stationary_Graph]:
			preview_cancel_button.show()

def lsv_write_summary_file(experiment_index, section):
	"""Write summary file for experiment."""
	global lsv_summary_file

	try:
		if section == "initial":
			lsv_summary_file = open(lsv_parameters['experiment_info_path_filename'], 'w', 1)
			lsv_summary_file.write("LSV EXPERIMENTS INFORMATION FILE\n********************************\n")

			lsv_summary_file.write(f"\nExperiment notes: {lsv_parameters['experiment_notes']}\n")

			lsv_summary_file.write("\nExperiment information file for the experiments stored in:\n")
			for file in lsv_parameters['path_filenames']:
				lsv_summary_file.write(f"{file}\n")
			lsv_summary_file.write("\n")

			potential_windows = []
			for startpot, stoppot in zip(lsv_parameters['unique_startpots'], lsv_parameters['unique_stoppots']):
				potential_windows.append((startpot, stoppot))

			potential_windows_str = ', '.join(f"[{startpot}, {stoppot}]" for startpot, stoppot in potential_windows)
			num_scanrates = len(lsv_parameters['unique_scanrates_mV/s'])

			current_ranges = []
			for i in range(len(current_range_list)):
				if cv_range_checkboxes[i].isChecked():
					current_ranges.append(current_range_list[i])
			current_range_str = ', '.join(f"{curr}" for curr in current_ranges)

			if lsv_parameters['OCP_bool']:
				pot_window_delay_str = "Wait for OCP equilibration."
			else:
				pot_window_delay_str = f"{lsv_parameters['pot_window_delay']}"

			lsv_summary_file.write("Linear sweep measurement parameters:\n")
			lsv_summary_file.write(f"Potential windows cycled through (V): {potential_windows_str}\n")
			lsv_summary_file.write(f"Scan rates cycled through (mV/s): {lsv_parameters['unique_scanrates_mV/s']}\n")
			lsv_summary_file.write(f"Number of experiments: {lsv_parameters['num_experiments']}\n")
			lsv_summary_file.write(f"Negative current limits per scan rate (µA): {lsv_parameters['neg_current_limit'][:num_scanrates]}\n")
			lsv_summary_file.write(f"Positive current limits per scan rate (µA): {lsv_parameters['pos_current_limit'][:num_scanrates]}\n")
			lsv_summary_file.write("\n")

			lsv_summary_file.write("Initialisation parameters:\n")
			lsv_summary_file.write(f"Scan rates (mV/s): {lsv_parameters['init_scanrate_mV/s'][:num_scanrates]}\n")
			lsv_summary_file.write(f"Hold time at start potential (s): {lsv_parameters['init_hold_time']}\n")
			lsv_summary_file.write("\n")

			lsv_summary_file.write(f"Samples to average per scan rate: {lsv_parameters['num_samples'][:num_scanrates]}\n")
			lsv_summary_file.write(f"Autoranging: {current_range_str}\n")
			lsv_summary_file.write(f"Pre-scan rate delay (s): {lsv_parameters['scanrate_delay']}\n")
			lsv_summary_file.write(f"Pre-potential window delay (s): {pot_window_delay_str}\n")

		elif section == "pot_window_delay":
			lsv_summary_file.write(f"\n*** Pre-potential window delay of {lsv_parameters['pot_window_delay']} seconds for experiment: {experiment_index + 1}/{lsv_parameters['num_experiments']} ***\n")

		elif section == "scanrate_delay":
			lsv_summary_file.write(f"\n*** Pre-scan rate delay of {lsv_parameters['scanrate_delay']} seconds for experiment: {experiment_index + 1}/{lsv_parameters['num_experiments']} ***\n")

		elif section == "experiment_start":
			lsv_summary_file.write("\n***************************************\n\tLSV MEASUREMENT STARTED\n***************************************\n")
			lsv_summary_file.write(f"Filepath: {lsv_parameters['path_filenames'][experiment_index]}\n")
			lsv_summary_file.write(f"Experiment number: {experiment_index + 1}/{lsv_parameters['num_experiments']}")
			lsv_summary_file.write("\n")

			lsv_summary_file.write(f"Start and stop potentials (V): {lsv_parameters['startpot'][experiment_index]}, {lsv_parameters['stoppot'][experiment_index]}\n")
			lsv_summary_file.write(f"Scan rate (mV/s): {lsv_parameters['scanrate_mV/s'][experiment_index]}\n")
			lsv_summary_file.write(f"Negative/positive current limits (µA): {lsv_parameters['neg_current_limit'][experiment_index]}, {lsv_parameters['pos_current_limit'][experiment_index]}\n")
			lsv_summary_file.write(f"Initialisation start potential (V): {lsv_parameters['init_potential'][experiment_index]}\n")
			lsv_summary_file.write(f"Initialisation scan rate (mV/s): {lsv_parameters['init_scanrate_mV/s'][experiment_index]}\n")
			lsv_summary_file.write(f"Initialisation hold time at start potential (s): {lsv_parameters['init_hold_time']}\n")
			lsv_summary_file.write(f"Samples to average: {lsv_parameters['num_samples'][experiment_index]}\n")
			lsv_summary_file.write("\n")

			lsv_summary_file.write(f"Experiment start time:  {lsv_data['starttime_readable'][experiment_index]}\n")

		elif section == "neg_current_limit_reached":
			lsv_summary_file.write("***** NEGATIVE CURRENT LIMIT REACHED *****\n")

		elif section == "pos_current_limit_reached":
			lsv_summary_file.write("***** POSITIVE CURRENT LIMIT REACHED *****\n")

		elif section == "experiment_end":
			lsv_summary_file.write(f"Experiment finish time: {lsv_data['finishtime_readable'][experiment_index]}\n")
			lsv_summary_file.write("\n")

			lsv_summary_file.write("*****************************************\n\tLSV MEASUREMENT COMPLETED\n*****************************************\n")

		elif section == "all_experiments_completed":
			lsv_summary_file.write("\n******************************************************\n\tALL EXPERIMENTS COMPLETED SUCCESSFULLY\n******************************************************\n")
			if lsv_parameters['OCP_bool']:
				OCP_values_str = ', '.join(f"{value:.6f}" for value in lsv_data['OCP_values'])
				OCP_times_str = ', '.join(f"{value:.3f}" for value in lsv_data['OCP_eq_elapsed_times'])

				lsv_summary_file.write("\n")
				lsv_summary_file.write("*** OCP drift information ***\n")
				lsv_summary_file.write(f"OCP value across potential windows (V): {OCP_values_str}\n")
				lsv_summary_file.write(f"OCP equilibration times (s): {OCP_times_str}\n")

			lsv_summary_file.write("\n")
			lsv_summary_file.write("********************\n")
			lsv_summary_file.write("EXPERIMENTS COMPLETE\n")
			lsv_summary_file.close()

		elif section == "OCP_eq_start":
			pot_window_exp_indexes = list(range(experiment_index, experiment_index + len(lsv_parameters['unique_scanrates_mV/s'])))

			lsv_summary_file.write("\n*********************************\n\tOCP EQUILIBRATING\n*********************************\n")
			lsv_summary_file.write(f"OCP equilibrating for experiments: [{', '.join(str(idx + 1) for idx in pot_window_exp_indexes)}]/{lsv_parameters['num_experiments']}\n")
			lsv_summary_file.write(f"Equilibration tolerance (mV): {global_software_settings['OCP_eq_tolerance']}\n")
			lsv_summary_file.write(f"Equilibration timescale (s): {global_software_settings['OCP_eq_timescale']}\n")
			lsv_summary_file.write(f"Equilibration timeout (s): {global_software_settings['OCP_eq_timeout']}\n")
			lsv_summary_file.write("\n")

			lsv_summary_file.write(f"OCP equilibration start time: {lsv_data['OCP_starttime_readable'][experiment_index]}\n")

		elif section == "OCP_equilibrated":
			lsv_summary_file.write(f"OCP equilibration finish time: {lsv_data['OCP_eq_finishtime_readable'][experiment_index]}\n")
			lsv_summary_file.write("\n")

			lsv_summary_file.write(f"Elapsed time (s): {lsv_data['OCP_eq_elapsed_times'][-1]:.3f}\n")
			lsv_summary_file.write(f"Starting potential (V): {lsv_data['OCP_eq_startpot'][experiment_index]:.5g}\n")
			lsv_summary_file.write(f"Final equilibrated OCP (V): {lsv_data['OCP_eq_stoppot'][experiment_index]:.5g}\n")
			lsv_summary_file.write(f"Total potential difference (V): {lsv_data['OCP_eq_total_pot_diff'][experiment_index]:.5g}\n")
			lsv_summary_file.write(f"Final ({global_software_settings['OCP_eq_timescale']}) s potential difference (V): {lsv_data['OCP_eq_timescale_pot_diff'][experiment_index]:.5g}\n")
			lsv_summary_file.write("\n")

			lsv_summary_file.write("***** OCP successfully equilibrated *****\n")

		elif section == "OCP_timeout":
			lsv_summary_file.write(f"OCP equilibration timeout time: {lsv_data['OCP_timeout_finishtime_readable'][experiment_index]}\n")

			lsv_summary_file.write("\n********************************************\n\tLSV MEASUREMENTS INTERRUPTED\n********************************************\n")
			lsv_summary_file.write(f"LSV experiments stopped due to OCP equilibration timeout.\n")
			lsv_summary_file.write(f"OCP did not equilibrate to within tolerance by the timeout threshold.\n")
			lsv_summary_file.write("\n")

			lsv_summary_file.write(f"OCP timeout threshold (s): {global_software_settings['OCP_eq_timeout']}\n")
			lsv_summary_file.write(f"Elapsed time (s): {lsv_data['OCP_timeout_time_elapsed'][experiment_index]:.3g}\n")
			lsv_summary_file.write(f"Starting potential (V): {lsv_data['OCP_timeout_startpot'][experiment_index]:.5g}\n")
			lsv_summary_file.write(f"Final potential (V): {lsv_data['OCP_timeout_stoppot'][experiment_index]:.5g}\n")
			lsv_summary_file.write(f"Total potential difference (V): {lsv_data['OCP_timeout_total_pot_diff'][experiment_index]:.5g}\n")
			lsv_summary_file.write(f"Final ({global_software_settings['OCP_eq_timescale']}) s potential difference (V): {lsv_data['OCP_timeout_timescale_pot_diff'][experiment_index]:.5g}\n")

			lsv_summary_file.write("\n")
			lsv_summary_file.write("**********************\n")
			lsv_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			lsv_summary_file.close()

		elif section == "interrupted":
			lsv_summary_file.write("\n********************************************\n\tLSV MEASUREMENTS INTERRUPTED\n********************************************\n")
			lsv_summary_file.write(f"Experiment interrupted: {experiment_index + 1}/{lsv_parameters['num_experiments']}\n")
			lsv_summary_file.write(f"Experiment interruption time: {lsv_data['finishtime_readable'][experiment_index]}\n")
			lsv_summary_file.write(f"LSV measurement interrupted: {lsv_parameters['startpot'][experiment_index]}/{lsv_parameters['stoppot'][experiment_index]} V; {lsv_parameters['scanrate_mV/s'][experiment_index]} mV/s\n")
			lsv_summary_file.write(f"Interrupted measurement data saved to: {lsv_parameters['path_filenames'][experiment_index]}\n")

			lsv_summary_file.write("\n")
			lsv_summary_file.write("**********************\n")
			lsv_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			lsv_summary_file.close()

		elif section == "error":
			lsv_summary_file.write("\n**********************************************\n\tERROR INITIALISING EXPERIMENTS\n**********************************************\n")

			lsv_summary_file.write("\n")
			lsv_summary_file.write("**********************\n")
			lsv_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			lsv_summary_file.close()

	except Exception as e:
		log_message(f"Write to summary file failed: {e}")

def lsv_calculate_experiment_time(experiment_index, initial):
	"""Calculate the total and remaining time for the experiments."""
	global lsv_total_time, lsv_remaining_time

	if initial:
		experiment_index = 0

	experiment_time_remaining = 0
	experiment_indexes = [exp for exp in range(lsv_parameters['num_experiments'])]
	experiment_indexes_remaining = experiment_indexes[experiment_index:]

	for i in experiment_indexes_remaining:

		# Skip the delay for the current experiment if not initial
		if initial or i != experiment_index:
			scanrate_idx = i % len(lsv_parameters['unique_scanrates_mV/s'])

			# If a new potential window
			if scanrate_idx == 0:
				if lsv_parameters['OCP_bool']:
					if 'lsv_data' not in globals() or not lsv_data.get('OCP_eq_elapsed_times', None):
						experiment_time_remaining += global_software_settings['OCP_eq_timescale']
					else:
						average_eq_time = sum(lsv_data['OCP_eq_elapsed_times']) / len(lsv_data['OCP_eq_elapsed_times'])
						experiment_time_remaining += average_eq_time

				else:
					experiment_time_remaining += lsv_parameters['pot_window_delay']

			# Moving to next scan rate
			else:
				experiment_time_remaining += lsv_parameters['scanrate_delay']

		if initial:
			init_startpot = potential
		elif not initial and i == experiment_index:
			init_startpot = lsv_parameters['init_potential'][i]
		else:
			init_startpot = lsv_parameters['stoppot'][i-1]

		init_scanrate = lsv_parameters['init_scanrate'][i]
		init_hold_time = lsv_parameters['init_hold_time']
		lsv_startpot = lsv_parameters['startpot'][i]
		lsv_stoppot = lsv_parameters['stoppot'][i]
		lsv_scanrate = lsv_parameters['scanrate'][i]

		if init_startpot == "OCP":
			init_startpot = lsv_parameters['current_OCP']
		if lsv_startpot == "OCP":
			lsv_startpot = lsv_parameters['current_OCP']
		if lsv_stoppot == "OCP":
			lsv_stoppot = lsv_parameters['current_OCP']

		if init_scanrate != "STEP":
			init_sweep_time = abs((lsv_startpot - init_startpot) / init_scanrate)
		else:
			init_sweep_time = 0

		lsv_sweep_time = abs((lsv_stoppot - lsv_startpot) / lsv_scanrate)

		experiment_time_remaining += init_sweep_time + init_hold_time + lsv_sweep_time

	lsv_remaining_time = experiment_time_remaining

	if initial:
		lsv_total_time = lsv_remaining_time

	elif not initial:
		total_time_elapsed = timeit.default_timer() - lsv_data['experiments_starttime']
		lsv_total_time = total_time_elapsed + lsv_remaining_time

	return lsv_total_time, lsv_remaining_time


"""LSV PLOT AND GUI FUNCTIONS"""

def lsv_update_plot(experiment_index):
	"""Update the plot with the current LSV experiment, and other experiments depending on GUI inputs."""

	# Clear the plot frame and legend
	plot_frame.clear()
	legend.clear()

	# Title for legend
	dummy_item = pyqtgraph.PlotDataItem([], [], pen=None)
	legend.addItem(dummy_item, "Linear sweep voltammetry experiments:")

	# Cache current experiment parameters
	startpot = lsv_parameters['startpot'][experiment_index]
	stoppot = lsv_parameters['stoppot'][experiment_index]
	current_potential_window = f"{startpot}/{stoppot}"
	current_scanrate_mV = lsv_parameters['scanrate_mV/s'][experiment_index]

	# Store whether plotting initialisation
	plot_init_bool = lsv_plot_options_init_segments_checkbox.isChecked()

	# Data masks by Initialising/Holding/Sweeping
	segment_types = numpy.array(lsv_data['segment_type'])
	init_indices = numpy.flatnonzero(numpy.isin(segment_types, ["Initialising", "Holding"]))
	lsv_indices = numpy.flatnonzero(segment_types == "Sweeping")

	# Add current experiment to the top of the legend
	legend.addItem(lsv_plot_curve, f"Current experiment: {current_potential_window} V; {current_scanrate_mV} mV/s")

	# Plot previous experiments
	if lsv_plot_options_prev_experiments_checkbox.isChecked():
		dropdown_index = lsv_plot_options_prev_experiments_dropdown.currentIndex()
		for potential_window, scanrate_dict in lsv_data['prev_potential'].items():
			for scanrate_mV, data in scanrate_dict.items():
				if dropdown_index == 1 and potential_window != current_potential_window:
					continue
				if dropdown_index == 2 and scanrate_mV != current_scanrate_mV:
					continue

				segment_types_prev = numpy.array(lsv_data['prev_segment_type'][potential_window][scanrate_mV])
				init_indices_prev = numpy.flatnonzero(numpy.isin(segment_types_prev, ["Initialising", "Holding"]))
				lsv_indices_prev = numpy.flatnonzero(segment_types_prev == "Sweeping")

				x_prev = numpy.array(lsv_data['prev_potential'][potential_window][scanrate_mV])[lsv_indices_prev]
				y_prev = numpy.array(lsv_data['prev_current'][potential_window][scanrate_mV])[lsv_indices_prev]
				color = lsv_parameters['plot_pen_color'][potential_window][scanrate_mV]
				pen = pyqtgraph.mkPen(color=color)
				lsv_prev_exp_curve = plot_frame.plot(x_prev, y_prev, pen=pen)
				legend.addItem(lsv_prev_exp_curve, f"Previous experiment: {potential_window} V; {scanrate_mV} mV/s")

				if plot_init_bool:
					x_init_prev = numpy.array(lsv_data['prev_potential'][potential_window][scanrate_mV])[init_indices_prev]
					y_init_prev = numpy.array(lsv_data['prev_current'][potential_window][scanrate_mV])[init_indices_prev]
					init_pen_prev = pyqtgraph.mkPen(color=color, style=QtCore.Qt.DashLine)
					plot_frame.plot(x_init_prev, y_init_prev, pen=init_pen_prev)

	# Plot current limits
	if lsv_plot_options_current_limits_checkbox.isChecked():
		neg_current = lsv_parameters['neg_current_limit'][experiment_index]
		if neg_current:
			line = pyqtgraph.InfiniteLine(
				pos=neg_current * 1e-6,
				angle=0,
				pen=pyqtgraph.mkPen('w', style=QtCore.Qt.DashLine)
			)
			plot_frame.addItem(line)
		pos_current = lsv_parameters['pos_current_limit'][experiment_index]
		if pos_current:
			line = pyqtgraph.InfiniteLine(
				pos=pos_current * 1e-6,
				angle=0,
				pen=pyqtgraph.mkPen('w', style=QtCore.Qt.DashLine)
			)
			plot_frame.addItem(line)

	# Plot current experiment over the top
	x = numpy.array(lsv_data['potential'])[lsv_indices]
	y = numpy.array(lsv_data['current'])[lsv_indices]
	lsv_plot_curve.setData(x, y)
	plot_frame.addItem(lsv_plot_curve)

	if plot_init_bool:
		x_init = numpy.array(lsv_data['potential'])[init_indices]
		y_init = numpy.array(lsv_data['current'])[init_indices]
		init_pen = pyqtgraph.mkPen(color='y', style=QtCore.Qt.DashLine)
		plot_frame.plot(x_init, y_init, pen=init_pen)

	if lsv_info_current_segment_entry.text() == "Initialising":
		if len(lsv_time_data.averagebuffer) > 1:
			legend.addItem(dummy_item, "")
			legend.addItem(dummy_item, "__________________")
			legend.addItem(dummy_item, "Initialisation sweep:")
			legend.addItem(dummy_item, f"Current potential: {potential:.3f}")
			legend.addItem(dummy_item, f"Next experiment start potential: {lsv_parameters['startpot'][experiment_index]:.3f}")
			legend.addItem(dummy_item, f"Time to start potential: {lsv_parameters['init_sweep_time'][experiment_index] - lsv_time_data.averagebuffer[-1]:.1f}")

	if lsv_info_current_segment_entry.text() == "Holding":
		if len(lsv_time_data.averagebuffer) > 1:
			hold_time_remaining = (lsv_parameters['init_hold_time'] + lsv_parameters['init_sweep_time'][experiment_index]) - lsv_time_data.averagebuffer[-1]
			legend.addItem(dummy_item, "")
			legend.addItem(dummy_item, "_____________")
			legend.addItem(dummy_item, "Holding at start potential:")
			legend.addItem(dummy_item, f"Hold time remaining: {hold_time_remaining:.1f}")

def lsv_experiment_progress_controller():
	"""Called periodically to control updates to the remaining time and progress bar."""
	global lsv_remaining_time

	if lsv_remaining_time is not None:
		interval = lsv_experiment_progress_update_interval / 1000  # Convert interval from ms to s
		if lsv_remaining_time >= interval:
			lsv_remaining_time -= interval  # Incrementally decrease remaining time
		elif lsv_remaining_time < interval:
			lsv_remaining_time = 0  # Prevent from decreasing below 0
		lsv_update_progress_bar()

def lsv_update_progress_bar():
	"""Update the progress bar to reflect percentage of total experiment time elapsed."""

	lsv_progress_bar.update_progress_bar(lsv_total_time, lsv_remaining_time)
	lsv_progress_bar.update()



"""_____GALVANOSTATIC CHARGE/DISCHARGE FUNCTIONS_____"""

"""GCD PARAMETER FUNCTIONS"""

def gcd_checkbutton_callback():
	"""Function to control the data-validation process, called when "CHECK" button pressed."""
	global gcd_parameters_checked, gcd_filenames_checked, gcd_cumulative_halfcyclenum

	# Initialise with gcd_parameters_checked = False, gcd_filenames_checked = False, and a check button style reset
	gcd_parameters_checked = False
	gcd_filenames_checked = False
	gcd_variables_checkbutton.setStyleSheet("")

	# Remove any previous program state entry
	gcd_info_program_state_entry.setText("No experiments running")

	# Check input parameter formats
	if gcd_validate_inputs():
		pass
	else:
		gcd_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
		log_message("GCD check button failed: Inputs are not in the correct format.")
		return False

	# Write input parameters to global dictionary
	if gcd_get_parameters():
		pass
	else:
		gcd_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
		log_message("GCD check button failed: Could not write experiment parameters to a global dictionary.")
		return False

	# If filename provided, check if filenames are acceptable
	if gcd_file_entry.text().strip() != "":
		if gcd_get_filenames():
			gcd_filenames_checked = True
		else:
			gcd_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
			log_message("GCD check button failed: Experiment filenames could not be constructed.")
			return False

	# Give any parameter warnings
	gcd_parameter_warnings()

	# Set global parameters_checked state as True
	gcd_parameters_checked = True

	# Make check button green
	gcd_variables_checkbutton.setStyleSheet("background-color: green; color: white;")

	# Calculate the number of halfcycles this experiment will perform
	gcd_calculate_experiment_halfcycles(initial=True)

	# Initialise cumulative halfcycles for progress bar
	gcd_cumulative_halfcyclenum = 0

	# Update progress bar and give green border
	gcd_update_progress_bar()
	gcd_progress_bar.set_solid_green_style()

	log_message(f"Check button successful! Experiments are ready to run. Halfcycles remaining: {gcd_total_halfcycles}")

	return True

def gcd_validate_inputs():
	"""Ensure inputs are of the correct format."""

	# Potential limits
	try:
		lbounds_str = gcd_params_lbound_entry.text().strip()
		ubounds_str = gcd_params_ubound_entry.text().strip()
		lbounds_list = [lbound.strip() for lbound in lbounds_str.split(",")]
		ubounds_list = [ubound.strip() for ubound in ubounds_str.split(",")]
		for i, lbound in enumerate(lbounds_list):
			if lbound.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				lbounds_list[i] = "OCP"
			else:
				lbounds_list[i] = float(lbound)
		for i, ubound in enumerate(ubounds_list):
			if ubound.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				ubounds_list[i] = "OCP"
			else:
				ubounds_list[i] = float(ubound)

		# Potential limits are the same length
		if len(lbounds_list) != len(ubounds_list):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Potential limits input",
				"Lower and upper potential limits must have the same number of inputs."
			)
			return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Potential limits input",
			"Lower and upper potential limits must be numeric values or 'OCP'."
		)
		return False

	# Upper potential limit is greater than the lower limit
	for lbound, ubound in zip(lbounds_list, ubounds_list):
		if lbound != "OCP" and ubound != "OCP":
			if lbound >= ubound:
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Potential limits input",
					"Upper potential limit must be greater than the lower potential limit for a given experiment."
				)
				return False
		elif lbound == "OCP" and ubound == "OCP":
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Potential limits input",
				"Lower and upper potential limits cannot both be set to 'OCP' for a given experiment."
			)
			return False


	# Charge/discharge currents
	try:
		charge_currents_str = gcd_params_chargecurrent_entry.text().strip()
		discharge_currents_str = gcd_params_dischargecurrent_entry.text().strip()
		charge_currents_list = [float(charge_current.strip()) for charge_current in charge_currents_str.split(",")]
		discharge_currents_list = [float(discharge_current.strip()) for discharge_current in discharge_currents_str.split(",")]

		# Charge/discharge currents are the same length
		if len(charge_currents_list) != len(discharge_currents_list):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Charge/discharge currents input",
				"Charge and discharge currents must have the same number of inputs."
			)
			return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Charge/discharge currents input",
			"Charge and discharge currents must be numeric values."
		)
		return False

	# Charge currents are positive
	if any(charge_current <= 0 for charge_current in charge_currents_list):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Charge currents input",
			"Charge currents must be positive."
		)
		return False

	# Discharge currents are negative
	if any(discharge_current >= 0 for discharge_current in discharge_currents_list):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Discharge currents input",
			"Discharge currents must be negative."
		)
		return False

	# Number of half cycles
	try:
		num_halfcycles = int(gcd_params_num_halfcycles_entry.text().strip())
		if num_halfcycles <= 0:
			raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Number of half cycles input",
			"Number of half cycles must be a positive integer value."
		)
		return False

	# Samples to average
	try:
		num_samples_str = gcd_params_num_samples_entry.text().strip()
		num_samples_list = [int(num_sample.strip()) for num_sample in num_samples_str.split(",")]
		if any(num_sample <= 0 for num_sample in num_samples_list):
			raise ValueError

		if len(num_samples_list) != 1 and len(num_samples_list) != len(charge_currents_list):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Samples to average input",
				"Samples to average input must be of either length 1 (to use for all charge/discharge currents) or the same length as the number of charge/discharge currents."
			)
			return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Samples to average input",
			"Samples to average must be a positive integer value or a csv list of positive integer values."
		)
		return False

	# Set num_samples to 1 for each charge/discharge current if only one given
	if len(num_samples_list) == 1 and len(charge_currents_list) > 1:
		num_samples = []
		for _ in range(len(charge_currents_list)):
			num_samples.append(num_samples_list[0])
		gcd_params_num_samples_entry.setText(f"{', '.join(map(str, num_samples))}")

	# Pre-charge/discharge current delay
	try:
		current_delay = float(gcd_params_current_delay_entry.text().strip())
		if current_delay < 0:
			raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Pre-charge/discharge current delay input",
			"Pre-charge/discharge current delay must be a non-negative numeric value."
		)
		return False

	# Determine if all potential windows require OCP equilibration
	if gcd_params_pot_window_delay_OCP_checkbox.isChecked():
		OCP_eq = True
	else:
		OCP_eq = False
		for lbound, ubound in zip(lbounds_list, ubounds_list):
			if "OCP" in (lbound, ubound):
				OCP_eq = True

	# Pre-potential window delay if required
	if not OCP_eq:
		try:
			pot_window_delay = float(gcd_params_pot_window_delay_entry.text().strip())
			if pot_window_delay < 0:
				raise ValueError

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Pre-potential window delay input",
				"Pre-potential window delay must be a non-negative numeric value and given if not waiting for OCP equilibration."
			)
			return False

	else:  # Remove the delay
		gcd_params_pot_window_delay_OCP_checkbox.setChecked(True)
		gcd_params_pot_window_delay_entry.setText("")

	return True

def gcd_get_parameters():
	"""Write experiment parameters to a global dictionary."""
	global gcd_parameters

	# Initialise parameter dictionary
	gcd_parameters = {'type': 'gcd'}

	try:
		# Track if OCP to be equilibrated before each potential window
		OCP_bool = False

		# Potential limits
		lbounds = gcd_params_lbound_entry.text().strip()
		ubounds = gcd_params_ubound_entry.text().strip()
		lbounds_list = [lbound.strip() for lbound in lbounds.split(",")]
		ubounds_list = [ubound.strip() for ubound in ubounds.split(",")]
		for i, lbound in enumerate(lbounds_list):
			if lbound.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				lbounds_list[i] = "OCP"
				OCP_bool = True
			else:
				lbounds_list[i] = float(lbound)
		for i, ubound in enumerate(ubounds_list):
			if ubound.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				ubounds_list[i] = "OCP"
				OCP_bool = True
			else:
				ubounds_list[i] = float(ubound)

		gcd_parameters['unique_lbounds'] = lbounds_list
		gcd_parameters['unique_ubounds'] = ubounds_list

		# Charge/discharge currents
		charge_currents = gcd_params_chargecurrent_entry.text().strip()
		discharge_currents = gcd_params_dischargecurrent_entry.text().strip()
		charge_currents_list = [float(charge_current.strip()) for charge_current in charge_currents.split(",")]
		discharge_currents_list = [float(discharge_current.strip()) for discharge_current in discharge_currents.split(",")]
		gcd_parameters['unique_charge_currents'] = charge_currents_list
		gcd_parameters['unique_discharge_currents'] = discharge_currents_list

		# Number of samples to average
		num_samples = [int(num_sample.strip()) for num_sample in gcd_params_num_samples_entry.text().strip().split(",")]

		# Make lists to loop through all experiments
		exp_lbounds = []
		exp_ubounds = []
		exp_charge_currents = []
		exp_discharge_currents = []
		exp_num_samples = []

		# Loop through each experiment and store parameters for that index
		for lbound, ubound in zip(lbounds_list, ubounds_list):
			for i, (charge_current, discharge_current) in enumerate(zip(charge_currents_list, discharge_currents_list)):
				exp_lbounds.append(lbound)
				exp_ubounds.append(ubound)
				exp_charge_currents.append(charge_current)
				exp_discharge_currents.append(discharge_current)
				exp_num_samples.append(num_samples[i])
		gcd_parameters['lbound'] = exp_lbounds
		gcd_parameters['ubound'] = exp_ubounds
		gcd_parameters['charge_current'] = exp_charge_currents
		gcd_parameters['discharge_current'] = exp_discharge_currents
		gcd_parameters['num_samples'] = exp_num_samples

		# Number of experiments
		gcd_parameters['num_experiments'] = len(gcd_parameters['lbound'])

		# Number of half and full cycles
		num_halfcycles = gcd_params_num_halfcycles_entry.text().strip()
		gcd_parameters['num_halfcycles'] = int(num_halfcycles)
		gcd_parameters['num_fullcycles'] = int(float(num_halfcycles) / 2)  # Number of whole cycles

		# Initialise prev_num_halfcycles for use in updating num_halfcycles
		gcd_parameters['prev_num_halfcycles'] = gcd_parameters['num_halfcycles']

		# Pre-charge/discharge current delay
		gcd_parameters['current_delay'] = float(gcd_params_current_delay_entry.text().strip())

		# Pre-potential window delay
		if gcd_params_pot_window_delay_OCP_checkbox.isChecked():
			OCP_bool = True
		else:
			gcd_parameters['pot_window_delay'] = float(gcd_params_pot_window_delay_entry.text().strip())

		# OCP equilibration pre-potential window
		gcd_parameters['OCP_bool'] = OCP_bool

		# Experiment notes
		experiment_notes = gcd_file_notes_entry.toPlainText()
		if not experiment_notes:
			experiment_notes = "No notes provided."
		gcd_parameters['experiment_notes'] = experiment_notes

		# Plot pen colours
		gcd_parameters['plot_pen_color'] = defaultdict(lambda: defaultdict(list))
		i = 0
		for lbound, ubound in zip(gcd_parameters['unique_lbounds'], gcd_parameters['unique_ubounds']):
			potential_window = f"{lbound}/{ubound}"
			for charge_current, discharge_current in zip(gcd_parameters['unique_charge_currents'], gcd_parameters['unique_discharge_currents']):
				cd_current = f"{charge_current}/{discharge_current}"
				gcd_parameters['plot_pen_color'][potential_window][cd_current] = CB_color_cycle[i % len(CB_color_cycle)]
				i += 1

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Input error",
			"One or more parameters are in the wrong format and cannot be written to a global dictionary."
		)
		return False

	return True

def gcd_parameter_warnings():
	"""Give GUI warnings for OCP values which may become invalid."""

	if "OCP" in gcd_parameters['lbound'] or "OCP" in gcd_parameters['ubound']:
		QtWidgets.QMessageBox.warning(
			mainwidget,
			"Warning: Possible OCP complications",
			"One or more experiments use 'OCP' as a potential limit.\n\n"
			"The open-circuit potential (OCP) can vary between experiments, which\n"
			"may cause your experiments to stop depending on your lower and upper potential limits.\n\n"
			"Please ensure that this is intentional!"
		)

	return True

def gcd_get_filenames():
	"""Construct filenames for the experiments."""
	global gcd_parameters

	try:
		filename_entry = str(gcd_file_entry.text().strip())
		if filename_entry == "":
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: No filename provided",
				"Please provide a base filename for the experiments."
			)
			return False

		directory_path = os.path.dirname(filename_entry)
		if directory_path == "":
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: No parent directory specified",
				f"The output files will be stored in the current working directory. Is this okay?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False
			directory_path = os.getcwd()
		elif not os.path.isdir(directory_path):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Directory does not exist",
				f"The directory {directory_path} does not exist."
			)
			return False

		if "_GCD_{experiment_info_here}" in filename_entry:
			filename_entry = filename_entry.split("_GCD_{experiment_info_here}")[0]
		filename = os.path.basename(filename_entry)
		if filename == "":  # If no filename given, only path
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: No filename provided",
				"Please provide a base filename for the experiments in addition to the path."
			)
			return False

		gcd_parameters['directory_path'] = directory_path
		gcd_parameters['base_filename'], _ = os.path.splitext(filename)

		exp_filenames = []
		exp_filenames_capacities = []
		for i in range(gcd_parameters['num_experiments']):
			lbound = gcd_parameters['lbound'][i]
			ubound = gcd_parameters['ubound'][i]
			charge_current = gcd_parameters['charge_current'][i]
			discharge_current = gcd_parameters['discharge_current'][i]
			exp_filename = f"{gcd_parameters['base_filename']}_GCD_exp{i+1}_{lbound}_{ubound}_V_{charge_current}_{discharge_current}_uA"
			exp_filename_capacities = f"{exp_filename}_capacities"
			exp_filenames.append(exp_filename)
			exp_filenames_capacities.append(exp_filename_capacities)

		gcd_parameters['filenames'] = [name + ".dat" for name in exp_filenames]
		gcd_parameters['filenames_capacities'] = [name + ".dat" for name in exp_filenames_capacities]

		gcd_parameters['path_filenames'] = [os.path.join(directory_path, name) for name in gcd_parameters['filenames']]
		gcd_parameters['path_filenames_capacities'] = [os.path.join(directory_path, name) for name in gcd_parameters['filenames_capacities']]

		gcd_parameters['experiment_info_filename'] = gcd_parameters['base_filename'] + "_GCD_experiment_info.txt"
		gcd_parameters['experiment_info_path_filename'] = os.path.join(directory_path, gcd_parameters['experiment_info_filename'])

		for file in gcd_parameters['path_filenames']:
			if os.path.isfile(file):
				if QtWidgets.QMessageBox.question(
					mainwidget,
					"Warning: File already exists",
					f"The output file {file} already exists. Do you want to overwrite it and the corresponding capacities file?",
					QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
					QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
					return False

		info_file = gcd_parameters['experiment_info_path_filename']
		if os.path.isfile(info_file):
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: Results file already exists",
				f"The experiment info output file {info_file} already exists. Do you want to overwrite it?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False

	except Exception as e:
		print(e)
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: File error",
			f"One or more output filepaths are not valid."
		)
		return False

	gcd_file_entry.setText(os.path.join(gcd_parameters['directory_path'], f"{gcd_parameters['base_filename']}_GCD_{{experiment_info_here}}"))

	return True

def gcd_validate_filenames():
	"""Check validity of files by creating and attempting to open them."""

	for file in gcd_parameters.get('path_filenames', []):
		try:
			with open(file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {file} is not valid."
			)
			return False

	for file in gcd_parameters.get('path_filenames_capacities', []):
		try:
			with open(file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {file} is not valid."
			)
			return False

	info_file = gcd_parameters.get('experiment_info_path_filename')
	if info_file:
		try:
			with open(info_file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {info_file} is not valid."
			)
			return False

	return True

def gcd_freeze_inputs(freeze):
	"""Function to freeze and unfreeze GUI inputs when experiments are running."""

	if freeze:
		gcd_params_lbound_entry.setEnabled(False)
		gcd_params_ubound_entry.setEnabled(False)
		gcd_params_chargecurrent_entry.setEnabled(False)
		gcd_params_dischargecurrent_entry.setEnabled(False)
		gcd_params_num_halfcycles_entry.setEnabled(False)
		gcd_params_num_samples_entry.setEnabled(False)
		gcd_params_current_delay_entry.setEnabled(False)
		gcd_params_pot_window_delay_entry.setEnabled(False)
		gcd_params_pot_window_delay_OCP_checkbox.setEnabled(False)
		gcd_file_entry.setEnabled(False)
		gcd_file_notes_entry.setEnabled(False)
		gcd_variables_checkbutton.setEnabled(False)
		gcd_update_num_halfcycles_input_button.setEnabled(True)  # Unfreeze update num_halfcycles button
		software_globals_menu_button.setEnabled(False)

	elif not freeze:
		gcd_params_lbound_entry.setEnabled(True)
		gcd_params_ubound_entry.setEnabled(True)
		gcd_params_chargecurrent_entry.setEnabled(True)
		gcd_params_dischargecurrent_entry.setEnabled(True)
		gcd_params_num_halfcycles_entry.setEnabled(True)
		gcd_params_num_samples_entry.setEnabled(True)
		gcd_params_current_delay_entry.setEnabled(True)
		gcd_params_pot_window_delay_entry.setEnabled(True)
		gcd_params_pot_window_delay_OCP_checkbox.setEnabled(True)
		gcd_file_entry.setEnabled(True)
		gcd_file_notes_entry.setEnabled(True)
		gcd_variables_checkbutton.setEnabled(True)
		gcd_update_num_halfcycles_input_button.setEnabled(False)  # Freeze update num_halfcycles button
		software_globals_menu_button.setEnabled(True)


"""GCD CORE FUNCTIONS"""

def gcd_initialise():
	"""Initialise GCD experiments."""
	global state, gcd_data, gcd_current_exp_index, gcd_cumulative_halfcyclenum

	# Ensure parameters have been verified using the "CHECK" button
	if not gcd_parameters_checked:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Parameters not checked",
			"Click the 'CHECK' button to ensure parameters are appropriate before starting your experiments."
		)
		return False

	# Ensure filenames have been checked
	if not gcd_filenames_checked:
		if gcd_get_filenames():
			pass
		else:
			return False

	if check_state([States.Idle, States.Stationary_Graph]):

		# Validate filenames before experiments begin
		if gcd_validate_filenames():
			pass
		else:
			gcd_reset_experiment_controller(mode="interrupted")
			log_message("GCD experiments could not initialise due to invalid output filename.")
			return False

		# Turn cell off if under manual control
		set_cell_status(False)

		# Freeze input fields and hide return to live graph button
		gcd_freeze_inputs(freeze=True)
		preview_cancel_button.hide()

		# Initialise experiment index
		gcd_current_exp_index = 0

		# Initialise cumulative halfcyclenum
		gcd_cumulative_halfcyclenum = 0

		# Write experiment info to summary file
		gcd_write_summary_file(gcd_current_exp_index, section="initial")

		# Update GUI
		gcd_info_expnum_entry.setText(f"{gcd_current_exp_index + 1}/{gcd_parameters['num_experiments']}")
		gcd_info_halfcyclenum_entry.setText(f"-/{gcd_parameters['num_halfcycles']}")
		log_message("Starting GCD experiments...")

		# Initialise gcd_data dictionary
		gcd_data = {
			'starttime': defaultdict(float),
			'starttime_readable': defaultdict(str),
			'finishtime_readable': defaultdict(str),

			'charges': collections.deque(maxlen=2),

			'time_data_finalcycle': defaultdict(lambda: defaultdict(list)),
			'potential_data_finalcycle': defaultdict(lambda: defaultdict(list)),
			'chg_charge_data_finalcycle': defaultdict(lambda: defaultdict(list)),
			'chg_potential_data_finalcycle': defaultdict(lambda: defaultdict(list)),
			'dis_charge_data_finalcycle': defaultdict(lambda: defaultdict(list)),
			'dis_potential_data_finalcycle': defaultdict(lambda: defaultdict(list)),

			'time_data_prev10_cycles': collections.deque(maxlen=10),
			'potential_data_prev10_cycles': collections.deque(maxlen=10),
			'chg_charge_data_prev10_cycles': collections.deque(maxlen=10),
			'chg_potential_data_prev10_cycles': collections.deque(maxlen=10),
			'dis_charge_data_prev10_cycles': collections.deque(maxlen=10),
			'dis_potential_data_prev10_cycles': collections.deque(maxlen=10),

			'nth_cycles_data': dict(),

			'time_data_currentcycle': [],
			'potential_data_currentcycle': [],
			'chg_charge_data_currentcycle': [],
			'chg_potential_data_currentcycle': [],
			'dis_charge_data_currentcycle': [],
			'dis_potential_data_currentcycle': [],
		}

		# Re-calculate remaining half cycles and update progress bar
		gcd_calculate_experiment_halfcycles(initial=True)
		gcd_update_progress_bar()

		# Pass to OCP equilibration controller if required
		if gcd_parameters['OCP_bool']:
			OCP_initialise_data_entries(gcd_data)
			OCP_equilibration_controller(gcd_parameters, gcd_data, gcd_current_exp_index, equilibrated=False)
		else:
			# Write pre-experiment delay info to summary file and update GUI
			gcd_write_summary_file(gcd_current_exp_index, section="pot_window_delay")
			gcd_info_program_state_entry.setText(f"Delay of {gcd_parameters['pot_window_delay']} s")

			# Update progress bar style to solid yellow border
			gcd_progress_bar.set_solid_yellow_style()

			# Update state
			state = States.Measuring_GCD_Delay

			# Launch the delay timer
			gcd_delay_timer.start(int(gcd_parameters['pot_window_delay'] * 1000))  # Delay input in ms

def gcd_start(experiment_index):

	global state, gcd_currentsetpoint
	global gcd_data, gcd_time_data, gcd_potential_data, gcd_current_data
	global gcd_output_file, gcd_output_file_capacities
	global gcd_current_cyclenum, gcd_current_halfcyclenum, gcd_cumulative_halfcyclenum
	global gcd_plot_curve, legend, legend_in_use

	if experiment_index is None:
		state = States.Stationary_Graph
		preview_cancel_button.show()
		gcd_write_summary_file(gcd_current_exp_index, section="error")
		gcd_reset_experiment_controller(mode="interrupted")
		log_message("Experiments could not start due to experiment_index initialisation error.")
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Initialisation",
			"The experiments could not initialise experiment_index correctly."
		)
		return

	# Write experiment information to summary file
	gcd_data['starttime_readable'][experiment_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]
	gcd_write_summary_file(gcd_current_exp_index, section="experiment_start")

	# Open output files and write headers
	try:
		gcd_output_file = open(gcd_parameters['path_filenames'][experiment_index], 'w', 1)
		gcd_output_file.write("Cycle number\tHalf cycle number\tElapsed time (s)\tPotential (V)\tCurrent (A)\n")
		gcd_output_file_capacities = open(gcd_parameters['path_filenames_capacities'][experiment_index], 'w', 1)
		gcd_output_file_capacities.write("Cycle number\tCharge capacity (Ah)\tDischarge capacity (Ah)\n")
	except Exception as e:
		log_message(f"Write to file failed: {e}")
		gcd_stop(experiment_index, interrupted=True)
		return

	# Initialise buffers for holding averaged elapsed time, potential, and current data for each half cycle
	gcd_time_data = AverageBuffer(gcd_parameters['num_samples'][experiment_index])
	gcd_potential_data = AverageBuffer(gcd_parameters['num_samples'][experiment_index])
	gcd_current_data = AverageBuffer(gcd_parameters['num_samples'][experiment_index])

	# Initialise current cyclenums
	gcd_current_cyclenum = 1
	gcd_current_halfcyclenum = 1

	# Initialise the plot area
	Legends.remove_all_legends()
	plot_frame.clear()
	plot_frame.enableAutoRange()
	plot_frame.getAxis('bottom').setTicks(None)
	plot_frame.setLabel('left', 'Potential', units='V')
	if gcd_plot_options_x_charge_radiobutton.isChecked():
		plot_frame.setLabel('bottom', 'Inserted/extracted charge', units='Ah')
	elif gcd_plot_options_x_time_radiobutton.isChecked():
		plot_frame.setLabel('bottom', 'Time elapsed per cycle', units='s')
	gcd_plot_curve = plot_frame.plot(pen='y')
	legend = pyqtgraph.LegendItem(offset=(60, 10))
	legend.setParentItem(plot_frame.plotItem)
	Legends.legends['gcd'] = legend
	legend_in_use = 'gcd'

	# If initial experiment, begin cumulative halfcyclenum at 1
	if gcd_cumulative_halfcyclenum == 0:
		gcd_cumulative_halfcyclenum = 1
	gcd_update_progress_bar()

	# Set progress bar style to solid green
	gcd_progress_bar.set_solid_green_style()

	# Display experiment info in GUI
	gcd_info_program_state_entry.setText(f"Measuring GCD")
	gcd_info_halfcyclenum_entry.setText(f"{gcd_current_halfcyclenum}/{gcd_parameters['num_halfcycles']}")
	log_message(f"Charge/discharge experiment {experiment_index+1}/{gcd_parameters['num_experiments']} started. Saving to: {gcd_parameters['filenames'][experiment_index]}")

	# Initialise DAC and cell for measurements
	gcd_currentsetpoint = gcd_parameters['charge_current'][experiment_index] * 1e-3  # Convert µA to mA
	hardware_manual_control_range_dropdown.setCurrentIndex(current_range_from_current(gcd_currentsetpoint)) # Determine the proper current range for the current setpoint
	set_current_range() # Set new current range
	set_output(1, gcd_currentsetpoint)  # Set current to setpoint
	set_control_mode(True)  # Galvanostatic control
	time.sleep(.2)  # Allow DAC some time to settle
	set_cell_status(True)  # Cell on

	# Begin calling gcd_update() through periodic_update()
	state = States.Measuring_GCD

	# Store experiment start time
	gcd_data['starttime'][experiment_index] = timeit.default_timer()

def gcd_update(experiment_index):
	"""Add a new data point to the GCD measurement - called regularly through periodic_update()"""
	global gcd_data, gcd_currentsetpoint, gcd_current_cyclenum, gcd_current_halfcyclenum, gcd_cumulative_halfcyclenum

	elapsed_time = timeit.default_timer() - gcd_data['starttime'][experiment_index]

	# If GCD experiment completed
	if gcd_current_halfcyclenum > gcd_parameters['num_halfcycles']:

		# Stop the experiment
		gcd_stop(experiment_index, interrupted=False)
		return

	else:  # Continue GCD cycling
		read_potential_current()
		gcd_time_data.add_sample(elapsed_time)
		gcd_potential_data.add_sample(potential)
		gcd_current_data.add_sample(1e-3 * current)  # Convert mA to A

		# Define current potential_window and cd_current for indexing gcd_data
		lbound, ubound = gcd_parameters['lbound'][experiment_index], gcd_parameters['ubound'][experiment_index]
		charge_current, discharge_current = gcd_parameters['charge_current'][experiment_index], gcd_parameters['discharge_current'][experiment_index]

		potential_window = f"{lbound}/{ubound}"
		cd_current = f"{charge_current}/{discharge_current}"

		if len(gcd_time_data.samples) == 0 and len(gcd_time_data.averagebuffer) > 0:  # A new average was just calculated

			# Write data to output file
			try:
				gcd_output_file.write("%d\t%d\t%e\t%e\t%e\n" % (
					gcd_current_cyclenum,
					gcd_current_halfcyclenum,
					gcd_time_data.averagebuffer[-1],
					gcd_potential_data.averagebuffer[-1],
					gcd_current_data.averagebuffer[-1]
				))
			except Exception as e:
				log_message(f"Write to file failed: {e}")
				gcd_stop(experiment_index, interrupted=True)
				return

			# Calculate the cumulative charge in Ah
			charge = numpy.abs(scipy.integrate.cumulative_trapezoid(gcd_current_data.averagebuffer, gcd_time_data.averagebuffer, initial=0.)/3600.)  # Cumulative charge in Ah

			# Store data for the current cycle
			gcd_data['time_data_currentcycle'].append(gcd_time_data.averagebuffer[-1])
			gcd_data['potential_data_currentcycle'].append(gcd_potential_data.averagebuffer[-1])
			if gcd_currentsetpoint > 0:  # Charging
				gcd_data['chg_charge_data_currentcycle'].append(charge[-1])
				gcd_data['chg_potential_data_currentcycle'].append(gcd_potential_data.averagebuffer[-1])
			elif gcd_currentsetpoint < 0:  # Discharging
				gcd_data['dis_charge_data_currentcycle'].append(charge[-1])
				gcd_data['dis_potential_data_currentcycle'].append(gcd_potential_data.averagebuffer[-1])

			# Update plot
			gcd_update_plot(experiment_index)

			# Handle OCP as inputs
			if lbound == "OCP":
				lbound = gcd_parameters['current_OCP']
			if ubound == "OCP":
				ubound = gcd_parameters['current_OCP']

			# If upper or lower bound reached - half cycle completed
			if (gcd_currentsetpoint > 0 and gcd_potential_data.averagebuffer[-1] >= ubound) or (gcd_currentsetpoint < 0 and gcd_potential_data.averagebuffer[-1] <= lbound):

				# Increment cumulative halfcyclenum and update progress bar
				gcd_cumulative_halfcyclenum += 1
				gcd_update_progress_bar()

				# Calculate half cycle charge and store in deque
				halfcycle_charge = numpy.abs(numpy.trapezoid(gcd_current_data.averagebuffer, gcd_time_data.averagebuffer) / 3600.)  # Charge in Ah
				gcd_data['charges'].append(halfcycle_charge)

				# If charge half cycle completed
				if gcd_currentsetpoint > 0:

					# Flip current and convert µA to mA
					gcd_currentsetpoint = discharge_current * 1e-3

					# Increment halfcyclenum and update GUI
					gcd_current_halfcyclenum += 1
					gcd_info_halfcyclenum_entry.setText(f"{gcd_current_halfcyclenum}/{gcd_parameters['num_halfcycles']}")

				# If discharge half cycle completed
				elif gcd_currentsetpoint < 0:

					# Flip current and convert µA to mA
					gcd_currentsetpoint = charge_current * 1e-3

					# Write to capacities output file
					try:
						gcd_output_file_capacities.write("%d\t%e\t%e\n" % (
							gcd_current_cyclenum,
							gcd_data['charges'][-2],
							gcd_data['charges'][-1]
						))
					except Exception as e:
						log_message(f"Write to file failed: {e}")
						gcd_stop(experiment_index, interrupted=True)
						return

					# Add completed cycle to previous 10 cycles deque
					gcd_data['time_data_prev10_cycles'].append(gcd_data['time_data_currentcycle'][:])
					gcd_data['potential_data_prev10_cycles'].append(gcd_data['potential_data_currentcycle'][:])
					gcd_data['chg_charge_data_prev10_cycles'].append(gcd_data['chg_charge_data_currentcycle'][:])
					gcd_data['chg_potential_data_prev10_cycles'].append(gcd_data['chg_potential_data_currentcycle'][:])
					gcd_data['dis_charge_data_prev10_cycles'].append(gcd_data['dis_charge_data_currentcycle'][:])
					gcd_data['dis_potential_data_prev10_cycles'].append(gcd_data['dis_potential_data_currentcycle'][:])

					# Store every nth cycle data (Set in global scope)
					if gcd_current_cyclenum % global_software_settings['gcd_nth_cycles'] == 0:
						cycle_key = gcd_current_cyclenum
						gcd_data['nth_cycles_data'][cycle_key] = {
							'time_data': gcd_data['time_data_currentcycle'][:],
							'potential_data': gcd_data['potential_data_currentcycle'][:],
							'chg_charge_data': gcd_data['chg_charge_data_currentcycle'][:],
							'chg_potential_data': gcd_data['chg_potential_data_currentcycle'][:],
							'dis_charge_data': gcd_data['dis_charge_data_currentcycle'][:],
							'dis_potential_data': gcd_data['dis_potential_data_currentcycle'][:],
						}
						# Add nth cycle to plot options dropdown
						gcd_plot_options_nth_cycles_dropdown.addItems([str(gcd_current_cyclenum)])

					# Store final full cycle data
					if gcd_current_cyclenum == gcd_parameters['num_fullcycles']:
						gcd_data['time_data_finalcycle'][potential_window][cd_current] = gcd_data['time_data_currentcycle'][:]
						gcd_data['potential_data_finalcycle'][potential_window][cd_current] = gcd_data['potential_data_currentcycle'][:]
						gcd_data['chg_charge_data_finalcycle'][potential_window][cd_current] = gcd_data['chg_charge_data_currentcycle'][:]
						gcd_data['chg_potential_data_finalcycle'][potential_window][cd_current] = gcd_data['chg_potential_data_currentcycle'][:]
						gcd_data['dis_charge_data_finalcycle'][potential_window][cd_current] = gcd_data['dis_charge_data_currentcycle'][:]
						gcd_data['dis_potential_data_finalcycle'][potential_window][cd_current] = gcd_data['dis_potential_data_currentcycle'][:]

					# Clear current cycle data
					gcd_data['time_data_currentcycle'] = []
					gcd_data['potential_data_currentcycle'] = []
					gcd_data['chg_charge_data_currentcycle'] = []
					gcd_data['chg_potential_data_currentcycle'] = []
					gcd_data['dis_charge_data_currentcycle'] = []
					gcd_data['dis_potential_data_currentcycle'] = []

					# Increment cyclenums
					gcd_current_cyclenum += 1
					gcd_current_halfcyclenum += 1

					# If final half cycle, send 0 current to the DAC
					if gcd_current_halfcyclenum > gcd_parameters['num_halfcycles']:
						gcd_currentsetpoint = 0.0

					# If not final half cycle, update GUI
					else:
						gcd_info_halfcyclenum_entry.setText(f"{gcd_current_halfcyclenum}/{gcd_parameters['num_halfcycles']}")

				# Clear average buffers for next half cycle
				for data in [gcd_time_data, gcd_potential_data, gcd_current_data]:
					data.clear()

				hardware_manual_control_range_dropdown.setCurrentIndex(current_range_from_current(gcd_currentsetpoint))  # Determine proper current range for new setpoint
				set_current_range()  # Set new current range
				set_output(1, gcd_currentsetpoint)  # Send current to the DAC

def gcd_stop(experiment_index, interrupted=True):
	"""Finish the GCD experiment."""
	global state, gcd_current_exp_index

	if check_state([States.Measuring_GCD, States.Measuring_GCD_OCP_eq, States.Measuring_GCD_Delay]):

		set_cell_status(False)  # Cell off
		state = States.Stationary_Graph
		set_output(1, 0.0)  # Send zero current to DAC

		# Close output files
		try:
			gcd_output_file.close()
			gcd_output_file_capacities.close()
		except:
			pass

		# Save experiment finish time
		gcd_data['finishtime_readable'][experiment_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]

		# Update cyclenum GUI
		gcd_plot_options_nth_cycles_dropdown.clear()  # Empty the dropdown box of nth cycles

		if interrupted:

			# Write to summmary file and close
			gcd_write_summary_file(experiment_index, section="interrupted")

			# Reset experiment
			gcd_reset_experiment_controller(mode="interrupted")

			# Update GUI
			log_message("*** EXPERIMENTS INTERRUPTED ***")
			QtWidgets.QMessageBox.information(
				mainwidget,
				"GCD experiments interrupted",
				"Oh no! GCD experiments have been interrupted.\n\nGlobal experiment parameters have been reset."
			)
			preview_cancel_button.show()

		elif not interrupted:

			# Write to summary file
			gcd_write_summary_file(experiment_index, section="experiment_end")
			log_message(f"*** Experiment {experiment_index + 1}/{gcd_parameters['num_experiments']} completed ***")

			# If not final experiment
			if experiment_index+1 != gcd_parameters['num_experiments']:

				# Re-initialise data for next experiment
				gcd_data['charges'] = collections.deque(maxlen=2)
				gcd_data['time_data_prev10_cycles'] = collections.deque(maxlen=10)
				gcd_data['potential_data_prev10_cycles'] = collections.deque(maxlen=10)
				gcd_data['chg_charge_data_prev10_cycles'] = collections.deque(maxlen=10)
				gcd_data['chg_potential_data_prev10_cycles'] = collections.deque(maxlen=10)
				gcd_data['dis_charge_data_prev10_cycles'] = collections.deque(maxlen=10)
				gcd_data['dis_potential_data_prev10_cycles'] = collections.deque(maxlen=10)
				gcd_data['nth_cycles_data'].clear()
				gcd_data['time_data_currentcycle'] = []
				gcd_data['potential_data_currentcycle'] = []
				gcd_data['chg_charge_data_currentcycle'] = []
				gcd_data['chg_potential_data_currentcycle'] = []
				gcd_data['dis_charge_data_currentcycle'] = []
				gcd_data['dis_potential_data_currentcycle'] = []

				# Increment the current experiment index and update GUI
				gcd_current_exp_index += 1
				gcd_info_expnum_entry.setText(f"{gcd_current_exp_index + 1}/{gcd_parameters['num_experiments']}")
				gcd_info_halfcyclenum_entry.setText(f"-/{gcd_parameters['num_halfcycles']}")

				# Determine if moving to the next potential window
				next_current_idx = gcd_current_exp_index % len(gcd_parameters['unique_charge_currents'])

				# If moving to next potential window
				if next_current_idx == 0:
					if gcd_parameters['OCP_bool']:
						OCP_equilibration_controller(gcd_parameters, gcd_data, gcd_current_exp_index, equilibrated=False)
					else:
						# Write pre-potential window delay to summary file and update GUI
						gcd_write_summary_file(gcd_current_exp_index, section="pot_window_delay")
						gcd_info_program_state_entry.setText(f"Delay of {gcd_parameters['pot_window_delay']} s")

						# Update progress bar style to solid yellow border
						gcd_progress_bar.set_solid_yellow_style()

						# Update state
						state = States.Measuring_GCD_Delay

						# Launch the pre-experiment delay timer
						gcd_delay_timer.start(int(gcd_parameters['pot_window_delay'] * 1000))  # Delay input in ms

				# Moving to next charge/discharge current
				else:
					# Write pre-charge/discharge current delay to summary file and update GUI
					gcd_write_summary_file(gcd_current_exp_index, section="current_delay")
					gcd_info_program_state_entry.setText(f"Delay of {gcd_parameters['current_delay']} s")

					# Update progress bar style to solid yellow border
					gcd_progress_bar.set_solid_yellow_style()

					# Update state
					state = States.Measuring_GCD_Delay

					# Launch pre-experiment delay timer
					gcd_delay_timer.start(int(gcd_parameters['current_delay'] * 1000))  # Delay input in ms

			# If final experiment completed
			elif experiment_index+1 == gcd_parameters['num_experiments']:

				# Write to summary file and close
				gcd_write_summary_file(experiment_index, section="all_experiments_completed")

				# Reset experiment
				gcd_reset_experiment_controller(mode="all_experiments_completed")

				# Update GUI
				gcd_info_program_state_entry.setText("All experiments completed")
				QtWidgets.QMessageBox.information(
					mainwidget,
					"GCD experiments completed",
					"CONGRATULATIONS! All GCD experiments have completed successfully.\n\nGlobal experiment parameters have been reset."
				)
				preview_cancel_button.show()

def gcd_reset_experiment_controller(mode):
	"""Controller for resetting global experiment parameters."""
	global gcd_parameters, gcd_data
	global gcd_parameters_checked, gcd_filenames_checked
	global gcd_current_exp_index, gcd_current_cyclenum, gcd_current_halfcyclenum
	global gcd_total_halfcycles, gcd_cumulative_halfcyclenum

	# Stop timer
	gcd_delay_timer.stop()

	if mode == "input_changed":
		if gcd_parameters_checked:  # If inputs have changed since last successful check

			# Reset globals
			gcd_parameters_checked = False
			gcd_filenames_checked = False
			gcd_variables_checkbutton.setStyleSheet("")

			# Reset progress bar
			gcd_total_halfcycles = None
			gcd_cumulative_halfcyclenum = None
			gcd_update_progress_bar()

			log_message("GCD input parameters or program state has changed since the last successful check - check-state has been reset.")

		return

	elif mode == "checkbutton_failed":

		# Reset progress bar
		gcd_total_halfcycles = None
		gcd_cumulative_halfcyclenum = None
		gcd_update_progress_bar()
		return

	# Ensure output files are closed
	for file in ('gcd_output_file', 'gcd_output_file_capacities','gcd_summary_file'):
		try:
			if file in globals():
				f = globals()[file]
				if f and hasattr(f, 'close'):
					f.close()
					globals()[file] = None  # Clear reference to file
		except Exception as e:
			log_message(f"Error closing {file}: {e}")

	# Reset globals
	gcd_parameters_checked = False
	gcd_filenames_checked = False
	gcd_variables_checkbutton.setStyleSheet("")
	gcd_parameters = {'type': 'gcd'}
	gcd_data = {}
	gcd_current_exp_index = None
	gcd_current_cyclenum = None
	gcd_current_halfcyclenum = None
	gcd_total_halfcycles = None
	gcd_cumulative_halfcyclenum = None

	# Reset GUI
	gcd_info_expnum_entry.setText("-/-")
	gcd_info_halfcyclenum_entry.setText("-/-")
	gcd_update_num_halfcycles_input_button.setStyleSheet("")
	gcd_plot_options_nth_cycles_dropdown.clear()

	# Unfreeze input fields
	gcd_freeze_inputs(freeze=False)

	if mode == "all_experiments_completed":
		gcd_progress_bar.set_completed_state()

	elif mode == "interrupted":
		gcd_info_program_state_entry.setText(f"Experiments interrupted")
		gcd_progress_bar.set_interrupted_state()

	elif mode == "OCP_interrupted":
		gcd_progress_bar.set_OCP_interrupted_state()


"""GCD ACCESSORY FUNCTIONS"""

def gcd_write_summary_file(experiment_index, section):
	"""Write summary file for the experiment."""
	global gcd_summary_file

	try:
		if section == "initial":
			gcd_summary_file = open(gcd_parameters['experiment_info_path_filename'], 'w', 1)
			gcd_summary_file.write("GCD EXPERIMENTS INFORMATION FILE\n********************************\n")

			gcd_summary_file.write(f"\nExperiment notes: {gcd_parameters['experiment_notes']}\n")

			gcd_summary_file.write("\nExperiment information file for the experiments stored in:\n")
			for file in gcd_parameters['path_filenames']:
				gcd_summary_file.write(f"{file}\n")
			gcd_summary_file.write("\n")

			gcd_summary_file.write("Calculated charge/discharge capacities for each full cycle are stored in:\n")
			for capacities_file in gcd_parameters['path_filenames_capacities']:
				gcd_summary_file.write(f"{capacities_file}\n")
			gcd_summary_file.write("\n")

			potential_windows = []
			cd_currents = []
			for lbound, ubound in zip(gcd_parameters['unique_lbounds'], gcd_parameters['unique_ubounds']):
				potential_windows.append((lbound, ubound))
			for charge_current, discharge_current in zip(gcd_parameters['unique_charge_currents'], gcd_parameters['unique_discharge_currents']):
				cd_currents.append((charge_current, discharge_current))

			potential_windows_str = ', '.join(f"[{lbound}, {ubound}]" for lbound, ubound in potential_windows)
			cd_currents_str = ', '.join(f"[{charge_current}, {discharge_current}]" for charge_current, discharge_current in cd_currents)
			num_cd_currents = len(gcd_parameters['unique_charge_currents'])

			if gcd_parameters['OCP_bool']:
				pot_window_delay_str = "Wait for OCP equilibration."
			else:
				pot_window_delay_str = f"{gcd_parameters['pot_window_delay']}"

			gcd_summary_file.write(f"Potential windows cycled through (V): {potential_windows_str}\n")
			gcd_summary_file.write(f"Charge/discharge currents cycled through (µA): {cd_currents_str}\n")
			gcd_summary_file.write(f"Number of experiments: {gcd_parameters['num_experiments']}\n")
			gcd_summary_file.write(f"Number of half cycles per charge/discharge current: {gcd_parameters['num_halfcycles']}\n")
			gcd_summary_file.write(f"Number of full cycles per charge/discharge current: {gcd_parameters['num_fullcycles']}\n")
			gcd_summary_file.write(f"Samples to average per charge/discharge current: {gcd_parameters['num_samples'][:num_cd_currents]}\n")
			gcd_summary_file.write(f"Pre-charge/discharge current delay (s): {gcd_parameters['current_delay']}\n")
			gcd_summary_file.write(f"Pre-potential window delay (s): {pot_window_delay_str}\n")

		elif section == "pot_window_delay":
			gcd_summary_file.write(f"\n*** Pre-potential window delay of {gcd_parameters['pot_window_delay']} seconds for experiment: {experiment_index + 1}/{gcd_parameters['num_experiments']} ***\n")

		elif section == "current_delay":
			gcd_summary_file.write(f"\n*** Pre-charge/discharge current delay of {gcd_parameters['current_delay']} seconds for experiment: {experiment_index + 1}/{gcd_parameters['num_experiments']} ***\n")

		elif section == "experiment_start":
			gcd_summary_file.write("\n***************************************\n\tGCD MEASUREMENT STARTED\n***************************************\n")
			gcd_summary_file.write(f"Filepath: {gcd_parameters['path_filenames'][experiment_index]}\n")
			gcd_summary_file.write(f"Experiment number: {experiment_index + 1}/{gcd_parameters['num_experiments']}")
			gcd_summary_file.write("\n")

			gcd_summary_file.write(f"Lower/upper potential limits (V): {gcd_parameters['lbound'][experiment_index]}, {gcd_parameters['ubound'][experiment_index]}\n")
			gcd_summary_file.write(f"Charge/discharge currents (µA): {gcd_parameters['charge_current'][experiment_index]}, {gcd_parameters['discharge_current'][experiment_index]}\n")
			gcd_summary_file.write(f"Number of half cycles: {gcd_parameters['num_halfcycles']}\n")
			gcd_summary_file.write(f"Number of full cycles: {gcd_parameters['num_fullcycles']}\n")
			gcd_summary_file.write(f"Samples to average: {gcd_parameters['num_samples'][experiment_index]}\n")
			gcd_summary_file.write("\n")

			gcd_summary_file.write(f"Experiment start time:  {gcd_data['starttime_readable'][experiment_index]}\n")

		elif section == "experiment_end":
			gcd_summary_file.write(f"Experiment finish time: {gcd_data['finishtime_readable'][experiment_index]}\n")
			gcd_summary_file.write("\n")

			gcd_summary_file.write("*****************************************\n\tGCD MEASUREMENT COMPLETED\n*****************************************\n")

		elif section == "all_experiments_completed":
			gcd_summary_file.write("\n******************************************************\n\tALL EXPERIMENTS COMPLETED SUCCESSFULLY\n******************************************************\n")
			if gcd_parameters['OCP_bool']:
				OCP_values_str = ', '.join(f"{value:.6f}" for value in gcd_data['OCP_values'])
				OCP_times_str = ', '.join(f"{value:.3f}" for value in gcd_data['OCP_eq_elapsed_times'])

				gcd_summary_file.write("\n")
				gcd_summary_file.write("*** OCP drift information ***\n")
				gcd_summary_file.write(f"OCP value across potential windows (V): {OCP_values_str}\n")
				gcd_summary_file.write(f"OCP equilibration times (s): {OCP_times_str}\n")

			gcd_summary_file.write("\n")
			gcd_summary_file.write("********************\n")
			gcd_summary_file.write("EXPERIMENTS COMPLETE\n")
			gcd_summary_file.close()

		elif section == "updated_num_halfcycles_input":
			gcd_summary_file.write("\n***** UPDATED NUMBER OF HALF CYCLES *****\n")
			gcd_summary_file.write(f"Number of half cycles updated during experiment: {experiment_index + 1}/{gcd_parameters['num_experiments']}\n")
			gcd_summary_file.write(f"Previous number of half cycles per charge/discharge current: {gcd_parameters['prev_num_halfcycles']}\n")
			gcd_summary_file.write(f"Updated number of half cycles per charge/discharge current: {gcd_parameters['num_halfcycles']}\n")
			gcd_summary_file.write("\n")
			gcd_summary_file.write("This updated number of half cycles will be applied to all further experiments.\n")
			gcd_summary_file.write("*****************************************\n\n")

		elif section == "OCP_eq_start":
			pot_window_exp_indexes = list(range(experiment_index, experiment_index + len(gcd_parameters['unique_charge_currents'])))

			gcd_summary_file.write("\n*********************************\n\tOCP EQUILIBRATING\n*********************************\n")
			gcd_summary_file.write(f"OCP equilibrating for experiments: [{', '.join(str(idx + 1) for idx in pot_window_exp_indexes)}]/{gcd_parameters['num_experiments']}\n")
			gcd_summary_file.write(f"Equilibration tolerance (mV): {global_software_settings['OCP_eq_tolerance']}\n")
			gcd_summary_file.write(f"Equilibration timescale (s): {global_software_settings['OCP_eq_timescale']}\n")
			gcd_summary_file.write(f"Equilibration timeout (s): {global_software_settings['OCP_eq_timeout']}\n")
			gcd_summary_file.write("\n")

			gcd_summary_file.write(f"OCP equilibration start time: {gcd_data['OCP_starttime_readable'][experiment_index]}\n")

		elif section == "OCP_equilibrated":
			gcd_summary_file.write(f"OCP equilibration finish time: {gcd_data['OCP_eq_finishtime_readable'][experiment_index]}\n")
			gcd_summary_file.write("\n")

			gcd_summary_file.write(f"Elapsed time (s): {gcd_data['OCP_eq_elapsed_times'][-1]:.3f}\n")
			gcd_summary_file.write(f"Starting potential (V): {gcd_data['OCP_eq_startpot'][experiment_index]:.5g}\n")
			gcd_summary_file.write(f"Final equilibrated OCP (V): {gcd_data['OCP_eq_stoppot'][experiment_index]:.5g}\n")
			gcd_summary_file.write(f"Total potential difference (V): {gcd_data['OCP_eq_total_pot_diff'][experiment_index]:.5g}\n")
			gcd_summary_file.write(f"Final {global_software_settings['OCP_eq_timescale']} s potential difference (V): {gcd_data['OCP_eq_timescale_pot_diff'][experiment_index]:.5g}\n")
			gcd_summary_file.write("\n")

			gcd_summary_file.write("***** OCP successfully equilibrated *****\n")

		elif section == "OCP_valid":
			pot_window_exp_indexes = list(range(experiment_index, experiment_index + len(gcd_parameters['unique_charge_currents'])))

			if gcd_parameters['OCP_parameters'] == []:
				gcd_summary_file.write(f"\nParameters set as OCP for experiments [{', '.join(str(idx + 1) for idx in pot_window_exp_indexes)}]/{gcd_parameters['num_experiments']}: None\n")
			else:
				gcd_summary_file.write(f"\nParameters set as OCP for experiments [{', '.join(str(idx + 1) for idx in pot_window_exp_indexes)}]/{gcd_parameters['num_experiments']}: {', '.join(gcd_parameters['OCP_parameters'])}\n")

			gcd_summary_file.write(f"{gcd_parameters['OCP_valid_text']}\n")

		elif section == "OCP_invalid":
			pot_window_exp_indexes = list(range(experiment_index, experiment_index + len(gcd_parameters['unique_charge_currents'])))

			gcd_summary_file.write("\n********************************************\n\tGCD MEASUREMENTS INTERRUPTED\n********************************************\n")
			gcd_summary_file.write("GCD experiments stopped due to OCP becoming incompatible with experiment parameters.\n")
			gcd_summary_file.write(f"OCP (V): {gcd_parameters['current_OCP']}\n")
			gcd_summary_file.write("\n")

			gcd_summary_file.write(f"Experiment parameters for experiments [{', '.join(str(idx + 1) for idx in pot_window_exp_indexes)}]/{gcd_parameters['num_experiments']}:\n")
			gcd_summary_file.write(f"Lower potential limit (V): {gcd_parameters['lbound'][experiment_index]}\n")
			gcd_summary_file.write(f"Upper potential limit (V): {gcd_parameters['ubound'][experiment_index]}\n")
			gcd_summary_file.write("\n")

			gcd_summary_file.write(f"{gcd_parameters['OCP_valid_text']}\n")

			gcd_summary_file.write("\n")
			gcd_summary_file.write("**********************\n")
			gcd_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			gcd_summary_file.close()

		elif section == "OCP_timeout":
			gcd_summary_file.write(f"OCP equilibration timeout time: {gcd_data['OCP_timeout_finishtime_readable'][experiment_index]}\n")

			gcd_summary_file.write("\n********************************************\n\tGCD MEASUREMENTS INTERRUPTED\n********************************************\n")
			gcd_summary_file.write(f"GCD experiments stopped due to OCP equilibration timeout.\n")
			gcd_summary_file.write(f"OCP did not equilibrate to within tolerance by the timeout threshold.\n")
			gcd_summary_file.write("\n")

			gcd_summary_file.write(f"OCP timeout threshold (s): {global_software_settings['OCP_eq_timeout']}\n")
			gcd_summary_file.write(f"Elapsed time (s): {gcd_data['OCP_timeout_time_elapsed'][experiment_index]:.3g}\n")
			gcd_summary_file.write(f"Starting potential (V): {gcd_data['OCP_timeout_startpot'][experiment_index]:.5g}\n")
			gcd_summary_file.write(f"Final potential (V): {gcd_data['OCP_timeout_stoppot'][experiment_index]:.5g}\n")
			gcd_summary_file.write(f"Total potential difference (V): {gcd_data['OCP_timeout_total_pot_diff'][experiment_index]:.5g}\n")
			gcd_summary_file.write(f"Final {global_software_settings['OCP_eq_timescale']} s potential difference (V): {gcd_data['OCP_timeout_timescale_pot_diff'][experiment_index]:.5g}\n")

			gcd_summary_file.write("\n")
			gcd_summary_file.write("**********************\n")
			gcd_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			gcd_summary_file.close()

		elif section == "interrupted":
			gcd_summary_file.write("\n********************************************\n\tGCD MEASUREMENTS INTERRUPTED\n********************************************\n")
			gcd_summary_file.write(f"Experiment interrupted: {experiment_index + 1}/{gcd_parameters['num_experiments']}\n")
			gcd_summary_file.write(f"Experiment interruption time: {gcd_data['finishtime_readable'][experiment_index]}\n")
			gcd_summary_file.write(f"GCD measurement interrupted: {gcd_parameters['lbound'][experiment_index]}/{gcd_parameters['ubound'][experiment_index]} V; {gcd_parameters['charge_current'][experiment_index]}/{gcd_parameters['discharge_current'][experiment_index]} µA\n")
			gcd_summary_file.write(f"Interrupted measurement data saved to: {gcd_parameters['path_filenames'][experiment_index]}\n")
			gcd_summary_file.write(f"Interrupted capacities data saved to: {gcd_parameters['path_filenames_capacities']}\n")

			gcd_summary_file.write("\n")
			gcd_summary_file.write("**********************\n")
			gcd_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			gcd_summary_file.close()

		elif section == "error":
			gcd_summary_file.write("\n**********************************************\n\tERROR INITIALISING EXPERIMENTS\n**********************************************\n")

			gcd_summary_file.write("\n")
			gcd_summary_file.write("**********************\n")
			gcd_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			gcd_summary_file.close()

	except Exception as e:
		log_message(f"Write to summary file failed: {e}")

def gcd_calculate_experiment_halfcycles(initial):
	"""Calculate the total halfcycles for the experiment, both initially and if the number of half cycles is
	changed mid-experiment."""
	global gcd_parameters, gcd_total_halfcycles

	if initial:
		exp_indexes = [exp for exp in range(gcd_parameters['num_experiments'])]
		halfcycles_per_exp = gcd_parameters['num_halfcycles']

		total_halfcycles = 0
		for _ in exp_indexes:
			total_halfcycles += halfcycles_per_exp
		gcd_total_halfcycles = total_halfcycles

	elif not initial:
		exp_indexes = [exp for exp in range(gcd_parameters['num_experiments'])]
		exp_indexes_remaining = exp_indexes[gcd_current_exp_index:]
		difference_per_exp = gcd_parameters['num_halfcycles'] - gcd_parameters['prev_num_halfcycles']

		total_halfcycles = gcd_total_halfcycles
		for _ in exp_indexes_remaining:
			total_halfcycles += difference_per_exp

		gcd_total_halfcycles = total_halfcycles

	return gcd_total_halfcycles

def gcd_update_num_halfcycles_input():
	global gcd_parameters, gcd_update_num_halfcycles_input_button_timer

	try:
		new_num_halfcycles = int(gcd_update_num_halfcycles_input_entry.text().strip())
		if new_num_halfcycles < 1:
			raise ValueError
	except ValueError:
		return False

	gcd_parameters['prev_num_halfcycles'] = gcd_parameters['num_halfcycles']  # Store previous num_halfcycles
	gcd_parameters['num_halfcycles'] = new_num_halfcycles
	gcd_parameters['num_fullcycles'] = int(float(new_num_halfcycles) / 2)

	# Reflect new number of half cycles in the input parameters
	gcd_params_num_halfcycles_entry.setEnabled(True)
	gcd_params_num_halfcycles_entry.blockSignals(True)
	gcd_params_num_halfcycles_entry.setText(f"{new_num_halfcycles}")
	gcd_params_num_halfcycles_entry.blockSignals(False)
	gcd_params_num_halfcycles_entry.setEnabled(False)

	# Update GUI
	gcd_info_halfcyclenum_entry.setText(f"{gcd_current_halfcyclenum}/{new_num_halfcycles}")

	# Remove new number of half cycles from input field and make button green
	gcd_update_num_halfcycles_input_entry.setText("")
	gcd_update_num_halfcycles_input_button.setStyleSheet("background-color: green; color: white;")

	# Start timer to reset button stylesheet
	if gcd_update_num_halfcycles_input_button_timer is None:
		gcd_update_num_halfcycles_input_button_timer = QtCore.QTimer()
		gcd_update_num_halfcycles_input_button_timer.setSingleShot(True)  # We only need it to trigger once
		gcd_update_num_halfcycles_input_button_timer.timeout.connect(lambda: gcd_update_num_halfcycles_input_button.setStyleSheet(""))  # Revert to original
	gcd_update_num_halfcycles_input_button_timer.start(1000)

	# Write updated number of half cycles to summary file
	gcd_write_summary_file(gcd_current_exp_index, section="updated_num_halfcycles_input")

	# Recalculate total number of half cycles with new input and update progress bar
	gcd_calculate_experiment_halfcycles(initial=False)
	gcd_update_progress_bar()

	# Update log
	log_message(f"Number of half cycles per experiment successfully updated to: {new_num_halfcycles}")

def gcd_OCP_valid_bool(experiment_index):

	lbound = gcd_parameters['lbound'][experiment_index]
	ubound = gcd_parameters['ubound'][experiment_index]
	OCP_value = gcd_parameters['current_OCP']

	if lbound == "OCP":
		if ubound <= OCP_value:
			return False, "OCP invalid: Lower potential limit must be less than the upper potential limit."

	if ubound == "OCP":
		if lbound >= OCP_value:
			return False, "OCP invalid: Upper potential limit must be greater than the lower potential limit."

	return True, "All experiment parameters are valid after OCP equilibration."


"""GCD PLOT AND GUI FUNCTIONS"""

def gcd_update_plot(experiment_index):
	"""Update the plot with the current GCD cycle, and other experiments/cycles depending on GUI inputs."""

	# Clear plot frame and legend
	plot_frame.clear()
	legend.clear()

	# Title for legend
	dummy_item = pyqtgraph.PlotDataItem([], [], pen=None)
	legend.addItem(dummy_item, "Galvanostatic charge/discharge experiments:")

	# Determine x-axis units
	if gcd_plot_options_x_time_radiobutton.isChecked():
		x_key = "time_data"
		plot_frame.setLabel('bottom', 'Time elapsed per cycle', units='s')
	elif gcd_plot_options_x_charge_radiobutton.isChecked():
		x_key = "charge_data"
		plot_frame.setLabel('bottom', 'Inserted/extracted charge', units='Ah')

	# Cache current experiment parameters
	lbound = gcd_parameters['lbound'][experiment_index]
	ubound = gcd_parameters['ubound'][experiment_index]
	chg_curr = gcd_parameters['charge_current'][experiment_index]
	dis_curr = gcd_parameters['discharge_current'][experiment_index]
	current_potential_window = f"{lbound}/{ubound}"
	current_cd_current = f"{chg_curr}/{dis_curr}"

	# Add current cycle to the top of the legend
	legend.addItem(gcd_plot_curve, f"Current experiment: {current_potential_window} V; {current_cd_current} µA")

	# Plot previous 10 cycles from this experiment
	if gcd_plot_options_prev10_cycles_radiobutton.isChecked():
		cycles = gcd_data.get('time_data_prev10_cycles', [])
		for i, cycle_time in enumerate(cycles):
			alpha = int(225 * (i + 1) / (len(cycles) + 1))
			pen = pyqtgraph.mkPen(color=(225, 225, 0, alpha))

			if x_key == "time_data":
				x = numpy.array(cycle_time)
				y = numpy.array(gcd_data['potential_data_prev10_cycles'][i])
				if x.size > 0:
					x -= x[0]
					plot_frame.plot(x, y, pen=pen)

			elif x_key == "charge_data":
				x1 = numpy.array(gcd_data['chg_charge_data_prev10_cycles'][i])
				y1 = numpy.array(gcd_data['chg_potential_data_prev10_cycles'][i])
				x2 = numpy.array(gcd_data['dis_charge_data_prev10_cycles'][i])
				y2 = numpy.array(gcd_data['dis_potential_data_prev10_cycles'][i])
				plot_frame.plot(x1, y1, pen=pen)
				plot_frame.plot(x2, y2, pen=pen)

	# Plot nth cycles from this experiment
	if gcd_plot_options_nth_cycles_radiobutton.isChecked():
		dropdown_index = gcd_plot_options_nth_cycles_dropdown.currentIndex()

		if dropdown_index != -1:  # Dropdown list is populated
			nth = int(gcd_plot_options_nth_cycles_dropdown.currentText())
			nth_cycles = [cycle for cycle in range(nth, gcd_current_cyclenum + 1, nth)]

			for i, cycle in enumerate(nth_cycles):
				alpha = int(225 * (i + 1) / (len(nth_cycles) + 1))
				pen = pyqtgraph.mkPen(color=(225, 225, 0, alpha))

				try:
					if x_key == "time_data":
						x = numpy.array(gcd_data['nth_cycles_data'][cycle]['time_data'])
						y = numpy.array(gcd_data['nth_cycles_data'][cycle]['potential_data'])
						if x.size > 0:
							x -= x[0]
							plot_frame.plot(x, y, pen=pen)
					elif x_key == "charge_data":
						x1 = numpy.array(gcd_data['nth_cycles_data'][cycle]['chg_charge_data'])
						y1 = numpy.array(gcd_data['nth_cycles_data'][cycle]['chg_potential_data'])
						x2 = numpy.array(gcd_data['nth_cycles_data'][cycle]['dis_charge_data'])
						y2 = numpy.array(gcd_data['nth_cycles_data'][cycle]['dis_potential_data'])
						plot_frame.plot(x1, y1, pen=pen)
						plot_frame.plot(x2, y2, pen=pen)

				except KeyError:
					continue

	# Plot final cycle from previous experiments
	if gcd_plot_options_prev_experiments_checkbox.isChecked():
		dropdown_index = gcd_plot_options_prev_experiments_dropdown.currentIndex()
		final_data = gcd_data.get('time_data_finalcycle', {})

		for window, currents in final_data.items():
			for curr in currents:
				if dropdown_index == 1 and window != current_potential_window:
					continue
				if dropdown_index == 2 and curr != current_cd_current:
					continue

				color = gcd_parameters['plot_pen_color'][window][curr]
				pen = pyqtgraph.mkPen(color=color)

				if x_key == "time_data":
					x = numpy.array(gcd_data.get(f'{x_key}_finalcycle', {}).get(window, {}).get(curr, []))
					y = numpy.array(gcd_data.get('potential_data_finalcycle', {}).get(window, {}).get(curr, []))
					if x.size > 0:
						x -= x[0]
						gcd_prev_exps_curve = plot_frame.plot(x, y, pen=pen)
				elif x_key == "charge_data":
					x1 = numpy.array(gcd_data['chg_charge_data_finalcycle'][window][curr])
					y1 = numpy.array(gcd_data['chg_potential_data_finalcycle'][window][curr])
					x2 = numpy.array(gcd_data['dis_charge_data_finalcycle'][window][curr])
					y2 = numpy.array(gcd_data['dis_potential_data_finalcycle'][window][curr])
					gcd_prev_exps_curve = plot_frame.plot(x1, y1, pen=pen)
					plot_frame.plot(x2, y2, pen=pen)

				legend.addItem(gcd_prev_exps_curve, f"Previous experiment: {window} V; {curr} µA")

	# Plot current cycle over the top
	if x_key == "time_data":
		x = numpy.array(gcd_data['time_data_currentcycle'])
		y = numpy.array(gcd_data['potential_data_currentcycle'])
		if x.size > 0:
			x -= x[0]
			gcd_plot_curve.setData(x, y)
			plot_frame.addItem(gcd_plot_curve)

	elif x_key == "charge_data":
		x1 = numpy.array(gcd_data.get('chg_charge_data_currentcycle', []))
		y1 = numpy.array(gcd_data.get('chg_potential_data_currentcycle', []))
		x2 = numpy.array(gcd_data.get('dis_charge_data_currentcycle', []))
		y2 = numpy.array(gcd_data.get('dis_potential_data_currentcycle', []))
		gcd_plot_curve.setData(x1, y1)
		plot_frame.addItem(gcd_plot_curve)
		plot_frame.plot(x2, y2, pen='y')

def gcd_update_progress_bar():
	"""Update the progress bar to reflect percentage of half cycles completed."""
	global gcd_cumulative_halfcyclenum

	if gcd_cumulative_halfcyclenum is not None and gcd_total_halfcycles is not None:
		if gcd_cumulative_halfcyclenum > gcd_total_halfcycles:
			gcd_cumulative_halfcyclenum = gcd_total_halfcycles

	gcd_progress_bar.update_progress_bar(gcd_total_halfcycles, gcd_cumulative_halfcyclenum)
	gcd_progress_bar.update()



"""_____CHRONOAMPEROMETRY FUNCTIONS_____"""

"""CA PARAMETER FUNCTIONS"""

def ca_checkbutton_callback():
	"""Function to control the data-validation process, called when "CHECK" button pressed."""
	global ca_parameters_checked, ca_filenames_checked, ca_current_segment_index, ca_total_segments

	# Initialise with parameters_checked = False, filenames_checked = False, and a check button style reset
	ca_parameters_checked = False
	ca_filenames_checked = False
	ca_variables_checkbutton.setStyleSheet("")

	# Remove any previous program state entry
	ca_info_program_state_entry.setText("No experiments running")

	# Check input parameter formats
	if ca_validate_inputs():
		pass
	else:
		ca_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
		log_message("CA check button failed: Inputs are not in the correct format.")
		return False

	# Write input parameters to global dictionary
	if ca_get_parameters():
		pass
	else:
		ca_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
		log_message("CA check button failed: Could not write experiment parameters to a global dictionary.")
		return False

	# If filename provided, check if filenames are acceptable
	if ca_file_entry.text().strip() != "":
		if ca_get_filenames():
			ca_filenames_checked = True
		else:
			ca_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
			log_message("CA check button failed: Experiment filenames could not be constructed.")
			return False

	# Give any parameter warnings
	ca_parameter_warnings()

	# Set global parameters_checked state as True
	ca_parameters_checked = True

	# Make check button green
	ca_variables_checkbutton.setStyleSheet("background-color: green; color: white;")

	# Store the number of halfcycles this experiment will perform
	ca_total_segments = ca_parameters['num_segments']

	# Initialise current_segment_index for progress bar
	ca_current_segment_index = -1

	# Update progress bar and give green border
	ca_update_progress_bar()
	ca_progress_bar.set_solid_green_style()

	log_message(f"Check button successful! Experiments are ready to run. Segments remaining: {ca_total_segments}")
	return True

def ca_validate_inputs():
	"""Ensure inputs are of the correct format."""

	dropdown_bool = ca_alternating_parameters_dropdown.dropdown_frame.isVisible()

	# Potential sequence
	try:
		potentials_str = ca_params_A_potential_entry.text().strip()
		potentials_list = [potential.strip() for potential in potentials_str.split(",")]
		for i, potential in enumerate(potentials_list):
			if potential.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				potentials_list[i] = "OCP"
			else:
				potentials_list[i] = float(potential)

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Potential sequence input" if not dropdown_bool else "Error: Potential sequence A input",
			"Potentials must be numeric values or 'OCP'." if not dropdown_bool else
			"Sequence A potentials must be numeric values or 'OCP'."
		)
		return False

	if dropdown_bool:
		try:
			potentials_B_str = ca_alternating_parameters_dropdown.ca_params_B_potential_entry.text().strip()
			potentials_list_B = [potential_B.strip() for potential_B in potentials_B_str.split(",")]
			for i, potential_B in enumerate(potentials_list_B):
				if potential_B.lower() == "ocp":
					potentials_list_B[i] = "OCP"
				else:
					potentials_list_B[i] = float(potential_B)

			# Check alternating lists are the correct length
			if len(potentials_list) != len(potentials_list_B) and len(potentials_list) != len(potentials_list_B) + 1:
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Alternating parameters potential sequence input",
					"If using alternating parameters, sequence A must be the same length as, or 1 input longer than, sequence B."
				)
				return False

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Potential sequence B input",
				"Sequence B potentials must be numeric values or 'OCP'."
			)
			return False

	# Hold times
	try:
		hold_times_str = ca_params_A_hold_time_entry.text().strip()
		hold_times_list = [float(hold_time.strip()) for hold_time in hold_times_str.split(",")]
		if any(hold_time < 0 for hold_time in hold_times_list):
			raise ValueError

		# Hold times same length as potential sequence
		if len(hold_times_list) != len(potentials_list):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Hold times input" if not dropdown_bool else "Error: Hold times A input",
				"Hold times must be of the same length as potential sequence." if not dropdown_bool else
				"Sequence A hold times must be of the same length as potential sequence A."
			)
			return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Hold times input" if not dropdown_bool else "Error: Hold times A input",
			"Hold times must be non-negative numeric values." if not dropdown_bool else
			"Sequence A hold times must be non-negative numeric values."
		)
		return False

	if dropdown_bool:
		try:
			hold_times_B_str = ca_alternating_parameters_dropdown.ca_params_B_hold_time_entry.text().strip()
			hold_times_list_B = [float(hold_time_B.strip()) for hold_time_B in hold_times_B_str.split(",")]
			if any(hold_time_B < 0 for hold_time_B in hold_times_list_B):
				raise ValueError

			# Hold times same length as potential sequence
			if len(hold_times_list_B) != len(potentials_list_B):
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Hold times B input",
					"Sequence B hold times must be of the same length as potential sequence B."
				)
				return False

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Hold times B input",
				"Sequence B hold times must be non-negative numeric values."
			)
			return False

	# Ramp rates
	try:
		ramp_rates_str = ca_params_A_ramp_rate_entry.text().strip()
		if ramp_rates_str == "":
			ramp_rates_list = []
		else:
			ramp_rates_list = [ramp_rate.strip() for ramp_rate in ramp_rates_str.split(",")]
			for i, ramp_rate in enumerate(ramp_rates_list):
				if ramp_rate.lower() == "step":  # Remove case-sensitivity for "STEP"
					ramp_rates_list[i] = "STEP"
				else:
					ramp_rates_list[i] = float(ramp_rate)

			# Ramp rates list same length as potential sequence
			if len(ramp_rates_list) != len(potentials_list):
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Ramp rates input" if not dropdown_bool else "Error: Ramp rates A input",
					"If ramp rates are given, there must be one per sequence potential." if not dropdown_bool else
					"If sequence A ramp rates are given, there must be one per sequence A potential."
				)
				return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Ramp rates input" if not dropdown_bool else "Error: Ramp rates A input",
			"Ramp rates must be positive numeric values or 'STEP'." if not dropdown_bool else
			"Sequence A ramp rates must be positive numeric values or 'STEP'."
		)
		return False

	if dropdown_bool:
		try:
			ramp_rates_B_str = ca_alternating_parameters_dropdown.ca_params_B_ramp_rate_entry.text().strip()
			if ramp_rates_B_str == "":
				ramp_rates_list_B = []
			else:
				ramp_rates_list_B = [ramp_rate_B.strip() for ramp_rate_B in ramp_rates_B_str.split(",")]
				for i, ramp_rate_B in enumerate(ramp_rates_list_B):
					if ramp_rate_B.lower() == "step":  # Remove case-sensitivity for "STEP"
						ramp_rates_list_B[i] = "STEP"
					else:
						ramp_rates_list_B[i] = float(ramp_rate_B)

				# Ramp rates list B same length as potential sequence B
				if len(ramp_rates_list_B) != len(potentials_list_B):
					QtWidgets.QMessageBox.critical(
						mainwidget,
						"Error: Ramp rates B input",
						"If sequence B ramp rates are given, there must be one per sequence B potential."
					)
					return False

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Ramp rates B input",
				"Sequence B ramp rates must be positive numeric values or 'STEP'."
			)
			return False

	# Ramp rates are positive
	if any(ramp_rate != "STEP" and ramp_rate <= 0 for ramp_rate in ramp_rates_list):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Ramp rates input" if not dropdown_bool else "Error: Ramp rates A input",
			"Ramp rates must be positive numeric values or 'STEP'." if not dropdown_bool else
			"Sequence A ramp rates must be positive numeric values or 'STEP'."
		)
		return False
	if dropdown_bool:
		if any(ramp_rate_B != "STEP" and ramp_rate_B <= 0 for ramp_rate_B in ramp_rates_list_B):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Ramp rates B input",
				"Sequence B ramp rates must be positive numeric values or 'STEP'."
			)
			return False

	# Ramp rates present if expected
	if ca_params_A_ramp_rate_checkbox.isChecked() and ramp_rates_list == []:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Ramp rates input" if not dropdown_bool else "Error: Ramp rates A input",
			"Ramp rates expected but not given." if not dropdown_bool else
			"Sequence A ramp rates expected but not given."
		)
		return False
	if dropdown_bool:
		if ca_alternating_parameters_dropdown.ca_params_B_ramp_rate_checkbox.isChecked() and ramp_rates_list_B == []:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Ramp rates B input",
				"Sequence B ramp rates expected but not given."
			)
			return False

	# Equilibration tolerance
	try:
		curr_eq_tolerance_str = ca_params_A_equilibration_tolerance_entry.text().strip()
		if curr_eq_tolerance_str == "":
			curr_eq_tolerance = None
		else:
			curr_eq_tolerance = float(curr_eq_tolerance_str)
			if curr_eq_tolerance < 0:
				raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Equilibration tolerance input" if not dropdown_bool else "Error: Equilibration tolerance A input",
			"Equilibration tolerance must be a non-negative numeric value." if not dropdown_bool else
			"Sequence A equilibration tolerance must be a non-negative numeric value."
		)
		return False

	if dropdown_bool:
		try:
			curr_eq_tolerance_B_str = ca_alternating_parameters_dropdown.ca_params_B_equilibration_tolerance_entry.text().strip()

			if curr_eq_tolerance_B_str == "":
				curr_eq_tolerance_B = None
			else:
				curr_eq_tolerance_B = float(curr_eq_tolerance_B_str)
				if curr_eq_tolerance_B < 0:
					raise ValueError

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Equilibration tolerance B input",
				"Sequence B equilibration tolerance must be a non-negative numeric value."
			)
			return False

	# Equilibration timescale
	try:
		curr_eq_timescale_str = ca_params_A_equilibration_timescale_entry.text().strip()
		if curr_eq_timescale_str == "":
			curr_eq_timescale = None
		else:
			curr_eq_timescale = float(curr_eq_timescale_str)
			if curr_eq_timescale <= 0:
				raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Equilibration timescale input" if not dropdown_bool else "Error: Equilibration timescale A input",
			"Equilibration timescale must be a positive numeric value." if not dropdown_bool else
			"Sequence A equilibration timescale must be a positive numeric value."
		)
		return False

	if dropdown_bool:
		try:
			curr_eq_timescale_B_str = ca_alternating_parameters_dropdown.ca_params_B_equilibration_timescale_entry.text().strip()
			if curr_eq_timescale_B_str == "":
				curr_eq_timescale_B = None
			else:
				curr_eq_timescale_B = float(curr_eq_timescale_B_str)
				if curr_eq_timescale_B <= 0:
					raise ValueError

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Equilibration timescale B input",
				"Sequence B equilibration timescale must be a positive numeric value."
			)
			return False

	# Current equilibration tolerance and timescale present if expected
	if ca_params_A_equilibration_checkbox.isChecked():
		if curr_eq_tolerance is None:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Equilibration tolerance input" if not dropdown_bool else "Error: Equilibration tolerance A input",
				"Equilibration tolerance expected but not given." if not dropdown_bool else
				"Sequence A equilibration tolerance expected but not given."
			)
			return False
		if curr_eq_timescale is None:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Equilibration timescale input" if not dropdown_bool else "Error: Equilibration timescale A input",
				"Equilibration timescale expected but not given." if not dropdown_bool else
				"Sequence A equilibration timescale expected but not given."
			)
			return False

	if dropdown_bool:
		if ca_alternating_parameters_dropdown.ca_params_B_equilibration_checkbox.isChecked():
			if curr_eq_tolerance_B is None:
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Equilibration tolerance B input",
					"Sequence B equilibration tolerance expected but not given."
				)
				return False
			if curr_eq_timescale_B is None:
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Equilibration timescale B input",
					"Sequence B equilibration timescale expected but not given."
				)
				return False

	# Current limits
	try:
		# Lower limit
		curr_limit_lower_str = ca_params_curr_limits_lower_entry.text().strip()
		if curr_limit_lower_str == "":
			curr_limit_lower = None
		elif curr_limit_lower_str.lower() == "none":
			curr_limit_lower = None
		else:
			curr_limit_lower = float(curr_limit_lower_str)

		# Upper limit
		curr_limit_upper_str = ca_params_curr_limits_upper_entry.text().strip()
		if curr_limit_upper_str == "":
			curr_limit_upper = None
		elif curr_limit_upper_str.lower() == "none":
			curr_limit_upper = None
		else:
			curr_limit_upper = float(curr_limit_upper_str)

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Current limits input",
			"Current limits must each be a numeric value or 'None'."
		)
		return False

	# Upper current limit greater than lower limit
	if curr_limit_lower is not None and curr_limit_upper is not None:
		if curr_limit_lower >= curr_limit_upper:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Current limits input",
				"Upper current limit must be greater than the lower current limit."
			)
			return False

	# Set the current limits to 'None' in GUI
	if ca_params_curr_limits_checkbox.isChecked():
		if curr_limit_lower is None:
			ca_params_curr_limits_lower_entry.setText("None")
		if curr_limit_upper is None:
			ca_params_curr_limits_upper_entry.setText("None")

	# Number of samples to average
	try:
		num_samples = int(ca_params_num_samples_entry.text().strip())
		if num_samples <= 0:
			raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Samples to average input",
			"Samples to average input must be a positive integer value."
		)
		return False

	# Determine whether OCP equilibration required
	if ca_params_delay_OCP_checkbox.isChecked():
		OCP_eq = True
	else:
		OCP_eq = False
		if any(potential == "OCP" for potential in potentials_list):
			OCP_eq = True
		if dropdown_bool:
			if any(potential_B == "OCP" for potential_B in potentials_list_B):
				OCP_eq = True

	# Delay if required
	if not OCP_eq:
		try:
			delay = float(ca_params_delay_entry.text().strip())
			if delay < 0:
				raise ValueError

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Pre-experiment delay input",
				"Pre-experiment delay must be a non-negative numeric value and given if not waiting for OCP equilibration."
			)
			return False

	else:  # Remove the delay
		ca_params_delay_OCP_checkbox.setChecked(True)
		ca_params_delay_entry.setText("")

	return True

def ca_get_parameters():
	""""Write experiment parameters to a global dictionary."""
	global ca_parameters

	ca_parameters = {'type': 'ca'}

	try:
		# Track if OCP to be equilibrated before the experiment
		OCP_bool = False

		# Potential sequence
		potentials = ca_params_A_potential_entry.text().strip()
		potentials_list = [segment_potential.strip() for segment_potential in potentials.split(",")]
		for i, segment_potential in enumerate(potentials_list):
			if segment_potential.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				potentials_list[i] = "OCP"
				OCP_bool = True
			else:
				potentials_list[i] = float(segment_potential)

		ca_parameters['potential_sequence_A'] = potentials_list

		# Potential hold times
		hold_times = ca_params_A_hold_time_entry.text().strip()
		hold_times_list = [float(hold_time.strip()) for hold_time in hold_times.split(",")]

		# Potential ramp rates
		if ca_params_A_ramp_rate_checkbox.isChecked():
			ramp_rates = ca_params_A_ramp_rate_entry.text().strip()
			ramp_rates_list = [ramp_rate.strip() for ramp_rate in ramp_rates.split(",")]
			ramp_rates_mV_list = [ramp_rate.strip() for ramp_rate in ramp_rates.split(",")]
			for i, ramp_rate in enumerate(ramp_rates_list):
				if ramp_rate.lower() == "step":  # Remove case-sensitivity for "STEP"
					ramp_rates_list[i] = "STEP"
					ramp_rates_mV_list[i] = "STEP"
				else:
					ramp_rates_list[i] = float(ramp_rate) * 1e-3  # Convert ramp rate from mV/s to V/s
					ramp_rates_mV_list[i] = float(ramp_rate)
		else:
			ramp_rates_list = ["STEP"] * len(potentials_list)
			ramp_rates_mV_list = ["STEP"] * len(potentials_list)

		# Current equilibration
		if ca_params_A_equilibration_checkbox.isChecked():

			# Tolerance
			curr_eq_tolerance = ca_params_A_equilibration_tolerance_entry.text().strip()
			curr_eq_tolerances_list = [float(curr_eq_tolerance)] * len(potentials_list)

			# Timescale
			curr_eq_timescale = ca_params_A_equilibration_timescale_entry.text().strip()
			curr_eq_timescales_list = [float(curr_eq_timescale)] * len(potentials_list)

		else:
			curr_eq_tolerances_list = [None] * len(potentials_list)
			curr_eq_timescales_list = [None] * len(potentials_list)

		# If alternating parameters are in use:
		if ca_alternating_parameters_dropdown.dropdown_frame.isHidden():
			ca_parameters['alternating_params_bool'] = False
		else:
			ca_parameters['alternating_params_bool'] = True

			# Potential sequence B
			potentials_B = ca_alternating_parameters_dropdown.ca_params_B_potential_entry.text().strip()
			potentials_list_B = [segment_potential.strip() for segment_potential in potentials_B.split(",")]
			for i, segment_potential in enumerate(potentials_list_B):
				if segment_potential.lower() == "ocp":  # Remove case-sensitivity for "OCP"
					potentials_list_B[i] = "OCP"
					OCP_bool = True
				else:
					potentials_list_B[i] = float(segment_potential)

			ca_parameters['potential_sequence_B'] = potentials_list_B

			# Potential hold times B
			hold_times_B = ca_alternating_parameters_dropdown.ca_params_B_hold_time_entry.text().strip()
			hold_times_list_B = [float(hold_time.strip()) for hold_time in hold_times_B.split(",")]

			# Potential ramp rates B
			if ca_alternating_parameters_dropdown.ca_params_B_ramp_rate_checkbox.isChecked():
				ramp_rates_B = ca_alternating_parameters_dropdown.ca_params_B_ramp_rate_entry.text().strip()
				ramp_rates_list_B = [ramp_rate.strip() for ramp_rate in ramp_rates_B.split(",")]
				ramp_rates_mV_list_B = [ramp_rate.strip() for ramp_rate in ramp_rates_B.split(",")]
				for i, ramp_rate in enumerate(ramp_rates_list_B):
					if ramp_rate.lower() == "step":  # Remove case-sensitivity for "STEP"
						ramp_rates_list_B[i] = "STEP"
						ramp_rates_mV_list_B[i] = "STEP"
					else:
						ramp_rates_list_B[i] = float(ramp_rate) * 1e-3  # Convert ramp rate from mV/s to V/s
						ramp_rates_mV_list_B[i] = float(ramp_rate)
			else:
				ramp_rates_list_B = ["STEP"] * len(potentials_list_B)
				ramp_rates_mV_list_B = ["STEP"] * len(potentials_list_B)

			# Current equilibration B
			if ca_alternating_parameters_dropdown.ca_params_B_equilibration_checkbox.isChecked():

				# Tolerance
				curr_eq_tolerance_B = ca_alternating_parameters_dropdown.ca_params_B_equilibration_tolerance_entry.text().strip()
				curr_eq_tolerances_list_B = [float(curr_eq_tolerance_B)] * len(potentials_list_B)

				# Timescale
				curr_eq_timescale_B = ca_alternating_parameters_dropdown.ca_params_B_equilibration_timescale_entry.text().strip()
				curr_eq_timescales_list_B = [float(curr_eq_timescale_B)] * len(potentials_list_B)

			else:
				curr_eq_tolerances_list_B = [None] * len(potentials_list_B)
				curr_eq_timescales_list_B = [None] * len(potentials_list_B)

			# Zip parameters A and B together in alternating fashion
			potentials_list = [item for pair in zip_longest(potentials_list, potentials_list_B) for item in pair]
			ramp_rates_list = [item for pair in zip_longest(ramp_rates_list, ramp_rates_list_B) for item in pair]
			ramp_rates_mV_list = [item for pair in zip_longest(ramp_rates_mV_list, ramp_rates_mV_list_B) for item in pair]
			hold_times_list = [item for pair in zip_longest(hold_times_list, hold_times_list_B) for item in pair]
			curr_eq_tolerances_list = [item for pair in zip_longest(curr_eq_tolerances_list, curr_eq_tolerances_list_B) for item in pair]
			curr_eq_timescales_list = [item for pair in zip_longest(curr_eq_timescales_list, curr_eq_timescales_list_B) for item in pair]

		# Store parameters for each segment in dictionary
		ca_parameters['potential_sequence'] = potentials_list
		ca_parameters['hold_time'] = hold_times_list
		ca_parameters['ramp_rate'] = ramp_rates_list
		ca_parameters['ramp_rate_mV/s'] = ramp_rates_mV_list
		ca_parameters['curr_eq_tolerance'] = curr_eq_tolerances_list
		ca_parameters['curr_eq_timescale'] = curr_eq_timescales_list

		# Current limits
		if ca_params_curr_limits_checkbox.isChecked():

			# Lower limit
			curr_limit_lower = ca_params_curr_limits_lower_entry.text().strip()
			if curr_limit_lower == "":
				curr_limit_lower = None
			elif curr_limit_lower.lower() == "none":    # Remove case-sensitivity for "None"
				curr_limit_lower = None
			else:
				curr_limit_lower = float(curr_limit_lower)

			# Upper limit
			curr_limit_upper = ca_params_curr_limits_upper_entry.text().strip()
			if curr_limit_upper == "":
				curr_limit_upper = None
			elif curr_limit_upper.lower() == "none":   # Remove case-sensitivity for "None"
				curr_limit_upper = None
			else:
				curr_limit_upper = float(curr_limit_upper)

		else:
			curr_limit_lower = None
			curr_limit_upper = None

		ca_parameters['curr_limit_lower'] = curr_limit_lower
		ca_parameters['curr_limit_upper'] = curr_limit_upper

		# Number of samples to average
		ca_parameters['num_samples'] = int(ca_params_num_samples_entry.text().strip())

		# Number of segments
		ca_parameters['num_segments'] = len(ca_parameters['potential_sequence'])

		# Pre-experiment delay
		if ca_params_delay_OCP_checkbox.isChecked():
			OCP_bool = True
		else:
			ca_parameters['delay'] = float(ca_params_delay_entry.text().strip())

		# OCP equilibration pre-experiment
		ca_parameters['OCP_bool'] = OCP_bool

		# Store current measured potential as the current OCP for ca_preview()
		if OCP_bool:
			ca_parameters['current_OCP'] = potential

		# Experiment notes
		experiment_notes = ca_file_notes_entry.toPlainText()
		if not experiment_notes:
			experiment_notes = "No notes provided."
		ca_parameters['experiment_notes'] = experiment_notes

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Input error",
			"One or more parameters are in the wrong format and cannot be written to a global dictionary."
		)
		return False

	return True

def ca_parameter_warnings():
	"""Give GUI warnings for any unused parameters."""

	# Unused ramp rates warning
	if ca_alternating_parameters_dropdown.dropdown_frame.isHidden():
		if ca_params_A_ramp_rate_entry.text().strip() != "" and not ca_params_A_ramp_rate_checkbox.isChecked():
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"Warning: Ramp rates input",
				"Ramp rates provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values, otherwise, the potential will be stepped."
			)
		if (ca_params_A_equilibration_tolerance_entry.text().strip() != "" or ca_params_A_equilibration_timescale_entry.text().strip() != "") and not ca_params_A_equilibration_checkbox.isChecked():
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"Warning: Equilibration input",
				"Equilibration conditions provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values."
			)

	elif not ca_alternating_parameters_dropdown.dropdown_frame.isHidden():
		if ca_params_A_ramp_rate_entry.text().strip() != "" and not ca_params_A_ramp_rate_checkbox.isChecked():
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"Warning: Sequence A ramp rates input",
				"Sequence A ramp rates provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values, otherwise, the potential will be stepped."
			)
		if (ca_params_A_equilibration_tolerance_entry.text().strip() != "" or ca_params_A_equilibration_timescale_entry.text().strip() != "") and not ca_params_A_equilibration_checkbox.isChecked():
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"Warning: Sequence A equilibration input",
				"Sequence A equilibration conditions provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values."
			)
		if ca_alternating_parameters_dropdown.ca_params_B_ramp_rate_entry.text().strip() != "" and not ca_alternating_parameters_dropdown.ca_params_B_ramp_rate_checkbox.isChecked():
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"Warning: Sequence B ramp rates input",
				"Sequence B ramp rates provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values, otherwise, the potential will be stepped."
			)
		if (ca_alternating_parameters_dropdown.ca_params_B_equilibration_tolerance_entry.text().strip() != "" or ca_alternating_parameters_dropdown.ca_params_B_equilibration_timescale_entry.text().strip() != "") and not ca_alternating_parameters_dropdown.ca_params_B_equilibration_checkbox.isChecked():
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"Warning: Sequence B equilibration input",
				"Sequence B equilibration conditions provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values."
			)

	# Unused current limits warning
	if not ca_params_curr_limits_checkbox.isChecked():
		if ca_params_curr_limits_lower_entry.text().strip().lower() != "" or ca_params_curr_limits_upper_entry.text().strip().lower() != "":
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"Warning: Current limits input",
				"Current limits provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values."
			)

	return True

def ca_get_filenames():
	"""Construct filenames for the experiments."""

	global ca_parameters

	try:
		filename_entry = str(ca_file_entry.text().strip())
		if filename_entry == "":
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: No filename provided",
				"Please provide a base filename for the experiments."
			)
			return False

		directory_path = os.path.dirname(filename_entry)
		if directory_path == "":
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: No parent directory specified",
				f"The output files will be stored in the current working directory. Is this okay?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False
			directory_path = os.getcwd()
		elif not os.path.isdir(directory_path):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Directory does not exist",
				f"The directory {directory_path} does not exist."
			)
			return False

		if "_CA_{experiment_info_here}" in filename_entry:
			filename_entry = filename_entry.split("_CA_{experiment_info_here}")[0]
		filename = os.path.basename(filename_entry)
		if filename == "":  # If no filename given, only path
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: No filename provided",
				"Please provide a base filename for the experiments in addition to the path."
			)
			return False

		ca_parameters['directory_path'] = directory_path
		ca_parameters['base_filename'], _ = os.path.splitext(filename)

		exp_filename = f"{ca_parameters['base_filename']}_CA"
		for potential in ca_parameters['potential_sequence']:
			exp_filename += f"_{potential}"
		exp_filename += "_V"

		ca_parameters['filename'] = exp_filename + ".dat"
		ca_parameters['path_filename'] = os.path.join(directory_path, ca_parameters['filename'])

		ca_parameters['experiment_info_filename'] = ca_parameters['base_filename'] + "_CA_experiment_info.txt"
		ca_parameters['experiment_info_path_filename'] = os.path.join(directory_path, ca_parameters['experiment_info_filename'])

		file = ca_parameters['path_filename']
		if os.path.isfile(file):
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: File already exists",
				f"The output file {file} already exists. Do you want to overwrite it?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False

		info_file = ca_parameters['experiment_info_path_filename']
		if os.path.isfile(info_file):
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: Results file already exists",
				f"The experiment info output file {info_file} already exists. Do you want to overwrite it?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False

	except Exception as e:
		print(e)
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: File error",
			f"One or more output filepaths are not valid."
		)
		return False

	ca_file_entry.setText(os.path.join(ca_parameters['directory_path'], f"{ca_parameters['base_filename']}_CA_{{experiment_info_here}}"))

	return True

def ca_validate_filenames():
	"""Check validity of files by creating and attempting to open them."""

	file = ca_parameters.get('path_filename')
	if file:
		try:
			with open(file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {file} is not valid."
			)
			return False

	info_file = ca_parameters.get('experiment_info_path_filename')
	if info_file:
		try:
			with open(info_file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {info_file} is not valid."
			)
			return False

	return True

def ca_freeze_inputs(freeze):
	"""Function to freeze and unfreeze GUI inputs when experiments are running."""

	if freeze:
		ca_params_A_potential_entry.setEnabled(False)
		ca_params_A_hold_time_entry.setEnabled(False)
		ca_params_A_ramp_rate_checkbox.setEnabled(False)
		ca_params_A_ramp_rate_entry.setEnabled(False)
		ca_params_A_equilibration_checkbox.setEnabled(False)
		ca_params_A_equilibration_tolerance_entry.setEnabled(False)
		ca_params_A_equilibration_timescale_entry.setEnabled(False)
		ca_params_curr_limits_checkbox.setEnabled(False)
		ca_params_curr_limits_lower_entry.setEnabled(False)
		ca_params_curr_limits_upper_entry.setEnabled(False)
		ca_params_num_samples_entry.setEnabled(False)
		ca_params_delay_entry.setEnabled(False)
		ca_params_delay_OCP_checkbox.setEnabled(False)
		ca_file_entry.setEnabled(False)
		ca_file_notes_entry.setEnabled(False)
		for i in range(len(current_range_list)):
			ca_range_checkboxes[i].setEnabled(False)
		ca_variables_checkbutton.setEnabled(False)
		ca_alternating_parameters_dropdown.freezeInputs(True)
		software_globals_menu_button.setEnabled(False)

	elif not freeze:
		ca_params_A_potential_entry.setEnabled(True)
		ca_params_A_hold_time_entry.setEnabled(True)
		ca_params_A_ramp_rate_checkbox.setEnabled(True)
		ca_params_A_ramp_rate_entry.setEnabled(True)
		ca_params_A_equilibration_checkbox.setEnabled(True)
		ca_params_A_equilibration_tolerance_entry.setEnabled(True)
		ca_params_A_equilibration_timescale_entry.setEnabled(True)
		ca_params_curr_limits_checkbox.setEnabled(True)
		ca_params_curr_limits_lower_entry.setEnabled(True)
		ca_params_curr_limits_upper_entry.setEnabled(True)
		ca_params_num_samples_entry.setEnabled(True)
		ca_params_delay_entry.setEnabled(True)
		ca_params_delay_OCP_checkbox.setEnabled(True)
		ca_file_entry.setEnabled(True)
		ca_file_notes_entry.setEnabled(True)
		for i in range(len(current_range_list)):
			ca_range_checkboxes[i].setEnabled(True)
		ca_variables_checkbutton.setEnabled(True)
		ca_alternating_parameters_dropdown.freezeInputs(False)
		software_globals_menu_button.setEnabled(True)


"""CA CORE FUNCTIONS"""

def ca_initialise():
	"""Initialise chronoamperometry experiment."""
	global state
	global ca_data, ca_current_segment_index
	global ca_time_data, ca_segment_time_data, ca_potential_data, ca_current_data

	# Ensure parameters have been verified using the "CHECK" button
	if not ca_parameters_checked:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Parameters not checked",
			"Click the 'CHECK' button to ensure parameters are appropriate before starting your experiments."
		)
		return False

	# Ensure filenames have been checked
	if not ca_filenames_checked:
		if ca_get_filenames():
			pass
		else:
			return False

	if check_state([States.Idle, States.Stationary_Graph]):

		# Validate filenames before experiment begins
		if ca_validate_filenames():
			pass
		else:
			ca_reset_experiment_controller(mode="interrupted")
			log_message("CA experiment could not initialise due to invalid output filename.")
			return False

		# Turn cell off if under manual control
		set_cell_status(False)

		# Freeze input fields and hide return to live graph button
		ca_freeze_inputs(freeze=True)
		preview_cancel_button.hide()

		# Initialise segment index
		ca_current_segment_index = 0

		# Write experiment info to summary file
		ca_write_summary_file(ca_current_segment_index, section="initial")

		# Update GUI
		ca_info_segmentnum_entry.setText(f"-/{ca_parameters['num_segments']}")
		log_message("Starting chronoamperometry experiment...")

		# Initialise ca_data dictionary
		ca_data = {
			'starttime': defaultdict(float),
			'starttime_readable': defaultdict(str),
			'finishtime_readable': defaultdict(str),

			'startpot': defaultdict(float),
			'time': defaultdict(list),
			'ramping/holding': defaultdict(list),
			'segment_time': defaultdict(list),
			'potential': defaultdict(list),
			'current': defaultdict(list),
		}

		# Pass to OCP equilibration controller if required
		if ca_parameters['OCP_bool']:
			OCP_initialise_data_entries(ca_data)
			OCP_equilibration_controller(ca_parameters, ca_data, ca_current_segment_index, equilibrated=False)
		else:
			# Write pre-experiment delay to summary file and update GUI
			ca_write_summary_file(ca_current_segment_index, section="delay")
			ca_info_program_state_entry.setText(f"Delay of {ca_parameters['delay']} s")

			# Update progress bar style to solid yellow border
			ca_progress_bar.set_solid_yellow_style()

			# Update state
			state = States.Measuring_CA_Delay

			# Launch the delay timer
			ca_delay_timer.start(int(ca_parameters['delay'] * 1000))  # Delay input in ms

def ca_start(segment_index):
	"""Begin or progress the chronoamperometry experiment."""
	global state, skipcounter
	global ca_data, ca_time_data, ca_segment_time_data, ca_potential_data, ca_current_data
	global ca_potential_hold, ca_curr_eq_history, ca_output_file
	global ca_potential_plot_curve, ca_current_plot_curve, legend, legend_in_use

	if segment_index is None:
		state = States.Stationary_Graph
		preview_cancel_button.show()
		ca_write_summary_file(ca_current_segment_index, section="error")
		ca_reset_experiment_controller(mode="interrupted")
		log_message("Experiments could not start due to segment_index initialisation error.")
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Initialisation",
			"The experiments could not initialise segment_index correctly."
		)
		return

	# Write segment information to summary file
	ca_data['starttime_readable'][segment_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]
	ca_write_summary_file(segment_index, section="segment_start")

	# Store start potential
	ca_data['startpot'][segment_index] = potential

	# Initialise state describing whether the potential is holding or ramping
	ca_potential_hold = False

	# If initial segment
	if segment_index == 0:

		# Open output file and write header
		try:
			ca_output_file = open(ca_parameters['path_filename'], 'w', 1)
			ca_output_file.write("Segment number\tRamping/Holding\tElapsed time (s)\tSegment time(s)\tPotential (V)\tCurrent (A)\n")
		except Exception as e:
			log_message(f"Write to file failed: {e}")
			ca_stop(segment_index, interrupted=True)
			return

		# Initialise buffers for holding averaged elapsed time, potential, and current data for each segment
		ca_time_data = AverageBuffer(ca_parameters['num_samples'])
		ca_segment_time_data = AverageBuffer(ca_parameters['num_samples'])
		ca_potential_data = AverageBuffer(ca_parameters['num_samples'])
		ca_current_data = AverageBuffer(ca_parameters['num_samples'])

		# Calculate ramp times and segment timelengths
		ca_parameters['ramp_time'] = []
		ca_parameters['segment_timelength'] = []
		for i, (segment_potential, ramp_rate) in enumerate(zip(ca_parameters['potential_sequence'], ca_parameters['ramp_rate'])):
			if ramp_rate == "STEP":
				ca_parameters['ramp_time'].append(0)
				ca_parameters['segment_timelength'].append(ca_parameters['hold_time'][i])
			else:
				segment_potential = segment_potential if segment_potential != "OCP" else ca_parameters['current_OCP']
				if i == 0:
					potential_to_sweep = segment_potential - ca_data['startpot'][0]
				else:
					prev_potential = ca_parameters['potential_sequence'][i-1]
					prev_potential = prev_potential if prev_potential != "OCP" else ca_parameters['current_OCP']
					potential_to_sweep = segment_potential - prev_potential

				ramp_time = abs(potential_to_sweep / ramp_rate)
				ca_parameters['ramp_time'].append(ramp_time)
				ca_parameters['segment_timelength'].append(ramp_time + ca_parameters['hold_time'][i])

		# Initialise the plot area
		Legends.remove_all_legends()
		plot_frame.clear()
		plot_frame.enableAutoRange()
		plot_frame.getAxis('bottom').setTicks(None)
		plot_frame.setLabel('bottom', 'Time', units="s")
		plot_frame.setLabel('left', 'Current', units="A")
		ca_current_plot_curve = plot_frame.plot(pen='r', name='Current')
		ca_potential_plot_curve = plot_frame.plot(pen='y', name='Potential (scaled)')
		legend = pyqtgraph.LegendItem(offset=(60, 10))
		legend.setParentItem(plot_frame.plotItem)
		Legends.legends['ca'] = legend
		legend_in_use = 'ca'

		# Update progress bar style to solid green border
		ca_progress_bar.set_solid_green_style()

		# Update GUI
		ca_info_program_state_entry.setText("Measuring CA")

		# Determine potential to send to the DAC
		if ca_parameters['ramp_rate'][0] == "STEP":
			setpot = ca_parameters['potential_sequence'][0]
			if setpot == "OCP":
				setpot = ca_parameters['current_OCP']
		else:
			setpot = ca_data['startpot'][0]

		# Initialise DAC for measurements
		set_output(0, setpot)
		set_control_mode(False)  # Potentiostatic control
		hardware_manual_control_range_dropdown.setCurrentIndex(0)  # Start at highest current range
		set_current_range()
		time.sleep(.1)  # Allow DAC some time to settle
		set_cell_status(True)  # Cell on
		time.sleep(.1)  # Allow feedback loop some time to settle
		read_potential_current()
		time.sleep(.1)
		read_potential_current()  # Two reads are necessary because each read actually returns the result of the previous conversion
		hardware_manual_control_range_dropdown.setCurrentIndex(
			get_next_enabled_current_range(current_range_from_current(current),
										   experiment="CA"))  # Autorange based on the measured current
		set_current_range()
		time.sleep(.1)
		read_potential_current()
		time.sleep(.1)
		read_potential_current()
		hardware_manual_control_range_dropdown.setCurrentIndex(
			get_next_enabled_current_range(current_range_from_current(current),
										   experiment="CA"))  # Another autorange, just to be sure
		set_current_range()

		# Begin calling ca_update() through periodic_update()
		state = States.Measuring_CA
		skipcounter = 2  # Skip first two data points to suppress artifacts

		# Store experiment start time
		ca_data['experiment_start_time'] = timeit.default_timer()

	# If not initial segment
	else:
		# Refresh data sample buffers
		for data in [ca_time_data, ca_segment_time_data, ca_potential_data, ca_current_data]:
			data.samples = []

	# Initialise deque to store current history for equilibration
	ca_curr_eq_history = collections.deque()

	# Update GUI and add segment to plot options dropdown lists
	ca_info_segmentnum_entry.setText(f"{segment_index + 1}/{ca_parameters['num_segments']}")
	ca_plot_options_segment1_dropdown.addItems([str(segment_index + 1)])
	ca_plot_options_update_segment_selector()
	ca_update_progress_bar()

	# Store segment start time
	ca_data['starttime'][segment_index] = timeit.default_timer()

def ca_update(segment_index):
	"""Add a new data point to the chronoamperometry measurement - called regularly through periodic_update()"""
	global skipcounter, ca_potential_hold

	# Calculate total and segment time elapsed
	elapsed_time = timeit.default_timer() - ca_data['experiment_start_time']
	segment_elapsed_time = timeit.default_timer() - ca_data['starttime'][segment_index]

	# If segment has completed
	if segment_elapsed_time >= ca_parameters['segment_timelength'][segment_index]:

		# Progress to the next segment
		ca_stop(segment_index, interrupted=False)

	else:
		# DAC control logic
		ramp_rate = ca_parameters['ramp_rate'][segment_index]
		ramp_time = ca_parameters['ramp_time'][segment_index]

		# Ramping
		if ramp_rate != "STEP" and segment_elapsed_time < ramp_time:
			ca_output = ca_sweep(segment_index, segment_elapsed_time)  # Determine potential to send to the DAC
			set_output(0, ca_output)  # Send potential to the DAC

		# Holding
		elif segment_elapsed_time >= ramp_time:
			if not ca_potential_hold:
				holdpot = ca_parameters['potential_sequence'][segment_index]
				if holdpot == "OCP":
					holdpot = ca_parameters['current_OCP']
				set_output(0, holdpot)  # Send potential to the DAC
				ca_potential_hold = True

		read_potential_current()  # Read new potential and current

		if skipcounter == 0:  # Process new measurements
			ca_time_data.add_sample(elapsed_time)
			ca_segment_time_data.add_sample(segment_elapsed_time)
			ca_potential_data.add_sample(potential)
			ca_current_data.add_sample(1e-3 * current)  # Convert from mA to A
			if len(ca_segment_time_data.samples) == 0 and len(ca_segment_time_data.averagebuffer) > 0:  # Check if a new average was just calculated

				if segment_elapsed_time >= ramp_time:
					ramp_or_hold = "Holding"
				else:
					ramp_or_hold = "Ramping"

				# Write new data to output file
				try:
					ca_output_file.write("%d\t%s\t%e\t%e\t%e\t%e\n" % (
						segment_index + 1,
						ramp_or_hold,
						ca_time_data.averagebuffer[-1],
						ca_segment_time_data.averagebuffer[-1],
						ca_potential_data.averagebuffer[-1],
						ca_current_data.averagebuffer[-1]
					))
				except Exception as e:
					log_message(f"Write to file failed: {e}")
					ca_stop(segment_index, interrupted=True)
					return

				# Append data to lists
				ca_data['time'][segment_index].append(ca_time_data.averagebuffer[-1])
				ca_data['ramping/holding'][segment_index].append(ramp_or_hold)
				ca_data['segment_time'][segment_index].append(ca_segment_time_data.averagebuffer[-1])
				ca_data['potential'][segment_index].append(ca_potential_data.averagebuffer[-1])
				ca_data['current'][segment_index].append(ca_current_data.averagebuffer[-1])

				# Update plot
				ca_update_plot(segment_index)

				# Check if current has equilibrated
				if ca_parameters['curr_eq_tolerance'][segment_index]:
					if ca_curr_eq_bool(segment_index, segment_elapsed_time):

						# Write to summary file and progress to next segment
						ca_write_summary_file(segment_index, section="current_equilibrated")
						log_message("*** Current equilibrated ***")
						ca_stop(segment_index, interrupted=False)
						return

				# Check if current limits have been surpassed
				if ca_parameters['curr_limit_lower']:
					if ca_current_data.averagebuffer[-1] <= ca_parameters['curr_limit_lower'] * 1e-3:  # Convert mA to A

						# Write to summary file and interrupt experiment
						ca_write_summary_file(segment_index, section="lower_curr_limit_reached")
						log_message("***** Lower current limit reached! *****")
						ca_stop(segment_index, interrupted=True)
						return

				if ca_parameters['curr_limit_upper']:
					if ca_current_data.averagebuffer[-1] >= ca_parameters['curr_limit_upper'] * 1e-3:  # Convert mA to A

						# Write to summary file and interrupt experiment
						ca_write_summary_file(segment_index, section="upper_curr_limit_reached")
						log_message("***** Upper current limit reached! *****")
						ca_stop(segment_index, interrupted=True)
						return

			skipcounter = auto_current_range(experiment="CA")  # Update the graph

		else:  # Wait until the required number of data points are skipped
			skipcounter -= 1

def ca_sweep(segment_index, segment_elapsed_time):

	ramp_time = ca_parameters['ramp_time'][segment_index]
	startpot = ca_data['startpot'][segment_index]
	holdpot = ca_parameters['potential_sequence'][segment_index]
	if holdpot == "OCP":
		holdpot = ca_parameters['current_OCP']

	if segment_elapsed_time >= ramp_time:
		return holdpot
	else:
		return startpot + (holdpot - startpot) * (segment_elapsed_time / ramp_time)

def ca_curr_eq_bool(segment_index, segment_elapsed_time):

	equilibrated = False
	if segment_elapsed_time >= ca_parameters['ramp_time'][segment_index]:
		ca_curr_eq_history.append((segment_elapsed_time, ca_current_data.averagebuffer[-1]))

		# Remove data points older than curr_eq_timescale from the history deque
		while ca_curr_eq_history and (segment_elapsed_time - ca_curr_eq_history[0][0]) > ca_parameters['curr_eq_timescale'][segment_index]:
			ca_curr_eq_history.popleft()

		# Start checking if the change in current is within tolerance after curr_eq_timescale following ramp
		if segment_elapsed_time >= ca_parameters['curr_eq_timescale'][segment_index] + ca_parameters['ramp_time'][segment_index]:
			current_current = ca_current_data.averagebuffer[-1]
			if ca_curr_eq_history:
				past_time, past_current = ca_curr_eq_history[0]

				# Check if the change in current across curr_eq_timescale is within tolerance
				if abs(current_current - past_current) < ca_parameters['curr_eq_tolerance'][segment_index] * 1e-6:  # Convert µA to A
					equilibrated = True

	return equilibrated

def ca_stop(segment_index, interrupted=True):
	global state, ca_current_segment_index

	if check_state([States.Measuring_CA, States.Measuring_CA_OCP_eq, States.Measuring_CA_Delay]):

		# Save segment finish time
		ca_data['finishtime_readable'][segment_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]

		if interrupted:

			set_cell_status(False)  # Cell off
			state = States.Stationary_Graph

			# Write to summary file and close
			ca_write_summary_file(segment_index, section="interrupted")

			# Reset experiment
			ca_reset_experiment_controller(mode="interrupted")

			# Update GUI
			log_message("*** EXPERIMENTS INTERRUPTED ***")
			QtWidgets.QMessageBox.information(
				mainwidget,
				"CA experiments interrupted",
				"Oh no! CA experiments have been interrupted.\n\nGlobal experiment parameters have been reset."
			)
			preview_cancel_button.show()

		elif not interrupted:

			# Write to summary file
			ca_write_summary_file(segment_index, section="segment_end")
			log_message(f"*** Segment {segment_index + 1}/{ca_parameters['num_segments']} completed ***")

			# If not final segment
			if segment_index + 1 != ca_parameters['num_segments']:

				# Increment the current segment index
				ca_current_segment_index += 1

				# Begin the next CA segment
				ca_start(ca_current_segment_index)

			# If final segment completed
			elif segment_index + 1 == ca_parameters['num_segments']:

				set_cell_status(False)  # Cell off
				state = States.Stationary_Graph

				# Write to summary file and close
				ca_write_summary_file(segment_index, section="all_segments_completed")

				# Reset experiment
				ca_reset_experiment_controller(mode="all_segments_completed")

				# Update GUI
				ca_info_program_state_entry.setText("All segments completed")
				QtWidgets.QMessageBox.information(
					mainwidget,
					"CA experiment completed",
					"CONGRATULATIONS! All CA segments have completed successfully.\n\nGlobal experiment parameters have been reset."
				)
				preview_cancel_button.show()

def ca_reset_experiment_controller(mode):
	global ca_parameters, ca_data
	global ca_parameters_checked, ca_filenames_checked
	global ca_total_segments, ca_current_segment_index
	global ca_potential_hold

	# Stop timer
	ca_delay_timer.stop()

	if mode == "input_changed":
		if ca_parameters_checked:  # If inputs have changed since last successful check

			# Reset globals
			ca_parameters_checked = False
			ca_filenames_checked = False
			ca_variables_checkbutton.setStyleSheet("")

			# Reset progress bar
			ca_total_segments = None
			ca_current_segment_index = None
			ca_update_progress_bar()

			# Remove preview if showing
			if legend == Legends.legends['ca_preview'] and legend_in_use == 'ca_preview':
				if state == States.NotConnected:
					plot_frame.clear()
					legend.clear()
				elif state in (States.Idle, States.Stationary_Graph):
					preview_cancel()

			log_message("CA input parameters or program state has changed since the last successful check - check-state has been reset.")

		return

	elif mode == "checkbutton_failed":

		# Reset progress bar
		ca_total_segments = None
		ca_current_segment_index = None
		ca_update_progress_bar()
		return

	# Ensure output files are closed
	for file in ('ca_output_file', 'ca_summary_file'):
		try:
			if file in globals():
				f = globals()[file]
				if f and hasattr(f, 'close'):
					f.close()
					globals()[file] = None  # Clear reference to file
		except Exception as e:
			log_message(f"Error closing {file}: {e}")

	# Reset globals
	ca_parameters_checked = False
	ca_filenames_checked = False
	ca_variables_checkbutton.setStyleSheet("")
	ca_parameters = {'type': 'ca'}
	ca_data = {}
	ca_total_segments = None
	ca_current_segment_index = None
	ca_potential_hold = None

	# Reset GUI
	ca_info_segmentnum_entry.setText("-/-")
	ca_plot_options_segment1_dropdown.clear()
	ca_plot_options_segment2_dropdown.clear()

	# Unfreeze input fields
	ca_freeze_inputs(freeze=False)

	if mode == "all_segments_completed":
		ca_progress_bar.set_completed_state()

	elif mode == "interrupted":
		ca_info_program_state_entry.setText(f"Experiments interrupted")
		ca_progress_bar.set_interrupted_state()

	elif mode == "OCP_interrupted":
		ca_progress_bar.set_OCP_interrupted_state()


"""CA ACCESSORY FUNCTIONS"""

def ca_preview():
	"""Generate a preview of the chronoamperometry potential profile in the plot window, based on the CA parameters currently entered in the GUI."""
	global legend, legend_in_use

	if ca_parameters_checked is False:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Parameters not checked",
			"Click the 'CHECK' button to ensure parameters are appropriate before constructing the preview."
		)
		return False

	if check_state([States.NotConnected, States.Idle, States.Stationary_Graph]):

		time_sequence = []
		potential_sequence = []
		segment_start_end, segment_labels = [], []

		running_time = 0.0
		OCP_bool = ca_parameters['OCP_bool']
		OCP_in_use = False  # Track whether OCP used as a potential
		num_segments = len(ca_parameters['potential_sequence'])

		delay = global_software_settings['OCP_eq_timescale'] if OCP_bool else ca_parameters['delay']
		running_time += delay

		potential_sequence.append(potential)
		time_sequence.append(running_time)

		# Construct potential profile
		for segment_index in range(num_segments):
			startpot = potential if segment_index == 0 else ca_parameters['potential_sequence'][segment_index - 1]
			ramp_rate = ca_parameters['ramp_rate'][segment_index]
			hold_time = ca_parameters['hold_time'][segment_index]
			holdpot = ca_parameters['potential_sequence'][segment_index]
			if holdpot == "OCP":
				holdpot = potential
				OCP_in_use = True

			segment_start = running_time

			if ramp_rate == "STEP":
				potential_sequence.append(holdpot)
				potential_sequence.append(holdpot)
				time_sequence.append(running_time)
				running_time += hold_time
				time_sequence.append(running_time)
			else:
				potential_sequence.append(holdpot)
				potential_sequence.append(holdpot)
				ramp_time = abs((holdpot - startpot) / ramp_rate)
				running_time += ramp_time
				time_sequence.append(running_time)
				running_time += hold_time
				time_sequence.append(running_time)

			segment_end = running_time
			segment_start_end.append([segment_start, segment_end])
			segment_labels.append(f"{segment_index + 1}")

		# Plot potential profile in the plot window
		Legends.remove_all_legends()
		plot_frame.clear()

		if len(set(potential_sequence)) == 1:  # Manually scale y-axis if potential sequence is flat
			pad = 0.01
			ymin = potential_sequence[0] - pad
			ymax = potential_sequence[0] + pad
			plot_frame.setYRange(ymin, ymax, padding=0)
		else:
			plot_frame.enableAutoRange()

		plot_frame.getAxis('bottom').setTicks(None)
		plot_frame.setLabel('bottom', 'Time', units='s')
		plot_frame.setLabel('left', 'Potential', units='V')

		legend = pyqtgraph.LegendItem(offset=(60, 10))
		legend.setParentItem(plot_frame.plotItem)
		Legends.legends['ca_preview'] = legend
		legend_in_use = 'ca_preview'

		potential_curve = pyqtgraph.PlotDataItem([], [], pen='g')
		legend.addItem(potential_curve, "Chronoamperometry potential profile")

		if OCP_in_use:
			dummy_item = pyqtgraph.PlotDataItem([], [], pen=None)
			legend.addItem(dummy_item, f"(estimating OCP as measured potential: {potential:.3f} V)")

		# Plot individual segment lines
		min_pot = min(potential_sequence)
		max_pot = max(potential_sequence)
		pot_range = max_pot - min_pot
		margin = pot_range * 0.1 if pot_range != 0 else 0.01  # Avoid zero range

		y0 = min_pot - margin
		y1 = max_pot + margin

		for i, (segment_start, segment_end) in enumerate(segment_start_end):
			line = pyqtgraph.PlotDataItem(
				x=[segment_start, segment_start],
				y=[y0, y1],
				pen=pyqtgraph.mkPen('w', width=1, style=QtCore.Qt.DashLine)
			)
			plot_frame.addItem(line)
			label = pyqtgraph.TextItem(f'{i+1}', color='w', anchor=(0.5, 1))
			label.setPos((segment_start + segment_end) / 2, max_pot)
			plot_frame.addItem(label)

		# Add pre-delay line and final line
		if delay > 0:
			start_line = pyqtgraph.PlotDataItem(
				x=[0, 0],
				y=[y0, y1],
				pen=pyqtgraph.mkPen('w', width=1, style=QtCore.Qt.DashLine)
			)
			plot_frame.addItem(start_line)
		end_line = pyqtgraph.PlotDataItem(
			x=[segment_start_end[-1][1], segment_start_end[-1][1]],
			y=[y0, y1],
			pen=pyqtgraph.mkPen('w', width=1, style=QtCore.Qt.DashLine)
		)
		plot_frame.addItem(end_line)

		# Plot potential profile
		plot_frame.plot(time_sequence, potential_sequence, pen='g')

		if state in [States.Idle, States.Stationary_Graph]:
			preview_cancel_button.show()

def ca_write_summary_file(segment_index, section):
	"""Write summary file for the experiment."""
	global ca_summary_file

	try:
		if section == "initial":
			ca_summary_file = open(ca_parameters['experiment_info_path_filename'], 'w', 1)
			ca_summary_file.write("CA EXPERIMENTS INFORMATION FILE\n*******************************\n")

			ca_summary_file.write(f"\nExperiment notes: {ca_parameters['experiment_notes']}\n")

			ca_summary_file.write("\nExperiment information file for the experiments stored in:\n")
			ca_summary_file.write(f"{ca_parameters['path_filename']}\n")
			ca_summary_file.write("\n")

			current_ranges = []
			for i in range(len(current_range_list)):
				if ca_range_checkboxes[i].isChecked():
					current_ranges.append(current_range_list[i])
			current_range_str = ', '.join(f"{curr}" for curr in current_ranges)

			if ca_parameters['OCP_bool']:
				delay_str = "Wait for OCP equilibration."
			else:
				delay_str = f"{ca_parameters['delay']}"

			if not ca_parameters['alternating_params_bool']:
				ca_summary_file.write("Alternating parameters?: No\n\n")
			else:
				ca_summary_file.write("Alternating parameters?: Yes\n")
				ca_summary_file.write("The following potential sequence, hold times, and ramp rates follow A-B-A-B-A-...\n\n")
			ca_summary_file.write(f"Potential sequence (V): {ca_parameters['potential_sequence']}\n")
			ca_summary_file.write(f"Hold times per potential (s): {ca_parameters['hold_time']}\n")
			ca_summary_file.write(f"Ramp rate per potential (mV/s): {ca_parameters['ramp_rate_mV/s']}\n")
			if not ca_parameters['alternating_params_bool']:
				ca_summary_file.write(f"Equilibration tolerance (µA): {ca_parameters['curr_eq_tolerance'][0]}\n")
				ca_summary_file.write(f"Equilibration timescale (s): {ca_parameters['curr_eq_timescale'][0]}\n")
			else:
				ca_summary_file.write(f"Equilibration tolerance A (µA): {ca_parameters['curr_eq_tolerance'][0]}\n")
				ca_summary_file.write(f"Equilibration timescale A (s): {ca_parameters['curr_eq_timescale'][0]}\n")
				ca_summary_file.write(f"Equilibration tolerance B (µA): {ca_parameters['curr_eq_tolerance'][1]}\n")
				ca_summary_file.write(f"Equilibration timescale B (s): {ca_parameters['curr_eq_timescale'][1]}\n")
			ca_summary_file.write(f"Autoranging: {current_range_str}\n")
			ca_summary_file.write(f"Lower current limit (mA): {ca_parameters['curr_limit_lower']}\n")
			ca_summary_file.write(f"Upper current limit (mA): {ca_parameters['curr_limit_upper']}\n")
			ca_summary_file.write(f"Samples to average: {ca_parameters['num_samples']}\n")
			ca_summary_file.write(f"Pre-experiment delay (s): {delay_str}\n")

		elif section == "delay":
			ca_summary_file.write(f"\n*** Pre-experiment delay of {ca_parameters['delay']} seconds ***\n")

		elif section == "segment_start":
			ca_summary_file.write("\n**********************************\n\tCA SEGMENT STARTED\n**********************************\n")
			ca_summary_file.write(f"Segment number: {segment_index + 1}/{ca_parameters['num_segments']}\n")
			ca_summary_file.write(f"Hold potential (V): {ca_parameters['potential_sequence'][segment_index]}\n")
			ca_summary_file.write(f"Hold time (s): {ca_parameters['hold_time'][segment_index]}\n")
			ca_summary_file.write(f"Ramp rate (mV/s): {ca_parameters['ramp_rate_mV/s'][segment_index]}\n")
			ca_summary_file.write(f"Equilibration tolerance (µA): {ca_parameters['curr_eq_tolerance'][segment_index]}\n")
			ca_summary_file.write(f"Equilibration timescale (s): {ca_parameters['curr_eq_timescale'][segment_index]}\n")
			ca_summary_file.write("\n")

			ca_summary_file.write(f"Segment start time:  {ca_data['starttime_readable'][segment_index]}\n")

		elif section == "current_equilibrated":
			ca_summary_file.write("\n***** CURRENT EQUILIBRATED *****\n")
			ca_summary_file.write(f"Current ({ca_parameters['curr_eq_timescale'][segment_index]}) s ago (µA): {ca_curr_eq_history[0][1] * 1e6:.3f}\n")
			ca_summary_file.write(f"Equilibrated current (µA): {ca_curr_eq_history[-1][1] * 1e6:.3f}\n")
			ca_summary_file.write(f"Equilibrated segment time (s): {ca_curr_eq_history[-1][0]:.3f}\n")
			ca_summary_file.write("\n")

		elif section == "segment_end":
			ca_summary_file.write(f"Segment finish time: {ca_data['finishtime_readable'][segment_index]}\n")

			ca_summary_file.write("\n************************************\n\tCA SEGMENT COMPLETED\n************************************\n")

		elif section == "all_segments_completed":
			ca_summary_file.write("\n***************************************************\n\tALL SEGMENTS COMPLETED SUCCESSFULLY\n***************************************************\n")

			ca_summary_file.write("\n")
			ca_summary_file.write("********************\n")
			ca_summary_file.write("EXPERIMENTS COMPLETE\n")
			ca_summary_file.close()

		elif section == "OCP_eq_start":
			ca_summary_file.write("\n*********************************\n\tOCP EQUILIBRATING\n*********************************\n")
			ca_summary_file.write(f"Equilibration tolerance (mV): {global_software_settings['OCP_eq_tolerance']}\n")
			ca_summary_file.write(f"Equilibration timescale (s): {global_software_settings['OCP_eq_timescale']}\n")
			ca_summary_file.write(f"Equilibration timeout (s): {global_software_settings['OCP_eq_timeout']}\n")
			ca_summary_file.write("\n")

			ca_summary_file.write(f"OCP equilibration start time: {ca_data['OCP_starttime_readable'][segment_index]}\n")

		elif section == "OCP_equilibrated":
			ca_summary_file.write(f"OCP equilibration finish time: {ca_data['OCP_eq_finishtime_readable'][segment_index]}\n")
			ca_summary_file.write("\n")

			ca_summary_file.write(f"Elapsed time (s): {ca_data['OCP_eq_elapsed_times'][-1]:.3f}\n")
			ca_summary_file.write(f"Starting potential (V): {ca_data['OCP_eq_startpot'][segment_index]:.5g}\n")
			ca_summary_file.write(f"Final equilibrated OCP (V): {ca_data['OCP_eq_stoppot'][segment_index]:.5g}\n")
			ca_summary_file.write(f"Total potential difference (V): {ca_data['OCP_eq_total_pot_diff'][segment_index]:.5g}\n")
			ca_summary_file.write(f"Final ({global_software_settings['OCP_eq_timescale']}) s potential difference (V): {ca_data['OCP_eq_timescale_pot_diff'][segment_index]:.5g}\n")
			ca_summary_file.write("\n")

			ca_summary_file.write("***** OCP successfully equilibrated *****\n")

		elif section == "OCP_timeout":
			ca_summary_file.write(f"OCP equilibration timeout time: {ca_data['OCP_timeout_finishtime_readable'][segment_index]}\n")

			ca_summary_file.write("\n*******************************************\n\tCA MEASUREMENTS INTERRUPTED\n*******************************************\n")
			ca_summary_file.write("CA experiments stopped due to OCP equilibration timeout.\n")
			ca_summary_file.write("OCP did not equilibrate to within tolerance by the timeout threshold.\n")
			ca_summary_file.write("\n")

			ca_summary_file.write(f"OCP timeout threshold (s): {global_software_settings['OCP_eq_timeout']}\n")
			ca_summary_file.write(f"Elapsed time (s): {ca_data['OCP_timeout_time_elapsed'][segment_index]:.3g}\n")
			ca_summary_file.write(f"Starting potential (V): {ca_data['OCP_timeout_startpot'][segment_index]:.5g}\n")
			ca_summary_file.write(f"Final potential (V): {ca_data['OCP_timeout_stoppot'][segment_index]:.5g}\n")
			ca_summary_file.write(f"Total potential difference (V): {ca_data['OCP_timeout_total_pot_diff'][segment_index]:.5g}\n")
			ca_summary_file.write(f"Final ({global_software_settings['OCP_eq_timescale']}) s potential difference (V): {ca_data['OCP_timeout_timescale_pot_diff'][segment_index]:.5g}\n")

			ca_summary_file.write("\n")
			ca_summary_file.write("**********************\n")
			ca_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			ca_summary_file.close()

		elif section == "interrupted":
			ca_summary_file.write("\n*******************************************\n\tCA MEASUREMENTS INTERRUPTED\n*******************************************\n")
			ca_summary_file.write(f"Segment interrupted: {segment_index + 1}/{ca_parameters['num_segments']}\n")
			ca_summary_file.write(f"Experiment interruption time: {ca_data['finishtime_readable'][segment_index]}\n")
			ca_summary_file.write(f"CA segment interrupted: {ca_parameters['potential_sequence'][segment_index]} V\n")

			ca_summary_file.write("\n")
			ca_summary_file.write("**********************\n")
			ca_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			ca_summary_file.close()

		elif section == "lower_curr_limit_reached":
			ca_summary_file.write("\n***** LOWER CURRENT LIMIT REACHED *****\n")
			ca_summary_file.write(f"Lower current limit (mA): {ca_parameters['curr_limit_lower']:.3f}\n")
			ca_summary_file.write(f"Last averaged current (mA): {ca_current_data.averagebuffer[-1] * 1e3:.3f}\n")
			ca_summary_file.write("***** INTERRUPTING MEASUREMENTS *****\n")

		elif section == "upper_curr_limit_reached":
			ca_summary_file.write("\n***** UPPER CURRENT LIMIT REACHED *****\n")
			ca_summary_file.write(f"Upper current limit (mA): {ca_parameters['curr_limit_upper']:.3f}\n")
			ca_summary_file.write(f"Last averaged current (mA): {ca_current_data.averagebuffer[-1] * 1e3:.3f}\n")
			ca_summary_file.write("***** INTERRUPTING MEASUREMENTS *****\n")

		elif section == "error":
			ca_summary_file.write("\n**********************************************\n\tERROR INITIALISING EXPERIMENTS\n**********************************************\n")

			ca_summary_file.write("\n")
			ca_summary_file.write("**********************\n")
			ca_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			ca_summary_file.close()

	except Exception as e:
		log_message(f"Write to summary file failed: {e}")


"""CA PLOT AND GUI FUNCTIONS"""

def ca_update_plot(segment_index):
	"""Update the plot with the current CA segment data, and other segments depending on GUI inputs."""

	def add_segment_data(start_idx, stop_idx, ref_time):
		"""Concatenate data from segments in range [start_idx, stop_idx] and normalise time by ref_time."""
		nonlocal time_data, current_data, potential_data, segment_centers, segment_labels

		for i in range(start_idx, stop_idx + 1):
			seg_time = numpy.array(ca_data['time'][i]) - ref_time
			seg_current = numpy.array(ca_data['current'][i])
			seg_potential = numpy.array(ca_data['potential'][i])

			segment_centers.append((seg_time[0] + seg_time[-1]) / 2)
			segment_labels.append(i + 1)

			time_data = numpy.concatenate([time_data, seg_time])
			current_data = numpy.concatenate([current_data, seg_current])
			potential_data = numpy.concatenate([potential_data, seg_potential])

			# Add vertical lines between segments
			if i > 0:
				line_pos = seg_time[0]
				line = pyqtgraph.InfiniteLine(
					pos=line_pos,
					angle=90,
					pen=pyqtgraph.mkPen('w', style=QtCore.Qt.DashLine)
				)
				plot_frame.addItem(line)

	# Clear plot frame and legend
	plot_frame.clear()
	legend.clear()

	# Title for legend
	dummy_item = pyqtgraph.PlotDataItem([], [], pen=None)
	legend.addItem(dummy_item, "Chronoamperometry experiment:")

	# Initialise plotting arrays and lists
	time_data = numpy.array([])
	current_data = numpy.array([])
	potential_data = numpy.array([])
	segment_centers = []
	segment_labels = []

	if ca_plot_options_all_segments_radiobutton.isChecked():
		ref_time = ca_data['time'][0][0]
		add_segment_data(0, segment_index, ref_time)

	elif ca_plot_options_current_segment_only_radiobutton.isChecked():
		ref_time = ca_data['time'][segment_index][0]
		add_segment_data(segment_index, segment_index, ref_time)

	elif ca_plot_options_segment_select_radiobutton.isChecked():
		seg1_idx = int(ca_plot_options_segment1_dropdown.currentText()) - 1
		seg2_idx = int(ca_plot_options_segment2_dropdown.currentText()) - 1
		ref_time = ca_data['time'][seg1_idx][0]
		add_segment_data(seg1_idx, seg2_idx, ref_time)

	# Scale potential data relative to current data
	if current_data.size > 0 and potential_data.size > 0:
		current_max = numpy.max(numpy.abs(current_data))
		potential_max = numpy.max(numpy.abs(potential_data))
		if potential_max != 0:
			potential_data *= (current_max / potential_max)

		# Add segment labels at max height
		for segment_label, segment_center in zip(segment_labels, segment_centers):
			label = pyqtgraph.TextItem(f'{segment_label}', color='w', anchor=(0.5, 1))
			label.setPos(segment_center, current_max)
			plot_frame.addItem(label)

	ca_current_plot_curve.setData(time_data, current_data)
	ca_potential_plot_curve.setData(time_data, potential_data)

	plot_frame.addItem(ca_current_plot_curve)
	plot_frame.addItem(ca_potential_plot_curve)

	legend.addItem(ca_current_plot_curve, "Current")
	legend.addItem(ca_potential_plot_curve, "Potential (scaled)")

	# Add equilibration info to legend
	if ca_parameters['curr_eq_tolerance'][segment_index] is not None:
		legend.addItem(dummy_item, "")
		legend.addItem(dummy_item, "Equilibration information:")
		legend.addItem(dummy_item, f"Tolerance (µA): {ca_parameters['curr_eq_tolerance'][segment_index]}")
		legend.addItem(dummy_item, f"Timescale (s): {ca_parameters['curr_eq_timescale'][segment_index]}")
		if ca_curr_eq_history:
			if ca_curr_eq_history[-1][0] >= ca_parameters['curr_eq_timescale'][segment_index]:
				legend.addItem(dummy_item, f"Current {ca_parameters['curr_eq_timescale'][segment_index]} s ago (µA): {ca_curr_eq_history[0][1] * 1e6:.3g}")
			else:
				legend.addItem(dummy_item, f"Current {ca_curr_eq_history[-1][0] - ca_curr_eq_history[0][0]:.3f} s ago (µA): {ca_curr_eq_history[0][1] * 1e6:.3g}")
			legend.addItem(dummy_item, f"Measured current (µA): {ca_curr_eq_history[-1][1] * 1e6:.3g}")
			curr_diff = (ca_curr_eq_history[-1][1] - ca_curr_eq_history[0][1])
			legend.addItem(dummy_item, f"Difference (µA): {curr_diff * 1e6:.3g}")

def ca_update_progress_bar():
	"""Update the progress bar to reflect percentage of segments completed."""

	ca_progress_bar.update_progress_bar(ca_total_segments, ca_current_segment_index)
	ca_progress_bar.update()



"""_____CHRONOPOTENTIOMETRY FUNCTIONS_____"""

"""CP PARAMETER FUNCTIONS"""

def cp_checkbutton_callback():
	"""Function to control the data-validation process, called when "CHECK" button pressed."""
	global cp_parameters_checked, cp_filenames_checked, cp_current_segment_index, cp_total_segments

	# Initialise with parameters_checked = False, filenames_checked = False, and a check button style reset
	cp_parameters_checked = False
	cp_filenames_checked = False
	cp_variables_checkbutton.setStyleSheet("")

	# Remove any previous program state entry
	cp_info_program_state_entry.setText("No experiments running")

	# Check input parameter formats
	if cp_validate_inputs():
		pass
	else:
		cp_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
		log_message("CP check button failed: Inputs are not in the correct format.")
		return False

	# Write input parameters to global dictionary
	if cp_get_parameters():
		pass
	else:
		cp_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
		log_message("CP check button failed: Could not write experiment parameters to a global dictionary.")
		return False

	# If filename provided, check if filenames are acceptable
	if cp_file_entry.text().strip() != "":
		if cp_get_filenames():
			cp_filenames_checked = True
		else:
			cp_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
			log_message("CP check button failed: Experiment filenames could not be constructed.")
			return False

	# Give any parameter warnings
	cp_parameter_warnings()

	# Set global parameters_checked state as True
	cp_parameters_checked = True

	# Make check button green
	cp_variables_checkbutton.setStyleSheet("background-color: green; color: white;")

	# Store the number of halfcycles this experiment will perform
	cp_total_segments = cp_parameters['num_segments']

	# Initialise current_segment_index for progress bar
	cp_current_segment_index = -1

	# Update progress bar and give green border
	cp_update_progress_bar()
	cp_progress_bar.set_solid_green_style()

	log_message(f"Check button successful! Experiments are ready to run. Segments remaining: {cp_total_segments}")
	return True

def cp_validate_inputs():
	"""Ensure inputs are of the correct format."""

	dropdown_bool = cp_alternating_parameters_dropdown.dropdown_frame.isVisible()

	# Current sequence
	try:
		currents_str = cp_params_A_current_entry.text().strip()
		currents_list = [float(current.strip()) for current in currents_str.split(",")]

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Current sequence input" if not dropdown_bool else "Error: Current sequence A input",
			"Currents must be numeric values." if not dropdown_bool else
			"Sequence A currents must be numeric values."
		)
		return False

	if dropdown_bool:
		try:
			currents_B_str = cp_alternating_parameters_dropdown.cp_params_B_current_entry.text().strip()
			currents_list_B = [float(current_B.strip()) for current_B in currents_B_str.split(",")]

			# Check alternating lists are the correct length
			if len(currents_list) != len(currents_list_B) and len(currents_list) != len(currents_list_B) + 1:
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Alternating parameters current sequence input",
					"If using alternating parameters, sequence A must be the same length as, or 1 input longer than, sequence B."
				)
				return False

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Current sequence B input",
				"Sequence B currents must be numeric values."
			)
			return False

	# Hold times
	try:
		hold_times_str = cp_params_A_hold_time_entry.text().strip()
		hold_times_list = [float(hold_time.strip()) for hold_time in hold_times_str.split(",")]
		if any(hold_time < 0 for hold_time in hold_times_list):
			raise ValueError

		# Hold times same length as current sequence
		if len(hold_times_list) != len(currents_list):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Hold times input" if not dropdown_bool else "Error: Hold times A input",
				"Hold times must be of the same length as current sequence." if not dropdown_bool else
				"Sequence A hold times must be of the same length as current sequence A."
			)
			return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Hold times input" if not dropdown_bool else "Error: Hold times A input",
			"Hold times must be non-negative numeric values." if not dropdown_bool else
			"Sequence A hold times must be non-negative numeric values."
		)
		return False

	if dropdown_bool:
		try:
			hold_times_B_str = cp_alternating_parameters_dropdown.cp_params_B_hold_time_entry.text().strip()
			hold_times_list_B = [float(hold_time_B.strip()) for hold_time_B in hold_times_B_str.split(",")]
			if any(hold_time_B < 0 for hold_time_B in hold_times_list_B):
				raise ValueError

			# Hold times same length as current sequence
			if len(hold_times_list_B) != len(currents_list_B):
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Hold times B input",
					"Sequence B hold times must be of the same length as current sequence B."
				)
				return False

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Hold times B input",
				"Sequence B hold times must be non-negative numeric values."
			)
			return False

	# Ramp rates
	try:
		ramp_rates_str = cp_params_A_ramp_rate_entry.text().strip()
		if ramp_rates_str == "":
			ramp_rates_list = []
		else:
			ramp_rates_list = [ramp_rate.strip() for ramp_rate in ramp_rates_str.split(",")]
			for i, ramp_rate in enumerate(ramp_rates_list):
				if ramp_rate.lower() == "step":  # Remove case-sensitivity for "STEP"
					ramp_rates_list[i] = "STEP"
				else:
					ramp_rates_list[i] = float(ramp_rate)

			# Ramp rates list same length as current sequence
			if len(ramp_rates_list) != len(currents_list):
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Ramp rates input" if not dropdown_bool else "Error: Ramp rates A input",
					"If ramp rates are given, there must be one per sequence current." if not dropdown_bool else
					"If sequence A ramp rates are given, there must be one per sequence A current."
				)
				return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Ramp rates input" if not dropdown_bool else "Error: Ramp rates A input",
			"Ramp rates must be positive numeric values or 'STEP'." if not dropdown_bool else
			"Sequence A ramp rates must be positive numeric values or 'STEP'."
		)
		return False

	if dropdown_bool:
		try:
			ramp_rates_B_str = cp_alternating_parameters_dropdown.cp_params_B_ramp_rate_entry.text().strip()
			if ramp_rates_B_str == "":
				ramp_rates_list_B = []
			else:
				ramp_rates_list_B = [ramp_rate_B.strip() for ramp_rate_B in ramp_rates_B_str.split(",")]
				for i, ramp_rate_B in enumerate(ramp_rates_list_B):
					if ramp_rate_B.lower() == "step":  # Remove case-sensitivity for "STEP"
						ramp_rates_list_B[i] = "STEP"
					else:
						ramp_rates_list_B[i] = float(ramp_rate_B)

				# Ramp rates list B same length as current sequence B
				if len(ramp_rates_list_B) != len(currents_list_B):
					QtWidgets.QMessageBox.critical(
						mainwidget,
						"Error: Ramp rates B input",
						"If sequences B ramp rates are given, there must be one per sequence B current."
					)
					return False

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Ramp rates B input",
				"Sequence B ramp rates must be positive numeric values or 'STEP'."
			)
			return False

	# Ramp rates are positive
	if any(ramp_rate != "STEP" and ramp_rate <= 0 for ramp_rate in ramp_rates_list):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Ramp rates input" if not dropdown_bool else "Error: Ramp rates A input",
			"Ramp rates must be positive." if not dropdown_bool else
			"Sequence A ramp rates must be positive."
		)
		return False
	if dropdown_bool:
		if any(ramp_rate_B != "STEP" and ramp_rate_B <= 0 for ramp_rate_B in ramp_rates_list_B):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Ramp rates B input",
				"Sequence B ramp rates must be positive."
			)
			return False

	# Ramp rates present if expected
	if cp_params_A_ramp_rate_checkbox.isChecked() and ramp_rates_list == []:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Ramp rates input" if not dropdown_bool else "Error: Ramp rates A input",
			"Ramp rates expected but not given." if not dropdown_bool else
			"Sequence A ramp rates expected but not given."
		)
		return False
	if dropdown_bool:
		if cp_alternating_parameters_dropdown.cp_params_B_ramp_rate_checkbox.isChecked() and ramp_rates_list_B == []:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Ramp rates B input",
				"Sequence B ramp rates expected but not given."
			)
			return False


	# Equilibration tolerance
	try:
		pot_eq_tolerance_str = cp_params_A_equilibration_tolerance_entry.text().strip()
		if pot_eq_tolerance_str == "":
			pot_eq_tolerance = None
		else:
			pot_eq_tolerance = float(pot_eq_tolerance_str)
			if pot_eq_tolerance < 0:
				raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Equilibration tolerance input" if not dropdown_bool else "Error: Equilibration tolerance A input",
			"Equilibration tolerance must be a non-negative numeric value." if not dropdown_bool else
			"Sequence A equilibration tolerance must be a non-negative numeric value."
		)
		return False

	if dropdown_bool:
		try:
			pot_eq_tolerance_B_str = cp_alternating_parameters_dropdown.cp_params_B_equilibration_tolerance_entry.text().strip()

			if pot_eq_tolerance_B_str == "":
				pot_eq_tolerance_B = None
			else:
				pot_eq_tolerance_B = float(pot_eq_tolerance_B_str)
				if pot_eq_tolerance_B < 0:
					raise ValueError

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Equilibration tolerance B input",
				"Sequence B equilibration tolerance must be a non-negative numeric value."
			)
			return False

	# Equilibration timescale
	try:
		pot_eq_timescale_str = cp_params_A_equilibration_timescale_entry.text().strip()
		if pot_eq_timescale_str == "":
			pot_eq_timescale = None
		else:
			pot_eq_timescale = float(pot_eq_timescale_str)
			if pot_eq_timescale <= 0:
				raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Equilibration timescale input" if not dropdown_bool else "Error: Equilibration timescale A input",
			"Equilibration timescale must be a positive numeric value." if not dropdown_bool else
			"Sequence A equilibration timescale must be a positive numeric value."
		)
		return False

	if dropdown_bool:
		try:
			pot_eq_timescale_B_str = cp_alternating_parameters_dropdown.cp_params_B_equilibration_timescale_entry.text().strip()
			if pot_eq_timescale_B_str == "":
				pot_eq_timescale_B = None
			else:
				pot_eq_timescale_B = float(pot_eq_timescale_B_str)
				if pot_eq_timescale_B <= 0:
					raise ValueError

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Equilibration timescale B input",
				"Sequence B equilibration timescale must be a positive numeric value."
			)
			return False

	# Potential equilibration tolerance and timescale present if expected
	if cp_params_A_equilibration_checkbox.isChecked():
		if pot_eq_tolerance is None:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Equilibration tolerance input" if not dropdown_bool else "Error: Equilibration tolerance A input",
				"Equilibration tolerance expected but not given." if not dropdown_bool else
				"Sequence A equilibration tolerance expected but not given."
			)
			return False
		if pot_eq_timescale is None:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Equilibration timescale input" if not dropdown_bool else "Error: Equilibration timescale A input",
				"Equilibration timescale expected but not given." if not dropdown_bool else
				"Sequence A equilibration timescale expected but not given."
			)
			return False

	if dropdown_bool:
		if cp_alternating_parameters_dropdown.cp_params_B_equilibration_checkbox.isChecked():
			if pot_eq_tolerance_B is None:
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Equilibration tolerance B input",
					"Sequence B equilibration tolerance expected but not given."
				)
				return False
			if pot_eq_timescale_B is None:
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Equilibration timescale B input",
					"Sequence B equilibration timescale expected but not given."
				)
				return False

	# Potential limits
	try:
		# Lower limit
		pot_limit_lower_str = cp_params_pot_limits_lower_entry.text().strip()
		if pot_limit_lower_str == "":
			pot_limit_lower = None
		elif pot_limit_lower_str.lower() == "none":
			pot_limit_lower = None
		elif pot_limit_lower_str.lower() == "ocp":
			pot_limit_lower = "OCP"
		else:
			pot_limit_lower = float(pot_limit_lower_str)

		# Upper limit
		pot_limit_upper_str = cp_params_pot_limits_upper_entry.text().strip()
		if pot_limit_upper_str == "":
			pot_limit_upper = None
		elif pot_limit_upper_str.lower() == "none":
			pot_limit_upper = None
		elif pot_limit_upper_str.lower() == "ocp":
			pot_limit_upper = "OCP"
		else:
			pot_limit_upper = float(pot_limit_upper_str)

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Potential limits input",
			"Potential limits must each be a numeric value, 'OCP', or 'None'."
		)
		return False

	# Upper potential limit greater than lower limit
	try:
		if isinstance(pot_limit_lower, (int, float)) and isinstance(pot_limit_upper, (int, float)):
			if pot_limit_lower >= pot_limit_upper:
				raise ValueError

		if pot_limit_lower == "OCP" and pot_limit_upper == "OCP":
			raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Potential limits input",
			"Upper potential limit must be greater than the lower potential limit."
		)
		return False

	# Set the potential limits to 'None' in GUI
	if cp_params_pot_limits_checkbox.isChecked():
		if pot_limit_lower is None:
			cp_params_pot_limits_lower_entry.setText("None")
		if pot_limit_upper is None:
			cp_params_pot_limits_upper_entry.setText("None")

	# Number of samples to average
	try:
		num_samples = int(cp_params_num_samples_entry.text().strip())
		if num_samples <= 0:
			raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Samples to average input",
			"Samples to average input must be a positive integer value."
		)
		return False

	# Determine whether OCP equilibration required
	if cp_params_delay_OCP_checkbox.isChecked():
		OCP_eq = True
	else:
		OCP_eq = False
		if pot_limit_lower == "OCP" or pot_limit_upper == "OCP":
			OCP_eq = True

	# Delay if required
	if not OCP_eq:
		try:
			delay = float(cp_params_delay_entry.text().strip())
			if delay < 0:
				raise ValueError

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Pre-experiment delay input",
				"Pre-experiment delay must be a non-negative numeric value and given if not waiting for OCP equilibration."
			)
			return False

	else:  # Remove the delay
		cp_params_delay_OCP_checkbox.setChecked(True)
		cp_params_delay_entry.setText("")

	return True

def cp_get_parameters():
	""""Write experiment parameters to a global dictionary."""
	global cp_parameters

	cp_parameters = {'type': 'cp'}

	try:
		# Track if OCP to be equilibrated before the experiment
		OCP_bool = False

		# Current sequence
		currents = cp_params_A_current_entry.text().strip()
		currents_list = [segment_current.strip() for segment_current in currents.split(",")]
		for i, segment_current in enumerate(currents_list):
			currents_list[i] = float(segment_current)

		cp_parameters['current_sequence_A'] = currents_list

		# Current hold times
		hold_times = cp_params_A_hold_time_entry.text().strip()
		hold_times_list = [float(hold_time.strip()) for hold_time in hold_times.split(",")]

		# Current ramp rates
		if cp_params_A_ramp_rate_checkbox.isChecked():
			ramp_rates = cp_params_A_ramp_rate_entry.text().strip()
			ramp_rates_list = [ramp_rate.strip() for ramp_rate in ramp_rates.split(",")]
			for i, ramp_rate in enumerate(ramp_rates_list):
				if ramp_rate.lower() == "step":  # Remove case-sensitivity for "STEP"
					ramp_rates_list[i] = "STEP"
				else:
					ramp_rates_list[i] = float(ramp_rate)
		else:
			ramp_rates_list = ["STEP"] * len(currents_list)

		# Current equilibration
		if cp_params_A_equilibration_checkbox.isChecked():

			# Tolerance
			pot_eq_tolerance = cp_params_A_equilibration_tolerance_entry.text().strip()
			pot_eq_tolerances_list = [float(pot_eq_tolerance)] * len(currents_list)

			# Timescale
			pot_eq_timescale = cp_params_A_equilibration_timescale_entry.text().strip()
			pot_eq_timescales_list = [float(pot_eq_timescale)] * len(currents_list)

		else:
			pot_eq_tolerances_list = [None] * len(currents_list)
			pot_eq_timescales_list = [None] * len(currents_list)

		# If alternating parameters are in use:
		if cp_alternating_parameters_dropdown.dropdown_frame.isHidden():
			cp_parameters['alternating_params_bool'] = False
		else:
			cp_parameters['alternating_params_bool'] = True

			# Current sequence B
			currents_B = cp_alternating_parameters_dropdown.cp_params_B_current_entry.text().strip()
			currents_list_B = [segment_current.strip() for segment_current in currents_B.split(",")]
			for i, segment_current in enumerate(currents_list_B):
				currents_list_B[i] = float(segment_current)

			cp_parameters['current_sequence_B'] = currents_list_B

			# Current hold times B
			hold_times_B = cp_alternating_parameters_dropdown.cp_params_B_hold_time_entry.text().strip()
			hold_times_list_B = [float(hold_time.strip()) for hold_time in hold_times_B.split(",")]

			# Current ramp rates B
			if cp_alternating_parameters_dropdown.cp_params_B_ramp_rate_checkbox.isChecked():
				ramp_rates_B = cp_alternating_parameters_dropdown.cp_params_B_ramp_rate_entry.text().strip()
				ramp_rates_list_B = [ramp_rate.strip() for ramp_rate in ramp_rates_B.split(",")]
				for i, ramp_rate in enumerate(ramp_rates_list_B):
					if ramp_rate.lower() == "step":  # Remove case-sensitivity for "STEP"
						ramp_rates_list_B[i] = "STEP"
					else:
						ramp_rates_list_B[i] = float(ramp_rate)
			else:
				ramp_rates_list_B = ["STEP"] * len(currents_list_B)

			# Potential equilibration B
			if cp_alternating_parameters_dropdown.cp_params_B_equilibration_checkbox.isChecked():

				# Tolerance
				pot_eq_tolerance_B = cp_alternating_parameters_dropdown.cp_params_B_equilibration_tolerance_entry.text().strip()
				pot_eq_tolerances_list_B = [float(pot_eq_tolerance_B)] * len(currents_list_B)

				# Timescale
				pot_eq_timescale_B = cp_alternating_parameters_dropdown.cp_params_B_equilibration_timescale_entry.text().strip()
				pot_eq_timescales_list_B = [float(pot_eq_timescale_B)] * len(currents_list_B)
			else:
				pot_eq_tolerances_list_B = [None] * len(currents_list_B)
				pot_eq_timescales_list_B = [None] * len(currents_list_B)

			# Zip parameters A and B together in alternating fashion
			currents_list = [item for pair in zip_longest(currents_list, currents_list_B) for item in pair]
			ramp_rates_list = [item for pair in zip_longest(ramp_rates_list, ramp_rates_list_B) for item in pair]
			hold_times_list = [item for pair in zip_longest(hold_times_list, hold_times_list_B) for item in pair]
			pot_eq_tolerances_list = [item for pair in zip_longest(pot_eq_tolerances_list, pot_eq_tolerances_list_B)
									   for item in pair]
			pot_eq_timescales_list = [item for pair in zip_longest(pot_eq_timescales_list, pot_eq_timescales_list_B)
									   for item in pair]

		# Store parameters for each segment in dictionary
		cp_parameters['current_sequence'] = currents_list
		cp_parameters['hold_time'] = hold_times_list
		cp_parameters['ramp_rate'] = ramp_rates_list
		cp_parameters['pot_eq_tolerance'] = pot_eq_tolerances_list
		cp_parameters['pot_eq_timescale'] = pot_eq_timescales_list

		# Potential limits
		if cp_params_pot_limits_checkbox.isChecked():
			# Lower limit
			pot_limit_lower = cp_params_pot_limits_lower_entry.text().strip()
			if pot_limit_lower == "":
				pot_limit_lower = None
			elif pot_limit_lower.lower() == "none":    # Remove case-sensitivity for "None"
				pot_limit_lower = None
			elif pot_limit_lower.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				pot_limit_lower = "OCP"
				OCP_bool = True
			else:
				pot_limit_lower = float(pot_limit_lower)

			# Upper limit
			pot_limit_upper = cp_params_pot_limits_upper_entry.text().strip()
			if pot_limit_lower == "":
				pot_limit_lower = None
			elif pot_limit_upper.lower() == "none":    # Remove case-sensitivity for "None"
				pot_limit_upper = None
			elif pot_limit_upper.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				pot_limit_upper = "OCP"
				OCP_bool = True
			else:
				pot_limit_upper = float(pot_limit_upper)

		else:
			pot_limit_lower = None
			pot_limit_upper = None

		cp_parameters['pot_limit_lower'] = pot_limit_lower
		cp_parameters['pot_limit_upper'] = pot_limit_upper

		# Number of samples to average
		cp_parameters['num_samples'] = int(cp_params_num_samples_entry.text().strip())

		# Number of segments
		cp_parameters['num_segments'] = len(cp_parameters['current_sequence'])

		# Pre-experiment delay
		if cp_params_delay_OCP_checkbox.isChecked():
			OCP_bool = True
		else:
			cp_parameters['delay'] = float(cp_params_delay_entry.text().strip())

		# OCP equilibration pre-experiment
		cp_parameters['OCP_bool'] = OCP_bool

		# Experiment notes
		experiment_notes = cp_file_notes_entry.toPlainText()
		if not experiment_notes:
			experiment_notes = "No notes provided."
		cp_parameters['experiment_notes'] = experiment_notes

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Input error",
			"One or more parameters are in the wrong format and cannot be written to a global dictionary."
		)
		return False

	return True

def cp_parameter_warnings():
	"""Give GUI warnings for unused parameters or OCP values which may become invalid."""

	# Unused ramp rates warning
	if cp_alternating_parameters_dropdown.dropdown_frame.isHidden():
		if cp_params_A_ramp_rate_entry.text().strip() != "" and not cp_params_A_ramp_rate_checkbox.isChecked():
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"Warning: Ramp rates input",
				"Ramp rates provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values, otherwise, the current will be stepped."
			)
		if (cp_params_A_equilibration_tolerance_entry.text().strip() != "" or cp_params_A_equilibration_timescale_entry.text().strip() != "") and not cp_params_A_equilibration_checkbox.isChecked():
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"Warning: Equilibration input",
				"Equilibration conditions provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values."
			)

	elif not cp_alternating_parameters_dropdown.dropdown_frame.isHidden():
		if cp_params_A_ramp_rate_entry.text().strip() != "" and not cp_params_A_ramp_rate_checkbox.isChecked():
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"Warning: Sequence A ramp rates input",
				"Sequence A ramp rates provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values, otherwise, the current will be stepped."
			)
		if (cp_params_A_equilibration_tolerance_entry.text().strip() != "" or cp_params_A_equilibration_timescale_entry.text().strip() != "") and not cp_params_A_equilibration_checkbox.isChecked():
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"Warning: Sequence A equilibration input",
				"Sequence A equilibration conditions provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values."
			)
		if cp_alternating_parameters_dropdown.cp_params_B_ramp_rate_entry.text().strip() != "" and not cp_alternating_parameters_dropdown.cp_params_B_ramp_rate_checkbox.isChecked():
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"Warning: Sequence B ramp rates input",
				"Sequence B ramp rates provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values, otherwise, the current will be stepped."
			)
		if (cp_alternating_parameters_dropdown.cp_params_B_equilibration_tolerance_entry.text().strip() != "" or cp_alternating_parameters_dropdown.cp_params_B_equilibration_timescale_entry.text().strip() != "") and not cp_alternating_parameters_dropdown.cp_params_B_equilibration_checkbox.isChecked():
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"Warning: Sequence B equilibration input",
				"Sequence B equilibration conditions provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values."
			)

	# Unused potential limits warning
	if not cp_params_pot_limits_checkbox.isChecked():
		if cp_params_pot_limits_lower_entry.text().strip().lower() != "" or cp_params_pot_limits_upper_entry.text().strip().lower() != "":
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"Warning: Potential limits input",
				"Potential limits provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values."
			)

	# OCP as a potential limit
	if (cp_parameters['pot_limit_lower'] == "OCP" and cp_parameters['pot_limit_upper'] is not None) or (cp_parameters['pot_limit_upper'] == "OCP" and cp_parameters['pot_limit_lower'] is not None):
		QtWidgets.QMessageBox.warning(
			mainwidget,
			"Warning: Possible OCP complications",
			"Both potential limits are set, with one of them set as 'OCP'.\n\n"
			"The open-circuit potential (OCP) can vary, which may cause your experiments\n"
			"to stop depending on your lower and upper potential limits.\n\n"
			"Please ensure that this is intentional!"
		)

	return True

def cp_get_filenames():
	"""Construct filenames for the experiments."""

	global cp_parameters

	try:
		filename_entry = str(cp_file_entry.text().strip())
		if filename_entry == "":
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: No filename provided",
				"Please provide a base filename for the experiments."
			)
			return False

		directory_path = os.path.dirname(filename_entry)
		if directory_path == "":
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: No parent directory specified",
				f"The output files will be stored in the current working directory. Is this okay?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False
			directory_path = os.getcwd()
		elif not os.path.isdir(directory_path):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Directory does not exist",
				f"The directory {directory_path} does not exist."
			)
			return False

		if "_CP_{experiment_info_here}" in filename_entry:
			filename_entry = filename_entry.split("_CP_{experiment_info_here}")[0]
		filename = os.path.basename(filename_entry)
		if filename == "":  # If no filename given, only path
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: No filename provided",
				"Please provide a base filename for the experiments in addition to the path."
			)
			return False

		cp_parameters['directory_path'] = directory_path
		cp_parameters['base_filename'], _ = os.path.splitext(filename)

		exp_filename = f"{cp_parameters['base_filename']}_CP"
		for current in cp_parameters['current_sequence']:
			exp_filename += f"_{current}"
		exp_filename += "_uA"

		cp_parameters['filename'] = exp_filename + ".dat"
		cp_parameters['path_filename'] = os.path.join(directory_path, cp_parameters['filename'])

		cp_parameters['experiment_info_filename'] = cp_parameters['base_filename'] + "_CP_experiment_info.txt"
		cp_parameters['experiment_info_path_filename'] = os.path.join(directory_path, cp_parameters['experiment_info_filename'])

		file = cp_parameters['path_filename']
		if os.path.isfile(file):
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: File already exists",
				f"The output file {file} already exists. Do you want to overwrite it?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False

		info_file = cp_parameters['experiment_info_path_filename']
		if os.path.isfile(info_file):
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: Results file already exists",
				f"The experiment info output file {info_file} already exists. Do you want to overwrite it?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False

	except Exception as e:
		print(e)
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: File error",
			f"One or more output filepaths are not valid."
		)
		return False

	cp_file_entry.setText(os.path.join(cp_parameters['directory_path'], f"{cp_parameters['base_filename']}_CP_{{experiment_info_here}}"))

	return True

def cp_validate_filenames():
	"""Check validity of files by creating and attempting to open them."""

	file = cp_parameters.get('path_filename')
	if file:
		try:
			with open(file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {file} is not valid."
			)
			return False

	info_file = cp_parameters.get('experiment_info_path_filename')
	if info_file:
		try:
			with open(info_file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {info_file} is not valid."
			)
			return False

	return True

def cp_freeze_inputs(freeze):
	"""Function to freeze and unfreeze GUI inputs when experiments are running."""

	if freeze:
		cp_params_A_current_entry.setEnabled(False)
		cp_params_A_hold_time_entry.setEnabled(False)
		cp_params_A_ramp_rate_checkbox.setEnabled(False)
		cp_params_A_ramp_rate_entry.setEnabled(False)
		cp_params_A_equilibration_checkbox.setEnabled(False)
		cp_params_A_equilibration_tolerance_entry.setEnabled(False)
		cp_params_A_equilibration_timescale_entry.setEnabled(False)
		cp_params_pot_limits_checkbox.setEnabled(False)
		cp_params_pot_limits_lower_entry.setEnabled(False)
		cp_params_pot_limits_upper_entry.setEnabled(False)
		cp_params_num_samples_entry.setEnabled(False)
		cp_params_delay_entry.setEnabled(False)
		cp_params_delay_OCP_checkbox.setEnabled(False)
		cp_file_entry.setEnabled(False)
		cp_file_notes_entry.setEnabled(False)
		cp_variables_checkbutton.setEnabled(False)
		cp_alternating_parameters_dropdown.freezeInputs(True)
		software_globals_menu_button.setEnabled(False)

	elif not freeze:
		cp_params_A_current_entry.setEnabled(True)
		cp_params_A_hold_time_entry.setEnabled(True)
		cp_params_A_ramp_rate_checkbox.setEnabled(True)
		cp_params_A_ramp_rate_entry.setEnabled(True)
		cp_params_A_equilibration_checkbox.setEnabled(True)
		cp_params_A_equilibration_tolerance_entry.setEnabled(True)
		cp_params_A_equilibration_timescale_entry.setEnabled(True)
		cp_params_pot_limits_checkbox.setEnabled(True)
		cp_params_pot_limits_lower_entry.setEnabled(True)
		cp_params_pot_limits_upper_entry.setEnabled(True)
		cp_params_num_samples_entry.setEnabled(True)
		cp_params_delay_entry.setEnabled(True)
		cp_params_delay_OCP_checkbox.setEnabled(True)
		cp_file_entry.setEnabled(True)
		cp_file_notes_entry.setEnabled(True)
		cp_variables_checkbutton.setEnabled(True)
		cp_alternating_parameters_dropdown.freezeInputs(False)
		software_globals_menu_button.setEnabled(True)


"""CP CORE FUNCTIONS"""

def cp_initialise():
	"""Initialise chronopotentiometry experiment."""
	global state
	global cp_data, cp_current_segment_index
	global cp_time_data, cp_segment_time_data, cp_potential_data, cp_current_data

	# Ensure parameters have been verified using the "CHECK" button
	if not cp_parameters_checked:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Parameters not checked",
			"Click the 'CHECK' button to ensure parameters are appropriate before starting your experiments."
		)
		return False

	# Ensure filenames have been checked
	if not cp_filenames_checked:
		if cp_get_filenames():
			pass
		else:
			return False

	if check_state([States.Idle, States.Stationary_Graph]):

		# Validate filenames before experiment begins
		if cp_validate_filenames():
			pass
		else:
			cp_reset_experiment_controller(mode="interrupted")
			log_message("CP experiment could not initialise due to invalid output filename.")
			return False

		# Turn cell off if under manual control
		set_cell_status(False)

		# Freeze input fields and hide return to live graph button
		cp_freeze_inputs(freeze=True)
		preview_cancel_button.hide()

		# Initialise segment index
		cp_current_segment_index = 0

		# Write experiment info to summary file
		cp_write_summary_file(cp_current_segment_index, section="initial")

		# Update GUI
		cp_info_segmentnum_entry.setText(f"-/{cp_parameters['num_segments']}")
		log_message("Starting chronopotentiometry experiment...")

		# Initialise cp_data dictionary
		cp_data = {
			'starttime': defaultdict(float),
			'starttime_readable': defaultdict(str),
			'finishtime_readable': defaultdict(str),

			'startcurr': defaultdict(float),
			'time': defaultdict(list),
			'ramping/holding': defaultdict(list),
			'segment_time': defaultdict(list),
			'potential': defaultdict(list),
			'current': defaultdict(list),
		}

		# Pass to OCP equilibration controller if required
		if cp_parameters['OCP_bool']:
			OCP_initialise_data_entries(cp_data)
			OCP_equilibration_controller(cp_parameters, cp_data, cp_current_segment_index, equilibrated=False)
		else:
			# Write pre-experiment delay to summary file and update GUI
			cp_write_summary_file(cp_current_segment_index, section="delay")
			cp_info_program_state_entry.setText(f"Delay of {cp_parameters['delay']} s")

			# Update progress bar style to solid yellow border
			cp_progress_bar.set_solid_yellow_style()

			# Update state
			state = States.Measuring_CP_Delay

			# Launch the delay timer
			cp_delay_timer.start(int(cp_parameters['delay'] * 1000))  # Delay input in ms

def cp_start(segment_index):
	"""Begin or progress the chronopotentiometry experiment."""
	global state
	global cp_data, cp_time_data, cp_segment_time_data, cp_potential_data, cp_current_data
	global cp_currentsetpoint_hold, cp_pot_eq_history, cp_output_file
	global cp_potential_plot_curve, cp_current_plot_curve, legend, legend_in_use

	if segment_index is None:
		state = States.Stationary_Graph
		preview_cancel_button.show()
		cp_write_summary_file(ca_current_segment_index, section="error")
		cp_reset_experiment_controller(mode="interrupted")
		log_message("Experiments could not start due to segment_index initialisation error.")
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Initialisation",
			"The experiments could not initialise segment_index correctly."
		)
		return

	# Write segment information to summary file
	cp_data['starttime_readable'][segment_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]
	cp_write_summary_file(segment_index, section="segment_start")

	# Store start current
	cp_data['startcurr'][segment_index] = current

	# Initialise state describing whether the current is holding or ramping
	cp_currentsetpoint_hold = False

	# If initial segment
	if segment_index == 0:

		# Open output file and write header
		try:
			cp_output_file = open(cp_parameters['path_filename'], 'w', 1)
			cp_output_file.write("Segment number\tRamping/Holding\tElapsed time (s)\tSegment time(s)\tPotential (V)\tCurrent (A)\n")
		except Exception as e:
			log_message(f"Write to file failed: {e}")
			cp_stop(segment_index, interrupted=True)
			return

		# Initialise buffers for holding averaged elapsed time, potential, and current data for each segment
		cp_time_data = AverageBuffer(cp_parameters['num_samples'])
		cp_segment_time_data = AverageBuffer(cp_parameters['num_samples'])
		cp_potential_data = AverageBuffer(cp_parameters['num_samples'])
		cp_current_data = AverageBuffer(cp_parameters['num_samples'])

		# Calculate ramp times and segment timelengths
		cp_parameters['ramp_time'] = []
		cp_parameters['segment_timelength'] = []
		for i, (segment_current, ramp_rate) in enumerate(zip(cp_parameters['current_sequence'], cp_parameters['ramp_rate'])):
			if ramp_rate == "STEP":
				cp_parameters['ramp_time'].append(0)
				cp_parameters['segment_timelength'].append(cp_parameters['hold_time'][i])
			else:
				if i == 0:
					current_to_sweep = segment_current - cp_data['startcurr'][segment_index] * 1e3  # Convert from mA to µA
				else:
					current_to_sweep = segment_current - cp_parameters['current_sequence'][i-1]

				ramp_time = abs(current_to_sweep / ramp_rate)
				cp_parameters['ramp_time'].append(ramp_time)
				cp_parameters['segment_timelength'].append(ramp_time + cp_parameters['hold_time'][i])

		# Initialise the plot area
		Legends.remove_all_legends()
		plot_frame.clear()
		plot_frame.enableAutoRange()
		plot_frame.getAxis('bottom').setTicks(None)
		plot_frame.setLabel('bottom', 'Time', units="s")
		plot_frame.setLabel('left', 'Potential', units="V")
		cp_potential_plot_curve = plot_frame.plot(pen='y', name='Potential')
		cp_current_plot_curve = plot_frame.plot(pen='r', name='Current (scaled)')
		legend = pyqtgraph.LegendItem(offset=(60, 10))
		legend.setParentItem(plot_frame.plotItem)
		Legends.legends['cp'] = legend
		legend_in_use = 'cp'

		# Update progress bar style to solid green border
		cp_progress_bar.set_solid_green_style()

		# Update GUI
		cp_info_program_state_entry.setText("Measuring CP")

		# Determine current to send to the DAC
		if cp_parameters['ramp_rate'][0] == "STEP":
			cp_currentsetpoint = cp_parameters['current_sequence'][0] * 1e-3  # Convert µA to mA
		else:
			cp_currentsetpoint = cp_data['startcurr'][0]  # Already in mA

		# Initialise DAC for measurements
		set_output(1, 0.0)  # Set zero current while range switching
		hardware_manual_control_range_dropdown.setCurrentIndex(current_range_from_current(cp_currentsetpoint))  # Determine the proper current range for the current setpoint
		set_current_range()  # Set new current range
		set_output(1, cp_currentsetpoint)  # Set current to setpoint
		set_control_mode(True)  # Galvanostatic control
		time.sleep(.2)  # Allow DAC some time to settle
		set_cell_status(True)  # Cell on

		# Begin calling cp_update() through periodic_update()
		state = States.Measuring_CP

		# Store experiment start time
		cp_data['experiment_start_time'] = timeit.default_timer()

	# If not initial segment
	else:
		# Refresh data sample buffers
		for data in [cp_time_data, cp_segment_time_data, cp_potential_data, cp_current_data]:
			data.samples = []

	# Initialise deque to store potential history for equilibration
	cp_pot_eq_history = collections.deque()

	# Update GUI and add segment to plot options dropdown lists
	cp_info_segmentnum_entry.setText(f"{segment_index + 1}/{cp_parameters['num_segments']}")
	cp_plot_options_segment1_dropdown.addItems([str(segment_index + 1)])
	cp_plot_options_update_segment_selector()
	cp_update_progress_bar()

	# Store segment start time
	cp_data['starttime'][segment_index] = timeit.default_timer()

def cp_update(segment_index):
	"""Add a new data point to the chronopotentiometry measurement - called regularly through periodic_update()"""
	global cp_currentsetpoint_hold

	# Calculate total and segment time elapsed
	elapsed_time = timeit.default_timer() - cp_data['experiment_start_time']
	segment_elapsed_time = timeit.default_timer() - cp_data['starttime'][segment_index]

	# If segment has completed
	if segment_elapsed_time >= cp_parameters['segment_timelength'][segment_index]:

		# Progress to the next segment
		cp_stop(segment_index, interrupted=False)
		return

	else:
		# DAC control logic
		ramp_rate = cp_parameters['ramp_rate'][segment_index]
		ramp_time = cp_parameters['ramp_time'][segment_index]

		# Ramping
		if ramp_rate != "STEP" and segment_elapsed_time < ramp_time:
			cp_currentsetpoint = cp_sweep(segment_index, segment_elapsed_time)  # Determine current to send to the DAC
			hardware_manual_control_range_dropdown.setCurrentIndex(current_range_from_current(cp_currentsetpoint))  # Determine the proper current range for the new setpoint
			set_current_range()  # Set new current range
			set_output(1, cp_currentsetpoint)  # Send current to the DAC

		# Holding
		elif segment_elapsed_time >= ramp_time:
			if not cp_currentsetpoint_hold:
				cp_currentsetpoint = cp_parameters['current_sequence'][segment_index] * 1e-3  # Convert µA to mA
				hardware_manual_control_range_dropdown.setCurrentIndex(current_range_from_current(cp_currentsetpoint))  # Determine the proper current range for the new setpoint
				set_current_range()  # Set new current range
				set_output(1, cp_currentsetpoint)  # Send current to the DAC
				cp_currentsetpoint_hold = True

		read_potential_current()  # Read new potential and current

		cp_time_data.add_sample(elapsed_time)
		cp_segment_time_data.add_sample(segment_elapsed_time)
		cp_potential_data.add_sample(potential)
		cp_current_data.add_sample(1e-3 * current)  # Convert mA to A

		if len(cp_segment_time_data.samples) == 0 and len(cp_segment_time_data.averagebuffer) > 0:  # A new average was just calculated

			if segment_elapsed_time >= ramp_time:
				ramp_or_hold = "Holding"
			else:
				ramp_or_hold = "Ramping"

			# Write new data to output file
			try:
				cp_output_file.write("%d\t%s\t%e\t%e\t%e\t%e\n" % (
					segment_index + 1,
					ramp_or_hold,
					cp_time_data.averagebuffer[-1],
					cp_segment_time_data.averagebuffer[-1],
					cp_potential_data.averagebuffer[-1],
					cp_current_data.averagebuffer[-1]
				))
			except Exception as e:
				log_message(f"Write to file failed: {e}")
				cp_stop(segment_index, interrupted=True)
				return

			# Append data to lists
			cp_data['time'][segment_index].append(cp_time_data.averagebuffer[-1])
			cp_data['ramping/holding'][segment_index].append(ramp_or_hold)
			cp_data['segment_time'][segment_index].append(cp_segment_time_data.averagebuffer[-1])
			cp_data['potential'][segment_index].append(cp_potential_data.averagebuffer[-1])
			cp_data['current'][segment_index].append(cp_current_data.averagebuffer[-1])

			# Update plot
			cp_update_plot(segment_index)

			# Check if potential has equilibrated
			if cp_parameters['pot_eq_tolerance'][segment_index]:
				if cp_pot_eq_bool(segment_index, segment_elapsed_time):

					# Write to summary file and progress to next segment
					cp_write_summary_file(segment_index, section="potential_equilibrated")
					log_message("*** Potential equilibrated ***")
					cp_stop(segment_index, interrupted=False)
					return

			# Check if lower potential limit has been surpassed
			if cp_parameters['pot_limit_lower']:
				lbound = cp_parameters['pot_limit_lower']
				if lbound == "OCP":
					lbound = cp_parameters['current_OCP']
				if cp_potential_data.averagebuffer[-1] <= lbound:

					# Write to summary file and interrupt experiment
					cp_write_summary_file(segment_index, section="lower_pot_limit_reached")
					log_message("***** Lower potential limit reached! *****")
					cp_stop(segment_index, interrupted=True)
					return

			# Check if upper potential limit has been surpassed
			if cp_parameters['pot_limit_upper']:
				ubound = cp_parameters['pot_limit_upper']
				if ubound == "OCP":
					ubound = cp_parameters['current_OCP']
				if cp_potential_data.averagebuffer[-1] >= ubound:

					# Write to summary file and interrupt experiment
					cp_write_summary_file(segment_index, section="upper_pot_limit_reached")
					log_message("***** Upper potential limit reached! *****")
					cp_stop(segment_index, interrupted=True)
					return

def cp_sweep(segment_index, segment_elapsed_time):

	ramp_time = cp_parameters['ramp_time'][segment_index]
	startcurr = cp_data['startcurr'][segment_index]  # Already in mA
	holdcurr = cp_parameters['current_sequence'][segment_index] * 1e-3  # Convert µA to mA

	if segment_elapsed_time >= ramp_time:
		return holdcurr
	else:
		return startcurr + (holdcurr - startcurr) * (segment_elapsed_time / ramp_time)

def cp_pot_eq_bool(segment_index, segment_elapsed_time):

	equilibrated = False
	if segment_elapsed_time >= cp_parameters['ramp_time'][segment_index]:
		cp_pot_eq_history.append((segment_elapsed_time, cp_potential_data.averagebuffer[-1]))

		# Remove data points older than pot_eq_timescale from the history deque
		while cp_pot_eq_history and (segment_elapsed_time - cp_pot_eq_history[0][0]) > cp_parameters['pot_eq_timescale'][segment_index]:
			cp_pot_eq_history.popleft()

		# Start checking if the change in potential is within tolerance after pot_eq_timescale following ramp
		if segment_elapsed_time >= cp_parameters['pot_eq_timescale'][segment_index] + cp_parameters['ramp_time'][segment_index]:
			current_potential = cp_potential_data.averagebuffer[-1]
			if cp_pot_eq_history:
				past_time, past_potential = cp_pot_eq_history[0]

				# Check if the change in potential across pot_eq_timescale is within tolerance
				if abs(current_potential - past_potential) < cp_parameters['pot_eq_tolerance'][segment_index] * 1e-3:  # Convert mV to V
					equilibrated = True

	return equilibrated

def cp_stop(segment_index, interrupted=True):
	global state, cp_current_segment_index

	if check_state([States.Measuring_CP, States.Measuring_CP_OCP_eq, States.Measuring_CP_Delay]):

		# Save segment finish time
		cp_data['finishtime_readable'][segment_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]

		if interrupted:

			set_cell_status(False)  # Cell off
			state = States.Stationary_Graph
			set_output(1, 0.0)  # Send zero current to DAC

			# Write to summary file and close
			cp_write_summary_file(segment_index, section="interrupted")

			# Reset experiment
			cp_reset_experiment_controller(mode="interrupted")

			# Update GUI
			log_message("*** EXPERIMENTS INTERRUPTED ***")
			QtWidgets.QMessageBox.information(
				mainwidget,
				"CP experiments interrupted",
				"Oh no! CP experiments have been interrupted.\n\nGlobal experiment parameters have been reset."
			)
			preview_cancel_button.show()

		elif not interrupted:

			# Write to summary file
			cp_write_summary_file(segment_index, section="segment_end")
			log_message(f"*** Segment {segment_index + 1}/{cp_parameters['num_segments']} completed ***")

			# If not final segment
			if segment_index + 1 != cp_parameters['num_segments']:

				# Increment the current segment index
				cp_current_segment_index += 1

				# Begin the next CP segment
				cp_start(cp_current_segment_index)

			# If final segment completed
			elif segment_index + 1 == cp_parameters['num_segments']:

				set_cell_status(False)  # Cell off
				state = States.Stationary_Graph
				set_output(1, 0.0)  # Send zero current to DAC

				# Write to summary file and close
				cp_write_summary_file(segment_index, section="all_segments_completed")

				# Reset experiment
				cp_reset_experiment_controller(mode="all_segments_completed")

				# Update GUI
				cp_info_program_state_entry.setText("All segments completed")
				QtWidgets.QMessageBox.information(
					mainwidget,
					"CP experiment completed",
					"CONGRATULATIONS! All CP segments have completed successfully.\n\nGlobal experiment parameters have been reset."
				)
				preview_cancel_button.show()

def cp_reset_experiment_controller(mode):
	global cp_parameters, cp_data
	global cp_parameters_checked, cp_filenames_checked
	global cp_total_segments, cp_current_segment_index
	global cp_currentsetpoint_hold

	# Stop timer
	cp_delay_timer.stop()

	if mode == "input_changed":
		if cp_parameters_checked:  # If inputs have changed since last successful check

			# Reset globals
			cp_parameters_checked = False
			cp_filenames_checked = False
			cp_variables_checkbutton.setStyleSheet("")

			# Reset progress bar
			cp_total_segments = None
			cp_current_segment_index = None
			cp_update_progress_bar()

			# Remove preview if showing
			if legend == Legends.legends['cp_preview'] and legend_in_use == 'cp_preview':
				if state == States.NotConnected:
					plot_frame.clear()
					legend.clear()
				elif state in (States.Idle, States.Stationary_Graph):
					preview_cancel()

			log_message("CP input parameters or program state has changed since the last successful check - check-state has been reset.")

		return

	elif mode == "checkbutton_failed":

		# Reset progress bar
		cp_total_segments = None
		cp_current_segment_index = None
		cp_update_progress_bar()
		return

	# Ensure output files are closed
	for file in ('cp_output_file', 'cp_summary_file'):
		try:
			if file in globals():
				f = globals()[file]
				if f and hasattr(f, 'close'):
					f.close()
					globals()[file] = None  # Clear reference to file
		except Exception as e:
			log_message(f"Error closing {file}: {e}")

	# Reset globals
	cp_parameters_checked = False
	cp_filenames_checked = False
	cp_variables_checkbutton.setStyleSheet("")
	cp_parameters = {'type': 'cp'}
	cp_data = {}
	cp_total_segments = None
	cp_current_segment_index = None
	cp_currentsetpoint_hold = False

	# Reset GUI
	cp_info_segmentnum_entry.setText("-/-")
	cp_plot_options_segment1_dropdown.clear()
	cp_plot_options_segment2_dropdown.clear()

	# Unfreeze input fields
	cp_freeze_inputs(freeze=False)

	if mode == "all_segments_completed":
		cp_progress_bar.set_completed_state()

	elif mode == "interrupted":
		cp_info_program_state_entry.setText(f"Experiments interrupted")
		cp_progress_bar.set_completed_state()

	elif mode == "OCP_interrupted":
		cp_progress_bar.set_OCP_interrupted_state()


"""CP ACCESSORY FUNCTIONS"""

def cp_preview():
	"""Generate a preview of the chronopotentiometry current profile in the plot window, based on the chronopotentiometry parameters currently entered in the GUI."""
	global legend, legend_in_use

	if cp_parameters_checked is False:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Parameters not checked",
			"Click the 'CHECK' button to ensure parameters are appropriate before constructing the preview."
		)
		return False

	if check_state([States.NotConnected, States.Idle, States.Stationary_Graph]):

		time_sequence = []
		current_sequence = []
		segment_start_end, segment_labels = [], []

		running_time = 0.0
		OCP_bool = cp_parameters['OCP_bool']
		num_segments = len(cp_parameters['current_sequence'])

		delay = global_software_settings['OCP_eq_timescale'] if OCP_bool else cp_parameters['delay']
		running_time += delay

		current_sequence.append(current * 1e-3)
		time_sequence.append(running_time)

		# Construct current profile
		for segment_index in range(num_segments):
			startcurr = current * 1e-3 if segment_index == 0 else cp_parameters['current_sequence'][segment_index - 1] * 1e-6
			ramp_rate = cp_parameters['ramp_rate'][segment_index]
			hold_time = cp_parameters['hold_time'][segment_index]
			holdcurr = cp_parameters['current_sequence'][segment_index] * 1e-6

			segment_start = running_time

			if ramp_rate == "STEP":
				current_sequence.append(holdcurr)
				current_sequence.append(holdcurr)
				time_sequence.append(running_time)
				running_time += hold_time
				time_sequence.append(running_time)
			else:
				current_sequence.append(holdcurr)
				current_sequence.append(holdcurr)
				ramp_time = abs((holdcurr - startcurr) / (ramp_rate * 1e-6))
				running_time += ramp_time
				time_sequence.append(running_time)
				running_time += hold_time
				time_sequence.append(running_time)

			segment_end = running_time
			segment_start_end.append([segment_start, segment_end])
			segment_labels.append(f"{segment_index + 1}")

		# Plot current profile in the plot window
		Legends.remove_all_legends()
		plot_frame.clear()

		if len(set(current_sequence)) == 1:  # Manually scale y-axis if current sequence is flat
			pad = 0.01
			ymin = current_sequence[0] - pad
			ymax = current_sequence[0] + pad
			plot_frame.setYRange(ymin, ymax, padding=0)
		else:
			plot_frame.enableAutoRange()

		plot_frame.getAxis('bottom').setTicks(None)
		plot_frame.setLabel('bottom', 'Time', units='s')
		plot_frame.setLabel('left', 'Current', units='A')

		legend = pyqtgraph.LegendItem(offset=(60, 10))
		legend.setParentItem(plot_frame.plotItem)
		Legends.legends['cp_preview'] = legend
		legend_in_use = 'cp_preview'

		current_curve = pyqtgraph.PlotDataItem([], [], pen='g')
		legend.addItem(current_curve, "Chronopotentiometry current profile")

		# Plot individual segment lines
		min_curr = min(current_sequence)
		max_curr = max(current_sequence)
		curr_range = max_curr - min_curr
		margin = curr_range * 0.1 if curr_range != 0 else 0.01  # Avoid zero range

		y0 = min_curr - margin
		y1 = max_curr + margin

		for i, (segment_start, segment_end) in enumerate(segment_start_end):
			line = pyqtgraph.PlotDataItem(
				x=[segment_start, segment_start],
				y=[y0, y1],
				pen=pyqtgraph.mkPen('w', width=1, style=QtCore.Qt.DashLine)
			)
			plot_frame.addItem(line)
			label = pyqtgraph.TextItem(f'{i + 1}', color='w', anchor=(0.5, 1))
			label.setPos((segment_start + segment_end) / 2, max_curr)
			plot_frame.addItem(label)

		# Add pre-delay line and final line
		if delay > 0:
			start_line = pyqtgraph.PlotDataItem(
				x=[0, 0],
				y=[y0, y1],
				pen=pyqtgraph.mkPen('w', width=1, style=QtCore.Qt.DashLine)
			)
			plot_frame.addItem(start_line)
		end_line = pyqtgraph.PlotDataItem(
			x=[segment_start_end[-1][1], segment_start_end[-1][1]],
			y=[y0, y1],
			pen=pyqtgraph.mkPen('w', width=1, style=QtCore.Qt.DashLine)
		)
		plot_frame.addItem(end_line)

		# Plot current profile
		plot_frame.plot(time_sequence, current_sequence, pen='g')

		if state in [States.Idle, States.Stationary_Graph]:
			preview_cancel_button.show()

def cp_write_summary_file(segment_index, section):
	"""Write summary file for the experiment."""
	global cp_summary_file

	try:
		if section == "initial":
			cp_summary_file = open(cp_parameters['experiment_info_path_filename'], 'w', 1)
			cp_summary_file.write(f"CP EXPERIMENTS INFORMATION FILE\n*******************************\n")

			cp_summary_file.write(f"\nExperiment notes: {cp_parameters['experiment_notes']}\n")

			cp_summary_file.write("\nExperiment information file for the experiments stored in:\n")
			cp_summary_file.write(f"{cp_parameters['path_filename']}\n")
			cp_summary_file.write(f"\n")

			if cp_parameters['OCP_bool']:
				delay_str = "Wait for OCP equilibration."
			else:
				delay_str = f"{cp_parameters['delay']}"

			if not cp_parameters['alternating_params_bool']:
				cp_summary_file.write("Alternating parameters?: No\n")
			else:
				cp_summary_file.write("Alternating parameters?: Yes\n")
				cp_summary_file.write("The following current sequence, hold times, and ramp rates follow A-B-A-B-A-...\n")
			cp_summary_file.write(f"Current sequence (µA): {cp_parameters['current_sequence']}\n")
			cp_summary_file.write(f"Hold times per current (s): {cp_parameters['hold_time']}\n")
			cp_summary_file.write(f"Ramp rate per current (µA/s): {cp_parameters['ramp_rate']}\n")
			if not cp_parameters['alternating_params_bool']:
				cp_summary_file.write(f"Equilibration tolerance (mV): {cp_parameters['pot_eq_tolerance'][0]}\n")
				cp_summary_file.write(f"Equilibration timescale (s): {cp_parameters['pot_eq_timescale'][0]}\n")
			else:
				cp_summary_file.write(f"Equilibration tolerance A (mV): {cp_parameters['pot_eq_tolerance'][0]}\n")
				cp_summary_file.write(f"Equilibration timescale A (s): {cp_parameters['pot_eq_timescale'][0]}\n")
				cp_summary_file.write(f"Equilibration tolerance B (mV): {cp_parameters['pot_eq_tolerance'][1]}\n")
				cp_summary_file.write(f"Equilibration timescale B (s): {cp_parameters['pot_eq_timescale'][1]}\n")
			cp_summary_file.write(f"Lower potential limit (V): {cp_parameters['pot_limit_lower']}\n")
			cp_summary_file.write(f"Upper potential limit (V): {cp_parameters['pot_limit_upper']}\n")
			cp_summary_file.write(f"Samples to average: {cp_parameters['num_samples']}\n")
			cp_summary_file.write(f"Pre-experiment delay (s): {delay_str}\n")

		elif section == "delay":
			cp_summary_file.write(f"\n*** Pre-experiment delay of {cp_parameters['delay']} seconds ***\n")

		elif section == "segment_start":
			cp_summary_file.write("\n**********************************\n\tCP SEGMENT STARTED\n**********************************\n")
			cp_summary_file.write(f"Segment number: {segment_index + 1}/{cp_parameters['num_segments']}\n")
			cp_summary_file.write(f"Hold current (µA): {cp_parameters['current_sequence'][segment_index]}\n")
			cp_summary_file.write(f"Hold time (s): {cp_parameters['hold_time'][segment_index]}\n")
			cp_summary_file.write(f"Ramp rate (µA/s): {cp_parameters['ramp_rate'][segment_index]}\n")
			cp_summary_file.write(f"Equilibration tolerance (mV): {cp_parameters['pot_eq_tolerance'][segment_index]}\n")
			cp_summary_file.write(f"Equilibration timescale (s): {cp_parameters['pot_eq_timescale'][segment_index]}\n")
			cp_summary_file.write("\n")

			cp_summary_file.write(f"Segment start time:  {cp_data['starttime_readable'][segment_index]}\n")

		elif section == "potential_equilibrated":
			cp_summary_file.write(f"\n***** POTENTIAL EQUILIBRATED *****\n")
			cp_summary_file.write(f"Potential ({cp_parameters['pot_eq_timescale'][segment_index]}) s ago (mV): {cp_pot_eq_history[0][1] * 1e3:.3f}\n")
			cp_summary_file.write(f"Equilibrated potential (mV): {cp_pot_eq_history[-1][1] * 1e3:.3f}\n")
			cp_summary_file.write(f"Equilibrated segment time (s): {cp_pot_eq_history[-1][0]:.3f}\n")
			cp_summary_file.write("\n")

		elif section == "segment_end":
			cp_summary_file.write(f"Segment finish time: {cp_data['finishtime_readable'][segment_index]}\n")

			cp_summary_file.write("\n************************************\n\tCP SEGMENT COMPLETED\n************************************\n")

		elif section == "all_segments_completed":
			cp_summary_file.write("\n***************************************************\n\tALL SEGMENTS COMPLETED SUCCESSFULLY\n***************************************************\n")

			cp_summary_file.write("\n")
			cp_summary_file.write("********************\n")
			cp_summary_file.write("EXPERIMENTS COMPLETE\n")
			cp_summary_file.close()

		elif section == "OCP_eq_start":
			cp_summary_file.write("\n*********************************\n\tOCP EQUILIBRATING\n*********************************\n")
			cp_summary_file.write(f"Equilibration tolerance (mV): {global_software_settings['OCP_eq_tolerance']}\n")
			cp_summary_file.write(f"Equilibration timescale (s): {global_software_settings['OCP_eq_timescale']}\n")
			cp_summary_file.write(f"Equilibration timeout (s): {global_software_settings['OCP_eq_timeout']}\n")
			cp_summary_file.write("\n")

			cp_summary_file.write(f"OCP equilibration start time: {cp_data['OCP_starttime_readable'][segment_index]}\n")


		elif section == "OCP_equilibrated":
			cp_summary_file.write(f"OCP equilibration finish time: {cp_data['OCP_eq_finishtime_readable'][segment_index]}\n")
			cp_summary_file.write("\n")

			cp_summary_file.write(f"Elapsed time (s): {cp_data['OCP_eq_elapsed_times'][-1]:.3f}\n")
			cp_summary_file.write(f"Starting potential (V): {cp_data['OCP_eq_startpot'][segment_index]:.5g}\n")
			cp_summary_file.write(f"Final equilibrated OCP (V): {cp_data['OCP_eq_stoppot'][segment_index]:.5g}\n")
			cp_summary_file.write(f"Total potential difference (V): {cp_data['OCP_eq_total_pot_diff'][segment_index]:.5g}\n")
			cp_summary_file.write(f"Final ({global_software_settings['OCP_eq_timescale']}) s potential difference (V): {cp_data['OCP_eq_timescale_pot_diff'][segment_index]:.5g}\n")
			cp_summary_file.write("\n")

			cp_summary_file.write("***** OCP successfully equilibrated *****\n")


		elif section == "OCP_valid":
			cp_summary_file.write("\n")
			cp_summary_file.write(f"{cp_parameters['OCP_valid_text']}\n")

		elif section == "OCP_invalid":
			cp_summary_file.write("\n*******************************************\n\tCP MEASUREMENTS INTERRUPTED\n*******************************************\n")
			cp_summary_file.write("CP experiments stopped due to OCP becoming incompatible with experiment parameters.")
			cp_summary_file.write(f"OCP (V): {cp_parameters['current_OCP']}\n")
			cp_summary_file.write("\n")

			cp_summary_file.write(f"Lower potential limit (V): {cp_parameters['pot_limit_lower']}\n")
			cp_summary_file.write(f"Upper potential limit (V): {cp_parameters['pot_limit_upper']}\n")
			cp_summary_file.write("\n")

			cp_summary_file.write(f"{cp_parameters['OCP_valid_text']}\n")

			cp_summary_file.write("\n")
			cp_summary_file.write("**********************\n")
			cp_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			cp_summary_file.close()

		elif section == "OCP_timeout":
			cp_summary_file.write(f"OCP equilibration timeout time: {cp_data['OCP_timeout_finishtime_readable'][segment_index]}\n")

			cp_summary_file.write("\n*******************************************\n\tCP MEASUREMENTS INTERRUPTED\n*******************************************\n")
			cp_summary_file.write("CP experiments stopped due to OCP equilibration timeout.\n")
			cp_summary_file.write("OCP did not equilibrate to within tolerance by the timeout threshold.\n")
			cp_summary_file.write("\n")

			cp_summary_file.write(f"OCP timeout threshold (s): {global_software_settings['OCP_eq_timeout']}\n")
			cp_summary_file.write(f"Elapsed time (s): {cp_data['OCP_timeout_time_elapsed'][segment_index]:.3g}\n")
			cp_summary_file.write(f"Starting potential (V): {cp_data['OCP_timeout_startpot'][segment_index]:.5g}\n")
			cp_summary_file.write(f"Final potential (V): {cp_data['OCP_timeout_stoppot'][segment_index]:.5g}\n")
			cp_summary_file.write(f"Total potential difference (V): {cp_data['OCP_timeout_total_pot_diff'][segment_index]:.5g}\n")
			cp_summary_file.write(f"Final ({global_software_settings['OCP_eq_timescale']}) s potential difference (V): {cp_data['OCP_timeout_timescale_pot_diff'][segment_index]:.5g}\n")

			cp_summary_file.write("\n")
			cp_summary_file.write("**********************\n")
			cp_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			cp_summary_file.close()

		elif section == "interrupted":
			cp_summary_file.write("\n*******************************************\n\tCP MEASUREMENTS INTERRUPTED\n*******************************************\n")
			cp_summary_file.write(f"Segment interrupted: {segment_index + 1}/{cp_parameters['num_segments']}\n")
			cp_summary_file.write(f"Experiment interruption time: {cp_data['finishtime_readable'][segment_index]}\n")
			cp_summary_file.write(f"CP segment interrupted: {cp_parameters['current_sequence'][segment_index]} µA\n")

			cp_summary_file.write("\n")
			cp_summary_file.write("**********************\n")
			cp_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			cp_summary_file.close()

		elif section == "lower_pot_limit_reached":
			cp_summary_file.write("\n***** LOWER POTENTIAL LIMIT REACHED *****\n")
			cp_summary_file.write(f"Lower potential limit (V): {cp_parameters['pot_limit_lower']}\n")
			cp_summary_file.write(f"Last averaged potential (V): {cp_potential_data.averagebuffer[-1]:.5g}\n")
			cp_summary_file.write("***** INTERRUPTING MEASUREMENTS *****\n")

		elif section == "upper_pot_limit_reached":
			cp_summary_file.write("\n***** UPPER POTENTIAL LIMIT REACHED *****\n")
			cp_summary_file.write(f"Upper potential limit (V): {cp_parameters['pot_limit_upper']}\n")
			cp_summary_file.write(f"Last averaged potential (V): {cp_potential_data.averagebuffer[-1]:.5g}\n")
			cp_summary_file.write("***** INTERRUPTING MEASUREMENTS *****\n")

		elif section == "error":
			cp_summary_file.write("\n**********************************************\n\tERROR INITIALISING EXPERIMENTS\n**********************************************\n")

			cp_summary_file.write("\n")
			cp_summary_file.write("**********************\n")
			cp_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			cp_summary_file.close()

	except Exception as e:
		log_message(f"Write to summary file failed: {e}")

def cp_OCP_valid_bool(segment_index):

	pot_limit_lower = cp_parameters['pot_limit_lower']
	pot_limit_upper = cp_parameters['pot_limit_upper']
	OCP_value = cp_parameters['current_OCP']

	if pot_limit_lower == "OCP" and pot_limit_upper is not None:
		if pot_limit_upper <= OCP_value:
			return False, "OCP invalid: Lower potential limit must be less than the upper potential limit."

	if pot_limit_upper == "OCP" and pot_limit_lower is not None:
		if pot_limit_lower >= OCP_value:
			return False, "OCP invalid: Upper potential limit must be greater than the lower potential limit."

	return True, "All experiment parameters are valid after OCP equilibration."


"""CP PLOT AND GUI FUNCTIONS"""

def cp_update_plot(segment_index):
	"""Update the plot with the current CP segment data, and other segments depending on GUI inputs."""

	def add_segment_data(start_idx, stop_idx, ref_time):
		"""Concatenate data from segments in range [start_idx, stop_idx] and normalise time by ref_time."""
		nonlocal time_data, current_data, potential_data, segment_centers, segment_labels

		for i in range(start_idx, stop_idx + 1):
			seg_time = numpy.array(cp_data['time'][i]) - ref_time
			seg_current = numpy.array(cp_data['current'][i])
			seg_potential = numpy.array(cp_data['potential'][i])

			segment_centers.append((seg_time[0] + seg_time[-1]) / 2)
			segment_labels.append(i + 1)

			time_data = numpy.concatenate([time_data, seg_time])
			current_data = numpy.concatenate([current_data, seg_current])
			potential_data = numpy.concatenate([potential_data, seg_potential])

			# Add vertical lines between segments
			if i > 0:
				line_pos = seg_time[0]
				line = pyqtgraph.InfiniteLine(
					pos=line_pos,
					angle=90,
					pen=pyqtgraph.mkPen('w', style=QtCore.Qt.DashLine)
				)
				plot_frame.addItem(line)

	# Clear plot frame and legend
	plot_frame.clear()
	legend.clear()

	# Title for legend
	dummy_item = pyqtgraph.PlotDataItem([], [], pen=None)
	legend.addItem(dummy_item, "Chronopotentiometry experiment:")

	time_data = numpy.array([])
	current_data = numpy.array([])
	potential_data = numpy.array([])
	segment_centers = []
	segment_labels = []

	if cp_plot_options_all_segments_radiobutton.isChecked():
		ref_time = cp_data['time'][0][0]
		add_segment_data(0, segment_index, ref_time)

	elif cp_plot_options_current_segment_only_radiobutton.isChecked():
		ref_time = cp_data['time'][segment_index][0]
		add_segment_data(segment_index, segment_index, ref_time)

	elif cp_plot_options_segment_select_radiobutton.isChecked():
		seg1_idx = int(cp_plot_options_segment1_dropdown.currentText()) - 1
		seg2_idx = int(cp_plot_options_segment2_dropdown.currentText()) - 1
		ref_time = cp_data['time'][seg1_idx][0]
		add_segment_data(seg1_idx, seg2_idx, ref_time)

	# Scale current data relative to potential data
	if current_data.size > 0 and potential_data.size > 0:
		current_max = numpy.max(numpy.abs(current_data))
		potential_max = numpy.max(numpy.abs(potential_data))
		if current_max != 0:
			current_data *= (potential_max / current_max)

		# Add segment labels at max height
		for segment_label, segment_center in zip(segment_labels, segment_centers):
			label = pyqtgraph.TextItem(f'{segment_label}', color='w', anchor=(0.5, 1))
			label.setPos(segment_center, potential_max)
			plot_frame.addItem(label)

	cp_current_plot_curve.setData(time_data, current_data)
	cp_potential_plot_curve.setData(time_data, potential_data)

	plot_frame.addItem(cp_current_plot_curve)
	plot_frame.addItem(cp_potential_plot_curve)

	legend.addItem(cp_potential_plot_curve, "Potential")
	legend.addItem(cp_current_plot_curve, "Current (scaled)")

	# Add equilibration info to legend
	if cp_parameters['pot_eq_tolerance'][segment_index] is not None:
		legend.addItem(dummy_item, "")
		legend.addItem(dummy_item, "Equilibration information:")
		legend.addItem(dummy_item, f"Tolerance (mV): {cp_parameters['pot_eq_tolerance'][segment_index]}")
		legend.addItem(dummy_item, f"Timescale (s): {cp_parameters['pot_eq_timescale'][segment_index]}")
		if cp_pot_eq_history:
			if cp_pot_eq_history[-1][0] >= cp_parameters['pot_eq_timescale'][segment_index]:
				legend.addItem(dummy_item, f"Potential {cp_parameters['pot_eq_timescale'][segment_index]} s ago (mV): {cp_pot_eq_history[0][1] * 1000:.3g}")
			else:
				legend.addItem(dummy_item, f"Potential {cp_pot_eq_history[-1][0] - cp_pot_eq_history[0][0]:.3f} s ago (mV): {cp_pot_eq_history[0][1] * 1000:.3g}")
			legend.addItem(dummy_item, f"Measured potential (mV): {cp_pot_eq_history[-1][1] * 1000:.3g}")
			pot_diff = (cp_pot_eq_history[-1][1] - cp_pot_eq_history[0][1])
			legend.addItem(dummy_item, f"Difference (mV): {pot_diff * 1000:.3g}")

def cp_update_progress_bar():
	"""Update the progress bar to reflect percentage of segments completed."""

	cp_progress_bar.update_progress_bar(cp_total_segments, cp_current_segment_index)
	cp_progress_bar.update()



"""_____SELF-DISCHARGE FUNCTIONS_____"""

"""SD PARAMETER FUNCTIONS"""

def sd_checkbutton_callback():
	"""Function to control the data-validation process, called when "CHECK" button pressed."""
	global sd_parameters_checked, sd_filenames_checked, sd_current_segment_index, sd_total_segments

	# Initialise with parameters_checked = False, filenames_checked = False, and a check button style reset
	sd_parameters_checked = False
	sd_filenames_checked = False
	sd_variables_checkbutton.setStyleSheet("")

	# Remove any previous program state entry
	sd_info_program_state_entry.setText("No experiments running")

	# Check input parameter formats
	if sd_validate_inputs():
		pass
	else:
		sd_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
		log_message("SD check button failed: Inputs are not in the correct format.")
		return False

	# Write input parameters to global dictionary
	if sd_get_parameters():
		pass
	else:
		sd_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
		log_message("SD check button failed: Could not write experiment parameters to a global dictionary.")
		return False

	# If filename provided, check if filenames are acceptable
	if sd_file_entry.text().strip() != "":
		if sd_get_filenames():
			sd_filenames_checked = True
		else:
			sd_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
			log_message("SD check button failed: Experiment filenames could not be constructed.")
			return False

	# Give any parameter warnings
	sd_parameter_warnings()

	# Set global parameters_checked state as True
	sd_parameters_checked = True

	# Make check button green
	sd_variables_checkbutton.setStyleSheet("background-color: green; color: white;")

	# Calculate the number of halfcycles this experiment will perform
	sd_total_segments = sd_parameters['num_segments']

	# Initialise current_segment_index for progress bar
	sd_current_segment_index = -1

	# Update progress bar and give green border
	sd_update_progress_bar()
	sd_progress_bar.set_solid_green_style()

	log_message(f"Check button successful! Experiments are ready to run. Segments remaining: {sd_total_segments}")
	return True

def sd_validate_inputs():
	"""Ensure inputs are of the correct format."""

	# Charge potentials
	try:
		charge_potentials_str = sd_params_charge_potential_entry.text().strip()
		charge_potentials_list = [charge_potential.strip() for charge_potential in charge_potentials_str.split(",")]
		for i, charge_potential in enumerate(charge_potentials_list):
			if charge_potential.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				charge_potentials_list[i] = "OCP"
			else:
				charge_potentials_list[i] = float(charge_potential)

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Charge potential sequence input",
			"Charge potentials must be numeric values or 'OCP'."
		)
		return False

	# Charge hold times
	try:
		charge_hold_times_str = sd_params_hold_time_entry.text().strip()
		charge_hold_times_list = [float(charge_hold_time.strip()) for charge_hold_time in charge_hold_times_str.split(",")]
		if any(charge_hold_time < 0 for charge_hold_time in charge_hold_times_list):
			raise ValueError

		# Hold times same length as potential sequence
		if len(charge_hold_times_list) != len(charge_potentials_list):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Charge hold times input",
				"Charge hold times must be of the same length as charge potential sequence."
			)
			return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Charge hold times input",
			"Charge hold times must be non-negative numeric values."
		)
		return False

	# Charging ramp rates
	try:
		charge_ramp_rates_str = sd_params_ramp_rate_entry.text().strip()
		if charge_ramp_rates_str == "":
			charge_ramp_rates_list = []
		else:
			charge_ramp_rates_list = [charge_ramp_rate.strip() for charge_ramp_rate in charge_ramp_rates_str.split(",")]
			for i, charge_ramp_rate in enumerate(charge_ramp_rates_list):
				if charge_ramp_rate.lower() == "step":  # Remove case-sensitivity for "STEP"
					charge_ramp_rates_list[i] = "STEP"
				else:
					charge_ramp_rates_list[i] = float(charge_ramp_rate)

			# Ramp rates list same length as potential sequence
			if len(charge_ramp_rates_list) != len(charge_potentials_list):
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Charge ramp rates input",
					"If charge ramp rates are given, there must be one per charge potential."
				)
				return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Charge ramp rates input",
			"Charge ramp rates must be positive numeric values or 'STEP'."
		)
		return False

	# Charge ramp rates are positive
	if any(charge_ramp_rate != "STEP" and charge_ramp_rate <= 0 for charge_ramp_rate in charge_ramp_rates_list):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Charge ramp rates input",
			"Charge ramp rates rates must be positive numeric values or 'STEP'."
		)
		return False

	# Charge ramp rates are present if expected
	if sd_params_ramp_rate_checkbox.isChecked() and charge_ramp_rates_list == []:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Charge ramp rates input",
			"Charge ramp rate input expected but not given."
		)
		return False

	# Acquisition times
	try:
		acquisition_times_str = sd_params_acquisition_time_entry.text().strip()
		acquisition_times_list = [float(acquisition_time.strip()) for acquisition_time in acquisition_times_str.split(",")]
		if any(acquisition_time <= 0 for acquisition_time in acquisition_times_list):
			raise ValueError

		# Acquisition times same length as charge potential sequence
		if len(acquisition_times_list) != len(charge_potentials_list):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Acquisition times input",
				"Self-discharge acquisition times must be of the same length as charge potential sequence."
			)
			return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Acquisition times input",
			"Self-discharge acquisition times must be positive numeric values."
		)
		return False

	# Acquisition potential cutoffs
	try:
		acquisition_cutoffs_str = sd_params_acquisition_cutoff_entry.text().strip()
		if acquisition_cutoffs_str == "":
			acquisition_cutoffs_list = []
		else:
			acquisition_cutoffs_list = [acquisition_cutoff.strip() for acquisition_cutoff in acquisition_cutoffs_str.split(",")]
			for i, acquisition_cutoff in enumerate(acquisition_cutoffs_list):
				if acquisition_cutoff.lower() == "ocp":
					acquisition_cutoffs_list[i] = "OCP"
				elif acquisition_cutoff.lower() == "none":
					acquisition_cutoffs_list[i] = None
				else:
					acquisition_cutoffs_list[i] = float(acquisition_cutoff)

			# Acquisition cutoffs same length as charge potentials
			if len(acquisition_cutoffs_list) != len(charge_potentials_list):
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Acquisition potential cutoffs input",
					"If acquisition potential cutoffs are given, there must be one per charge potential."
				)
				return False

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Acquisition potential cutoff input",
			"Acquisition potential cutoffs must be numeric values, 'OCP', or 'None'."
		)
		return False

	# Acquisition cutoffs expected but not given
	if sd_params_acquisition_cutoff_checkbox.isChecked() and acquisition_cutoffs_list == []:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Acquisition potential cutoffs input",
			"Acquisition potential cutoffs expected but not given."
		)
		return False

	# Equilibration tolerance
	try:
		pot_eq_tolerance_str = sd_params_acquisition_equilibration_tolerance_entry.text().strip()
		if pot_eq_tolerance_str == "":
			pot_eq_tolerance = None
		else:
			pot_eq_tolerance = float(pot_eq_tolerance_str)
			if pot_eq_tolerance < 0:
				raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Equilibration tolerance input",
			"Equilibration tolerance must be a non-negative numeric value."
		)
		return False

	# Equilibration timescale
	try:
		pot_eq_timescale_str = sd_params_acquisition_equilibration_timescale_entry.text().strip()
		if pot_eq_timescale_str == "":
			pot_eq_timescale = None
		else:
			pot_eq_timescale = float(pot_eq_timescale_str)
			if pot_eq_timescale <= 0:
				raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Equilibration timescale input",
			"Equilibration timescale must be a positive numeric value."
		)
		return False

	# Potential equilibration tolerance and timescale present if expected
	if sd_params_acquisition_equilibration_checkbox.isChecked():
		if pot_eq_tolerance is None:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Equilibration tolerance input",
				"Equilibration tolerance expected but not given."
			)
			return False
		if pot_eq_timescale is None:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Equilibration timescale input",
				"Equilibration timescale expected but not given."
			)
			return False

	# Number of samples to average
	try:
		num_samples = int(sd_params_num_samples_entry.text().strip())
		if num_samples <= 0:
			raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Samples to average input",
			"Samples to average input must be a positive integer value."
		)
		return False

	# Determine whether OCP equilibration required
	if sd_params_delay_OCP_checkbox.isChecked():
		OCP_eq = True
	else:
		OCP_eq = False
		if any(chargepot == "OCP" for chargepot in charge_potentials_list):
			OCP_eq = True
		if any(cutoff == "OCP" for cutoff in acquisition_cutoffs_list):
			OCP_eq = True

	# Delay if required
	if not OCP_eq:
		try:
			delay = float(sd_params_delay_entry.text().strip())
			if delay < 0:
				raise ValueError

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Pre-experiment delay input",
				"Pre-experiment delay must be a non-negative numeric value and given if not waiting for OCP equilibration."
			)
			return False

	else:  # Remove the delay
		sd_params_delay_OCP_checkbox.setChecked(True)
		sd_params_delay_entry.setText("")

	return True

def sd_get_parameters():
	""""Write experiment parameters to a global dictionary."""
	global sd_parameters

	sd_parameters = {'type': 'sd'}

	try:
		# Track if OCP to be equilibrated before the experiment
		OCP_bool = False

		# Charging potential sequence
		charge_potentials = sd_params_charge_potential_entry.text().strip()
		charge_potentials_list = [charge_potential.strip() for charge_potential in charge_potentials.split(",")]
		for i, charge_potential in enumerate(charge_potentials_list):
			if charge_potential.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				charge_potentials_list[i] = "OCP"
				OCP_bool = True
			else:
				charge_potentials_list[i] = float(charge_potential)

		# Potential hold times
		hold_times = sd_params_hold_time_entry.text().strip()
		hold_times_list = [float(hold_time.strip()) for hold_time in hold_times.split(",")]

		# Potential ramp rates
		if sd_params_ramp_rate_checkbox.isChecked():
			ramp_rates = sd_params_ramp_rate_entry.text().strip()
			ramp_rates_list = [ramp_rate.strip() for ramp_rate in ramp_rates.split(",")]
			ramp_rates_mV_list = [ramp_rate.strip() for ramp_rate in ramp_rates.split(",")]
			for i, ramp_rate in enumerate(ramp_rates_list):
				if ramp_rate.lower() == "step":  # Remove case-sensitivity for "STEP"
					ramp_rates_list[i] = "STEP"
					ramp_rates_mV_list[i] = "STEP"
				else:
					ramp_rates_list[i] = float(ramp_rate) * 1e-3  # Convert ramp rate from mV/s to V/s
					ramp_rates_mV_list[i] = float(ramp_rate)
		else:
			ramp_rates_list = ["STEP"] * len(charge_potentials_list)
			ramp_rates_mV_list = ["STEP"] * len(charge_potentials_list)


		#Self-discharge acquisition parameters

		# Self-discharge acquisition times
		acquisition_times = sd_params_acquisition_time_entry.text().strip()
		acquisition_times_list = [float(acquisition_time.strip()) for acquisition_time in acquisition_times.split(",")]

		# Self-discharge potential cutoffs
		if sd_params_acquisition_cutoff_checkbox.isChecked():
			potential_cutoffs = sd_params_acquisition_cutoff_entry.text().strip()
			potential_cutoffs_list = [pot_cutoff.strip() for pot_cutoff in potential_cutoffs.split(",")]
			for i, potential_cutoff in enumerate(potential_cutoffs_list):
				if potential_cutoff.lower() == "none":  # Remove case-sensitivity for "None"
					potential_cutoffs_list[i] = None
				elif potential_cutoff.lower() == "ocp":  # Remove case-sensitivity for "OCP"
					potential_cutoffs_list[i] = "OCP"
					OCP_bool = True
				else:
					potential_cutoffs_list[i] = float(potential_cutoff)
		else:
			potential_cutoffs_list = [None] * len(charge_potentials_list)

		# Self-discharge equilibration
		if sd_params_acquisition_equilibration_checkbox.isChecked():
			pot_eq_tolerance = sd_params_acquisition_equilibration_tolerance_entry.text().strip()
			pot_eq_timescale = sd_params_acquisition_equilibration_timescale_entry.text().strip()
			pot_eq_tolerances_list = [float(pot_eq_tolerance)] * len(charge_potentials_list)
			pot_eq_timescales_list = [float(pot_eq_timescale)] * len(charge_potentials_list)
		else:
			pot_eq_tolerances_list = [None] * len(charge_potentials_list)
			pot_eq_timescales_list = [None] * len(charge_potentials_list)

		# Store parameters in global dictionary
		sd_parameters['charge_potential'] = charge_potentials_list
		sd_parameters['hold_time'] = hold_times_list
		sd_parameters['ramp_rate'] = ramp_rates_list
		sd_parameters['ramp_rate_mV/s'] = ramp_rates_mV_list
		sd_parameters['acquisition_time'] = acquisition_times_list
		sd_parameters['pot_cutoff'] = potential_cutoffs_list
		sd_parameters['pot_eq_tolerance'] = pot_eq_tolerances_list
		sd_parameters['pot_eq_timescale'] = pot_eq_timescales_list

		# Number of samples to average
		sd_parameters['num_samples'] = int(sd_params_num_samples_entry.text().strip())

		# Number of segments
		sd_parameters['num_segments'] = len(charge_potentials_list)

		sd_parameters['segment_timelength'] = [None] * len(charge_potentials_list)
		sd_parameters['ramp_time'] = [None] * len(charge_potentials_list)

		# Pre-experiment delay
		if sd_params_delay_OCP_checkbox.isChecked():
			OCP_bool = True
		else:
			sd_parameters['delay'] = float(sd_params_delay_entry.text().strip())

		# OCP equilibration pre-experiment
		sd_parameters['OCP_bool'] = OCP_bool

		# Experiment notes
		experiment_notes = sd_file_notes_entry.toPlainText()
		if not experiment_notes:
			experiment_notes = "No notes provided."
		sd_parameters['experiment_notes'] = experiment_notes

		# Plot pen colours
		sd_parameters['plot_pen_color'] = defaultdict(lambda: defaultdict(list))
		for i in range(sd_parameters['num_segments']):
			sd_parameters['plot_pen_color'][i] = CB_color_cycle[i % len(CB_color_cycle)]

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Input error",
			"One or more parameters are in the wrong format and cannot be written to a global dictionary."
		)
		return False

	return True

def sd_parameter_warnings():
	"""Give GUI warnings for unused parameters."""

	# Unused charging ramp rates warning
	if sd_params_ramp_rate_entry.text().strip() != "" and not sd_params_ramp_rate_checkbox.isChecked():
		QtWidgets.QMessageBox.warning(
			mainwidget,
			"Warning: Charging ramp rates input",
			"Charging ramp rates provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values, otherwise, the potential will be stepped."
		)

	# Unused potential cutoff thresholds warning
	if sd_params_acquisition_cutoff_entry.text().strip() != "" and not sd_params_acquisition_cutoff_checkbox.isChecked():
		QtWidgets.QMessageBox.warning(
			mainwidget,
			"Warning: Potential cutoffs input",
			"Potential cutoffs provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values."
		)

	# Unused equilibration parameters warning
	if (sd_params_acquisition_equilibration_tolerance_entry.text().strip() != "" or sd_params_acquisition_equilibration_timescale_entry.text().strip() != "") and not sd_params_acquisition_equilibration_checkbox.isChecked():
		QtWidgets.QMessageBox.warning(
			mainwidget,
			"Warning: Equilibration input",
			"Equilibration conditions provided as inputs but not used.\n\nPlease ensure that this is deliberate!\n\nTick the checkbox if you wish to use these values."
		)

	return True

def sd_get_filenames():
	"""Construct filenames for the experiments."""
	global sd_parameters

	try:
		filename_entry = str(sd_file_entry.text().strip())
		if filename_entry == "":
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: No filename provided",
				"Please provide a base filename for the experiments."
			)
			return False

		directory_path = os.path.dirname(filename_entry)
		if directory_path == "":
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: No parent directory specified",
				f"The output files will be stored in the current working directory. Is this okay?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False
			directory_path = os.getcwd()
		elif not os.path.isdir(directory_path):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Directory does not exist",
				f"The directory {directory_path} does not exist."
			)
			return False

		if "_SD_{experiment_info_here}" in filename_entry:
			filename_entry = filename_entry.split("_SD_{experiment_info_here}")[0]
		filename = os.path.basename(filename_entry)
		if filename == "":  # If no filename given, only path
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: No filename provided",
				"Please provide a base filename for the experiments in addition to the path."
			)
			return False

		sd_parameters['directory_path'] = directory_path
		sd_parameters['base_filename'], _ = os.path.splitext(filename)

		exp_filename = f"{sd_parameters['base_filename']}_SD"
		for potential in sd_parameters['charge_potential']:
			exp_filename += f"_{potential}"
		exp_filename += "_V"

		sd_parameters['filename'] = exp_filename + ".dat"
		sd_parameters['path_filename'] = os.path.join(directory_path, sd_parameters['filename'])

		sd_parameters['experiment_info_filename'] = sd_parameters['base_filename'] + "_SD_experiment_info.txt"
		sd_parameters['experiment_info_path_filename'] = os.path.join(directory_path, sd_parameters['experiment_info_filename'])

		file = sd_parameters['path_filename']
		if os.path.isfile(file):
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: File already exists",
				f"The output file {file} already exists. Do you want to overwrite it?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False

		info_file = sd_parameters['experiment_info_path_filename']
		if os.path.isfile(info_file):
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: Results file already exists",
				f"The experiment info output file {info_file} already exists. Do you want to overwrite it?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False

	except Exception as e:
		print(e)
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: File error",
			f"One or more output filepaths are not valid."
		)
		return False

	sd_file_entry.setText(os.path.join(sd_parameters['directory_path'], f"{sd_parameters['base_filename']}_SD_{{experiment_info_here}}"))

	return True

def sd_validate_filenames():
	"""Check validity of files by creating and attempting to open them."""

	file = sd_parameters.get('path_filename')
	if file:
		try:
			with open(file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {file} is not valid."
			)
			return False

	info_file = sd_parameters.get('experiment_info_path_filename')
	if info_file:
		try:
			with open(info_file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {info_file} is not valid."
			)
			return False

	return True

def sd_freeze_inputs(freeze):
	"""Function to freeze and unfreeze GUI inputs when experiments are running."""

	if freeze:
		sd_params_charge_potential_entry.setEnabled(False)
		sd_params_hold_time_entry.setEnabled(False)
		sd_params_ramp_rate_checkbox.setEnabled(False)
		sd_params_ramp_rate_entry.setEnabled(False)
		sd_params_acquisition_time_entry.setEnabled(False)
		sd_params_acquisition_cutoff_checkbox.setEnabled(False)
		sd_params_acquisition_cutoff_entry.setEnabled(False)
		sd_params_acquisition_equilibration_checkbox.setEnabled(False)
		sd_params_acquisition_equilibration_tolerance_entry.setEnabled(False)
		sd_params_acquisition_equilibration_timescale_entry.setEnabled(False)
		sd_params_num_samples_entry.setEnabled(False)
		sd_params_delay_entry.setEnabled(False)
		sd_params_delay_OCP_checkbox.setEnabled(False)
		sd_file_entry.setEnabled(False)
		sd_file_notes_entry.setEnabled(False)
		sd_variables_checkbutton.setEnabled(False)
		software_globals_menu_button.setEnabled(False)

	elif not freeze:
		sd_params_charge_potential_entry.setEnabled(True)
		sd_params_hold_time_entry.setEnabled(True)
		sd_params_ramp_rate_checkbox.setEnabled(True)
		sd_params_ramp_rate_entry.setEnabled(True)
		sd_params_acquisition_time_entry.setEnabled(True)
		sd_params_acquisition_cutoff_checkbox.setEnabled(True)
		sd_params_acquisition_cutoff_entry.setEnabled(True)
		sd_params_acquisition_equilibration_checkbox.setEnabled(True)
		sd_params_acquisition_equilibration_tolerance_entry.setEnabled(True)
		sd_params_acquisition_equilibration_timescale_entry.setEnabled(True)
		sd_params_num_samples_entry.setEnabled(True)
		sd_params_delay_entry.setEnabled(True)
		sd_params_delay_OCP_checkbox.setEnabled(True)
		sd_file_entry.setEnabled(True)
		sd_file_notes_entry.setEnabled(True)
		sd_variables_checkbutton.setEnabled(True)
		software_globals_menu_button.setEnabled(True)


"""SD CORE FUNCTIONS"""

def sd_initialise():
	"""Initialise chronoamperometry experiment."""
	global state
	global sd_data, sd_current_segment_index
	global sd_time_data, sd_segment_time_data, sd_potential_data, sd_current_data

	# Ensure parameters have been verified using the "CHECK" button
	if not sd_parameters_checked:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Parameters not checked",
			"Click the 'CHECK' button to ensure parameters are appropriate before starting your experiments."
		)
		return False

	# Ensure filenames have been checked
	if not sd_filenames_checked:
		if sd_get_filenames():
			pass
		else:
			return False

	if check_state([States.Idle, States.Stationary_Graph]):

		# Validate filenames before experiment begins
		if sd_validate_filenames():
			pass
		else:
			sd_reset_experiment_controller(mode="interrupted")
			log_message("SD experiment could not initialise due to invalid output filename.")
			return False

		# Turn cell off if under manual control
		set_cell_status(False)

		# Freeze input fields and hide return to live graph button
		sd_freeze_inputs(freeze=True)
		preview_cancel_button.hide()

		# Initialise segment index
		sd_current_segment_index = 0

		# Write experiment info to summary file
		sd_write_summary_file(sd_current_segment_index, section="initial")

		# Update GUI
		sd_info_segmentnum_entry.setText(f"-/{sd_parameters['num_segments']}")
		log_message("Starting self-discharge experiment...")

		# Initialise sd_data dictionary
		sd_data = {
			'starttime': defaultdict(float),
			'starttime_readable': defaultdict(str),
			'finishtime_readable': defaultdict(str),

			'startpot': defaultdict(float),
			'time': defaultdict(list),
			'segment_type': defaultdict(list),
			'segment_time': defaultdict(list),
			'potential': defaultdict(list),
			'current': defaultdict(list),
		}

		sd_parameters['ramp_time'] = defaultdict(float)
		sd_parameters['charging_time'] = defaultdict(float)
		sd_parameters['segment_timelength'] = defaultdict(float)
		sd_parameters['charging/self-discharging'] = ["Charging"] * sd_parameters['num_segments']

		# Pass to OCP equilibration controller if required
		if sd_parameters['OCP_bool']:
			OCP_initialise_data_entries(sd_data)
			OCP_equilibration_controller(sd_parameters, sd_data, sd_current_segment_index, equilibrated=False)
		else:
			# Write pre-experiment delay to summary file and update GUI
			sd_write_summary_file(sd_current_segment_index, section="delay")
			sd_info_program_state_entry.setText(f"Delay of {sd_parameters['delay']} s")

			# Update progress bar style to solid yellow border
			sd_progress_bar.set_solid_yellow_style()

			# Update state
			state = States.Measuring_SD_Delay

			# Launch the delay timer
			sd_delay_timer.start(int(sd_parameters['delay'] * 1000))  # Delay input in ms

def sd_start(segment_index):
	"""Begin or progress the self-discharge experiment."""
	global state, skipcounter
	global sd_data, sd_time_data, sd_segment_time_data, sd_potential_data, sd_current_data
	global sd_charge_hold, sd_acquiring, sd_pot_eq_history, sd_output_file
	global sd_charge_potential_plot_curve, sd_discharge_potential_plot_curve
	global legend, legend_in_use

	if segment_index is None:
		state = States.Stationary_Graph
		preview_cancel_button.show()
		sd_write_summary_file(sd_current_segment_index, section="error")
		sd_reset_experiment_controller(mode="interrupted")
		log_message("Experiments could not start due to segment_index initialisation error.")
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Initialisation",
			"The experiments could not initialise segment_index correctly."
		)
		return

	# Write segment information to summary file
	sd_data['starttime_readable'][segment_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]
	sd_write_summary_file(segment_index, section="segment_start")

	# Store start potential
	sd_data['startpot'][segment_index] = potential

	# Initialise state describing whether the system is holding at charge potential
	sd_charge_hold = False

	# Initialise state describing whether the system is self-discharging
	sd_acquiring = False

	# If initial segment
	if segment_index == 0:

		# Open output file and write header
		try:
			sd_output_file = open(sd_parameters['path_filename'], 'w', 1)
			sd_output_file.write("Segment number\tCharging/Self-discharging?\tElapsed time (s)\tSegment time (s)\tPotential (V)\tCurrent (A)\n")
		except Exception as e:
			log_message(f"Write to file failed: {e}")
			sd_stop(segment_index, interrupted=True)
			return

		# Initialise buffers for holding averaged elapsed time, potential, and current data for each segment
		sd_time_data = AverageBuffer(sd_parameters['num_samples'])
		sd_segment_time_data = AverageBuffer(sd_parameters['num_samples'])
		sd_potential_data = AverageBuffer(sd_parameters['num_samples'])
		sd_current_data = AverageBuffer(sd_parameters['num_samples'])

		# Initialise the plot area
		Legends.remove_all_legends()
		plot_frame.clear()
		plot_frame.enableAutoRange()
		plot_frame.getAxis('bottom').setTicks(None)
		plot_frame.setLabel('bottom', 'Time', units="s")
		plot_frame.setLabel('left', 'Potential', units="V")
		sd_charge_potential_plot_curve = plot_frame.plot(pen='w', name='Charging potential')
		sd_discharge_potential_plot_curve = plot_frame.plot(pen='y', name='Self-discharge potential')
		legend = pyqtgraph.LegendItem(offset=(60, 10))
		legend.setParentItem(plot_frame.plotItem)
		Legends.legends['sd'] = legend
		legend_in_use = 'sd'

		# Update progress bar style to solid green border
		sd_progress_bar.set_solid_green_style()

		# Update GUI
		sd_info_program_state_entry.setText("Measuring SD")

		# Begin calling sd_update() through periodic_update()
		state = States.Measuring_SD

		# Store experiment start time
		sd_data['experiment_start_time'] = timeit.default_timer()

	# If not initial segment
	else:
		# Refresh data buffers
		for data in [sd_time_data, sd_segment_time_data, sd_potential_data, sd_current_data]:
			data.samples = []

	# Handle OCP as charge potential
	chargepot = sd_parameters['charge_potential'][segment_index]
	if chargepot == "OCP":
		chargepot = sd_parameters['current_OCP']

	# Calculate ramp times, charging times, and segment timelengths
	if sd_parameters['ramp_rate'][segment_index] == "STEP":
		sd_parameters['ramp_time'][segment_index] = 0
	else:
		potential_to_sweep = chargepot - sd_data['startpot'][segment_index]
		sd_parameters['ramp_time'][segment_index] = abs(potential_to_sweep / sd_parameters['ramp_rate'][segment_index])

	# Calculate segment charging time
	sd_parameters['charging_time'][segment_index] = sd_parameters['ramp_time'][segment_index] + \
													sd_parameters['hold_time'][segment_index]

	# Calculate segment timelength
	sd_parameters['segment_timelength'][segment_index] = sd_parameters['acquisition_time'][segment_index] + \
														 sd_parameters['charging_time'][segment_index]

	# Determine potential to send to the DAC
	if sd_parameters['ramp_rate'][segment_index] == "STEP":
		setpot = chargepot
	else:
		setpot = sd_data['startpot'][segment_index]

	# Initialise DAC for charging
	set_output(0, setpot)
	set_control_mode(False)  # Potentiostatic control
	hardware_manual_control_range_dropdown.setCurrentIndex(
		0)  # Set as highest current range
	set_current_range()
	time.sleep(.1)  # Allow DAC some time to settle
	set_cell_status(True)  # Cell on
	time.sleep(.1)  # Allow feedback loop some time to settle
	read_potential_current()
	time.sleep(.1)
	read_potential_current()  # Two reads are necessary because each read actually returns the result of the previous conversion

	# Initialise deque to store potential history for equilibration
	sd_pot_eq_history = collections.deque()

	# Update GUI
	sd_info_segmentnum_entry.setText(f"{segment_index + 1}/{sd_parameters['num_segments']}")
	sd_info_charge_self_discharge_entry.setText("Charging")
	sd_update_progress_bar()

	skipcounter = 2  # Skip first two data points to suppress artifacts

	sd_data['starttime'][segment_index] = timeit.default_timer()

def sd_update(segment_index):
	global skipcounter, sd_charge_hold, sd_acquiring

	# Calculate total and segment time elapsed
	elapsed_time = timeit.default_timer() - sd_data['experiment_start_time']
	segment_elapsed_time = timeit.default_timer() - sd_data['starttime'][segment_index]

	# If segment has completed
	if segment_elapsed_time >= sd_parameters['segment_timelength'][segment_index]:

		# Progress to next segment
		sd_stop(segment_index, interrupted=False)
		return

	else:
		# DAC control logic
		ramp_rate = sd_parameters['ramp_rate'][segment_index]
		ramp_time = sd_parameters['ramp_time'][segment_index]
		total_charge_time = sd_parameters['charging_time'][segment_index]

		# Ramping
		if ramp_rate != "STEP" and segment_elapsed_time < ramp_time:
			sd_output = sd_sweep(segment_index, segment_elapsed_time)  # Determine potential to send to the DAC
			set_output(0, sd_output)  # Send potential to the DAC

		# Holding
		elif segment_elapsed_time < total_charge_time:
			if not sd_charge_hold:
				chargepot = sd_parameters['charge_potential'][segment_index]
				if chargepot == "OCP":
					chargepot = sd_parameters['current_OCP']
				set_output(0, chargepot)
				sd_charge_hold = True

		# Measuring self-discharge
		elif segment_elapsed_time >= total_charge_time:
			if not sd_acquiring:
				set_cell_status(False)  # Switch cell off for self-discharge measurement
				sd_info_charge_self_discharge_entry.setText("Self-discharging")
				sd_parameters['charging/self-discharging'][segment_index] = "Self-discharging"
				sd_acquiring = True

		read_potential_current()  # Read new potential and current

		if skipcounter == 0:  # Process new measurements
			sd_time_data.add_sample(elapsed_time)
			sd_segment_time_data.add_sample(segment_elapsed_time)
			sd_potential_data.add_sample(potential)
			sd_current_data.add_sample(1e-3 * current)  # Convert from mA to A
			if len(sd_time_data.samples) == 0 and len(sd_time_data.averagebuffer) > 0:  # Check if a new average was just calculated

				if segment_elapsed_time >= total_charge_time:
					charge_or_self_discharge = "Self-discharging"
				else:
					charge_or_self_discharge = "Charging"

				# Write new data to output file
				try:
					sd_output_file.write("%d\t%s\t%e\t%e\t%e\t%e\n" % (
						segment_index+1,
						charge_or_self_discharge,
						sd_time_data.averagebuffer[-1],
						sd_segment_time_data.averagebuffer[-1],
						sd_potential_data.averagebuffer[-1],
						sd_current_data.averagebuffer[-1]
					))
				except Exception as e:
					log_message(f"Write to file failed: {e}")
					sd_stop(segment_index, interrupted=True)
					return

				# Append data to lists
				sd_data['time'][segment_index].append(sd_time_data.averagebuffer[-1])
				sd_data['segment_type'][segment_index].append(charge_or_self_discharge)
				sd_data['segment_time'][segment_index].append(sd_segment_time_data.averagebuffer[-1])
				sd_data['potential'][segment_index].append(sd_potential_data.averagebuffer[-1])
				sd_data['current'][segment_index].append(sd_current_data.averagebuffer[-1])

				# Update plot
				sd_update_plot(segment_index)

				# If self-discharging
				if sd_parameters['charging/self-discharging'][segment_index] == "Self-discharging":

					# Check if potential has equilibrated
					if sd_parameters['pot_eq_tolerance'][segment_index]:
						if sd_pot_eq_bool(segment_index, segment_elapsed_time):

							# Write to summary file and progress to next segment
							sd_write_summary_file(segment_index, section="acquisition_potential_equilibrated")
							log_message("*** Potential equilibrated ***")
							sd_stop(segment_index, interrupted=False)
							return

					# Check if potential cutoff has been surpassed
					if sd_parameters['pot_cutoff'][segment_index]:
						pot_cutoff = sd_parameters['pot_cutoff'][segment_index]
						if pot_cutoff == "OCP":
							pot_cutoff = sd_parameters['current_OCP']
						if sd_potential_data.averagebuffer[-1] <= pot_cutoff:

							# Write to summary file and progress to next segment
							sd_write_summary_file(segment_index, section="acquisition_potential_cutoff_reached")
							log_message("*** Cutoff potential reached ***")
							sd_stop(segment_index, interrupted=False)
							return

		else:  # Wait until the required number of data points are skipped
			skipcounter -= 1

def sd_sweep(segment_index, segment_elapsed_time):

	ramp_time = sd_parameters['ramp_time'][segment_index]
	startpot = sd_data['startpot'][segment_index]
	chargepot = sd_parameters['charge_potential'][segment_index]
	if chargepot == "OCP":
		chargepot = sd_parameters['current_OCP']

	if segment_elapsed_time >= ramp_time:
		return chargepot
	else:
		return startpot + (chargepot - startpot) * (segment_elapsed_time / ramp_time)

def sd_pot_eq_bool(segment_index, segment_elapsed_time):

	equilibrated = False
	if segment_elapsed_time >= sd_parameters['charging_time'][segment_index]:
		sd_pot_eq_history.append((segment_elapsed_time, sd_potential_data.averagebuffer[-1]))

		# Remove data points older than pot_eq_timescale from the history deque
		while sd_pot_eq_history and (segment_elapsed_time - sd_pot_eq_history[0][0]) > sd_parameters['pot_eq_timescale'][segment_index]:
			sd_pot_eq_history.popleft()

		# Start checking if the change in potential is within tolerance after pot_eq_timescale once self-discharging
		if segment_elapsed_time >= sd_parameters['pot_eq_timescale'][segment_index] + sd_parameters['charging_time'][segment_index]:
			current_potential = sd_potential_data.averagebuffer[-1]
			if sd_pot_eq_history:
				past_time, past_potential = sd_pot_eq_history[0]

				# Check if the change in potential across pot_eq_timescale is within tolerance
				if abs(current_potential - past_potential) < sd_parameters['pot_eq_tolerance'][segment_index] * 1e-3:  # Convert mV to V
					equilibrated = True

	return equilibrated

def sd_stop(segment_index, interrupted=True):
	global state, sd_current_segment_index

	if check_state([States.Measuring_SD, States.Measuring_SD_OCP_eq, States.Measuring_SD_Delay]):

		# Save segment finish time
		sd_data['finishtime_readable'][segment_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]

		if interrupted:

			set_cell_status(False)  # Cell off
			state = States.Stationary_Graph

			# Write to summary file and close
			sd_write_summary_file(segment_index, section="interrupted")

			# Reset experiment
			sd_reset_experiment_controller(mode="interrupted")

			# Update GUI
			log_message("*** EXPERIMENTS INTERRUPTED ***")
			QtWidgets.QMessageBox.information(
				mainwidget,
				"SD experiments interrupted",
				"Oh no! SD experiments have been interrupted.\n\nGlobal experiment parameters have been reset."
			)
			preview_cancel_button.show()

		elif not interrupted:

			# Write to summary file
			sd_write_summary_file(segment_index, section="segment_end")
			log_message(f"*** Segment {segment_index + 1}/{sd_parameters['num_segments']} completed ***")

			# If not final segment
			if segment_index + 1 != sd_parameters['num_segments']:

				# Increment the current segment index
				sd_current_segment_index += 1

				# Begin the next SD segment
				sd_start(sd_current_segment_index)

			# If final segment completed
			elif segment_index + 1 == sd_parameters['num_segments']:

				set_cell_status(False)  # Cell off
				state = States.Stationary_Graph

				# Write to summary file and close
				sd_write_summary_file(segment_index, section="all_segments_completed")

				# Reset experiment
				sd_reset_experiment_controller(mode="all_segments_completed")

				# Update GUI
				sd_info_program_state_entry.setText("All segments completed")
				QtWidgets.QMessageBox.information(
					mainwidget,
					"SD experiment completed",
					"CONGRATULATIONS! All SD segments have completed successfully.\n\nGlobal experiment parameters have been reset."
				)
				preview_cancel_button.show()

def sd_reset_experiment_controller(mode):
	global sd_parameters, sd_data
	global sd_parameters_checked, sd_filenames_checked
	global sd_total_segments, sd_current_segment_index
	global sd_charge_hold, sd_acquiring

	# Stop timer
	sd_delay_timer.stop()

	if mode == "input_changed":
		if sd_parameters_checked:  # If inputs have changed since last successful check

			# Reset globals
			sd_parameters_checked = False
			sd_filenames_checked = False
			sd_variables_checkbutton.setStyleSheet("")

			# Reset progress bar
			sd_total_segments = None
			sd_current_segment_index = None
			sd_update_progress_bar()

			log_message("SD input parameters or program state has changed since the last successful check - check-state has been reset.")

		return

	elif mode == "checkbutton_failed":

		# Reset globals
		sd_parameters_checked = False
		sd_filenames_checked = False
		sd_variables_checkbutton.setStyleSheet("")

		# Reset progress bar
		sd_total_segments = None
		sd_current_segment_index = None
		sd_update_progress_bar()
		return

	# Ensure output files are closed
	for file in ('sd_output_file', 'sd_summary_file'):
		try:
			if file in globals():
				f = globals()[file]
				if f and hasattr(f, 'close'):
					f.close()
					globals()[file] = None  # Clear reference to file
		except Exception as e:
			log_message(f"Error closing {file}: {e}")

	# Reset globals
	sd_parameters_checked = False
	sd_filenames_checked = False
	sd_variables_checkbutton.setStyleSheet("")
	sd_parameters = {'type': 'sd'}
	sd_data = {}
	sd_total_segments = None
	sd_current_segment_index = None
	sd_charge_hold = False
	sd_acquiring = False

	# Reset GUI
	sd_info_segmentnum_entry.setText("-/-")
	sd_info_charge_self_discharge_entry.setText("-")

	# Unfreeze input fields
	sd_freeze_inputs(freeze=False)

	if mode == "all_segments_completed":
		sd_progress_bar.set_completed_state()

	elif mode == "interrupted":
		sd_info_program_state_entry.setText(f"Experiments interrupted")
		sd_progress_bar.set_interrupted_state()

	elif mode == "OCP_interrupted":
		sd_progress_bar.set_OCP_interrupted_state()


"""SD ACCESSORY FUNCTIONS"""

def sd_write_summary_file(segment_index, section):
	"""Write summary file for the experiment."""
	global sd_summary_file

	try:
		if section == "initial":
			sd_summary_file = open(sd_parameters['experiment_info_path_filename'], 'w', 1)
			sd_summary_file.write(f"SD EXPERIMENTS INFORMATION FILE\n*******************************\n")

			sd_summary_file.write(f"\nExperiment notes: {sd_parameters['experiment_notes']}\n")

			sd_summary_file.write("\nExperiment information file for the experiments stored in:\n")
			sd_summary_file.write(f"{sd_parameters['path_filename']}\n")
			sd_summary_file.write(f"\n")

			if sd_parameters['OCP_bool']:
				delay_str = "Wait for OCP equilibration."
			else:
				delay_str = f"{sd_parameters['delay']}"

			sd_summary_file.write("Charging parameters:\n")
			sd_summary_file.write(f"Charge potentials (V): {sd_parameters['charge_potential']}\n")
			sd_summary_file.write(f"Hold times per potential (s): {sd_parameters['hold_time']}\n")
			sd_summary_file.write(f"Ramp rate per potential (mV/s): {sd_parameters['ramp_rate_mV/s']}\n")
			sd_summary_file.write("\n")

			sd_summary_file.write("Self-discharge acquisition parameters:\n")
			sd_summary_file.write(f"Acquisition times per potential (s): {sd_parameters['acquisition_time']}\n")
			sd_summary_file.write(f"Potential cutoff thresholds per potential (V): {sd_parameters['pot_cutoff']}\n")
			sd_summary_file.write(f"Equilibration tolerance (V): {sd_parameters['pot_eq_tolerance'][0]}\n")
			sd_summary_file.write(f"Equilibration timescale (s): {sd_parameters['pot_eq_timescale'][0]}\n")
			sd_summary_file.write("\n")

			sd_summary_file.write(f"Samples to average: {sd_parameters['num_samples']}\n")
			sd_summary_file.write(f"Pre-experiment delay (s): {delay_str}\n")

		elif section == "delay":
			sd_summary_file.write(f"\n*** Pre-experiment delay of {sd_parameters['delay']} seconds ***\n")

		elif section == "segment_start":
			sd_summary_file.write("\n**********************************\n\tSD SEGMENT STARTED\n**********************************\n")
			sd_summary_file.write(f"Segment number: {segment_index + 1}/{sd_parameters['num_segments']}\n")
			sd_summary_file.write("\n")

			sd_summary_file.write("Charging parameters:\n")
			sd_summary_file.write(f"Charge potential (V): {sd_parameters['charge_potential'][segment_index]}\n")
			sd_summary_file.write(f"Hold time (s): {sd_parameters['hold_time'][segment_index]}\n")
			sd_summary_file.write(f"Ramp rate (mV/s): {sd_parameters['ramp_rate_mV/s'][segment_index]}\n")
			sd_summary_file.write("\n")

			sd_summary_file.write("Self-discharge acquisition parameters:\n")
			sd_summary_file.write(f"Potential cutoff threshold (V): {sd_parameters['pot_cutoff'][segment_index]}\n")
			sd_summary_file.write(f"Equilibration tolerance (mV): {sd_parameters['pot_eq_tolerance'][segment_index]}\n")
			sd_summary_file.write(f"Equilibration timescale (s): {sd_parameters['pot_eq_timescale'][segment_index]}\n")
			sd_summary_file.write("\n")

			sd_summary_file.write(f"Segment start time:  {sd_data['starttime_readable'][segment_index]}\n")

		elif section == "acquisition_potential_equilibrated":
			sd_summary_file.write(f"\n***** POTENTIAL EQUILIBRATED *****\n")
			sd_summary_file.write(f"Potential ({sd_parameters['pot_eq_timescale'][0]}) s ago (mV): {sd_pot_eq_history[0][1] * 1e3:.3f}\n")
			sd_summary_file.write(f"Equilibrated potential (mV): {sd_pot_eq_history[-1][1] * 1e3:.3f}\n")
			sd_summary_file.write(f"Equilibrated segment time (s): {sd_pot_eq_history[-1][0]:.3f}\n")
			sd_summary_file.write("\n")

		elif section == "acquisition_potential_cutoff_reached":
			sd_summary_file.write(f"\n***** POTENTIAL CUTOFF REACHED *****\n")
			sd_summary_file.write(f"Cutoff potential threshold (V): {sd_parameters['pot_cutoff'][segment_index]}\n")
			sd_summary_file.write(f"Last averaged potential (V): {sd_potential_data.averagebuffer[-1]:.5g}\n")
			sd_summary_file.write(f"Self-discharge elapsed time at cutoff (s): {sd_segment_time_data.averagebuffer[-1] - sd_parameters['charging_time'][segment_index]:.3f}\n")
			sd_summary_file.write("\n")

		elif section == "segment_end":
			sd_summary_file.write(f"Segment finish time: {sd_data['finishtime_readable'][segment_index]}\n")

			sd_summary_file.write("\n************************************\n\tSD SEGMENT COMPLETED\n************************************\n")

		elif section == "all_segments_completed":
			sd_summary_file.write("\n***************************************************\n\tALL SEGMENTS COMPLETED SUCCESSFULLY\n***************************************************\n")

			sd_summary_file.write("\n")
			sd_summary_file.write("********************\n")
			sd_summary_file.write("EXPERIMENTS COMPLETE\n")
			sd_summary_file.close()

		elif section == "OCP_eq_start":
			sd_summary_file.write("\n*********************************\n\tOCP EQUILIBRATING\n*********************************\n")
			sd_summary_file.write(f"Equilibration tolerance (mV): {global_software_settings['OCP_eq_tolerance']}\n")
			sd_summary_file.write(f"Equilibration timescale (s): {global_software_settings['OCP_eq_timescale']}\n")
			sd_summary_file.write(f"Equilibration timeout (s): {global_software_settings['OCP_eq_timeout']}\n")
			sd_summary_file.write("\n")

			sd_summary_file.write(f"OCP equilibration start time: {sd_data['OCP_starttime_readable'][segment_index]}\n")

		elif section == "OCP_equilibrated":
			sd_summary_file.write(f"OCP equilibration finish time: {sd_data['OCP_eq_finishtime_readable'][segment_index]}\n")
			sd_summary_file.write("\n")

			sd_summary_file.write(f"Elapsed time (s): {sd_data['OCP_eq_elapsed_times'][-1]:.3f}\n")
			sd_summary_file.write(f"Starting potential (V): {sd_data['OCP_eq_startpot'][segment_index]:.5g}\n")
			sd_summary_file.write(f"Final equilibrated OCP (V): {sd_data['OCP_eq_stoppot'][segment_index]:.5g}\n")
			sd_summary_file.write(f"Total potential difference (V): {sd_data['OCP_eq_total_pot_diff'][segment_index]:.5g}\n")
			sd_summary_file.write(f"Final ({global_software_settings['OCP_eq_timescale']}) s potential difference (V): {sd_data['OCP_eq_timescale_pot_diff'][segment_index]:.5g}\n")
			sd_summary_file.write("\n")

			sd_summary_file.write("***** OCP successfully equilibrated *****\n")

		elif section == "OCP_timeout":
			sd_summary_file.write(f"OCP equilibration timeout time: {sd_data['OCP_timeout_finishtime_readable'][segment_index]}\n")

			sd_summary_file.write("\n*******************************************\n\tSD MEASUREMENTS INTERRUPTED\n*******************************************\n")
			sd_summary_file.write("SD experiments stopped due to OCP equilibration timeout.\n")
			sd_summary_file.write("OCP did not equilibrate to within tolerance by the timeout threshold.\n")
			sd_summary_file.write("\n")

			sd_summary_file.write(f"OCP timeout threshold (s): {global_software_settings['OCP_eq_timeout']}\n")
			sd_summary_file.write(f"Elapsed time (s): {sd_data['OCP_timeout_time_elapsed'][segment_index]:.3g}\n")
			sd_summary_file.write(f"Starting potential (V): {sd_data['OCP_timeout_startpot'][segment_index]:.5g}\n")
			sd_summary_file.write(f"Final potential (V): {sd_data['OCP_timeout_stoppot'][segment_index]:.5g}\n")
			sd_summary_file.write(f"Total potential difference (V): {sd_data['OCP_timeout_total_pot_diff'][segment_index]:.5g}\n")
			sd_summary_file.write(f"Final ({global_software_settings['OCP_eq_timescale']}) s potential difference (V): {sd_data['OCP_timeout_timescale_pot_diff'][segment_index]:.5g}\n")

			sd_summary_file.write("\n")
			sd_summary_file.write("**********************\n")
			sd_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			sd_summary_file.close()

		elif section == "interrupted":
			sd_summary_file.write("\n*******************************************\n\tSD MEASUREMENTS INTERRUPTED\n*******************************************\n")
			sd_summary_file.write(f"Segment interrupted: {segment_index + 1}/{sd_parameters['num_segments']}\n")
			sd_summary_file.write(f"Experiment interruption time: {sd_data['finishtime_readable'][segment_index]}\n")
			sd_summary_file.write(f"SD segment interrupted: {sd_parameters['charge_potential'][segment_index]} V\n")

			sd_summary_file.write("\n")
			sd_summary_file.write("**********************\n")
			sd_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			sd_summary_file.close()

		elif section == "error":
			sd_summary_file.write("\n**********************************************\n\tERROR INITIALISING EXPERIMENTS\n**********************************************\n")

			sd_summary_file.write("\n")
			sd_summary_file.write("**********************\n")
			sd_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			sd_summary_file.close()

	except Exception as e:
		log_message(f"Write to summary file failed: {e}")


"""SD PLOT AND GUI FUNCTIONS"""

def sd_update_plot(segment_index):
	"""Update the plot with the current self-discharge experiment, and other experiments depending on GUI inputs."""

	# Clear the plot frame and legend
	plot_frame.clear()
	legend.clear()

	# Title for legend
	dummy_item = pyqtgraph.PlotDataItem([], [], pen=None)
	legend.addItem(dummy_item, "Self-discharge experiments:")

	# Store whether plotting charging
	plot_chg_bool = sd_plot_options_show_charging_checkbox.isChecked()

	# Data masks by Charging/Self-discharging
	segment_time = numpy.array(sd_data['segment_time'][segment_index])
	sd_indices = numpy.flatnonzero(numpy.array(sd_data['segment_type'][segment_index]) == "Self-discharging")
	chg_indices = numpy.flatnonzero(numpy.array(sd_data['segment_type'][segment_index]) == "Charging")

	# Determine where to set 0 time
	if sd_indices.size > 0:
		ref_time = segment_time[sd_indices[0]]  # At start of discharge
	else:
		ref_time = sd_parameters['charging_time'][segment_index]  # At estimated start of discharge

	# Add current segment to the top of the legend
	legend.addItem(sd_discharge_potential_plot_curve, f"Current self-discharge: {sd_parameters['charge_potential'][segment_index]}")

	# Plot all previous segments
	if sd_plot_options_all_segments_radiobutton.isChecked():
		for seg_idx, segment_types in sd_data['segment_type'].items():
			if seg_idx != segment_index:  # Skip this segment

				segment_time_prev = numpy.array(sd_data['segment_time'][seg_idx])
				sd_indices_prev = numpy.flatnonzero(numpy.array(sd_data['segment_type'][seg_idx]) == "Self-discharging")
				chg_indices_prev = numpy.flatnonzero(numpy.array(sd_data['segment_type'][seg_idx]) == "Charging")
				if sd_indices_prev.size > 0:
					ref_time_prev = segment_time_prev[sd_indices_prev[0]]
				else:
					ref_time_prev = sd_parameters['charging_time'][seg_idx]

				x_sd_prev = segment_time_prev[sd_indices_prev] - ref_time_prev
				y_sd_prev = numpy.array(sd_data['potential'][seg_idx])[sd_indices_prev]
				color = sd_parameters['plot_pen_color'][seg_idx]
				pen = pyqtgraph.mkPen(color=color)
				sd_prev_seg_potential_curve = plot_frame.plot(x_sd_prev, y_sd_prev, pen=pen)
				legend.addItem(sd_prev_seg_potential_curve, f"Previous segment: {sd_parameters['charge_potential'][seg_idx]} V")

				if plot_chg_bool:
					x_chg_prev = segment_time_prev[chg_indices_prev] - ref_time_prev
					y_chg_prev = numpy.array(sd_data['potential'][seg_idx])[chg_indices_prev]
					plot_frame.plot(x_chg_prev, y_chg_prev, pen='w')

	# Plot potential cutoff
	if sd_plot_options_pot_cutoff_checkbox.isChecked():
		pot_cutoff = sd_parameters['pot_cutoff'][segment_index]
		if pot_cutoff == "OCP":
			pot_cutoff = sd_parameters['current_OCP']
		if pot_cutoff is not None:
			line = pyqtgraph.InfiniteLine(
				pos=pot_cutoff,
				angle=0,
				pen=pyqtgraph.mkPen('y', style=QtCore.Qt.DashLine)
			)
			plot_frame.addItem(line)

	# Plot current segment over the top
	x_sd = segment_time[sd_indices] - ref_time
	y_sd = numpy.array(sd_data['potential'][segment_index])[sd_indices]
	sd_discharge_potential_plot_curve.setData(x_sd, y_sd)
	plot_frame.addItem(sd_discharge_potential_plot_curve)

	if plot_chg_bool:
		x_chg = segment_time[chg_indices] - ref_time
		y_chg = numpy.array(sd_data['potential'][segment_index])[chg_indices]
		sd_charge_potential_plot_curve.setData(x_chg, y_chg)
		plot_frame.addItem(sd_charge_potential_plot_curve)
		legend.addItem(sd_charge_potential_plot_curve, f"Charging potential")

	# Update legend for acquisition segments
	if sd_parameters['charging/self-discharging'][segment_index] == "Self-discharging":

		if sd_parameters['pot_cutoff'][segment_index]:
			legend.addItem(dummy_item, "")
			legend.addItem(dummy_item, "Current segment potential cutoff information:")
			pot_cutoff = sd_parameters['pot_cutoff'][segment_index]
			if pot_cutoff == "OCP":
				pot_cutoff = sd_parameters['current_OCP']
				pot_cutoff_str = f"{sd_parameters['current_OCP']:.3g}"
			else:
				pot_cutoff_str = f"{sd_parameters['pot_cutoff'][segment_index]}"
			pot_cutoff_diff = (y_sd[-1] - pot_cutoff)
			legend.addItem(dummy_item, f"Cutoff potential (V): {pot_cutoff_str}")
			legend.addItem(dummy_item, f"Distance to cutoff (V): {pot_cutoff_diff:.3g}")

		if sd_parameters['pot_eq_tolerance'][segment_index]:
			legend.addItem(dummy_item, "")
			legend.addItem(dummy_item, "Current segment equilibration information:")
			legend.addItem(dummy_item, f"Equilibration tolerance (mV): {sd_parameters['pot_eq_tolerance'][segment_index]}")
			legend.addItem(dummy_item, f"Equilibration timescale (s): {sd_parameters['pot_eq_timescale'][segment_index]}")
			if sd_pot_eq_history:
				if sd_pot_eq_history[-1][0] >= sd_parameters['pot_eq_timescale'][segment_index] + sd_parameters['charging_time'][segment_index]:
					legend.addItem(dummy_item, f"Potential {sd_parameters['pot_eq_timescale'][segment_index]} s ago (mV): {sd_pot_eq_history[0][1] * 1000:.3g}")
				else:
					legend.addItem(dummy_item, f"Potential {sd_pot_eq_history[-1][0] - sd_pot_eq_history[0][0]:.3f} s ago (mV): {sd_pot_eq_history[0][1] * 1000:.3g}")
				legend.addItem(dummy_item, f"Measured potential (mV): {sd_pot_eq_history[-1][1] * 1000:.3g}")
				pot_diff = (sd_pot_eq_history[-1][1] - sd_pot_eq_history[0][1])
				legend.addItem(dummy_item, f"Difference (mV): {pot_diff * 1000:.3g}")

def sd_update_progress_bar():
	"""Update the progress bar to reflect percentage of segments completed."""

	sd_progress_bar.update_progress_bar(sd_total_segments, sd_current_segment_index)
	sd_progress_bar.update()



"""_____RATE TESTING FUNCTIONS_____"""

"""RATE PARAMETER FUNCTIONS"""

def rate_checkbutton_callback():
	"""Function to control the data-validation process, called when "CHECK" button pressed."""
	global rate_parameters_checked, rate_filenames_checked, rate_total_c_rates, rate_cumulative_c_rate

	# Initialise with parameters_checked = False, filenames_checked = False, and a check button style reset
	rate_parameters_checked = False
	rate_filenames_checked = False
	rate_variables_checkbutton.setStyleSheet("")

	# Remove any previous program state entry
	rate_info_program_state_entry.setText("No experiments running")

	# Check input parameter formats
	if rate_validate_inputs():
		pass
	else:
		rate_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
		log_message("Rate-testing check button failed: Inputs are not in the correct format.")
		return False

	# Write input parameters to global dictionary
	if rate_get_parameters():
		pass
	else:
		rate_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
		log_message("Rate-testing check button failed: Could not write experiment parameters to a global dictionary.")
		return False

	# If filename provided, check if filenames are acceptable
	if rate_file_entry.text().strip() != "":
		if rate_get_filenames():
			rate_filenames_checked = True
		else:
			rate_reset_experiment_controller(mode="checkbutton_failed")  # Clear any saved global parameters
			log_message("Rate-testing check button failed: Experiment filenames could not be constructed.")
			return False

	# Give any parameter warnings
	rate_parameter_warnings()

	# Set global parameters_checked state as True
	rate_parameters_checked = True

	# Make check button green
	rate_variables_checkbutton.setStyleSheet("background-color: green; color: white;")

	# Store the number of C-rates this experiment will perform
	rate_total_c_rates = rate_parameters['total_c_rates']

	# Initialise cumulative C-rate for progress bar
	rate_cumulative_c_rate = -1

	# Update progress bar and give green border
	rate_update_progress_bar()
	rate_progress_bar.set_solid_green_style()

	log_message(f"Check button successful! Experiments are ready to run. C-rates remaining: {rate_total_c_rates}")

	return True

def rate_validate_inputs():
	"""Ensure inputs are of the correct format."""

	# Potential limits
	try:
		lbounds_str = rate_params_lbound_entry.text().strip()
		ubounds_str = rate_params_ubound_entry.text().strip()
		lbounds_list = [lbound.strip() for lbound in lbounds_str.split(",")]
		ubounds_list = [ubound.strip() for ubound in ubounds_str.split(",")]
		for i, lbound in enumerate(lbounds_list):
			if lbound.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				lbounds_list[i] = "OCP"
			else:
				lbounds_list[i] = float(lbound)
		for i, ubound in enumerate(ubounds_list):
			if ubound.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				ubounds_list[i] = "OCP"
			else:
				ubounds_list[i] = float(ubound)

		# Potential limits are the same length
		if len(lbounds_list) != len(ubounds_list):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Potential limits input",
				"Lower and upper potential limits must have the same number of inputs."
			)
			return False


	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Potential limits input",
			"Lower and upper potential limits must be numeric values or 'OCP'."
		)
		return False

	# Upper potential limit is greater than the lower limit
	for lbound, ubound in zip(lbounds_list, ubounds_list):
		if lbound != "OCP" and ubound != "OCP":
			if lbound >= ubound:
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: Potential limits input",
					"Upper potential limit must be greater than the lower potential limit for a given experiment."
				)
				return False
		elif lbound == "OCP" and ubound == "OCP":
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Potential limits input",
				"Lower and upper potential limits cannot both be set to 'OCP' for a given experiment."
			)
			return False

	# Manual 1C values input
	if not rate_params_one_c_calc_checkbox.isChecked():
		try:
			one_c_str = rate_params_one_c_entry.text().strip()
			if one_c_str == "":
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: 1C values input",
					"If not auto-calculating, a 1C value must be given for each potential window."
				)
				return False

			one_c_list = [float(one_c.strip()) for one_c in one_c_str.split(",")]
			if any(one_c <= 0 for one_c in one_c_list):
				raise ValueError

			# 1C current provided for each potential window
			if len(one_c_list) != len(lbounds_list):
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Error: 1C values input",
					"If not auto-calculating, a 1C value must be given for each potential window."
				)
				return False

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: 1C values input",
				"1C values must be positive numeric values."
			)
			return False

	# Auto-calculate 1C values
	else:
		# Current for 1C calculation
		try:
			one_c_calc_current = float(rate_one_c_calc_dropdown.rate_params_one_c_calc_current_entry.text().strip())
			if one_c_calc_current <= 0:
				raise ValueError

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: 1C calculation current input",
				"If auto-calculating 1C, a positive numeric value for charge/discharge current must be given."
			)
			return False

		# Number of cycles for 1C calculation
		try:
			one_c_calc_num_cycles = int(rate_one_c_calc_dropdown.rate_params_one_c_calc_num_cycles_entry.text().strip())
			if one_c_calc_num_cycles <= 0:
				raise ValueError

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: 1C calculation number of cycles input",
				"If auto-calculating 1C, a positive integer value for number of cycles must be given."
			)
			return False

		# Number of samples to average for 1C calculation
		try:
			one_c_calc_num_samples = int(rate_one_c_calc_dropdown.rate_params_one_c_calc_num_samples_entry.text().strip())
			if one_c_calc_num_samples <= 0:
				raise ValueError

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: 1C calculation samples to average input",
				"If auto-calculating 1C, a positive integer value for number of samples to average must be given."
			)
			return False

		# Post-1C calculation delay
		try:
			if rate_one_c_calc_dropdown.rate_params_one_c_calc_post_delay_OCP_checkbox.isChecked():
				rate_one_c_calc_dropdown.rate_params_one_c_calc_post_delay_entry.setText("")
			else:
				post_delay = float(rate_one_c_calc_dropdown.rate_params_one_c_calc_post_delay_entry.text().strip())
				if post_delay < 0:
					raise ValueError

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Post-1C calculation delay input",
				"If auto-calculating 1C, a non-negative numeric value for post-calculation delay must be given if not waiting for OCP equilibration."
			)
			return False

	# C-rates per experiment
	try:
		c_rates_str = rate_params_c_rate_entry.text().strip()
		c_rates_list = [float(c_rate.strip()) for c_rate in c_rates_str.split(",")]
		if any(c_rate <= 0 for c_rate in c_rates_list):
			raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: C-rates input",
			"C-rates must be positive numeric values."
		)
		return False

	# Cycles per C-rate
	try:
		num_cycles = int(rate_params_c_rate_num_cycles_entry.text().strip())
		if num_cycles <= 0:
			raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Cycles per C-rate input",
			"Number of cycles per C-rate must be a positive integer value."
		)
		return False

	# Pre-C-rate delay
	try:
		c_rate_delay = float(rate_params_c_rate_delay_entry.text().strip())
		if c_rate_delay < 0:
			raise ValueError

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Pre-C-rate delay input",
			"Pre-C-rate delay must be a non-negative numeric value."
		)
		return False

	# Determine if all experiments require OCP equilibration
	if rate_params_delay_OCP_checkbox.isChecked():
		OCP_eq = True
	else:
		OCP_eq = False
		for lbound, ubound in zip(lbounds_list, ubounds_list):
			if "OCP" in (lbound, ubound):
				OCP_eq = True

	# Pre-potential window delay if required
	if not OCP_eq:
		try:
			delay = float(rate_params_delay_entry.text().strip())
			if delay < 0:
				raise ValueError

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Pre-potential window delay input",
				"Pre-potential window delay must be a non-negative numeric value and given if not waiting for OCP equilibration."
			)
			return False

	else:  # Remove the delay
		rate_params_delay_OCP_checkbox.setChecked(True)
		rate_params_delay_entry.setText("")

	# Remove 1C calculation dropdown entries or 1C values if unused
	if not rate_params_one_c_calc_checkbox.isChecked():
		rate_one_c_calc_dropdown.rate_params_one_c_calc_current_entry.setText("")
		rate_one_c_calc_dropdown.rate_params_one_c_calc_num_cycles_entry.setText("")
		rate_one_c_calc_dropdown.rate_params_one_c_calc_num_samples_entry.setText("")
		rate_one_c_calc_dropdown.rate_params_one_c_calc_post_delay_entry.setText("")
		rate_one_c_calc_dropdown.rate_params_one_c_calc_post_delay_OCP_checkbox.setChecked(False)
	else:
		rate_params_one_c_entry.setText("")

	return True

def rate_get_parameters():
	"""Parse and validate GUI experiment parameters into the global rate_parameters dictionary."""
	global rate_parameters

	# Initialise parameter dictionary
	rate_parameters = {'type': 'rate'}

	try:
		# Track if OCP to be equilibrated before experiment
		OCP_bool = False

		# Potential limits
		lbounds = rate_params_lbound_entry.text().strip()
		ubounds = rate_params_ubound_entry.text().strip()
		lbounds_list = [lbound.strip() for lbound in lbounds.split(",")]
		ubounds_list = [ubound.strip() for ubound in ubounds.split(",")]
		for i, lbound in enumerate(lbounds_list):
			if lbound.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				lbounds_list[i] = "OCP"
				OCP_bool = True
			else:
				lbounds_list[i] = float(lbound)
		for i, ubound in enumerate(ubounds_list):
			if ubound.lower() == "ocp":  # Remove case-sensitivity for "OCP"
				ubounds_list[i] = "OCP"
				OCP_bool = True
			else:
				ubounds_list[i] = float(ubound)

		rate_parameters['lbound'] = lbounds_list
		rate_parameters['ubound'] = ubounds_list

		# Number of experiments
		rate_parameters['num_experiments'] = len(lbounds_list)

		# If 1C currents are to be calculated
		if rate_params_one_c_calc_checkbox.isChecked():
			rate_parameters['one_c_calc_bool'] = True
			rate_parameters['one_c_calc_completed_bool'] = [False] * rate_parameters['num_experiments']
			rate_parameters['one_c_calc_current'] = float(rate_one_c_calc_dropdown.rate_params_one_c_calc_current_entry.text().strip())
			rate_parameters['one_c_calc_num_samples'] = int(rate_one_c_calc_dropdown.rate_params_one_c_calc_num_samples_entry.text().strip())
			rate_parameters['one_c_calc_num_cycles'] = int(rate_one_c_calc_dropdown.rate_params_one_c_calc_num_cycles_entry.text().strip())
			rate_parameters['one_c_calc_num_halfcycles'] = 2 * int(rate_one_c_calc_dropdown.rate_params_one_c_calc_num_cycles_entry.text().strip())
			if rate_one_c_calc_dropdown.rate_params_one_c_calc_post_delay_OCP_checkbox.isChecked():
				rate_parameters['one_c_calc_post_delay'] = "OCP"
			else:
				rate_parameters['one_c_calc_post_delay'] = float(rate_one_c_calc_dropdown.rate_params_one_c_calc_post_delay_entry.text().strip())

			# Initialise list for storing calculated 1C values
			rate_parameters['one_c'] = [None] * rate_parameters['num_experiments']

		# If 1C currents are provided
		else:
			rate_parameters['one_c_calc_bool'] = False
			one_c_values = rate_params_one_c_entry.text().strip()
			one_c_values_list = [float(one_c_value.strip()) for one_c_value in one_c_values.split(",")]
			rate_parameters['one_c'] = one_c_values_list

		# C-rates per experiment
		c_rates = rate_params_c_rate_entry.text().strip()
		c_rates_list = [float(c_rate.strip()) for c_rate in c_rates.split(",")]
		rate_parameters['c_rates'] = c_rates_list
		rate_parameters['num_c_rates'] = len(c_rates_list)

		# Number of samples to average
		rate_parameters['num_samples'] = [max(1, int(36./c_rate)) for c_rate in rate_parameters['c_rates']]  # Set an appropriate amount of samples to average for the C-rates; results in approx. 1000 points per curve

		# Initialise dict to store calculated currents per C-rate per experiment
		rate_parameters['currents'] = defaultdict(lambda: defaultdict(float))

		# Total C-rates
		rate_parameters['total_c_rates'] = int(rate_parameters['num_c_rates'] * rate_parameters['num_experiments'])

		# Cycles per C-rate
		rate_parameters['num_cycles'] = int(rate_params_c_rate_num_cycles_entry.text().strip())
		rate_parameters['num_halfcycles'] = 2 * int(rate_params_c_rate_num_cycles_entry.text().strip())

		# Pre-C-rate delay
		rate_parameters['c_rate_delay'] = float(rate_params_c_rate_delay_entry.text().strip())

		# Pre-experiment delay
		if rate_params_delay_OCP_checkbox.isChecked():
			OCP_bool = True
		else:
			rate_parameters['pre_exp_delay'] = float(rate_params_delay_entry.text().strip())

		# OCP equilibration pre-experiment
		rate_parameters['OCP_bool'] = OCP_bool
		if OCP_bool:
			rate_parameters['OCP'] = defaultdict(float)

		# Experiment notes
		experiment_notes = rate_file_notes_entry.toPlainText()
		if not experiment_notes:
			experiment_notes = "No notes provided."
		rate_parameters['experiment_notes'] = experiment_notes

		# Plot pen colours
		rate_parameters['plot_pen_color'] = defaultdict(list)
		for i in range(len(lbounds_list)):
			rate_parameters['plot_pen_color'][i] = CB_color_cycle[i % len(CB_color_cycle)]

	except (ValueError, TypeError):
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Input error",
			"One or more parameters are in the wrong format and cannot be written to a global dictionary."
		)
		return False

	return True

def rate_parameter_warnings():
	"""Give GUI warnings for OCP values which may become invalid."""

	if "OCP" in rate_parameters['lbound'] or "OCP" in rate_parameters['ubound']:
		QtWidgets.QMessageBox.warning(
			mainwidget,
			"Warning: Possible OCP complications",
			"One or more experiments use 'OCP' as a potential limit.\n\n"
			"The open-circuit potential (OCP) can vary between experiments, which\n"
			"may cause your experiments to stop depending on your lower and upper potential limits.\n\n"
			"Please ensure that this is intentional!"
		)

	return True

def rate_get_filenames():
	"""Construct filenames for the experiments."""
	global rate_parameters

	try:
		filename_entry = str(rate_file_entry.text().strip())
		if filename_entry == "":
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: No filename provided",
				"Please provide a base filename for the experiments."
			)
			return False

		directory_path = os.path.dirname(filename_entry)
		if directory_path == "":
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: No parent directory specified",
				"The output files will be stored in the current working directory. Is this okay?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False
			directory_path = os.getcwd()
		elif not os.path.isdir(directory_path):
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: Directory does not exist",
				f"The directory {directory_path} does not exist."
			)
			return False

		if "_RT_{experiment_info_here}" in filename_entry:
			filename_entry = filename_entry.split("_RT_{experiment_info_here}")[0]
		filename = os.path.basename(filename_entry)
		if filename == "":  # If no filename given, only path
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: No filename provided",
				"Please provide a base filename for the experiments in addition to the path."
			)
			return False

		rate_parameters['directory_path'] = directory_path
		rate_parameters['base_filename'], _ = os.path.splitext(filename)

		exp_filenames = []
		exp_filenames_capacities = []
		for i in range(rate_parameters['num_experiments']):
			lbound = rate_parameters['lbound'][i]
			ubound = rate_parameters['ubound'][i]
			c_rate_list = '_'.join(f"{c_rate:g}" for c_rate in rate_parameters['c_rates'])
			exp_filename = f"{rate_parameters['base_filename']}_RT_exp{i + 1}_{lbound}_{ubound}_V_{c_rate_list}_C"
			exp_filename_capacities = f"{exp_filename}_capacities"
			exp_filenames.append(exp_filename)
			exp_filenames_capacities.append(exp_filename_capacities)

		rate_parameters['filenames'] = [name + ".dat" for name in exp_filenames]
		rate_parameters['filenames_capacities'] = [name + ".dat" for name in exp_filenames_capacities]

		rate_parameters['path_filenames'] = [os.path.join(directory_path, name) for name in rate_parameters['filenames']]
		rate_parameters['path_filenames_capacities'] = [os.path.join(directory_path, name) for name in rate_parameters['filenames_capacities']]

		rate_parameters['experiment_info_filename'] = rate_parameters['base_filename'] + "_RT_experiment_info.txt"
		rate_parameters['experiment_info_path_filename'] = os.path.join(directory_path, rate_parameters['experiment_info_filename'])

		if rate_parameters['one_c_calc_bool']:
			one_c_calc_filenames = []
			one_c_calc_filenames_capacities = []
			for i in range(rate_parameters['num_experiments']):
				lbound = rate_parameters['lbound'][i]
				ubound = rate_parameters['ubound'][i]
				c_rate_list = '_'.join(f"{c_rate:g}" for c_rate in rate_parameters['c_rates'])
				one_c_calc_filename = f"{rate_parameters['base_filename']}_RT_exp{i + 1}_1C_Calc_{lbound}_{ubound}_V_{c_rate_list}_C"
				one_c_calc_filename_capacities = f"{one_c_calc_filename}_capacities"
				one_c_calc_filenames.append(one_c_calc_filename)
				one_c_calc_filenames_capacities.append(one_c_calc_filename_capacities)

			rate_parameters['one_c_calc_filenames'] = [name + ".dat" for name in one_c_calc_filenames]
			rate_parameters['one_c_calc_filenames_capacities'] = [name + ".dat" for name in one_c_calc_filenames_capacities]

			rate_parameters['one_c_calc_path_filenames'] = [os.path.join(directory_path, name) for name in rate_parameters['one_c_calc_filenames']]
			rate_parameters['one_c_calc_path_filenames_capacities'] = [os.path.join(directory_path, name) for name in rate_parameters['one_c_calc_filenames_capacities']]

		for file in rate_parameters['path_filenames']:
			if os.path.isfile(file):
				if QtWidgets.QMessageBox.question(
					mainwidget,
					"Warning: File already exists",
					f"The output file {file} already exists.\n\n"
					f"Do you want to overwrite it and the corresponding capacities file, and any 1C calculation files?",
					QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
					QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
					return False

		info_file = rate_parameters['experiment_info_path_filename']
		if os.path.isfile(info_file):
			if QtWidgets.QMessageBox.question(
				mainwidget,
				"Warning: Results file already exists",
				f"The experiment info output file {info_file} already exists. Do you want to overwrite it?",
				QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
				QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
				return False

	except Exception as e:
		print(e)
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: File error",
			f"One or more output filepaths are not valid."
		)
		return False

	rate_file_entry.setText(os.path.join(rate_parameters['directory_path'], f"{rate_parameters['base_filename']}_RT_{{experiment_info_here}}"))

	return True

def rate_validate_filenames():
	"""Check the validity of files by creating and attempting to open them."""

	for file in rate_parameters.get('path_filenames', []):
		try:
			with open(file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {file} is not valid."
			)
			return False

	for capacities_file in rate_parameters.get('path_filenames_capacities', []):
		try:
			with open(capacities_file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {capacities_file} is not valid."
			)
			return False

	for one_c_calc_file in rate_parameters.get('one_c_calc_filenames', []):
		try:
			with open(one_c_calc_file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {one_c_calc_file} is not valid."
			)
			return False

	for one_c_calc_capacities_file in rate_parameters.get('one_c_calc_filenames_capacities', []):
		try:
			with open(one_c_calc_capacities_file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {one_c_calc_capacities_file} is not valid."
			)
			return False

	info_file = rate_parameters.get('experiment_info_path_filename')
	if info_file:
		try:
			with open(info_file, 'w', buffering=1) as tryfile:
				pass
		except IOError:
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Error: File error",
				f"The output filename or path for {info_file} is not valid."
			)
			return False

	return True

def rate_freeze_inputs(freeze):
	"""Function to freeze and unfreeze GUI inputs when experiments are running."""

	if freeze:
		rate_params_lbound_entry.setEnabled(False)
		rate_params_ubound_entry.setEnabled(False)
		rate_params_one_c_entry.setEnabled(False)
		rate_params_one_c_calc_checkbox.setEnabled(False)
		rate_params_c_rate_entry.setEnabled(False)
		rate_params_c_rate_num_cycles_entry.setEnabled(False)
		rate_params_c_rate_delay_entry.setEnabled(False)
		rate_params_delay_entry.setEnabled(False)
		rate_params_delay_OCP_checkbox.setEnabled(False)
		rate_one_c_calc_dropdown.freezeInputs(True)  # 1C calculation dropdown
		rate_file_entry.setEnabled(False)
		rate_file_notes_entry.setEnabled(False)
		rate_variables_checkbutton.setEnabled(False)
		software_globals_menu_button.setEnabled(False)

	elif not freeze:
		rate_params_lbound_entry.setEnabled(True)
		rate_params_ubound_entry.setEnabled(True)
		rate_params_one_c_entry.setEnabled(True)
		rate_params_one_c_calc_checkbox.setEnabled(True)
		rate_params_c_rate_entry.setEnabled(True)
		rate_params_c_rate_num_cycles_entry.setEnabled(True)
		rate_params_c_rate_delay_entry.setEnabled(True)
		rate_params_delay_entry.setEnabled(True)
		rate_params_delay_OCP_checkbox.setEnabled(True)
		rate_one_c_calc_dropdown.freezeInputs(False)  # 1C calculation dropdown
		rate_file_entry.setEnabled(True)
		rate_file_notes_entry.setEnabled(True)
		rate_variables_checkbutton.setEnabled(True)
		software_globals_menu_button.setEnabled(True)


"""RATE CORE FUNCTIONS"""

def rate_initialise():
	"""Initialise rate experiments."""
	global state, rate_data, rate_current_exp_index

	# Ensure parameters have been verified using the "CHECK" button
	if not rate_parameters_checked:
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Parameters not checked",
			"Click the 'CHECK' button to ensure parameters are appropriate before starting your experiments."
		)
		return False

	# Ensure filenames have been checked
	if not rate_filenames_checked:
		if rate_get_filenames():
			pass
		else:
			return False

	if check_state([States.Idle, States.Stationary_Graph]):

		# Validate filenames before experiments begin
		if rate_validate_filenames():
			pass
		else:
			rate_reset_experiment_controller(mode="interrupted")
			log_message("Rate-testing experiments could not initialise due to invalid output filename.")
			return False

		# Turn cell off if under manual control
		set_cell_status(False)

		# Freeze the input fields and hide return to live graph button
		rate_freeze_inputs(freeze=True)
		preview_cancel_button.hide()

		# Initialise experiment index
		rate_current_exp_index = 0

		# Write experiment info to summary file
		rate_write_summary_file(rate_current_exp_index, section="initial")

		# Update GUI and log
		rate_info_expnum_entry.setText(f"{rate_current_exp_index + 1}/{rate_parameters['num_experiments']}")
		rate_info_c_rate_entry.setText(f"-/{rate_parameters['num_c_rates']}")
		log_message("Starting rate-testing experiments...")

		# Initialise rate_data dictionary
		rate_data = {
			'starttime': defaultdict(float),
			'starttime_readable': defaultdict(str),
			'finishtime_readable': defaultdict(str),
			'one_c_calc_starttime': defaultdict(float),
			'one_c_calc_starttime_readable': defaultdict(str),
			'one_c_calc_finishtime_readable': defaultdict(str),

			'c_rate_delay_starttime': defaultdict(lambda: defaultdict(float)),
			'c_rate_starttime': defaultdict(lambda: defaultdict(float)),
			'c_rate_finishtime_readable': defaultdict(lambda: defaultdict(str)),

			'charges': collections.deque(maxlen=2),
			'one_c_calc_charges': collections.deque(maxlen=2),
			'final_chg_charge': defaultdict(lambda: defaultdict(float)),
			'final_dis_charge': defaultdict(lambda: defaultdict(float)),

			'one_c_calc_chg_time_data_currentcycle': [],
			'one_c_calc_chg_charge_data_currentcycle': [],
			'one_c_calc_chg_potential_data_currentcycle': [],
			'one_c_calc_dis_time_data_currentcycle': [],
			'one_c_calc_dis_charge_data_currentcycle': [],
			'one_c_calc_dis_potential_data_currentcycle': [],
		}

		# Pass to OCP equilibration controller if required
		if rate_parameters['OCP_bool']:
			OCP_initialise_data_entries(rate_data)
			OCP_equilibration_controller(rate_parameters, rate_data, rate_current_exp_index, equilibrated=False)
		else:
			delay = rate_parameters['pre_exp_delay']
			rate_write_summary_file(rate_current_exp_index, section="pre_exp_delay")
			rate_info_program_state_entry.setText(f"Delay of {rate_parameters['pre_exp_delay']} s")

			# Update progress bar style to solid yellow border
			rate_progress_bar.set_solid_yellow_style()

			# Update state
			state = States.Measuring_Rate_Delay

			# Launch the delay timer
			rate_delay_timer.start(int(delay * 1000))  # Delay input in ms

def rate_start(experiment_index):
	global state, rate_data, rate_parameters
	global rate_waiting_for_next_c_rate
	global rate_current_exp_index, rate_current_c_rate_index
	global rate_currentsetpoint, rate_one_c_calc_currentsetpoint
	global rate_time_data, rate_c_rate_time_data, rate_potential_data, rate_current_data
	global rate_one_c_calc_time_data, rate_one_c_calc_potential_data, rate_one_c_calc_current_data
	global rate_current_cyclenum, rate_current_halfcyclenum, rate_cumulative_c_rate
	global rate_one_c_calc_current_cyclenum, rate_one_c_calc_current_halfcyclenum
	global rate_output_file, rate_output_file_capacities
	global rate_one_c_calc_output_file, rate_one_c_calc_output_file_capacities
	global rate_chg_plot_scatter, rate_dis_plot_scatter, rate_one_c_calc_plot_curve, legend, legend_in_use

	# Stop the experiments if there is an issue
	if experiment_index is None:
		state = States.Stationary_Graph
		preview_cancel_button.show()
		rate_write_summary_file(rate_current_exp_index, section="error")
		rate_reset_experiment_controller(mode="interrupted")
		log_message("Experiments could not start due to experiment_index initialisation error.")
		QtWidgets.QMessageBox.critical(
			mainwidget,
			"Error: Initialisation",
			"The experiments could not initialise experiment_index correctly."
		)
		return

	# If proceeding to 1C calculation
	if rate_parameters['one_c_calc_bool'] and not rate_parameters['one_c_calc_completed_bool'][experiment_index]:

		# Write experiment information to summary file
		rate_data['one_c_calc_starttime_readable'][experiment_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]
		rate_write_summary_file(rate_current_exp_index, section="one_c_calc_start")

		# Open output files and write headers
		try:
			rate_one_c_calc_output_file = open(rate_parameters['one_c_calc_path_filenames'][experiment_index], 'w', 1)
			rate_one_c_calc_output_file.write("Cycle number\tHalf cycle number\tElapsed time (s)\tPotential (V)\tCurrent (A)\n")
			rate_one_c_calc_output_file_capacities = open(rate_parameters['one_c_calc_path_filenames_capacities'][experiment_index], 'w', 1)
			rate_one_c_calc_output_file_capacities.write("Cycle number\tCharge capacity (Ah)\tDischarge capacity (Ah)\n")
		except Exception as e:
			log_message(f"Write to file failed: {e}")
			rate_stop(experiment_index, interrupted=True)

		# Initialise buffers for holding averaged elapsed time, potential, and current data for each half cycle of the 1C calculation
		rate_one_c_calc_time_data = AverageBuffer(rate_parameters['one_c_calc_num_samples'])
		rate_one_c_calc_potential_data = AverageBuffer(rate_parameters['one_c_calc_num_samples'])
		rate_one_c_calc_current_data = AverageBuffer(rate_parameters['one_c_calc_num_samples'])

		# Initialise cyclenums
		rate_one_c_calc_current_cyclenum = 1
		rate_one_c_calc_current_halfcyclenum = 1

		# Initialise the plot area
		Legends.remove_all_legends()
		plot_frame.clear()
		plot_frame.enableAutoRange()
		plot_frame.getAxis('bottom').setTicks(None)
		plot_frame.setLabel('left', 'Potential', units='V')
		plot_frame.setLabel('bottom', 'Inserted/extracted charge', units='Ah')
		rate_one_c_calc_plot_curve = plot_frame.plot(pen='y')
		legend = pyqtgraph.LegendItem(offset=(60, 10))
		legend.setParentItem(plot_frame.plotItem)
		Legends.legends['rate_one_c_calc'] = legend
		legend_in_use = 'rate_one_c_calc'

		# Update progress bar style to dashed yellow border
		rate_progress_bar.set_dashed_yellow_style()

		# Display 1C calculation info in GUI
		rate_info_program_state_entry.setText("Calculating 1C")
		log_message(f"Calculation of 1C for experiment {experiment_index + 1}/{rate_parameters['num_experiments']} started. Saving to: {rate_parameters['one_c_calc_filenames'][experiment_index]}")

		# Initialise DAC and cell for measurements
		rate_one_c_calc_currentsetpoint = rate_parameters['one_c_calc_current'] * 1e-3  # Convert µA to mA
		hardware_manual_control_range_dropdown.setCurrentIndex(current_range_from_current(rate_one_c_calc_currentsetpoint))  # Determine the proper current range for the current setpoint
		set_current_range()  # Set new current range
		set_output(1, rate_one_c_calc_currentsetpoint)  # Set current to setpoint
		set_control_mode(True)  # Galvanostatic control
		time.sleep(.2)  # Allow DAC some time to settle
		set_cell_status(True)  # Cell on

		# Begin calling rate_one_c_calc_update() through periodic_update()
		state = States.Measuring_Rate_One_C_Calc

		# Store 1C calculation start time
		rate_data['one_c_calc_starttime'][experiment_index] = timeit.default_timer()

	else:  # Continue to start the rate-testing experiment

		# Write experiment information to summary file
		rate_data['starttime_readable'][experiment_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]
		rate_write_summary_file(rate_current_exp_index, section="experiment_start")

		# Open output files and write headers
		try:
			rate_output_file = open(rate_parameters['path_filenames'][experiment_index], 'w', 1)
			rate_output_file.write("C-rate\tCycle number\tHalf cycle number\tElapsed time (s)\tElapsed C-rate time (s)\tPotential (V)\tCurrent (A)\n")
			rate_output_file_capacities = open(rate_parameters['path_filenames_capacities'][experiment_index], 'w', 1)
			rate_output_file_capacities.write("C-rate\tCycle number\tCharge capacity (Ah)\tDischarge capacity (Ah)\n")
		except Exception as e:
			log_message(f"Write to file failed: {e}")
			rate_stop(experiment_index, interrupted=True)
			return

		# Initialise buffers for holding averaged elapsed time, potential, and current data for each half cycle
		rate_time_data = AverageBuffer(rate_parameters['num_samples'][0])
		rate_c_rate_time_data = AverageBuffer(rate_parameters['num_samples'][0])
		rate_potential_data = AverageBuffer(rate_parameters['num_samples'][0])
		rate_current_data = AverageBuffer(rate_parameters['num_samples'][0])

		# Calculate C-rate currents in µA
		rate_parameters['currents'][experiment_index] = [c_rate * rate_parameters['one_c'][experiment_index] for c_rate in rate_parameters['c_rates']]

		# Initialise current cyclenums and C-rate index
		rate_current_cyclenum = 1
		rate_current_halfcyclenum = 1
		rate_current_c_rate_index = 0

		if experiment_index == 0:
			rate_cumulative_c_rate = 0
			rate_update_progress_bar()
		else:
			rate_cumulative_c_rate += 1
			rate_update_progress_bar()

		# Initialise state describing whether waiting for next C-rate
		rate_waiting_for_next_c_rate = False

		# Initialise the plot area
		Legends.remove_all_legends()
		plot_frame.clear()
		plot_frame.enableAutoRange()
		plot_frame.getAxis('bottom').setTicks(None)
		plot_frame.setLabel('left', 'Inserted/extracted charge', units='Ah')
		if rate_plot_options_c_rate_scale_linear_radiobutton.isChecked():
			plot_frame.setLabel('bottom', 'C-rate', units=None)
		elif rate_plot_options_c_rate_scale_log_radiobutton.isChecked():
			plot_frame.setLabel('bottom', 'log\u2081\u2080 of C-rate', units=None)
		rate_chg_plot_scatter = plot_frame.plot(symbol='t1', pen=None, symbolPen='y', symbolBrush='y', name='Charge')  # Filled upward triangles
		rate_dis_plot_scatter = plot_frame.plot(symbol='t', pen=None, symbolPen='y', symbolBrush=None, name='Discharge')  # Unfilled downward triangles
		legend = pyqtgraph.LegendItem(anchor=(1, 1), offset=(-10, 10))
		legend.setParentItem(plot_frame.plotItem)
		Legends.legends['rate'] = legend
		legend_in_use = 'rate'

		# Update plot to show legend
		rate_update_plot(experiment_index)

		# Update progress bar style to solid green border
		rate_progress_bar.set_solid_green_style()

		# Display experiment info in GUI
		rate_info_program_state_entry.setText("Measuring C-rate")
		rate_info_c_rate_entry.setText(f"{rate_current_c_rate_index + 1}/{rate_parameters['num_c_rates']}")
		log_message(f"Rate-testing experiment {experiment_index + 1}/{rate_parameters['num_experiments']} started. Saving to: {rate_parameters['filenames'][experiment_index]}")

		# Initialise DAC and cell for measurements
		rate_currentsetpoint = rate_parameters['currents'][experiment_index][0] * 1e-3  # Convert µA to mA
		set_output(1, 0.0)  # Set zero current while range switching
		hardware_manual_control_range_dropdown.setCurrentIndex(current_range_from_current(rate_currentsetpoint))  # Determine the proper current range for the current setpoint
		set_current_range()  # Set new current range
		set_output(1, rate_currentsetpoint)  # Set current to setpoint
		set_control_mode(True)  # Galvanostatic control
		time.sleep(.2)  # Allow DAC some time to settle
		set_cell_status(True)  # Cell on

		# Begin calling rate_update() through periodic_update()
		state = States.Measuring_Rate

		# Store experiment and initial C-rate start time
		rate_data['starttime'][experiment_index] = timeit.default_timer()
		rate_data['c_rate_starttime'][experiment_index][rate_current_c_rate_index] = timeit.default_timer()

def rate_one_c_calc_update(experiment_index):
	"""Add a new data point to the 1C calculation in the rate-testing experiment - called regularly through periodic_update()"""
	global rate_data, rate_one_c_calc_currentsetpoint, rate_one_c_calc_current_cyclenum, rate_one_c_calc_current_halfcyclenum

	elapsed_time = timeit.default_timer() - rate_data['one_c_calc_starttime'][experiment_index]

	# If 1C calculation experiment completed
	if rate_one_c_calc_current_cyclenum > rate_parameters['one_c_calc_num_cycles']:
		rate_one_c_calc_stop(experiment_index)

	else:  # Continue 1C calculation
		read_potential_current()
		rate_one_c_calc_time_data.add_sample(elapsed_time)
		rate_one_c_calc_potential_data.add_sample(potential)
		rate_one_c_calc_current_data.add_sample(1e-3 * current)  # Convert mA to A

		lbound, ubound = rate_parameters['lbound'][experiment_index], rate_parameters['ubound'][experiment_index]
		charge_current, discharge_current = rate_parameters['one_c_calc_current'], -rate_parameters['one_c_calc_current']

		if len(rate_one_c_calc_time_data.samples) == 0 and len(rate_one_c_calc_time_data.averagebuffer) > 0:  # A new average was just calculated

			# Write data to output data file
			try:
				rate_one_c_calc_output_file.write("%d\t%d\t%e\t%e\t%e\n" % (
					rate_one_c_calc_current_cyclenum,
					rate_one_c_calc_current_halfcyclenum,
					rate_one_c_calc_time_data.averagebuffer[-1],
					rate_one_c_calc_potential_data.averagebuffer[-1],
					rate_one_c_calc_current_data.averagebuffer[-1]
				))
			except Exception as e:
				log_message(f"Write to file failed: {e}")
				rate_stop(experiment_index, interrupted=True)
				return

			# Calculate cumulative charge in Ah
			charge = numpy.abs(scipy.integrate.cumulative_trapezoid(rate_one_c_calc_current_data.averagebuffer, rate_one_c_calc_time_data.averagebuffer, initial=0.)/3600.)  # Cumulative charge in Ah

			# Store data for the current cycle
			if rate_one_c_calc_currentsetpoint > 0:  # If charging
				rate_data['one_c_calc_chg_time_data_currentcycle'].append(rate_one_c_calc_time_data.averagebuffer[-1])
				rate_data['one_c_calc_chg_charge_data_currentcycle'].append(charge[-1])
				rate_data['one_c_calc_chg_potential_data_currentcycle'].append(rate_one_c_calc_potential_data.averagebuffer[-1])
			elif rate_one_c_calc_currentsetpoint < 0:  # If discharging
				rate_data['one_c_calc_dis_time_data_currentcycle'].append(rate_one_c_calc_time_data.averagebuffer[-1])
				rate_data['one_c_calc_dis_charge_data_currentcycle'].append(charge[-1])
				rate_data['one_c_calc_dis_potential_data_currentcycle'].append(rate_one_c_calc_potential_data.averagebuffer[-1])

			# Update plot
			rate_one_c_calc_update_plot(experiment_index)

			# Handle OCP as inputs
			if lbound == "OCP":
				lbound = rate_parameters['current_OCP']
			if ubound == "OCP":
				ubound = rate_parameters['current_OCP']

			# If upper or lower bound reached - half cycle completed
			if (rate_one_c_calc_currentsetpoint > 0 and rate_one_c_calc_potential_data.averagebuffer[-1] >= ubound) or (rate_one_c_calc_currentsetpoint < 0 and rate_one_c_calc_potential_data.averagebuffer[-1] <= lbound):

				# Calculate half cycle charge and store in deque
				halfcycle_charge = numpy.abs(numpy.trapezoid(rate_one_c_calc_current_data.averagebuffer, rate_one_c_calc_time_data.averagebuffer) / 3600.)  # Charge in Ah
				rate_data['one_c_calc_charges'].append(halfcycle_charge)

				# If charge half cycle completed
				if rate_one_c_calc_currentsetpoint > 0:

					# Flip current and convert µA to mA
					rate_one_c_calc_currentsetpoint = discharge_current * 1e-3

					# Increment halfcyclenum
					rate_one_c_calc_current_halfcyclenum += 1

				# If discharge half cycle completed
				elif rate_one_c_calc_currentsetpoint < 0:

					# Flip current and convert µA to mA
					rate_one_c_calc_currentsetpoint = charge_current * 1e-3

					# Write to capacities output file
					try:
						rate_one_c_calc_output_file_capacities.write("%d\t%e\t%e\n" % (
							rate_one_c_calc_current_cyclenum,
							rate_data['one_c_calc_charges'][-2],
							rate_data['one_c_calc_charges'][-1]
						))
					except Exception as e:
						log_message(f"Write to file failed: {e}")
						rate_stop(experiment_index, interrupted=True)
						return

					# Clear 1C calculation current cycle data
					rate_data['one_c_calc_chg_time_data_currentcycle'] = []
					rate_data['one_c_calc_chg_charge_data_currentcycle'] = []
					rate_data['one_c_calc_chg_potential_data_currentcycle'] = []
					rate_data['one_c_calc_dis_time_data_currentcycle'] = []
					rate_data['one_c_calc_dis_charge_data_currentcycle'] = []
					rate_data['one_c_calc_dis_potential_data_currentcycle'] = []

					# Increment cyclenums
					rate_one_c_calc_current_cyclenum += 1
					rate_one_c_calc_current_halfcyclenum += 1

					# If final discharge half cycle, send 0 current to the DAC
					if rate_one_c_calc_current_cyclenum > rate_parameters['one_c_calc_num_cycles']:
						rate_one_c_calc_currentsetpoint = 0.0

				# Clear average buffers for next half cycle
				for data in [rate_one_c_calc_time_data, rate_one_c_calc_potential_data, rate_one_c_calc_current_data]:
					data.clear()

				# Send new current to DAC
				hardware_manual_control_range_dropdown.setCurrentIndex(current_range_from_current(rate_one_c_calc_currentsetpoint))
				set_current_range()  # Set new current range
				set_output(1, rate_one_c_calc_currentsetpoint)  # Send current to the DAC

def rate_one_c_calc_stop(experiment_index):
	"""Finish the 1C calculation for the rate-testing experiment."""
	global state, rate_one_c_calc_current_cyclenum, rate_one_c_calc_current_halfcyclenum

	if check_state([States.Measuring_Rate_One_C_Calc]):

		set_cell_status(False)  # Cell off
		state = States.Stationary_Graph
		set_output(1, 0.0)  # Send zero current to DAC

		# Close output files
		try:
			rate_one_c_calc_output_file.close()
			rate_one_c_calc_output_file_capacities.close()
		except:
			pass

		# Store calculated 1C from final discharge
		rate_parameters['one_c'][experiment_index] = rate_data['one_c_calc_charges'][-1] * 1e6  # Convert Ah to µAh

		# Update one_c_calc_completed_bool to reflect it is now calculated
		rate_parameters['one_c_calc_completed_bool'][experiment_index] = True

		# Save 1C calculation finish time and write to summary file
		rate_data['one_c_calc_finishtime_readable'][experiment_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]
		rate_write_summary_file(rate_current_exp_index, section="one_c_calc_completed")
		log_message("*** Calculation of 1C completed ***")

		# Clear cyclenums
		rate_one_c_calc_current_cyclenum = None
		rate_one_c_calc_current_halfcyclenum = None

		# Clear 1C calculation data
		rate_data['one_c_calc_charges'] = collections.deque(maxlen=2)
		rate_data['one_c_calc_chg_time_data_currentcycle'] = []
		rate_data['one_c_calc_chg_charge_data_currentcycle'] = []
		rate_data['one_c_calc_chg_potential_data_currentcycle'] = []
		rate_data['one_c_calc_dis_time_data_currentcycle'] = []
		rate_data['one_c_calc_dis_charge_data_currentcycle'] = []
		rate_data['one_c_calc_dis_potential_data_currentcycle'] = []

		# Pass back to OCP equilibration after 1C calculation if required
		one_c_calc_post_delay = rate_parameters['one_c_calc_post_delay']
		if one_c_calc_post_delay == "OCP":
			OCP_equilibration_controller(rate_parameters, rate_data, rate_current_exp_index, equilibrated=False)
		else:
			# Write post-1C-calculation delay info to summary file and update GUI
			rate_write_summary_file(experiment_index, section="one_c_calc_post_delay")
			rate_info_program_state_entry.setText(f"Post-1C calculation delay of {one_c_calc_post_delay} s")

			# Update progress bar style to solid yellow border
			rate_progress_bar.set_solid_yellow_style()

			# Update state
			state = States.Measuring_Rate_Delay

			# Launch the pre-experiment delay timer
			rate_delay_timer.start(int(one_c_calc_post_delay * 1000))  # Delay input in ms

def rate_update(experiment_index, c_rate_index):
	"""Add a new data point to the rate-testing measurement - called regularly through periodic_update()"""
	global state, rate_currentsetpoint
	global rate_current_c_rate_index, rate_cumulative_c_rate
	global rate_current_cyclenum, rate_current_halfcyclenum
	global rate_waiting_for_next_c_rate

	if rate_waiting_for_next_c_rate:

		# If finished waiting
		if timeit.default_timer() - rate_data['c_rate_delay_starttime'][experiment_index][c_rate_index] >= rate_parameters['c_rate_delay']:
			rate_waiting_for_next_c_rate = False

			# Update progress bar and GUI
			rate_cumulative_c_rate += 1
			rate_update_progress_bar()
			rate_info_program_state_entry.setText(f"Measuring C-rate")
			rate_info_c_rate_entry.setText(f"{c_rate_index + 1}/{rate_parameters['num_c_rates']}")

			# Update progress bar style to solid green border
			rate_progress_bar.set_solid_green_style()

			# Determine next C-rate current and send to DAC
			rate_currentsetpoint = rate_parameters['currents'][experiment_index][c_rate_index] * 1e-3  # Convert µA to mA
			set_output(1, 0.0)  # Set zero current while range switching
			hardware_manual_control_range_dropdown.setCurrentIndex(current_range_from_current(rate_currentsetpoint))  # Determine the proper current range for the new setpoint
			set_current_range()  # Set the new current range
			set_output(1, rate_currentsetpoint)

			# Clear data buffers and update num_samples
			for data in [rate_time_data, rate_c_rate_time_data, rate_potential_data, rate_current_data]:
				data.clear()
				data.number_of_samples_to_average = rate_parameters['num_samples'][c_rate_index]

			# Store start time
			rate_data['c_rate_starttime'][experiment_index][c_rate_index] = timeit.default_timer()

		# Still waiting, do nothing this update cycle
		else:
			return

	elapsed_time = timeit.default_timer() - rate_data['starttime'][experiment_index]
	c_rate_elapsed_time = timeit.default_timer() - rate_data['c_rate_starttime'][experiment_index][c_rate_index]

	read_potential_current()
	rate_time_data.add_sample(elapsed_time)
	rate_c_rate_time_data.add_sample(c_rate_elapsed_time)
	rate_potential_data.add_sample(potential)
	rate_current_data.add_sample(1e-3 * current)  # Convert mA to A

	if len(rate_time_data.samples) == 0 and len(rate_time_data.averagebuffer) > 0:  # A new average was just calculated

		# Write data to output data file
		try:
			rate_output_file.write("%s\t%d\t%d\t%e\t%e\t%e\t%e\n" % (
				f"{rate_parameters['c_rates'][c_rate_index]:g}",
				rate_current_cyclenum,
				rate_current_halfcyclenum,
				rate_time_data.averagebuffer[-1],
				rate_c_rate_time_data.averagebuffer[-1],
				rate_potential_data.averagebuffer[-1],
				rate_current_data.averagebuffer[-1]
			))
		except Exception as e:
			log_message(f"Write to file failed: {e}")
			rate_stop(experiment_index, interrupted=True)
			return

	# Cache lbound/ubound and handle OCP as inputs
	lbound, ubound = rate_parameters['lbound'][experiment_index], rate_parameters['ubound'][experiment_index]
	if lbound == "OCP":
		lbound = rate_parameters['OCP'][experiment_index]
	if ubound == "OCP":
		ubound = rate_parameters['OCP'][experiment_index]

	# If upper or lower bound reached - half cycle completed
	if (rate_currentsetpoint > 0 and potential >= ubound) or (rate_currentsetpoint < 0 and potential <= lbound):

		# Calculate charge and store in deque
		halfcycle_charge = numpy.abs(numpy.trapezoid(rate_current_data.averagebuffer, rate_time_data.averagebuffer) / 3600.)  # Charge in Ah
		rate_data['charges'].append(halfcycle_charge)

		# If charge half cycle completed
		if rate_currentsetpoint > 0:

			# Flip current and convert µA to mA
			rate_currentsetpoint = -rate_parameters['currents'][experiment_index][c_rate_index] * 1e-3

			# Increment halfcyclenum
			rate_current_halfcyclenum += 1

			# If final charge for this C-rate
			if rate_current_cyclenum == rate_parameters['num_cycles']:

				# Store charge from final charge cycle
				rate_data['final_chg_charge'][experiment_index][c_rate_index] = halfcycle_charge

		# If discharge half cycle completed
		elif rate_currentsetpoint < 0:

			# Flip current and convert µA to mA
			rate_currentsetpoint = rate_parameters['currents'][experiment_index][c_rate_index] * 1e-3

			# Write to capacities output file
			try:
				rate_output_file_capacities.write("%s\t%d\t%e\t%e\n" % (
					f"{rate_parameters['c_rates'][c_rate_index]:g}",
					rate_current_cyclenum,
					rate_data['charges'][-2],
					rate_data['charges'][-1]
				))
			except Exception as e:
				log_message(f"Write to file failed: {e}")
				rate_stop(experiment_index, interrupted=True)
				return

			# If final discharge half cycle for this C-rate
			if rate_current_cyclenum == rate_parameters['num_cycles']:

				# Store charge from final discharge cycle
				rate_data['final_dis_charge'][experiment_index][c_rate_index] = halfcycle_charge

				# Update plot
				rate_update_plot(experiment_index)

				# Write to summary file
				rate_data['c_rate_finishtime_readable'][experiment_index][c_rate_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]
				rate_write_summary_file(experiment_index, section="c_rate_completed")
				log_message(f"*** C-rate complete: {rate_parameters['c_rates'][c_rate_index]:g} ***")

				# If final C-rate for this experiment
				if c_rate_index + 1 == len(rate_parameters['c_rates']):
					rate_stop(experiment_index, interrupted=False)
					return

				# If progressing to new C-rate
				else:
					# Update C-rate index and reset cyclenums
					rate_current_c_rate_index += 1
					rate_current_cyclenum = 1
					rate_current_halfcyclenum = 1

					# Update plot to reflect halfcycle reset
					rate_update_plot(experiment_index)

					# Store bool, delay start time, and set zero current
					rate_waiting_for_next_c_rate = True
					rate_data['c_rate_delay_starttime'][experiment_index][rate_current_c_rate_index] = timeit.default_timer()
					set_output(1, 0.0)  # Set zero current while waiting

					# Write to summary file
					rate_write_summary_file(experiment_index, section="c_rate_delay")

					# Update GUI
					rate_info_program_state_entry.setText(f"Delay of {rate_parameters['c_rate_delay']} s for next C-rate")
					rate_info_c_rate_entry.setText(f"-/{rate_parameters['num_c_rates']}")

					# Update progress bar style to solid yellow border
					rate_progress_bar.set_solid_yellow_style()

					return

			# Increment cyclenums
			rate_current_cyclenum += 1
			rate_current_halfcyclenum += 1

		# Update plot to reflect halfcycle increment
		rate_update_plot(experiment_index)

		# Clear average buffers for next half cycle
		for data in [rate_time_data, rate_c_rate_time_data, rate_potential_data, rate_current_data]:
			data.clear()

		# Send new current to DAC
		hardware_manual_control_range_dropdown.setCurrentIndex(current_range_from_current(rate_currentsetpoint))
		set_current_range()  # Set new current range
		set_output(1, rate_currentsetpoint)  # Send current to the DAC

def rate_stop(experiment_index, interrupted=True):
	"""Finish the rate-testing experiment."""
	global state, rate_current_exp_index

	if check_state([States.Measuring_Rate, States.Measuring_Rate_One_C_Calc, States.Measuring_Rate_OCP_eq, States.Measuring_Rate_Delay]):

		set_cell_status(False)  # Cell off
		state = States.Stationary_Graph
		set_output(1, 0.0)  # Send zero current to DAC

		# Close output files
		try:
			rate_output_file.close()
			rate_output_file_capacities.close()
			rate_one_c_calc_output_file.close()
			rate_one_c_calc_output_file_capacities.close()
		except:
			pass

		# Save experiment finish time
		rate_data['finishtime_readable'][experiment_index] = datetime.now().strftime(f"%Y-%m-%d %H:%M:%S.%f")[:-3]

		if interrupted:

			# Write to summary file and close
			rate_write_summary_file(experiment_index, section="interrupted")

			# Reset experiment
			rate_reset_experiment_controller(mode="interrupted")

			# Update GUI
			log_message("*** EXPERIMENTS INTERRUPTED ***")
			QtWidgets.QMessageBox.information(
				mainwidget,
				"Rate-testing experiments interrupted",
				"Oh no! Rate-testing experiments have been interrupted.\n\nGlobal experiment parameters have been reset."
			)
			preview_cancel_button.show()

		elif not interrupted:

			# Write to summary file
			rate_write_summary_file(experiment_index, section="experiment_end")
			log_message(f"*** Experiment {experiment_index + 1}/{rate_parameters['num_experiments']} completed ***")

			# If not final experiment
			if experiment_index + 1 != rate_parameters['num_experiments']:

				# Re-initialise data for next experiment
				rate_data['charges'] = collections.deque(maxlen=2)

				# Increment the current experiment counter and update GUI
				rate_current_exp_index += 1
				rate_info_expnum_entry.setText(f"{rate_current_exp_index + 1}/{rate_parameters['num_experiments']}")
				rate_info_c_rate_entry.setText(f"-/{rate_parameters['num_c_rates']}")

				# Pass to OCP equilibration if required
				if rate_parameters['OCP_bool']:
					OCP_equilibration_controller(rate_parameters, rate_data, rate_current_exp_index, equilibrated=False)
				else:
					# Write pre-experiment delay info to summary file and update GUI
					delay = rate_parameters['pre_exp_delay']
					rate_write_summary_file(rate_current_exp_index, section="pre_exp_delay")
					rate_info_program_state_entry.setText(f"Delay of {rate_parameters['pre_exp_delay']} s")

					# Update progress bar style to solid yellow border
					rate_progress_bar.set_solid_yellow_style()

					# Update state
					state = States.Measuring_Rate_Delay

					# Launch the pre-experiment delay timer
					rate_delay_timer.start(int(delay * 1000))  # Delay input in ms

			# If final experiment completed
			elif experiment_index+1 == rate_parameters['num_experiments']:

				# Write to summary file and close
				rate_write_summary_file(experiment_index, section="all_experiments_completed")

				# Reset experiment
				rate_reset_experiment_controller(mode="all_experiments_completed")

				# Update GUI
				rate_info_program_state_entry.setText("All experiments completed")
				QtWidgets.QMessageBox.information(
					mainwidget,
					"Rate-testing experiments completed",
					"CONGRATULATIONS! All rate-testing experiments have completed successfully.\n\nGlobal experiment parameters have been reset."
				)
				preview_cancel_button.show()

def rate_reset_experiment_controller(mode):
	"""Controller for resetting global experiment data and parameters."""
	global rate_parameters, rate_data
	global rate_parameters_checked, rate_filenames_checked
	global rate_current_exp_index, rate_current_c_rate_index, rate_total_c_rates, rate_cumulative_c_rate
	global rate_current_cyclenum, rate_current_halfcyclenum
	global rate_one_c_calc_current_cyclenum, rate_one_c_calc_current_halfcyclenum
	global rate_waiting_for_next_c_rate

	# Stop timer
	rate_delay_timer.stop()

	if mode == "input_changed":
		if rate_parameters_checked:  # If inputs have changed since last successful check

			# Reset globals
			rate_parameters_checked = False
			rate_filenames_checked = False
			rate_variables_checkbutton.setStyleSheet("")

			# Reset progress bar
			rate_total_c_rates = None
			rate_cumulative_c_rate = None
			rate_update_progress_bar()

			log_message("Rate-testing input parameters or program state has changed since the last successful check - check-state has been reset.")

		return

	elif mode == "checkbutton_failed":

		# Reset progress bar
		rate_total_c_rates = None
		rate_cumulative_c_rate = None
		rate_update_progress_bar()
		return

	# Ensure output files are closed
	for file in ('rate_output_file', 'rate_output_file_capacities', 'rate_one_c_calc_output_file', 'rate_one_c_calc_output_file_capacities', 'rate_summary_file'):
		try:
			if file in globals():
				f = globals()[file]
				if f and hasattr(f, 'close'):
					f.close()
					globals()[file] = None  # Clear reference to file
		except Exception as e:
			log_message(f"Error closing {file}: {e}")

	# Reset globals
	rate_parameters_checked = False
	rate_filenames_checked = False
	rate_variables_checkbutton.setStyleSheet("")
	rate_parameters = {'type': 'rate'}
	rate_data = {}
	rate_total_c_rates = None
	rate_cumulative_c_rate = None
	rate_current_exp_index = None
	rate_current_c_rate_index = None
	rate_current_cyclenum = None
	rate_current_halfcyclenum = None
	rate_one_c_calc_current_cyclenum = None
	rate_one_c_calc_current_halfcyclenum = None
	rate_waiting_for_next_c_rate = False

	# Reset GUI
	rate_info_expnum_entry.setText("-/-")
	rate_info_c_rate_entry.setText("-/-")

	# Unfreeze input fields
	rate_freeze_inputs(freeze=False)

	if mode == "all_experiments_completed":
		rate_progress_bar.set_completed_state()

	elif mode == "interrupted":
		rate_info_program_state_entry.setText(f"Experiments interrupted")
		rate_progress_bar.set_interrupted_state()

	elif mode == "OCP_interrupted":
		rate_progress_bar.set_OCP_interrupted_state()


"""RATE ACCESSORY FUNCTIONS"""

def rate_write_summary_file(experiment_index, section):
	"""Write summary file for the experiment."""
	global rate_summary_file

	try:
		if section == "initial":
			rate_summary_file = open(rate_parameters['experiment_info_path_filename'], 'w', 1)
			rate_summary_file.write("RATE-TESTING EXPERIMENTS INFORMATION FILE\n*****************************************\n")

			rate_summary_file.write(f"\nExperiment notes: {rate_parameters['experiment_notes']}\n")

			rate_summary_file.write("\nExperiment information file for the experiments stored in:\n")
			for file in rate_parameters['path_filenames']:
				rate_summary_file.write(f"{file}\n")
			rate_summary_file.write("\n")

			rate_summary_file.write("Calculated charge/discharge capacities for each full cycle are stored in:\n")
			for capacities_file in rate_parameters['path_filenames_capacities']:
				rate_summary_file.write(f"{capacities_file}\n")
			rate_summary_file.write("\n")

			if rate_parameters['one_c_calc_bool']:
				rate_summary_file.write("1C calculation experiments stored in:\n")
				for file in rate_parameters['one_c_calc_path_filenames']:
					rate_summary_file.write(f"{file}\n")
				rate_summary_file.write("\n")

				rate_summary_file.write("1C calculation charge/discharge capacities for each full cycle are stored in:\n")
				for capacities_file in rate_parameters['one_c_calc_path_filenames_capacities']:
					rate_summary_file.write(f"{capacities_file}\n")
				rate_summary_file.write("\n")

			potential_windows = []
			for lbound, ubound in zip(rate_parameters['lbound'], rate_parameters['ubound']):
				potential_windows.append((lbound, ubound))
			potential_windows_str = ', '.join(f"[{lbound}, {ubound}]" for lbound, ubound in potential_windows)

			if rate_parameters['OCP_bool']:
				pre_exp_delay_str = "Wait for OCP equilibration."
			else:
				pre_exp_delay_str = f"{rate_parameters['pre_exp_delay']}"

			rate_summary_file.write(f"Potential windows (V): {potential_windows_str}\n")
			rate_summary_file.write(f"C-rates: {', '.join((f'{c_rate:g}' for c_rate in rate_parameters['c_rates']))}\n")

			if not rate_parameters['one_c_calc_bool']:
				rate_summary_file.write("Auto-calculating 1C currents?: No\n")
				rate_summary_file.write(f"1C per potential window (µAh): {', '.join(str(one_c_val) for one_c_val in rate_parameters['one_c'])}\n")
			else:
				if rate_parameters['one_c_calc_post_delay'] == "OCP":
					one_c_calc_post_delay_str = "Wait for OCP equilibration."
				else:
					one_c_calc_post_delay_str = f"{rate_parameters['one_c_calc_post_delay']}"

				rate_summary_file.write("Auto-calculating 1C currents?: Yes\n")
				rate_summary_file.write("\n")
				rate_summary_file.write("1C calculation parameters:\n")
				rate_summary_file.write(f"1C calculation charge/discharge current (µA): {rate_parameters['one_c_calc_current']}\n")
				rate_summary_file.write(f"1C calculation number of cycles: {rate_parameters['one_c_calc_num_cycles']}\n")
				rate_summary_file.write(f"1C calculation number of samples to average: {rate_parameters['one_c_calc_num_samples']}\n")
				rate_summary_file.write(f"1C calculation post-calculation delay (s): {one_c_calc_post_delay_str}\n")
				rate_summary_file.write("\n")

			rate_summary_file.write(f"Number of samples to average: {', '.join(str(num_sample) for num_sample in rate_parameters['num_samples'])}\n")
			rate_summary_file.write(f"Number of cycles per C-rate: {rate_parameters['num_cycles']}\n")
			rate_summary_file.write(f"Pre-experiment delay (s): {pre_exp_delay_str}\n")

		elif section == "one_c_calc_post_delay":
			rate_summary_file.write(f"\n*** Post-1C calculation delay of {rate_parameters['one_c_calc_post_delay']} seconds for experiment: {experiment_index + 1}/{rate_parameters['num_experiments']} ***\n")

		elif section == "pre_exp_delay":
			rate_summary_file.write(f"\n*** Pre-experiment delay of {rate_parameters['pre_exp_delay']} seconds for experiment: {experiment_index + 1}/{rate_parameters['num_experiments']} ***\n")

		elif section == "c_rate_delay":
			rate_summary_file.write(f"\n*** Delay of {rate_parameters['c_rate_delay']} seconds for next C-rate: {rate_parameters['c_rates'][rate_current_c_rate_index]:g} ***\n")

		elif section == "one_c_calc_start":
			rate_summary_file.write(f"\n**************************************\n\t1C CALCULATION STARTED\n**************************************\n")
			rate_summary_file.write(f"Filepath: {rate_parameters['one_c_calc_path_filenames'][experiment_index]}\n")
			rate_summary_file.write(f"Capacities filepath: {rate_parameters['one_c_calc_path_filenames_capacities'][experiment_index]}\n")
			rate_summary_file.write(f"Experiment number: {experiment_index + 1}/{rate_parameters['num_experiments']}\n")
			rate_summary_file.write("\n")

			rate_summary_file.write(f"Lower/upper potential limits (V): {rate_parameters['lbound'][experiment_index]}, {rate_parameters['ubound'][experiment_index]}\n")
			rate_summary_file.write(f"Charge/discharge current (µA): {rate_parameters['one_c_calc_current']}\n")
			rate_summary_file.write(f"Number of cycles: {rate_parameters['one_c_calc_num_cycles']}\n")
			rate_summary_file.write(f"Samples to average: {rate_parameters['one_c_calc_num_samples']}\n")
			rate_summary_file.write("\n")

			rate_summary_file.write(f"1C calculation start time: {rate_data['one_c_calc_starttime_readable'][experiment_index]}\n")

		elif section == "one_c_calc_completed":
			rate_summary_file.write(f"1C calculation finish time: {rate_data['one_c_calc_finishtime_readable'][experiment_index]}\n")
			rate_summary_file.write(f"Calculated 1C (µAh): {rate_parameters['one_c'][experiment_index]:.5g}\n")
			rate_summary_file.write("\n")

			rate_summary_file.write("***** 1C calculation completed *****\n")

		elif section == "experiment_start":
			rate_summary_file.write(f"\n***********************************************\n\tRATE-TESTING EXPERIMENT STARTED\n***********************************************\n")
			rate_summary_file.write(f"Filepath: {rate_parameters['path_filenames'][experiment_index]}\n")
			rate_summary_file.write(f"Capacities filepath: {rate_parameters['path_filenames_capacities'][experiment_index]}\n")
			rate_summary_file.write(f"Experiment number: {experiment_index + 1}/{rate_parameters['num_experiments']}\n")
			rate_summary_file.write(f"Upper/lower potential limits (V): {rate_parameters['lbound'][experiment_index]}, {rate_parameters['ubound'][experiment_index]}\n")
			rate_summary_file.write(f"1C (µAh): {rate_parameters['one_c'][experiment_index]:.5g}\n")
			rate_summary_file.write(f"C-rates: {', '.join((f'{c_rate:g}' for c_rate in rate_parameters['c_rates']))}\n")
			rate_summary_file.write(f"Currents (µA): {', '.join((f'{current:.5g}' for current in rate_parameters['currents'][experiment_index]))}\n")
			rate_summary_file.write("\n")

			rate_summary_file.write(f"Experiment start time:  {rate_data['starttime_readable'][experiment_index]}\n")

		elif section == "c_rate_completed":
			rate_summary_file.write(f"\n*** C-rate complete: {rate_parameters['c_rates'][rate_current_c_rate_index]:g} ***\n")
			rate_summary_file.write(f"Completion time: {rate_data['c_rate_finishtime_readable'][experiment_index][rate_current_c_rate_index]}\n")

		elif section == "experiment_end":
			rate_summary_file.write(f"\nExperiment finish time: {rate_data['finishtime_readable'][experiment_index]}\n")
			rate_summary_file.write("\n")

			rate_summary_file.write("*************************************************\n\tRATE-TESTING EXPERIMENT COMPLETED\n*************************************************\n")

		elif section == "all_experiments_completed":
			rate_summary_file.write("\n******************************************************\n\tALL EXPERIMENTS COMPLETED SUCCESSFULLY\n******************************************************\n")
			if rate_parameters['OCP_bool']:
				OCP_values_str = ', '.join(f"{value:.6f}" for value in rate_data['OCP_values'])
				OCP_times_str = ', '.join(f"{value:.3f}" for value in rate_data['OCP_eq_elapsed_times'])

				rate_summary_file.write("\n")
				rate_summary_file.write("*** OCP drift information ***\n")
				rate_summary_file.write(f"OCP value across experiments (V): {OCP_values_str}\n")
				rate_summary_file.write(f"OCP equilibration times (s): {OCP_times_str}\n")

			rate_summary_file.write("\n")
			rate_summary_file.write("********************\n")
			rate_summary_file.write("EXPERIMENTS COMPLETE\n")
			rate_summary_file.close()

		elif section == "OCP_eq_start":
			rate_summary_file.write(f"\n*********************************\n\tOCP EQUILIBRATING\n*********************************\n")
			rate_summary_file.write(f"OCP equilibrating for rate-testing experiment: {experiment_index + 1}/{rate_parameters['num_experiments']}\n")
			rate_summary_file.write(f"Equilibration tolerance (mV): {global_software_settings['OCP_eq_tolerance']}\n")
			rate_summary_file.write(f"Equilibration timescale (s): {global_software_settings['OCP_eq_timescale']}\n")
			rate_summary_file.write(f"Equilibration timeout (s): {global_software_settings['OCP_eq_timeout']}\n")
			rate_summary_file.write("\n")

			rate_summary_file.write(f"OCP equilibration start time:  {rate_data['OCP_starttime_readable'][experiment_index]}\n")

		elif section == "OCP_equilibrated":
			rate_summary_file.write(f"OCP equilibration finish time: {rate_data['OCP_eq_finishtime_readable'][experiment_index]}\n")
			rate_summary_file.write("\n")

			rate_summary_file.write(f"Elapsed time (s): {rate_data['OCP_eq_elapsed_times'][-1]:.3f}\n")
			rate_summary_file.write(f"Starting potential (V): {rate_data['OCP_eq_startpot'][experiment_index]:.5g}\n")
			rate_summary_file.write(f"Final equilibrated OCP (V): {rate_data['OCP_eq_stoppot'][experiment_index]:.5g}\n")
			rate_summary_file.write(f"Total potential difference (V): {rate_data['OCP_eq_total_pot_diff'][experiment_index]:.5g}\n")
			rate_summary_file.write(f"Final {global_software_settings['OCP_eq_timescale']} s potential difference (V): {rate_data['OCP_eq_timescale_pot_diff'][experiment_index]:.5g}\n")
			rate_summary_file.write("\n")

			rate_summary_file.write("***** OCP successfully equilibrated *****\n")

		elif section == "OCP_valid":
			if rate_parameters['OCP_parameters'] == []:
				rate_summary_file.write("\nParameters set as OCP for the next experiment: None\n")
			else:
				rate_summary_file.write(f"\nParameters set as OCP for the next experiment: {', '.join(rate_parameters['OCP_parameters'])}\n")

			rate_summary_file.write(f"{rate_parameters['OCP_valid_text']}\n")

		elif section == "OCP_invalid":
			rate_summary_file.write("\n****************************************************\n\tRATE-TESTING EXPERIMENTS INTERRUPTED\n****************************************************\n")
			rate_summary_file.write("Rate-testing experiments stopped due to OCP becoming incompatible with experiment parameters.\n")
			rate_summary_file.write(f"OCP (V): {rate_parameters['current_OCP']}\n")
			rate_summary_file.write("\n")

			rate_summary_file.write(f"Experiment parameters for experiment {experiment_index + 1}/{rate_parameters['num_experiments']}:\n")
			rate_summary_file.write(f"Lower potential limit (V): {rate_parameters['lbound'][experiment_index]}\n")
			rate_summary_file.write(f"Upper potential limit (V): {rate_parameters['ubound'][experiment_index]}\n")
			rate_summary_file.write("\n")

			rate_summary_file.write(f"{rate_parameters['OCP_valid_text']}\n")

			rate_summary_file.write("\n")
			rate_summary_file.write("**********************\n")
			rate_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			rate_summary_file.close()

		elif section == "OCP_equilibrated_post_1C_calc":
			rate_summary_file.write(f"OCP equilibration finish time: {rate_data['OCP_eq_finishtime_readable'][experiment_index]}\n")
			rate_summary_file.write("\n")

			rate_summary_file.write(f"Elapsed time (s): {rate_data['OCP_eq_elapsed_times'][-1]:.3f}\n")
			rate_summary_file.write(f"Starting potential (V): {rate_data['OCP_eq_startpot'][experiment_index]:.5g}\n")
			rate_summary_file.write(f"Final equilibrated OCP (V): {rate_data['OCP_eq_stoppot'][experiment_index]:.5g}\n")
			rate_summary_file.write(f"Total potential difference (V): {rate_data['OCP_eq_total_pot_diff'][experiment_index]:.5g}\n")
			rate_summary_file.write(f"Final {global_software_settings['OCP_eq_timescale']} s potential difference (V): {rate_data['OCP_eq_timescale_pot_diff'][experiment_index]:.5g}\n")
			rate_summary_file.write("\n")

			rate_summary_file.write("***** OCP successfully equilibrated *****\n")

			rate_summary_file.write("\n")
			rate_summary_file.write("*****\n")
			rate_summary_file.write("NOTE: THIS OCP VALUE IS NOT USED AS AN EXPERIMENT PARAMETER - IT IS EQUILIBRATED FOR CELL RELAXAITON PURPOSES ONLY!\n")
			rate_summary_file.write("      IF ANY POTENTIAL LIMITS ARE SET AS OCP, THEY WILL USE THE VALUE EQUILIBRATED PRE-1C CALCULATION.\n")
			rate_summary_file.write("*****\n")

		elif section == "OCP_timeout":
			rate_summary_file.write(f"OCP equilibration timeout time: {rate_data['OCP_timeout_finishtime_readable'][experiment_index]}\n")

			rate_summary_file.write("\n****************************************************\n\tRATE-TESTING EXPERIMENTS INTERRUPTED\n****************************************************\n")
			rate_summary_file.write(f"Rate-testing experiments stopped due to OCP equilibration timeout.\n")
			rate_summary_file.write(f"OCP did not equilibrate to within tolerance by the timeout threshold.\n")
			rate_summary_file.write("\n")

			rate_summary_file.write(f"OCP timeout threshold (s): {global_software_settings['OCP_eq_timeout']}\n")
			rate_summary_file.write(f"Elapsed time (s): {rate_data['OCP_timeout_time_elapsed']:.3g}\n")
			rate_summary_file.write(f"Starting potential (V): {rate_data['OCP_timeout_startpot']:.5g}\n")
			rate_summary_file.write(f"Final potential (V): {rate_data['OCP_timeout_stoppot']:.5g}\n")
			rate_summary_file.write(f"Total potential difference (V): {rate_data['OCP_timeout_total_pot_diff']:.5g}\n")
			rate_summary_file.write(f"Final {global_software_settings['OCP_eq_timescale']} s potential difference (V): {rate_data['OCP_timeout_timescale_pot_diff']:.5g}\n")

			rate_summary_file.write("\n")
			rate_summary_file.write("**********************\n")
			rate_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			rate_summary_file.close()

		elif section == "interrupted":
			rate_summary_file.write("\n****************************************************\n\tRATE-TESTING EXPERIMENTS INTERRUPTED\n****************************************************\n")
			rate_summary_file.write(f"Experiment interrupted: {experiment_index + 1}/{rate_parameters['num_experiments']}\n")
			rate_summary_file.write(f"Experiment interruption time: {rate_data['finishtime_readable'][experiment_index]}\n")
			rate_summary_file.write(f"Measurement interrupted: {rate_parameters['lbound'][experiment_index]}/{rate_parameters['ubound'][experiment_index]} V\n")
			if rate_current_c_rate_index is not None:
				rate_summary_file.write(f"C-rate interrupted: {rate_parameters['c_rates'][rate_current_c_rate_index]:g} C\n")
			rate_summary_file.write(f"Interrupted measurement data saved to: {rate_parameters['path_filenames'][experiment_index]}\n")
			rate_summary_file.write(f"Interrupted capacities data saved to: {rate_parameters['path_filenames_capacities'][experiment_index]}\n")

			rate_summary_file.write("\n")
			rate_summary_file.write("**********************\n")
			rate_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			rate_summary_file.close()

		elif section == "error":
			rate_summary_file.write("\n**********************************************\n\tERROR INITIALISING EXPERIMENTS\n**********************************************\n")

			rate_summary_file.write("\n")
			rate_summary_file.write("**********************\n")
			rate_summary_file.write("EXPERIMENTS INCOMPLETE\n")
			rate_summary_file.close()

	except Exception as e:
		log_message(f"Write to summary file failed: {e}")

def rate_OCP_valid_bool(experiment_index):
	lbound = rate_parameters['lbound'][experiment_index]
	ubound = rate_parameters['ubound'][experiment_index]
	OCP_value = rate_parameters['OCP'][experiment_index]

	if lbound == "OCP":
		if ubound <= OCP_value:
			return False, "OCP invalid: Lower potential limit must be less than the upper potential limit."

	if ubound == "OCP":
		if lbound >= OCP_value:
			return False, "OCP invalid: Upper potential limit must be greater than the lower potential limit."

	return True, "All experiment parameters are valid after OCP equilibration."


"""RATE PLOT AND GUI FUNCTIONS"""

def rate_update_plot(experiment_index):
	"""Update the plot with the inserted/extracted charge for rate-testing experiments."""

	# Clear plot frame and legend
	plot_frame.clear()
	legend.clear()
	plot_frame.getAxis('bottom').setTicks(None)

	# Title for legend
	dummy_item = pyqtgraph.PlotDataItem([], [], pen=None)
	legend.addItem(dummy_item, "Rate-testing experiments:")
	legend.addItem(dummy_item, f"Current experiment C-rate: {rate_parameters['c_rates'][rate_current_c_rate_index]} C")
	legend.addItem(dummy_item, f"Current experiment halfcycle: {rate_current_halfcyclenum}/{rate_parameters['num_halfcycles']}")
	legend.addItem(dummy_item, "")

	c_rates = rate_parameters['c_rates']
	chg_vals = rate_data['final_chg_charge'].get(experiment_index, {})
	dis_vals = rate_data['final_dis_charge'].get(experiment_index, {})

	# Determine whether to use log scale
	log_scale_bool = rate_plot_options_c_rate_scale_log_radiobutton.isChecked()

	if log_scale_bool:
		plot_frame.setLabel('bottom', 'log\u2081\u2080 of C-rate', units=None)
	else:
		plot_frame.setLabel('bottom', 'C-rate', units=None)

	def scale_c_rate(rate):
		return numpy.log10(rate) if log_scale_bool else rate

	# Calculated charging values
	chg_indices = [i for i in range(len(c_rates)) if i in chg_vals]
	chg_x_data_curr = [scale_c_rate(c_rates[i]) for i in chg_indices]
	chg_y_data_curr = [chg_vals[i] for i in chg_indices]

	# Calculated discharging values
	dis_indices = [i for i in range(len(c_rates)) if i in dis_vals]
	dis_x_data_curr = [scale_c_rate(c_rates[i]) for i in dis_indices]
	dis_y_data_curr = [dis_vals[i] for i in dis_indices]

	# Track all C-rates that have been plotted for x-ticks
	all_plotted_c_rates = set([c_rates[i] for i in chg_indices] + [c_rates[i] for i in dis_indices])

	# Add current charges to the top of the legend
	if chg_x_data_curr:
		legend.addItem(rate_chg_plot_scatter, f"Charge from current experiment: {rate_parameters['lbound'][experiment_index]}/{rate_parameters['ubound'][experiment_index]} V")

	if dis_x_data_curr:
		legend.addItem(rate_dis_plot_scatter, f"Discharge from current experiment: {rate_parameters['lbound'][experiment_index]}/{rate_parameters['ubound'][experiment_index]} V")

	# Plot data from all previous experiments
	if rate_plot_options_all_prev_experiments_radiobutton.isChecked():
		for i in range(experiment_index):
			all_plotted_c_rates.update(rate_parameters['c_rates'])

			color = rate_parameters['plot_pen_color'][i]
			pen = pyqtgraph.mkPen(color=color)

			chg_y_data_prev = [rate_data['final_chg_charge'][i].get(j, None) for j in range(len(c_rates))]
			dis_y_data_prev = [rate_data['final_dis_charge'][i].get(j, None) for j in range(len(c_rates))]

			chg_y_data_prev = [val if val is not None else numpy.nan for val in chg_y_data_prev]
			dis_y_data_prev = [val if val is not None else numpy.nan for val in dis_y_data_prev]

			x_data_prev = [scale_c_rate(rate) for rate in c_rates]

			rate_prev_exp_chg_curve = plot_frame.plot(x_data_prev, chg_y_data_prev, pen=None, symbol='t1', symbolPen=pen, symbolBrush=color)
			rate_prev_exp_dis_curve = plot_frame.plot(x_data_prev, dis_y_data_prev, pen=None, symbol='t', symbolPen=pen, symbolBrush=None)

			legend.addItem(rate_prev_exp_chg_curve, f"Charge from previous experiment: {rate_parameters['lbound'][i]}/{rate_parameters['ubound'][i]} V")
			legend.addItem(rate_prev_exp_dis_curve, f"Discharge from previous experiment: {rate_parameters['lbound'][i]}/{rate_parameters['ubound'][i]} V")

	# Plot current charges over the top
	if chg_x_data_curr:
		rate_chg_plot_scatter.setData(chg_x_data_curr, chg_y_data_curr)
		plot_frame.addItem(rate_chg_plot_scatter)
	if dis_x_data_curr:
		rate_dis_plot_scatter.setData(dis_x_data_curr, dis_y_data_curr)
		plot_frame.addItem(rate_dis_plot_scatter)

	# Set x-ticks
	xticks = sorted(all_plotted_c_rates)
	tick_labels = [(scale_c_rate(x), f"{scale_c_rate(x):g}") for x in xticks]
	plot_frame.getAxis('bottom').setTicks([tick_labels])

def rate_one_c_calc_update_plot(experiment_index):
	"""Update the plot with the current 1C calculation cycle."""

	# Clear plot frame and legend
	plot_frame.clear()
	legend.clear()

	# Plot current cycle
	x1_data = numpy.array(rate_data['one_c_calc_chg_charge_data_currentcycle'])
	y1_data = numpy.array(rate_data['one_c_calc_chg_potential_data_currentcycle'])
	x2_data = numpy.array(rate_data['one_c_calc_dis_charge_data_currentcycle'])
	y2_data = numpy.array(rate_data['one_c_calc_dis_potential_data_currentcycle'])
	rate_one_c_calc_plot_curve.setData(x1_data, y1_data)
	plot_frame.addItem(rate_one_c_calc_plot_curve)
	if x2_data.size > 0 and y2_data.size > 0:
		plot_frame.plot(x2_data, y2_data, pen='y')

	# Title for legend
	dummy_item = pyqtgraph.PlotDataItem([], [], pen=None)
	legend.addItem(dummy_item, "Calculating 1C:")
	legend.addItem(rate_one_c_calc_plot_curve, f"Calculating for experiment: {rate_parameters['lbound'][experiment_index]}/{rate_parameters['ubound'][experiment_index]} V")
	legend.addItem(dummy_item, f"Charge/Discharge current: {rate_parameters['one_c_calc_current']} µA")
	legend.addItem(dummy_item, f"Cycle: {rate_one_c_calc_current_cyclenum}/{rate_parameters['one_c_calc_num_cycles']}")

def rate_update_progress_bar():
	"""Update the progress bar to reflect percentage of C-rates completed."""

	rate_progress_bar.update_progress_bar(rate_total_c_rates, rate_cumulative_c_rate)
	rate_progress_bar.update()



"""_____OCP EQUILIBRATION FUNCTIONS_____"""

def OCP_initialise_data_entries(experiment_data):
	"""Initialise dictionaries for OCP equilibration data."""

	experiment_data['OCP_values'] = []
	experiment_data['OCP_eq_elapsed_times'] = []
	experiment_data['OCP_starttime'] = defaultdict(float)
	experiment_data['OCP_starttime_readable'] = defaultdict(str)
	experiment_data['OCP_eq_finishtime_readable'] = defaultdict(str)
	experiment_data['OCP_eq_startpot'] = defaultdict(float)
	experiment_data['OCP_eq_stoppot'] = defaultdict(float)
	experiment_data['OCP_eq_total_pot_diff'] = defaultdict(float)
	experiment_data['OCP_eq_timescale_pot_diff'] = defaultdict(float)

def OCP_equilibration_controller(experiment_parameters, experiment_data, experiment_index, equilibrated=False):
	"""Controller to handle OCP equilibration."""
	global state, skipcounter
	global OCP_time_data, OCP_potential_data, OCP_eq_history
	global OCP_potential_plot_curve, legend, legend_in_use
	global cv_total_time, cv_remaining_time
	global lsv_total_time, lsv_remaining_time

	param_type = experiment_parameters.get("type")
	param_map = {
		"cv": ("CV", cv_reset_experiment_controller, cv_write_summary_file, cv_info_program_state_entry, cv_progress_bar),
		"lsv": ("LSV", lsv_reset_experiment_controller, lsv_write_summary_file, lsv_info_program_state_entry, lsv_progress_bar),
		"gcd": ("GCD", gcd_reset_experiment_controller, gcd_write_summary_file, gcd_info_program_state_entry, gcd_progress_bar),
		"ca": ("CA", ca_reset_experiment_controller, ca_write_summary_file, ca_info_program_state_entry, ca_progress_bar),
		"cp": ("CP", cp_reset_experiment_controller, cp_write_summary_file, cp_info_program_state_entry, cp_progress_bar),
		"sd": ("SD", sd_reset_experiment_controller, sd_write_summary_file, sd_info_program_state_entry, sd_progress_bar),
		"rate": ("Rate-testing", rate_reset_experiment_controller, rate_write_summary_file, rate_info_program_state_entry, rate_progress_bar),
	}
	exp_str, reset_experiment_controller, write_summary_file, info_program_state_entry, progress_bar = param_map[param_type]

	if equilibrated is False:

		# Initialise equilibration data buffers
		OCP_time_data = AverageBuffer(global_software_settings['OCP_eq_num_samples'])
		OCP_potential_data = AverageBuffer(global_software_settings['OCP_eq_num_samples'])
		OCP_eq_history = collections.deque()

		# Initialise the plot area
		Legends.remove_all_legends()
		plot_frame.clear()
		plot_frame.enableAutoRange()
		plot_frame.getAxis('bottom').setTicks(None)
		plot_frame.setLabel('bottom', 'Time', units="s")
		plot_frame.setLabel('left', 'Potential', units="V")
		OCP_potential_plot_curve = plot_frame.plot(pen='y')
		legend = pyqtgraph.LegendItem(offset=(60, 30))
		legend.setParentItem(plot_frame.plotItem)
		Legends.legends['OCP_eq'] = legend
		legend_in_use = 'OCP_eq'

		# Write to summary file, update GUI, and set progress bar to dashed yellow border
		experiment_data['OCP_starttime_readable'][experiment_index] = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
		write_summary_file(experiment_index, section="OCP_eq_start")
		info_program_state_entry.setText("Equilibrating OCP")
		progress_bar.set_dashed_yellow_style()

		if param_type == "cv":
			state = States.Measuring_CV_OCP_eq
		elif param_type == "lsv":
			state = States.Measuring_LSV_OCP_eq
		elif param_type == "gcd":
			state = States.Measuring_GCD_OCP_eq
		elif param_type == "ca":
			state = States.Measuring_CA_OCP_eq
		elif param_type == "cp":
			state = States.Measuring_CP_OCP_eq
		elif param_type == "sd":
			state = States.Measuring_SD_OCP_eq
		elif param_type == "rate":
			state = States.Measuring_Rate_OCP_eq

		set_cell_status(False)  # Turn cell off for equilibration
		skipcounter = 2  # Skip first two data points to suppress artifacts

		# Store OCP equilibration start time
		experiment_data['OCP_starttime'][experiment_index] = timeit.default_timer()


	elif equilibrated == "OCP_equilibrated":

		# Store equilibrated OCP and equilibration data
		OCP_value = OCP_potential_data.averagebuffer[-1]
		experiment_parameters['current_OCP'] = OCP_value
		experiment_data['OCP_values'].append(OCP_value)
		experiment_data['OCP_eq_elapsed_times'].append(OCP_time_data.averagebuffer[-1])

		experiment_data['OCP_eq_finishtime_readable'][experiment_index] = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
		experiment_data['OCP_eq_startpot'][experiment_index] = OCP_potential_data.averagebuffer[0]
		experiment_data['OCP_eq_stoppot'][experiment_index] = OCP_potential_data.averagebuffer[-1]
		experiment_data['OCP_eq_total_pot_diff'][experiment_index] = abs(OCP_potential_data.averagebuffer[-1] - OCP_potential_data.averagebuffer[0])
		experiment_data['OCP_eq_timescale_pot_diff'][experiment_index] = abs(OCP_eq_history[-1][1] - OCP_eq_history[0][1])

		if param_type == "cv":

			# Determine potential window index and update GUI if parameters set to OCP
			pot_window_index = experiment_index // len(cv_parameters['unique_scanrates_mV/s'])
			lbound_list = [lbound.strip() for lbound in cv_params_lbound_entry.text().strip().split(',')]
			ubound_list = [ubound.strip() for ubound in cv_params_ubound_entry.text().strip().split(',')]
			startpot_list = [startpot.strip() for startpot in cv_params_startpot_entry.text().strip().split(',')]
			stoppot_list = [stoppot.strip() for stoppot in cv_params_stoppot_entry.text().strip().split(',')]
			OCP_parameters = []  # List to store parameters set to OCP in these experiments

			if cv_parameters['lbound'][experiment_index] == "OCP":
				OCP_parameters.append("Lower potential limit")
				lbound_list[pot_window_index] = f"{OCP_value:.3g}"
				cv_params_lbound_entry.setEnabled(True)
				cv_params_lbound_entry.blockSignals(True)
				cv_params_lbound_entry.setText(f"{', '.join(str(val) for val in lbound_list)}")
				cv_params_lbound_entry.blockSignals(False)
				cv_params_lbound_entry.setEnabled(False)

			if cv_parameters['ubound'] == "OCP":
				OCP_parameters.append("Upper potential limit")
				ubound_list[pot_window_index] = f"{OCP_value:.3g}"
				cv_params_ubound_entry.setEnabled(True)
				cv_params_ubound_entry.blockSignals(True)
				cv_params_ubound_entry.setText(f"{', '.join(str(val) for val in ubound_list)}")
				cv_params_ubound_entry.blockSignals(False)
				cv_params_ubound_entry.setEnabled(False)

			if cv_parameters['startpot'][experiment_index] == "OCP":
				OCP_parameters.append("Start potential")
				startpot_list[pot_window_index] = f"{OCP_value:.3g}"
				cv_params_startpot_entry.setEnabled(True)
				cv_params_startpot_entry.blockSignals(True)
				cv_params_startpot_entry.setText(f"{', '.join(str(val) for val in startpot_list)}")
				cv_params_startpot_entry.blockSignals(False)
				cv_params_startpot_entry.setEnabled(False)

			if cv_parameters['stoppot'][experiment_index] == "OCP":
				OCP_parameters.append("Stop potential")
				stoppot_list[pot_window_index] = f"{OCP_value:.3g}"
				cv_params_stoppot_entry.setEnabled(True)
				cv_params_stoppot_entry.blockSignals(True)
				cv_params_stoppot_entry.setText(f"{', '.join(str(val) for val in stoppot_list)}")
				cv_params_stoppot_entry.blockSignals(False)
				cv_params_stoppot_entry.setEnabled(False)

			# Update total and remaining time based on how long the OCP equilibration took
			if cv_total_time is not None and cv_remaining_time is not None:
				if len(cv_data['OCP_eq_elapsed_times']) == 1:  # If the previous calculation was based on global_software_settings['OCP_eq_timescale']
					cv_total_time += (OCP_time_data.averagebuffer[-1] - global_software_settings['OCP_eq_timescale'])
					cv_remaining_time += (OCP_time_data.averagebuffer[-1] - global_software_settings['OCP_eq_timescale'])
				elif len(cv_data['OCP_eq_elapsed_times']) > 1:  # If the previous calculation was based on the average of OCP equilibration times so far
					calculated_average = sum(cv_data['OCP_eq_elapsed_times']) / len(cv_data['OCP_eq_elapsed_times'])
					cv_total_time += (OCP_time_data.averagebuffer[-1] - calculated_average)
					cv_remaining_time += (OCP_time_data.averagebuffer[-1] - calculated_average)

			# Write to summary file
			cv_parameters['OCP_parameters'] = OCP_parameters
			cv_write_summary_file(experiment_index, section="OCP_equilibrated")

			# Check if OCP is valid for the experiment parameters at this potential window
			OCP_valid_bool, OCP_valid_text = cv_OCP_valid_bool(experiment_index, write_errors=True, write_warnings=True)
			cv_parameters['OCP_valid_text'] = OCP_valid_text

			if OCP_valid_bool:

				# Check if OCP valid for future experiments
				num_scanrates = len(cv_parameters['unique_scanrates_mV/s'])
				num_experiments = cv_parameters['num_experiments']
				future_pot_window_indexes = [i * num_scanrates for i in range(pot_window_index + 1, num_experiments // num_scanrates)]
				future_OCP_valid_bool = True  # Initialise as True
				for idx in future_pot_window_indexes:
					OCP_valid_bool, _ = cv_OCP_valid_bool(idx, write_errors=False, write_warnings=False)
					if not OCP_valid_bool:
						future_OCP_valid_bool = False

				if future_OCP_valid_bool:
					cv_parameters['future_OCP_valid_bool'] = True
				else:
					cv_parameters['future_OCP_valid_bool'] = False
					log_message("WARNING: Equilibrated OCP is invalid with future experiments after this potential window.")

				# Write to summary file
				cv_write_summary_file(experiment_index, section="OCP_valid")

				# Begin the next experiment
				cv_start(experiment_index)

			else:
				state = States.Stationary_Graph

				# Write to summary file
				cv_write_summary_file(experiment_index, section="OCP_invalid")

				# Construct error string
				error_str = ""
				for err_str in cv_parameters['OCP_invalid_strings']:
					error_str += f"{err_str}\n"

				# Reset experiment
				cv_reset_experiment_controller(mode="OCP_interrupted")

				# Update GUI
				cv_info_program_state_entry.setText("Experiments interrupted")
				log_message("*** EXPERIMENTS INTERRUPTED DUE TO OCP INCOMPATIBILITY ***")
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"OCP error: CV experiments stopped",
					f"Oh no! CV experiments stopped due to OCP becoming incompatible with input parameters.\n\n"
					f"{OCP_valid_text}\n"
					f"{error_str}"
				)
				preview_cancel_button.show()

		elif param_type == "lsv":

			# Determine potential window index and update GUI if parameters set to OCP
			pot_window_index = experiment_index // len(lsv_parameters['unique_scanrates_mV/s'])
			startpot_list = [startpot.strip() for startpot in lsv_params_startpot_entry.text().strip().split(',')]
			stoppot_list = [stoppot.strip() for stoppot in lsv_params_stoppot_entry.text().strip().split(',')]

			if lsv_parameters['startpot'][experiment_index] == "OCP":
				startpot_list[pot_window_index] = f"{OCP_value:.3g}"
				lsv_params_startpot_entry.setEnabled(True)
				lsv_params_startpot_entry.blockSignals(True)
				lsv_params_startpot_entry.setText(f"{', '.join(str(val) for val in startpot_list)}")
				lsv_params_startpot_entry.blockSignals(False)
				lsv_params_startpot_entry.setEnabled(False)

			if lsv_parameters['stoppot'][experiment_index] == "OCP":
				stoppot_list[pot_window_index] = f"{OCP_value:.3g}"
				lsv_params_stoppot_entry.setEnabled(True)
				lsv_params_stoppot_entry.blockSignals(True)
				lsv_params_stoppot_entry.setText(f"{', '.join(str(val) for val in stoppot_list)}")
				lsv_params_stoppot_entry.blockSignals(False)
				lsv_params_stoppot_entry.setEnabled(False)

			# Update total and remaining time based on how long OCP equilibration took
			if len(lsv_data['OCP_eq_elapsed_times']) == 1:  # If the previous calculation was based on global_software_settings['OCP_eq_timescale']
				lsv_total_time += (OCP_time_data.averagebuffer[-1] - global_software_settings['OCP_eq_timescale'])
				lsv_remaining_time += (OCP_time_data.averagebuffer[-1] - global_software_settings['OCP_eq_timescale'])
			else:  # If the previous calculation was based on the average of OCP equilibration times so far
				calculated_average = sum(lsv_data['OCP_eq_elapsed_times']) / len(lsv_data['OCP_eq_elapsed_times'])
				lsv_total_time += (OCP_time_data.averagebuffer[-1] - calculated_average)
				lsv_remaining_time += (OCP_time_data.averagebuffer[-1] - calculated_average)

			# Write to summary file
			lsv_write_summary_file(experiment_index, section="OCP_equilibrated")

			# Begin the next experiment
			lsv_start(experiment_index)

		elif param_type == "gcd":

			# Determine potential window index and update GUI if parameters set to OCP
			pot_window_index = experiment_index // len(gcd_parameters['unique_charge_currents'])
			lbound_list = [lbound.strip() for lbound in gcd_params_lbound_entry.text().strip().split(',')]
			ubound_list = [lbound.strip() for lbound in gcd_params_lbound_entry.text().strip().split(',')]
			OCP_parameters = []  # List to store parameters set to OCP in these experiments

			if gcd_parameters['lbound'][experiment_index] == "OCP":
				OCP_parameters.append("Lower potential limit")
				lbound_list[pot_window_index] = f"{OCP_value:.3g}"
				gcd_params_lbound_entry.setEnabled(True)
				gcd_params_lbound_entry.blockSignals(True)
				gcd_params_lbound_entry.setText(f"{', '.join(str(val) for val in lbound_list)}")
				gcd_params_lbound_entry.blockSignals(False)
				gcd_params_lbound_entry.setEnabled(False)

			if gcd_parameters['ubound'][experiment_index] == "OCP":
				OCP_parameters.append("Upper potential limit")
				ubound_list[pot_window_index] = f"{OCP_value:.3g}"
				gcd_params_ubound_entry.setEnabled(True)
				gcd_params_ubound_entry.blockSignals(True)
				gcd_params_ubound_entry.setText(f"{', '.join(str(val) for val in ubound_list)}")
				gcd_params_ubound_entry.blockSignals(False)
				gcd_params_ubound_entry.setEnabled(False)

			# Write to summary file
			gcd_parameters['OCP_parameters'] = OCP_parameters
			gcd_write_summary_file(experiment_index, section="OCP_equilibrated")

			# Check if OCP is valid for the experiment parameters at this potential window
			OCP_valid_bool, OCP_valid_text = gcd_OCP_valid_bool(experiment_index)
			gcd_parameters['OCP_valid_text'] = OCP_valid_text

			if OCP_valid_bool:
				# Write to summary file
				gcd_write_summary_file(experiment_index, section="OCP_valid")

				# Begin the next experiment
				gcd_start(experiment_index)

			else:
				state = States.Stationary_Graph

				# Write to summary file
				gcd_write_summary_file(experiment_index, section="OCP_invalid")

				# Reset experiment
				gcd_reset_experiment_controller(mode="OCP_interrupted")

				# Update GUI
				gcd_info_program_state_entry.setText("Experiments interrupted")
				log_message("*** EXPERIMENTS INTERRUPTED DUE TO OCP INCOMPATIBILITY ***")
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"OCP error: GCD experiments stopped",
					f"Oh no! GCD experiments stopped due to OCP becoming incompatible with input parameters.\n\n"
					f"OCP invalid: {OCP_valid_text}"
				)
				preview_cancel_button.show()

		elif param_type == "ca":

			# Update GUI if parameters set to OCP
			if "OCP" in ca_parameters['potential_sequence_A']:
				pot_sequence_A = [f"{OCP_value:.3g}" if pot_A == "OCP" else str(pot_A) for pot_A in ca_parameters['potential_sequence_A']]
				ca_params_A_potential_entry.setEnabled(True)
				ca_params_A_potential_entry.blockSignals(True)
				ca_params_A_potential_entry.setText(f"{', '.join(pot_sequence_A)}")
				ca_params_A_potential_entry.blockSignals(False)
				ca_params_A_potential_entry.setEnabled(False)

			if ca_parameters['alternating_params_bool'] and "OCP" in ca_parameters['potential_sequence_B']:
				pot_sequence_B = [f"{OCP_value:.3g}" if pot_B == "OCP" else str(pot_B) for pot_B in ca_parameters['potential_sequence_B']]
				ca_alternating_parameters_dropdown.ca_params_B_potential_entry.setEnabled(True)
				ca_alternating_parameters_dropdown.ca_params_B_potential_entry.blockSignals(True)
				ca_alternating_parameters_dropdown.ca_params_B_potential_entry.setText(f"{', '.join(pot_sequence_B)}")
				ca_alternating_parameters_dropdown.ca_params_B_potential_entry.blockSignals(False)
				ca_alternating_parameters_dropdown.ca_params_B_potential_entry.setEnabled(False)

			# Write to summary file
			ca_write_summary_file(experiment_index, section="OCP_equilibrated")

			# Start the experiment
			ca_start(experiment_index)

		elif param_type == "cp":

			# Update GUI if parameters set to OCP
			if cp_parameters['pot_limit_lower'] == "OCP":
				cp_params_pot_limits_lower_entry.setEnabled(True)
				cp_params_pot_limits_lower_entry.blockSignals(True)
				cp_params_pot_limits_lower_entry.setText(f"{OCP_value:.3g}")
				cp_params_pot_limits_lower_entry.blockSignals(False)
				cp_params_pot_limits_lower_entry.setEnabled(False)

			if cp_parameters['pot_limit_upper'] == "OCP":
				cp_params_pot_limits_upper_entry.setEnabled(True)
				cp_params_pot_limits_upper_entry.blockSignals(True)
				cp_params_pot_limits_upper_entry.setText(f"{OCP_value:.3g}")
				cp_params_pot_limits_upper_entry.blockSignals(False)
				cp_params_pot_limits_upper_entry.setEnabled(False)

			# Write to summary file
			cp_write_summary_file(experiment_index, section="OCP_equilibrated")

			# Check if OCP is valid for the experiment parameters
			OCP_valid_bool, OCP_valid_text = cp_OCP_valid_bool(experiment_index)
			cp_parameters['OCP_valid_text'] = OCP_valid_text

			if OCP_valid_bool:
				# Write to summary file
				cp_write_summary_file(experiment_index, section="OCP_valid")

				# Start the experiment
				cp_start(experiment_index)

			else:
				state = States.Stationary_Graph

				# Write to summary file
				cp_write_summary_file(experiment_index, section="OCP_invalid")

				# Reset experiment
				cp_reset_experiment_controller(mode="OCP_interrupted")

				# Update GUI
				cp_info_program_state_entry.setText("Experiment interrupted")
				log_message("*** EXPERIMENTS INTERRUPTED DUE TO OCP INCOMPATIBILITY ***")
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"OCP error: CP experiments stopped",
					f"Oh no! CP experiments stopped due to OCP becoming incompatible with input parameters.\n\n"
					f"OCP invalid: {OCP_valid_text}"
				)
				preview_cancel_button.show()

		elif param_type == "sd":

			# Update GUI if parameters set to OCP
			if "OCP" in sd_parameters['charge_potential']:
				charge_pot_list = [f"{OCP_value:.3g}" if charge_pot == "OCP" else str(charge_pot) for charge_pot in sd_parameters['charge_potential']]
				sd_params_charge_potential_entry.setEnabled(True)
				sd_params_charge_potential_entry.blockSignals(True)
				sd_params_charge_potential_entry.setText(f"{', '.join(charge_pot_list)}")
				sd_params_charge_potential_entry.blockSignals(False)
				sd_params_charge_potential_entry.setEnabled(False)

			if "OCP" in sd_parameters['pot_cutoff']:
				pot_cutoff_list = [f"{OCP_value:.3g}" if pot_cutoff == "OCP" else "None" if pot_cutoff is None else str(pot_cutoff) for pot_cutoff in sd_parameters['pot_cutoff']]
				sd_params_acquisition_cutoff_entry.setEnabled(True)
				sd_params_acquisition_cutoff_entry.blockSignals(True)
				sd_params_acquisition_cutoff_entry.setText(f"{', '.join(pot_cutoff_list)}")
				sd_params_acquisition_cutoff_entry.blockSignals(False)
				sd_params_acquisition_cutoff_entry.setEnabled(False)

			# Write to summary file
			sd_write_summary_file(experiment_index, section="OCP_equilibrated")

			# Start the experiment
			sd_start(experiment_index)

		elif param_type == "rate":

			# If 1C calculation not performed or pre-1C calculation
			if not rate_parameters['one_c_calc_bool'] or (rate_parameters['one_c_calc_bool'] and not rate_parameters['one_c_calc_completed_bool'][experiment_index]):

				# Store OCP value
				rate_parameters['OCP'][experiment_index] = OCP_value

				# Update GUI if parameters set to OCP
				lbound_list = [lbound.strip() for lbound in rate_params_lbound_entry.text().strip().split(',')]
				ubound_list = [ubound.strip() for ubound in rate_params_ubound_entry.text().strip().split(',')]
				OCP_parameters = []  # List to store parameters set to OCP in these experiments

				if rate_parameters['lbound'][experiment_index] == "OCP":
					OCP_parameters.append("Lower potential limit")
					lbound_list[experiment_index] = f"{OCP_value:.3g}"
					rate_params_lbound_entry.setEnabled(True)
					rate_params_lbound_entry.blockSignals(True)
					rate_params_lbound_entry.setText(f"{', '.join(str(val) for val in lbound_list)}")
					rate_params_lbound_entry.blockSignals(False)
					rate_params_lbound_entry.setEnabled(False)

				if rate_parameters['ubound'][experiment_index] == "OCP":
					OCP_parameters.append("Upper potential_limit")
					ubound_list[experiment_index] = f"{OCP_value:.3g}"
					rate_params_ubound_entry.setEnabled(True)
					rate_params_ubound_entry.blockSignals(True)
					rate_params_ubound_entry.setText(f"{', '.join(str(val) for val in ubound_list)}")
					rate_params_ubound_entry.blockSignals(False)
					rate_params_ubound_entry.setEnabled(False)

				# Write to summary file
				rate_parameters['OCP_parameters'] = OCP_parameters
				rate_write_summary_file(experiment_index, section="OCP_equilibrated")

				# Check if OCP is valid for the experiment parameters at this potential window
				OCP_valid_bool, OCP_valid_text = rate_OCP_valid_bool(experiment_index)
				rate_parameters['OCP_valid_text'] = OCP_valid_text

				if OCP_valid_bool:
					# Write to summary file
					rate_write_summary_file(experiment_index, section="OCP_valid")

					# Begin the next experiment
					rate_start(experiment_index)

				else:
					state = States.Stationary_Graph

					# Write to summary file
					rate_write_summary_file(experiment_index, section="OCP_invalid")

					# Reset experiment
					rate_reset_experiment_controller(mode="OCP_interrupted")

					# Update GUI
					rate_info_program_state_entry.setText("Experiments interrupted")
					log_message("*** EXPERIMENTS INTERRUPTED DUE TO OCP INCOMPATIBILITY ***")
					QtWidgets.QMessageBox.critical(
						mainwidget,
						"OCP error: Rate-testing experiments stopped",
						f"Oh no! Rate-testing experiments stopped due to OCP becoming incompatible with input parameters.\n\n"
						f"OCP invalid: {OCP_valid_text}"
					)
					preview_cancel_button.show()

			# Post-1C calculation
			elif rate_parameters['one_c_calc_bool'] and rate_parameters['one_c_calc_completed_bool'][experiment_index]:
				# Write to summary file
				rate_write_summary_file(experiment_index, section="OCP_equilibrated_post_1C_calc")

				# Begin the next experiment
				rate_start(experiment_index)


	elif equilibrated == "OCP_timeout":

		state = States.Stationary_Graph

		# Store OCP timeout data
		experiment_data['OCP_timeout_finishtime_readable'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
		experiment_data['OCP_timeout_time_elapsed'] = OCP_time_data.averagebuffer[-1]
		experiment_data['OCP_timeout_startpot'] = OCP_potential_data.averagebuffer[0]
		experiment_data['OCP_timeout_stoppot'] = OCP_potential_data.averagebuffer[-1]
		experiment_data['OCP_timeout_total_pot_diff'] = abs(OCP_potential_data.averagebuffer[-1] - OCP_potential_data.averagebuffer[0])
		experiment_data['OCP_timeout_timescale_pot_diff'] = abs(OCP_eq_history[-1][1] - OCP_eq_history[0][1])

		# Write to summary file
		write_summary_file(experiment_index, section="OCP_timeout")

		# Reset experiment
		reset_experiment_controller(mode="OCP_interrupted")

		# Update GUI
		info_program_state_entry.setText("Experiments interrupted")
		QtWidgets.QMessageBox.critical(
			mainwidget,
			f"OCP timeout: {exp_str} experiments stopped",
			f"Oh no! {exp_str} experiments stopped due to OCP not equilibrating before the equilibration timeout. The global input parameters dictionary, current experiment index, and check-status of parameters have been cleared."
		)
		preview_cancel_button.show()

def OCP_equilibration_loop(experiment_parameters, experiment_data, experiment_index):
	"""Loop to equilibrate potential continuously called through periodic_update()"""
	global skipcounter

	equilibrated = False
	elapsed_time = timeit.default_timer() - experiment_data['OCP_starttime'][experiment_index]

	read_potential_current()  # Read new potential and current

	if skipcounter == 0:  # Process new measurements
		OCP_time_data.add_sample(elapsed_time)
		OCP_potential_data.add_sample(potential)

		# Check if a new average was just calculated
		if len(OCP_time_data.samples) == 0 and len(OCP_time_data.averagebuffer) > 0:

			# Add the newly averaged time and potential to the history deque
			OCP_eq_history.append((OCP_time_data.averagebuffer[-1], OCP_potential_data.averagebuffer[-1]))

			# Remove data points older than global_software_settings['OCP_eq_timescale'] seconds from the history deque
			while OCP_eq_history and (OCP_time_data.averagebuffer[-1] - OCP_eq_history[0][0]) > global_software_settings['OCP_eq_timescale']:
				OCP_eq_history.popleft()

			# Start checking if the change in potential is within tolerance after global_software_settings['OCP_eq_timescale'] seconds have passed
			if OCP_time_data.averagebuffer[-1] >= global_software_settings['OCP_eq_timescale']:
				current_potential = OCP_potential_data.averagebuffer[-1]
				# Get potential history from global_software_settings['OCP_eq_timescale'] seconds ago
				if OCP_eq_history:
					past_time, past_potential = OCP_eq_history[0]
					# Check if the change in potential across global_software_settings['OCP_eq_timescale'] is within tolerance
					if abs(current_potential - past_potential) < global_software_settings['OCP_eq_tolerance'] * 1e-3:  # Convert mV to V
						equilibrated = True

			# Update plot to show equilibration progress
			OCP_equilibration_update_live_graph(experiment_parameters, elapsed_time)

		if experiment_parameters == cv_parameters:
			skipcounter = auto_current_range("CV")
		elif experiment_parameters == lsv_parameters:
			skipcounter = auto_current_range("LSV")
		elif experiment_parameters == ca_parameters:
			skipcounter = auto_current_range("CA")

	else:  # Wait until the required number of data points are skipped
		skipcounter -= 1

	# Once equilibrated pass back to equilibration controller
	if equilibrated == True:
		OCP_equilibration_controller(experiment_parameters, experiment_data, experiment_index, equilibrated="OCP_equilibrated")

	# If equilibration time surpasses timeout threshold, pass back to equilibration controller
	elif elapsed_time > global_software_settings['OCP_eq_timeout']:
		OCP_equilibration_controller(experiment_parameters, experiment_data, experiment_index, equilibrated="OCP_timeout")

def OCP_equilibration_update_live_graph(experiment_parameters, elapsed_time):
	"""Update the plot with OCP equilibration data."""

	if len(OCP_eq_history) >= 1:

		# Clear plot frame and legend
		plot_frame.clear()
		legend.clear()

		times = [data[0] for data in OCP_eq_history]
		potentials = [data[1] for data in OCP_eq_history]
		OCP_potential_plot_curve.setData(times, potentials)

		plot_frame.addItem(OCP_potential_plot_curve)

		experiment_names = {
			"cv": "CYCLIC VOLTAMMETRY",
			"lsv": "LINEAR SWEEP VOLTAMMETRY",
			"gcd": "GALVANOSTATIC CHARGE-DISCHARGE",
			"ca": "CHRONOAMPEROMETRY",
			"cp": "CHRONOPOTENTIOMETRY",
			"sd": "SELF-DISCHARGE",
			"rate": "RATE-TESTING",
		}
		param_type = experiment_parameters.get("type")
		experiment_str = experiment_names.get(param_type, "UNKNOWN EXPERIMENT")

		dummy_item = pyqtgraph.PlotDataItem([], [], pen=None)
		legend.addItem(OCP_potential_plot_curve, "Potential (V)")
		legend.addItem(dummy_item, "")
		legend.addItem(dummy_item, f"OCP EQUILIBRATION INFO FOR {experiment_str} EXPERIMENTS:")
		legend.addItem(dummy_item, f"Equilibration tolerance (mV): {float(global_software_settings['OCP_eq_tolerance']):.3g}")
		legend.addItem(dummy_item, f"Equilibration timescale (s): {global_software_settings['OCP_eq_timescale']:.3g}")
		legend.addItem(dummy_item, f"Equilibration timeout (s): {global_software_settings['OCP_eq_timeout']:.3g}")
		legend.addItem(dummy_item, "")
		legend.addItem(dummy_item, f"Elapsed time (s): {elapsed_time:.3f}")
		if elapsed_time >= global_software_settings['OCP_eq_timescale']:
			legend.addItem(dummy_item, f"Potential {global_software_settings['OCP_eq_timescale']} s ago (mV): {OCP_eq_history[0][1] * 1000:.3g}")
		else:
			legend.addItem(dummy_item, f"Potential {elapsed_time:.3f} s ago (mV): {OCP_eq_history[0][1] * 1000:.3g}")
		legend.addItem(dummy_item, f"Measured potential (mV): {OCP_eq_history[-1][1] * 1000:.3g}")
		potential_diff = (OCP_eq_history[-1][1] - OCP_eq_history[0][1])
		legend.addItem(dummy_item, f"Difference (mV): {potential_diff * 1000:.3g}")


"""_____PROGRESS BARS_____"""

class GenericProgressBar(QtWidgets.QProgressBar):
	def __init__(self, parent=None, formatter=None):
		super().__init__(parent)
		self.set_solid_green_style()
		self.setFormat("")
		self._progress_value = 0
		self._progress_max = 100
		self._formatter = formatter or (lambda: "")

	def set_style(self, border_color="green", chunk_color="green", border_style="solid"):
		self.setStyleSheet(f"""
			QProgressBar {{
				border: 2px {border_style} {border_color};
				border-radius: 5px;
				text-align: center;
			}}
			QProgressBar::chunk {{
				background-color: {chunk_color};
			}}
		""")

	def set_solid_green_style(self):
		self.set_style("green", "green", "solid")

	def set_solid_yellow_style(self):
		self.set_style("#ebdb34", "green", "solid")

	def set_dashed_yellow_style(self):
		self.set_style("#ebdb34", "green", "dashed")

	def set_solid_red_style(self):
		self.set_style("red", "red", "solid")

	def set_dashed_yellow_red_style(self):
		self.set_style("#ebdb34", "red", "dashed")

	def set_progress(self, maximum, value):
		self._progress_max = maximum
		self._progress_value = value
		if maximum is not None and value is not None and maximum > 0:
			percent = int((value / maximum) * 100)
			self.setValue(percent)
		else:
			self.setValue(0)

	def paintEvent(self, event):
		"""Override the paint event to display remaining time in the progress bar with dynamic color change."""
		super().paintEvent(event)
		painter = QtGui.QPainter(self)
		try:
			# Get the display text
			display_text = self._formatter()

			# Determine the progress percentage and the bar width
			progress_percentage = self.value() / self.maximum() if self.maximum() else 0
			progress_width = int(self.width() * progress_percentage)

			# Determine text position and size
			rect = self.rect()
			font_metrics = QtGui.QFontMetrics(self.font())
			text_width = font_metrics.horizontalAdvance(display_text)
			text_height = font_metrics.height()
			text_x = (rect.width() - text_width) // 2
			text_y = (rect.height() + text_height) // 2 - 7  # Vertical centering of the text

			# Draw each character with appropriate colour
			current_x = text_x
			for char in display_text:
				char_width = font_metrics.horizontalAdvance(char)
				char_rect = QtCore.QRect(current_x, text_y - (text_height // 2), char_width, text_height)

				# Determine whether horizontal center of character surpassed by the progress bar fill
				if progress_width - current_x > 0.5 * char_width:
					painter.setPen(QtGui.QColor(255, 255, 255))  # White text for filled area
				else:
					painter.setPen(QtGui.QColor(0, 0, 0))  # Black text for unfilled area
				painter.drawText(char_rect, QtCore.Qt.AlignLeft, char)
				current_x += char_width
		finally:
			painter.end()  # Ensure that QPainter is properly ended

class TimeProgressBar(GenericProgressBar):
	def __init__(self, parent=None):
		self.total_time = None
		self.remaining_time = None
		super().__init__(parent, formatter=self.format_time)

	def update_progress_bar(self, total_time, remaining_time):
		if total_time is None or remaining_time is None:
			self.reset()
			return
		self.total_time = total_time
		self.remaining_time = remaining_time
		self.set_progress(total_time, total_time - remaining_time)

	def format_time(self):
		if self.remaining_time is None:
			return "--:--:--"
		hours, remainder = divmod(self.remaining_time, 3600)
		minutes, seconds = divmod(remainder, 60)
		return f"{int(hours):02}:{int(minutes):02}:{int(seconds):02}"

	def reset(self):
		self.total_time = None
		self.remaining_time = None
		self.setValue(0)
		self.set_solid_green_style()
		self.update()

	def set_interrupted_state(self):
		self.total_time = None
		self.remaining_time = None
		self.set_solid_red_style()
		self.update()

	def set_OCP_interrupted_state(self):
		self.total = None
		self.current_value = None
		self.set_dashed_yellow_red_style()
		self.update()

	def set_completed_state(self):
		self.set_progress(100, 100)
		self.total_time = None
		self.remaining_time = None
		self.update()

class LabeledProgressBar(GenericProgressBar):
	def __init__(self, label="Item", parent=None, offset=0):
		self.total = None
		self.current_value = None
		self.label = label
		self.offset = offset
		super().__init__(parent, formatter=self.format_label)

	def update_progress_bar(self, total, current_value):
		if total is None or current_value is None:
			self.reset()
			return
		self.total = total
		self.current_value = current_value
		self.set_progress(self.total, self.current_value)

	def format_label(self):
		if self.total is None or self.current_value is None:
			return f"{self.label} -- of --"
		return f"{self.label} {self.current_value + self.offset} of {self.total}"

	def reset(self):
		self.total = None
		self.current_value = None
		self.setValue(0)
		self.set_solid_green_style()
		self.update()

	def set_interrupted_state(self):
		self.total = None
		self.current_value = None
		self.set_solid_red_style()
		self.update()

	def set_OCP_interrupted_state(self):
		self.total = None
		self.current_value = None
		self.set_dashed_yellow_red_style()
		self.update()

	def set_completed_state(self):
		self.set_progress(100, 100)
		self.total = None
		self.current_value = None
		self.update()



"""_____SET UP THE GUI_____"""


"""MAIN WINDOW"""

# Set up the GUI - Main Window
app = QtWidgets.QApplication([])
win = QtWidgets.QMainWindow()
win.setGeometry(250, 200, 1200, 650)
win.setWindowTitle('USB potentiostat/galvanostat controller')
#win.setWindowIcon(QtGui.QIcon('icon/icon.png'))

potential_monitor, potential_monitor_box = make_groupbox_indicator("Measured potential", "+#.### V")
potential_monitor.setFont(QtGui.QFont("monospace", 8))
current_monitor, current_monitor_box = make_groupbox_indicator("Measured current", "+#.### mA")
current_monitor.setFont(QtGui.QFont("monospace", 8))
potential_current_display_frame = QtWidgets.QHBoxLayout()
potential_current_display_frame.setSpacing(1)
potential_current_display_frame.setContentsMargins(0, 0, 0, 0)
potential_current_display_frame.addWidget(potential_monitor_box)
potential_current_display_frame.addWidget(current_monitor_box)

mode_display_frame = QtWidgets.QHBoxLayout()
mode_display_frame.setSpacing(1)
mode_display_frame.setContentsMargins(0, 0, 0, 5)
cell_status_monitor, cell_status_monitor_box = make_groupbox_indicator("Cell status", "        ")
cell_status_monitor.setFont(custom_size_font(8))
control_mode_monitor, control_mode_monitor_box = make_groupbox_indicator("Control mode", "             ")
control_mode_monitor.setFont(custom_size_font(8))
current_range_monitor, current_range_monitor_box = make_groupbox_indicator("Current range", "     ")
current_range_monitor.setFont(custom_size_font(8))
mode_display_frame.addWidget(cell_status_monitor_box)
mode_display_frame.addWidget(control_mode_monitor_box)
mode_display_frame.addWidget(current_range_monitor_box)
pyqtgraph.setConfigOptions(foreground="#e5e5e5", background="#00304f")
plot_frame = pyqtgraph.PlotWidget()

display_plot_frame = QtWidgets.QVBoxLayout()
display_plot_frame.setSpacing(0)
display_plot_frame.setContentsMargins(0, 0, 0, 0)
display_plot_frame.addLayout(potential_current_display_frame)
display_plot_frame.addLayout(mode_display_frame)
display_plot_frame.addWidget(plot_frame)

preview_cancel_vlayout = QtWidgets.QVBoxLayout(plot_frame)
preview_cancel_hlayout = QtWidgets.QHBoxLayout()
preview_cancel_vlayout.setAlignment(QtCore.Qt.AlignTop)
preview_cancel_vlayout.addLayout(preview_cancel_hlayout)
preview_cancel_hlayout.setAlignment(QtCore.Qt.AlignRight)
preview_cancel_button = QtWidgets.QPushButton("Back to live graph")
preview_cancel_button.clicked.connect(preview_cancel)
preview_cancel_hlayout.addWidget(preview_cancel_button)
preview_cancel_button.hide()


"""_____TAB FRAMES_____"""

tab_frame = QtWidgets.QTabWidget()

tab_names = ["Hardware", "CV", "LSV", "GCD", "CA", "CP", "Self-discharge", "C-Rate", "Plotting"]
tabs = [add_my_tab(tab_frame, tab_name) for tab_name in tab_names]

# Make spacer widget
def create_spacer(height):
	spacer = QtWidgets.QWidget()
	spacer.setFixedHeight(height)
	return spacer

def create_line():
	line = QtWidgets.QFrame()
	line.setFrameShape(QtWidgets.QFrame.HLine)
	line.setFrameShadow(QtWidgets.QFrame.Sunken)
	return line


"""HARDWARE TAB"""

# Set up the GUI - Hardware tab
hardware_vbox = QtWidgets.QVBoxLayout()
hardware_vbox.setAlignment(QtCore.Qt.AlignTop)

hardware_usb_box = QtWidgets.QGroupBox(title="USB Interface", flat=False)
format_box_for_parameter(hardware_usb_box)
hardware_usb_box_layout = QtWidgets.QVBoxLayout()
hardware_usb_box.setLayout(hardware_usb_box_layout)
hardware_usb_vid_pid_layout = QtWidgets.QHBoxLayout()
hardware_usb_box_layout.addLayout(hardware_usb_vid_pid_layout)
hardware_usb_vid = make_label_entry(hardware_usb_vid_pid_layout, "VID")
hardware_usb_vid.setText(usb_vid)
hardware_usb_pid = make_label_entry(hardware_usb_vid_pid_layout, "PID")
hardware_usb_pid.setText(usb_pid)
hardware_usb_connectButton = QtWidgets.QPushButton("Connect")
hardware_usb_connectButton.clicked.connect(connect_disconnect_usb)
hardware_usb_box_layout.addWidget(hardware_usb_connectButton)
hardware_usb_box_layout.setSpacing(5)
hardware_usb_box_layout.setContentsMargins(3, 10, 3, 3)
hardware_vbox.addWidget(hardware_usb_box)

hardware_device_info_box = QtWidgets.QGroupBox(title="Device Information", flat=False)
format_box_for_parameter(hardware_device_info_box)
hardware_device_info_box_layout = QtWidgets.QVBoxLayout()
hardware_device_info_box.setLayout(hardware_device_info_box_layout)
hardware_device_info_text = QtWidgets.QLabel("Manufacturer: \nProduct: \nSerial #: ")
hardware_device_info_box_layout.addWidget(hardware_device_info_text)
hardware_device_info_box_layout.setSpacing(5)
hardware_device_info_box_layout.setContentsMargins(3, 10, 3, 3)
#hardware_vbox.addWidget(hardware_device_info_box)

hardware_calibration_box = QtWidgets.QGroupBox(title="Calibration", flat=False)
format_box_for_parameter(hardware_calibration_box)
hardware_calibration_box_layout = QtWidgets.QVBoxLayout()
hardware_calibration_box.setLayout(hardware_calibration_box_layout)
hardware_calibration_dac_hlayout = QtWidgets.QHBoxLayout()
hardware_calibration_box_layout.addLayout(hardware_calibration_dac_hlayout)
hardware_calibration_dac_vlayout = QtWidgets.QVBoxLayout()
hardware_calibration_dac_hlayout.addLayout(hardware_calibration_dac_vlayout)
hardware_calibration_dac_offset = make_label_entry(hardware_calibration_dac_vlayout, "DAC Offset")
hardware_calibration_dac_gain = make_label_entry(hardware_calibration_dac_vlayout, "DAC Gain")
hardware_calibration_dac_calibrate_button = QtWidgets.QPushButton("Auto\nCalibrate")
hardware_calibration_dac_calibrate_button.setMaximumHeight(50)
hardware_calibration_dac_calibrate_button.clicked.connect(dac_calibrate)
hardware_calibration_dac_hlayout.addWidget(hardware_calibration_dac_calibrate_button)

hardware_calibration_offset_hlayout = QtWidgets.QHBoxLayout()
hardware_calibration_box_layout.addLayout(hardware_calibration_offset_hlayout)
hardware_calibration_offset_vlayout = QtWidgets.QVBoxLayout()
hardware_calibration_offset_hlayout.addLayout(hardware_calibration_offset_vlayout)
hardware_calibration_potential_offset = make_label_entry(hardware_calibration_offset_vlayout, "Pot. Offset")
hardware_calibration_potential_offset.editingFinished.connect(offset_changed_callback)
hardware_calibration_potential_offset.setText("0")
hardware_calibration_current_offset = make_label_entry(hardware_calibration_offset_vlayout, "Curr. Offset")
hardware_calibration_current_offset.editingFinished.connect(offset_changed_callback)
hardware_calibration_current_offset.setText("0")
hardware_calibration_adc_measure_button = QtWidgets.QPushButton("Auto\nZero")
hardware_calibration_adc_measure_button.setMaximumHeight(50)
hardware_calibration_adc_measure_button.clicked.connect(zero_offset)
hardware_calibration_offset_hlayout.addWidget(hardware_calibration_adc_measure_button)

hardware_calibration_shunt_resistor_layout = QtWidgets.QHBoxLayout()
hardware_calibration_box_layout.addLayout(hardware_calibration_shunt_resistor_layout)
hardware_calibration_shuntvalues = [make_label_entry(hardware_calibration_shunt_resistor_layout, "R%d" % i) for i in range(1, 4)]
for i in range(0,3):
	hardware_calibration_shuntvalues[i].editingFinished.connect(shunt_calibration_changed_callback)
	hardware_calibration_shuntvalues[i].setText("%.4f" % shunt_calibration[i])

hardware_calibration_button_layout = QtWidgets.QHBoxLayout()
hardware_calibration_get_button = QtWidgets.QPushButton("Load from device")
hardware_calibration_get_button.clicked.connect(get_calibration)
hardware_calibration_button_layout.addWidget(hardware_calibration_get_button)
hardware_calibration_set_button = QtWidgets.QPushButton("Save to device")
hardware_calibration_set_button.clicked.connect(set_calibration)
hardware_calibration_button_layout.addWidget(hardware_calibration_set_button)

hardware_calibration_box_layout.addLayout(hardware_calibration_button_layout)
hardware_calibration_box_layout.setSpacing(5)
hardware_calibration_box_layout.setContentsMargins(3, 10, 3, 3)
hardware_vbox.addWidget(hardware_calibration_box)

hardware_manual_control_box = QtWidgets.QGroupBox(title="Manual Control", flat=False)
format_box_for_parameter(hardware_manual_control_box)
hardware_manual_control_box_layout = QtWidgets.QVBoxLayout()
hardware_manual_control_box.setLayout(hardware_manual_control_box_layout)

hardware_manual_control_warning = QtWidgets.QLabel("<html><body>"
    "<b>Caution:</b> 'Cell connection: On' applies the last DAC command.<br>"
    "Use the fields below to update this first."
	"</body></html>"
)
hardware_manual_control_warning.setWordWrap(True)
hardware_manual_control_box_layout.addWidget(hardware_manual_control_warning)

hardware_manual_control_cell_layout = QtWidgets.QHBoxLayout()
hardware_manual_control_cell_layout.addWidget(QtWidgets.QLabel("Cell connection"))
hardware_manual_control_cell_on_button = QtWidgets.QPushButton("On")
hardware_manual_control_cell_on_button.clicked.connect(lambda: (set_cell_status(True), checkstate_reset(mode="manual_control")))
hardware_manual_control_cell_layout.addWidget(hardware_manual_control_cell_on_button)
hardware_manual_control_cell_off_button = QtWidgets.QPushButton("Off")
hardware_manual_control_cell_off_button.clicked.connect(lambda: (set_cell_status(False), checkstate_reset(mode="manual_control")))
hardware_manual_control_cell_layout.addWidget(hardware_manual_control_cell_off_button)
hardware_manual_control_box_layout.addLayout(hardware_manual_control_cell_layout)

hardware_manual_control_mode_layout = QtWidgets.QHBoxLayout()
hardware_manual_control_mode_layout.addWidget(QtWidgets.QLabel("Mode"))
hardware_manual_control_mode_potentiostat_button = QtWidgets.QPushButton("Potentiostatic")
hardware_manual_control_mode_potentiostat_button.clicked.connect(lambda: (set_control_mode(False), checkstate_reset(mode="manual_control")))
hardware_manual_control_mode_layout.addWidget(hardware_manual_control_mode_potentiostat_button)
hardware_manual_control_mode_galvanostat_button = QtWidgets.QPushButton("Galvanostatic")
hardware_manual_control_mode_galvanostat_button.clicked.connect(lambda: (set_control_mode(True), checkstate_reset(mode="manual_control")))
hardware_manual_control_mode_layout.addWidget(hardware_manual_control_mode_galvanostat_button)
hardware_manual_control_box_layout.addLayout(hardware_manual_control_mode_layout)

hardware_manual_control_range_layout = QtWidgets.QHBoxLayout()
hardware_manual_control_range_layout.addWidget(QtWidgets.QLabel("Current range"))
hardware_manual_control_range_dropdown = QtWidgets.QComboBox()
hardware_manual_control_range_dropdown.addItems(current_range_list)
hardware_manual_control_range_layout.addWidget(hardware_manual_control_range_dropdown)
hardware_manual_control_range_set_button = QtWidgets.QPushButton("Set")
hardware_manual_control_range_set_button.clicked.connect(set_current_range)
hardware_manual_control_range_layout.addWidget(hardware_manual_control_range_set_button)
hardware_manual_control_box_layout.addLayout(hardware_manual_control_range_layout)

hardware_manual_control_output_layout = QtWidgets.QHBoxLayout()
hardware_manual_control_output_dropdown = QtWidgets.QComboBox()
hardware_manual_control_output_dropdown.addItems(units_list)
hardware_manual_control_output_layout.addWidget(hardware_manual_control_output_dropdown)
hardware_manual_control_output_entry = QtWidgets.QLineEdit()
hardware_manual_control_output_entry.returnPressed.connect(lambda: (set_output_from_gui(), checkstate_reset(mode="manual_control")))
hardware_manual_control_output_layout.addWidget(hardware_manual_control_output_entry)
hardware_manual_control_output_set_button = QtWidgets.QPushButton("Set")
hardware_manual_control_output_set_button.clicked.connect(lambda: (set_output_from_gui(), checkstate_reset(mode="manual_control")))
hardware_manual_control_output_layout.addWidget(hardware_manual_control_output_set_button)
hardware_manual_control_box_layout.addLayout(hardware_manual_control_output_layout)

hardware_manual_control_box_layout.setSpacing(5)
hardware_manual_control_box_layout.setContentsMargins(3, 10, 3, 3)

hardware_vbox.addWidget(hardware_manual_control_box)

hardware_log_box = QtWidgets.QGroupBox(title="Log to file", flat=False)
format_box_for_parameter(hardware_log_box)
hardware_log_box_layout = QtWidgets.QHBoxLayout()
hardware_log_box.setLayout(hardware_log_box_layout)
hardware_log_checkbox = QtWidgets.QCheckBox("Log")
hardware_log_checkbox.stateChanged.connect(toggle_logging)
hardware_log_box_layout.addWidget(hardware_log_checkbox)
hardware_log_filename = QtWidgets.QLineEdit()
hardware_log_box_layout.addWidget(hardware_log_filename)
hardware_log_choose_button = QtWidgets.QPushButton("...")
hardware_log_choose_button.setFixedWidth(32)
hardware_log_choose_button.clicked.connect(lambda: choose_file(hardware_log_filename, "Choose where to save the log data"))
hardware_log_box_layout.addWidget(hardware_log_choose_button)

hardware_log_box_layout.setSpacing(5)
hardware_log_box_layout.setContentsMargins(3, 10, 3, 3)

hardware_vbox.addWidget(hardware_log_box)

class SoftwareGlobalsDialog(QtWidgets.QDialog):
	def __init__(self, parent=None):
		super().__init__(parent)
		self.setWindowTitle("Global Software Settings")
		self.setModal(True)
		self.setWindowFlags(self.windowFlags() | QtCore.Qt.WindowStaysOnTopHint)

		layout = QtWidgets.QFormLayout()

		self.entry_ocp_tol = QtWidgets.QLineEdit(str(global_software_settings['OCP_eq_tolerance']))
		self.entry_ocp_timescale = QtWidgets.QLineEdit(str(global_software_settings['OCP_eq_timescale']))
		self.entry_ocp_timeout = QtWidgets.QLineEdit(str(global_software_settings['OCP_eq_timeout']))
		self.entry_ocp_num_samples = QtWidgets.QLineEdit(str(global_software_settings['OCP_eq_num_samples']))
		self.entry_cv_lockout = QtWidgets.QLineEdit(str(global_software_settings['cv_reverse_current_lockout']))
		self.entry_gcd_nth_cycles = QtWidgets.QLineEdit(str(global_software_settings['gcd_nth_cycles']))
		self.entry_tab_frame_width = QtWidgets.QLineEdit(str(global_software_settings['tab_frame_width']))

		self.entry_ocp_tol.setToolTip("Tolerance (mV) to determine if OCP has stabilised.")
		self.entry_ocp_timescale.setToolTip("Timescale (s) over which OCP stability is measured.")
		self.entry_ocp_timeout.setToolTip("Maximum duration (s) to wait for OCP to stabilise.")
		self.entry_ocp_num_samples.setToolTip("Number of samples to average during OCP equilibration.")
		self.entry_cv_lockout.setToolTip("Lockout time (s) during which a re-trigger of the same reverse current is ignored.")
		self.entry_gcd_nth_cycles.setToolTip(
			"<html><body>"
			"How often cycles are stored during a GCD experiment for plotting. Each nth cycle is stored in the data dictionary.<br><br>"
			"<b>Warning:</b> This can use a lot of memory if you are running long experiments."
			"</body></html>"
		)
		self.entry_tab_frame_width.setToolTip("Width of the GUI tab frame (in spaces)")

		layout.addRow(QtWidgets.QLabel("<html><body><b>NOTE:</b> Changes to these parameters are saved for this session only.</body></html>"))
		layout.addRow("OCP equilibration tolerance (mV):", self.entry_ocp_tol)
		layout.addRow("OCP equilibration timescale (s):", self.entry_ocp_timescale)
		layout.addRow("OCP equilibration timeout (s):", self.entry_ocp_timeout)
		layout.addRow("OCP equilibration number of samples to average:", self.entry_ocp_num_samples)
		layout.addRow("CV reverse current lockout time (s):", self.entry_cv_lockout)
		layout.addRow("GCD store nth cycles for plotting:", self.entry_gcd_nth_cycles)
		layout.addRow("GUI tab frame width (spaces):", self.entry_tab_frame_width)

		button_layout = QtWidgets.QHBoxLayout()
		save_button = QtWidgets.QPushButton("Save")
		save_button.clicked.connect(self.save_settings)
		cancel_button = QtWidgets.QPushButton("Cancel")
		cancel_button.clicked.connect(self.reject)
		button_layout.addWidget(save_button)
		button_layout.addWidget(cancel_button)
		layout.addRow(button_layout)

		self.setLayout(layout)

	def save_settings(self):
		try:
			OCP_tolerance = float(self.entry_ocp_tol.text().strip())
			OCP_timescale = float(self.entry_ocp_timescale.text().strip())
			OCP_timeout = float(self.entry_ocp_timeout.text().strip())
			OCP_num_samples = int(self.entry_ocp_num_samples.text().strip())
			lockout = float(self.entry_cv_lockout.text().strip())
			nth_cycles = int(self.entry_gcd_nth_cycles.text().strip())
			tab_frame_width = int(self.entry_tab_frame_width.text().strip())

			# Validate all are positive
			if not all(x > 0 for x in [OCP_tolerance, OCP_timescale, OCP_timeout, OCP_num_samples, lockout, nth_cycles, tab_frame_width]):
				raise ValueError

			changes = []

			def update_if_changed(key, label, new_val):
				old_val = global_software_settings[key]
				if old_val != new_val:
					global_software_settings[key] = new_val
					changes.append(f"{label}: {old_val} to {new_val}")

					# Reset check state if OCP timescale has changed and in use it is used to calculate times and previews
					if key == "OCP_eq_timescale":
						checkstate_reset(mode="OCP_eq_timescale_changed")

			update_if_changed('OCP_eq_tolerance', "OCP eq tolerance (mV)", OCP_tolerance)
			update_if_changed('OCP_eq_timescale', "OCP eq timescale (s)", OCP_timescale)
			update_if_changed('OCP_eq_timeout', "OCP eq timeout (s)", OCP_timeout)
			update_if_changed('OCP_eq_num_samples', "OCP eq number of samples", OCP_num_samples)
			update_if_changed('cv_reverse_current_lockout', "CV reverse current lockout (s)", lockout)
			update_if_changed('gcd_nth_cycles', "GCD nth cycles", nth_cycles)
			update_if_changed('tab_frame_width', "GUI tab frame width", tab_frame_width)

			apply_tab_frame_width()

			if changes:
				log_message("Updated software globals: " + ", ".join(changes))
			else:
				log_message("Software global parameters unchanged.")

			self.accept()

		except (ValueError, TypeError):
			QtWidgets.QMessageBox.critical(
				self,
				"Invalid Input",
				f"One or more fields have invalid values:\n"
				"- All parameters must be positive numeric values.\n"
				"- Number of samples to average must be an integer.\n"
				"- nth cycles must be an integer.\n"
				"- Tab frame width must be an integer.\n\n"
				"The parameters were not saved."
			)


def open_software_globals_menu():
	dialog = SoftwareGlobalsDialog(mainwidget)
	dialog.exec_()


def apply_tab_frame_width():
	tab_frame_width = global_software_settings.get('tab_frame_width', 125)
	font_metrics = QtGui.QFontMetrics(statustext.font())
	char_width = font_metrics.horizontalAdvance(' ')
	tab_frame.setFixedWidth(tab_frame_width * char_width)

software_globals_menu_box = QtWidgets.QGroupBox(title="Software options", flat=False)
format_box_for_parameter(software_globals_menu_box)
software_globals_menu_layout = QtWidgets.QHBoxLayout()
software_globals_menu_box.setLayout(software_globals_menu_layout)

software_globals_menu_button = QtWidgets.QPushButton("Software options menu")
software_globals_menu_button.clicked.connect(open_software_globals_menu)
software_globals_menu_layout.addWidget(software_globals_menu_button)

software_globals_menu_layout.setSpacing(5)
software_globals_menu_layout.setContentsMargins(3, 10, 3, 3)
hardware_vbox.addWidget(software_globals_menu_box)

hardware_vbox.setSpacing(5)
hardware_vbox.setContentsMargins(3, 3, 3, 3)


# Make scrollable area
hardware_widget = QtWidgets.QWidget()
hardware_widget.setLayout(hardware_vbox)
hardware_widget.setContentsMargins(0, 0, 0, 0)

hardware_scroll_area = QtWidgets.QScrollArea()
hardware_scroll_area.setWidgetResizable(True)
hardware_scroll_area.setWidget(hardware_widget)
hardware_scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
hardware_scroll_area.setContentsMargins(0, 0, 0, 0)

hardware_layout = QtWidgets.QVBoxLayout()
hardware_layout.setContentsMargins(0, 0, 0, 0)
hardware_layout.addWidget(hardware_scroll_area)
tabs[0].setLayout(hardware_layout)


"""CYCLIC VOLTAMMETRY TAB"""

cv_vbox = QtWidgets.QVBoxLayout()
cv_vbox.setAlignment(QtCore.Qt.AlignTop)

# Parameters box
cv_params_box = QtWidgets.QGroupBox(title="Cyclic voltammetry parameters", flat=False)
format_box_for_parameter(cv_params_box)
cv_params_box_layout = QtWidgets.QVBoxLayout()
cv_params_box.setLayout(cv_params_box_layout)

# Lower potential bounds
cv_params_lbound_hlayout = QtWidgets.QHBoxLayout()
cv_params_lbound_label = QtWidgets.QLabel(text="Lower potential bounds (csv, V)")
cv_params_lbound_label.setToolTip(
	"Lower potential bounds for experiments to cycle through.\n\n"
	"Accepts numeric values or 'OCP'."
)
cv_params_lbound_entry = QtWidgets.QLineEdit()

cv_params_lbound_hlayout.addWidget(cv_params_lbound_label)
cv_params_lbound_hlayout.addWidget(cv_params_lbound_entry)
cv_params_box_layout.addLayout(cv_params_lbound_hlayout)

# Upper potential bounds
cv_params_ubound_hlayout = QtWidgets.QHBoxLayout()
cv_params_ubound_label = QtWidgets.QLabel(text="Upper potential bounds (csv, V)")
cv_params_ubound_label.setToolTip(
	"Upper potential bounds for experiments to cycle through.\n\n"
	"Accepts numeric values or 'OCP'."
)
cv_params_ubound_entry = QtWidgets.QLineEdit()

cv_params_ubound_hlayout.addWidget(cv_params_ubound_label)
cv_params_ubound_hlayout.addWidget(cv_params_ubound_entry)
cv_params_box_layout.addLayout(cv_params_ubound_hlayout)

# Startpots
cv_params_startpot_hlayout = QtWidgets.QHBoxLayout()
cv_params_startpot_label = QtWidgets.QLabel(text="Start potentials (V)")
cv_params_startpot_label.setToolTip(
	"Starting potentials to cycle through.\n"
	"Each start potential corresponds to a potential window.\n\n"
	"Accepts numeric values or 'OCP'."
)
cv_params_startpot_entry = QtWidgets.QLineEdit()

cv_params_startpot_hlayout.addWidget(cv_params_startpot_label)
cv_params_startpot_hlayout.addWidget(cv_params_startpot_entry)
cv_params_box_layout.addLayout(cv_params_startpot_hlayout)

# Stoppots
cv_params_stoppot_hlayout = QtWidgets.QHBoxLayout()
cv_params_stoppot_label = QtWidgets.QLabel(text="Stop potentials (V)")
cv_params_stoppot_label.setToolTip(
	"Stop potentials to cycle through.\n"
	"Each stop potential corresponds to a potential window.\n\n"
	"Accepts numeric values or 'OCP'."
)
cv_params_stoppot_entry = QtWidgets.QLineEdit()

cv_params_stoppot_hlayout.addWidget(cv_params_stoppot_label)
cv_params_stoppot_hlayout.addWidget(cv_params_stoppot_entry)
cv_params_box_layout.addLayout(cv_params_stoppot_hlayout)

cv_params_box_layout.addWidget(create_line())

# Scanrates
cv_params_scanrate_hlayout = QtWidgets.QHBoxLayout()
cv_params_scanrate_label = QtWidgets.QLabel(text="Scan rates (csv, mV/s)")
cv_params_scanrate_label.setToolTip(
	"Scan rates to cycle through.\n\n"
	"Experiments will cycle through each scan rate\n"
	"at each potential window.\n\n"
	"Must be non-zero numeric values."
)
cv_params_scanrate_entry = QtWidgets.QLineEdit()

cv_params_scanrate_hlayout.addWidget(cv_params_scanrate_label)
cv_params_scanrate_hlayout.addWidget(cv_params_scanrate_entry)
cv_params_box_layout.addLayout(cv_params_scanrate_hlayout)

# Negative reverse currents
cv_params_reverse_current_negative_hlayout = QtWidgets.QHBoxLayout()
cv_params_reverse_current_negative_label = QtWidgets.QLabel(text="Negative reverse currents (csv, µA)")
cv_params_reverse_current_negative_label.setToolTip(
	"Negative current threshold for cycle reversal.\n"
	"Each current corresponds to a scan rate.\n\n"
	"Accepts numeric values or 'None'."
)
cv_params_reverse_current_negative_entry = QtWidgets.QLineEdit()
cv_params_reverse_current_negative_checkbox = QtWidgets.QCheckBox()
cv_params_reverse_current_negative_checkbox.setToolTip(
	"If checked, the given negative reverse currents are applied.\n\n"
	"If unchecked, the negative reverse currents are not applied."
)

cv_params_reverse_current_negative_hlayout.addWidget(cv_params_reverse_current_negative_label)
cv_params_reverse_current_negative_hlayout.addWidget(cv_params_reverse_current_negative_entry)
cv_params_reverse_current_negative_hlayout.addWidget(cv_params_reverse_current_negative_checkbox)
cv_params_box_layout.addLayout(cv_params_reverse_current_negative_hlayout)

# Positive reverse currents
cv_params_reverse_current_positive_hlayout = QtWidgets.QHBoxLayout()
cv_params_reverse_current_positive_label = QtWidgets.QLabel(text="Positive reverse currents (csv, µA)")
cv_params_reverse_current_positive_label.setToolTip(
	"Positive current threshold for cycle reversal.\n"
	"Each current corresponds to a scan rate.\n\n"
	"Accepts numeric values or 'None'."
)
cv_params_reverse_current_positive_entry = QtWidgets.QLineEdit()
cv_params_reverse_current_positive_checkbox = QtWidgets.QCheckBox()
cv_params_reverse_current_positive_checkbox.setToolTip(
	"If checked, the given positive reverse currents are applied.\n\n"
	"If unchecked, the positive reverse currents are not applied."
)

cv_params_reverse_current_positive_hlayout.addWidget(cv_params_reverse_current_positive_label)
cv_params_reverse_current_positive_hlayout.addWidget(cv_params_reverse_current_positive_entry)
cv_params_reverse_current_positive_hlayout.addWidget(cv_params_reverse_current_positive_checkbox)
cv_params_box_layout.addLayout(cv_params_reverse_current_positive_hlayout)

# Number of cycles per scan rate
cv_params_num_cycles_hlayout = QtWidgets.QHBoxLayout()
cv_params_num_cycles_label = QtWidgets.QLabel(text="Number of cycles per scan rate")
cv_params_num_cycles_label.setToolTip(
	"Number of complete potential cycles per experiment.\n\n"
	"A full potential cycle is defined as a full sweep between\n"
	"potential bounds, or reverse current thresholds if applied.\n\n"
	"Must be a positive integer value."
)
cv_params_num_cycles_entry = QtWidgets.QLineEdit()

cv_params_num_cycles_hlayout.addWidget(cv_params_num_cycles_label)
cv_params_num_cycles_hlayout.addWidget(cv_params_num_cycles_entry)
cv_params_box_layout.addLayout(cv_params_num_cycles_hlayout)

# Number of samples to average
cv_params_num_samples_hlayout = QtWidgets.QHBoxLayout()
cv_params_num_samples_label = QtWidgets.QLabel(text="Number of samples to average per scan rate")
cv_params_num_samples_label.setToolTip(
	"Number of data points to average for each scan rate.\n\n"
	"Must be a single positive integer value or a positive\n"
	"integer value for each scan rate.\n\n"
	"If a single integer is given but multiple scan rates exist,\n"
	"the number of samples to average will be scaled based on\n"
	"scan rate to maintain sampling resolution in the potential domain.\n"
	"This scaling uses the given integer as the reference for\n"
	"the lowest scan rate, adjusting as scan rate increases."
)
cv_params_num_samples_entry = QtWidgets.QLineEdit()

cv_params_num_samples_hlayout.addWidget(cv_params_num_samples_label)
cv_params_num_samples_hlayout.addWidget(cv_params_num_samples_entry)
cv_params_box_layout.addLayout(cv_params_num_samples_hlayout)

cv_params_box_layout.addWidget(create_line())

# Pre-scan rate delay
cv_params_scanrate_delay_hlayout = QtWidgets.QHBoxLayout()
cv_params_scanrate_delay_label = QtWidgets.QLabel(text="Pre-scan rate delay (s)")
cv_params_scanrate_delay_label.setToolTip(
	"Time delay before the experiment at the next scan rate begins to allow the system to settle.\n\n"
	"Must be a non-negative numeric value."
)
cv_params_scanrate_delay_entry = QtWidgets.QLineEdit()

cv_params_scanrate_delay_hlayout.addWidget(cv_params_scanrate_delay_label)
cv_params_scanrate_delay_hlayout.addWidget(cv_params_scanrate_delay_entry)
cv_params_box_layout.addLayout(cv_params_scanrate_delay_hlayout)

# Pre-potential window delay
cv_params_pot_window_delay_hlayout = QtWidgets.QHBoxLayout()
cv_params_pot_window_delay_label = QtWidgets.QLabel(text="Pre-potential window delay (s)")
cv_params_pot_window_delay_label.setToolTip(
	"Time delay before experiments at each potential window begin to allow the system to settle.\n\n"
	"Required if not waiting for OCP equilibration.\n\n"
	"Must be a non-negative numeric value."
)
cv_params_pot_window_delay_entry = QtWidgets.QLineEdit()
cv_params_pot_window_delay_OCP_checkbox = QtWidgets.QCheckBox("Wait for OCP equilibration")
cv_params_pot_window_delay_OCP_checkbox.setToolTip(
	"<html><body>"
	"Wait for OCP equilibration before experiments at each potential window begin instead of using a fixed delay.<br><br>"
	"If checked, OCP equilibration is performed before every potential window to ensure consistent starting conditions.<br><br>"
	"<b>NOTE:</b> Mixing equilibration and fixed delays between potential windows within the same set of experiments is not supported. For users wishing to do this, please run separate sets of experiments.<br><br>"
	"Automatically checked if any start potential, stop potential, or potential limit set as 'OCP'."
	"</body></html>"
)

cv_params_pot_window_delay_hlayout.addWidget(cv_params_pot_window_delay_label)
cv_params_pot_window_delay_hlayout.addWidget(cv_params_pot_window_delay_entry)
cv_params_pot_window_delay_hlayout.addWidget(cv_params_pot_window_delay_OCP_checkbox)
cv_params_box_layout.addLayout(cv_params_pot_window_delay_hlayout)

cv_params_box_layout.setSpacing(5)
cv_params_box_layout.setContentsMargins(3, 10, 3, 3)
cv_vbox.addWidget(cv_params_box)

# Checkbutton box
cv_checking_box = QtWidgets.QGroupBox(title="THIS BUTTON MUST BE PRESSED", flat=False)
format_box_for_parameter_centered_title(cv_checking_box)
cv_checking_layout = QtWidgets.QHBoxLayout()
cv_checking_box.setLayout(cv_checking_layout)

cv_variables_checkbutton = QtWidgets.QPushButton("CHECK")
cv_variables_checkbutton.clicked.connect(cv_checkbutton_callback)
cv_checking_layout.addWidget(cv_variables_checkbutton)

cv_checking_layout.setSpacing(5)
cv_checking_layout.setContentsMargins(3, 10, 3, 3)
cv_vbox.addWidget(cv_checking_box)

# Autoranging box
cv_range_box = QtWidgets.QGroupBox(title="Autoranging", flat=False)
cv_range_box.setToolTip(
	"Enable autoranging to automatically select the optimal current range\n"
	"based on the measured current during the experiment.\n\n"
	"This feature allows the device to accurately measure currents across\n"
	"a wide dynamic range, from nanoamps up to 25 mA, by adapting to the\n"
	"current level in real time.\n\n"
	"If specific current ranges are undesirable for these experiments,\n"
	"untick their checkboxes to prevent them from being selected by autoranging."
)
format_box_for_parameter(cv_range_box)
cv_range_layout = QtWidgets.QHBoxLayout()
cv_range_box.setLayout(cv_range_layout)
cv_range_layout.setAlignment(QtCore.Qt.AlignCenter)
cv_range_checkboxes = []
for curr in current_range_list:
	checkbox = QtWidgets.QCheckBox(curr)
	cv_range_checkboxes.append(checkbox)
	cv_range_layout.addWidget(checkbox)
	checkbox.setChecked(True)

cv_range_layout.setSpacing(50)
cv_range_layout.setContentsMargins(3, 10, 3, 3)
cv_vbox.addWidget(cv_range_box)

# File box
cv_file_box = QtWidgets.QGroupBox(title="Output filepath (experiment info will be appended)", flat=False)
format_box_for_parameter(cv_file_box)
cv_file_layout = QtWidgets.QVBoxLayout()
cv_file_box.setLayout(cv_file_layout)

# Filename
cv_file_choose_hlayout = QtWidgets.QHBoxLayout()
cv_file_entry = QtWidgets.QLineEdit()
cv_file_entry.setToolTip(
	"Base output filename for these experiments.\n\n"
	"Each experiment's data file will append '_CV' and details for\n"
	"the corresponding potential window and scan rate, saved as\n"
	"a .dat file.\n\n"
	"A summary file will also be created as a .txt file using the\n"
	"given base output filename."
)
cv_file_choose_button = QtWidgets.QPushButton("...")
cv_file_choose_button.setFixedWidth(32)
cv_file_choose_button.clicked.connect(lambda: choose_file(cv_file_entry, "Choose where to save the CV measurement data"))
cv_file_choose_hlayout.addWidget(cv_file_entry)
cv_file_choose_hlayout.addWidget(cv_file_choose_button)
cv_file_layout.addLayout(cv_file_choose_hlayout)

# Notes
cv_file_notes_entry = QtWidgets.QTextEdit()
cv_file_notes_entry.setPlaceholderText("*** Optional experiment notes to write in summary file ***")
cv_file_notes_entry.setStyleSheet("""
	QTextEdit {
		color: black;
	}
	QTextEdit: empty {
		color:grey;
	}
""")
line_height = cv_file_notes_entry.fontMetrics().lineSpacing()
num_lines = 5
cv_file_notes_entry.setMaximumHeight(line_height * num_lines)
cv_file_notes_entry.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
cv_file_layout.addWidget(cv_file_notes_entry)

cv_file_layout.setSpacing(5)
cv_file_layout.setContentsMargins(3, 10, 3, 3)
cv_vbox.addWidget(cv_file_box)

# Check button reset behaviour if parameter or file inputs change
cv_params_lbound_entry.textChanged.connect(lambda: cv_reset_experiment_controller(mode="input_changed"))
cv_params_ubound_entry.textChanged.connect(lambda: cv_reset_experiment_controller(mode="input_changed"))
cv_params_startpot_entry.textChanged.connect(lambda: cv_reset_experiment_controller(mode="input_changed"))
cv_params_stoppot_entry.textChanged.connect(lambda: cv_reset_experiment_controller(mode="input_changed"))
cv_params_scanrate_entry.textChanged.connect(lambda: cv_reset_experiment_controller(mode="input_changed"))
cv_params_reverse_current_negative_entry.textChanged.connect(lambda: cv_reset_experiment_controller(mode="input_changed"))
cv_params_reverse_current_negative_checkbox.stateChanged.connect(lambda: cv_reset_experiment_controller(mode="input_changed"))
cv_params_reverse_current_positive_entry.textChanged.connect(lambda: cv_reset_experiment_controller(mode="input_changed"))
cv_params_reverse_current_positive_checkbox.stateChanged.connect(lambda: cv_reset_experiment_controller(mode="input_changed"))
cv_params_num_cycles_entry.textChanged.connect(lambda: cv_reset_experiment_controller(mode="input_changed"))
cv_params_num_samples_entry.textChanged.connect(lambda: cv_reset_experiment_controller(mode="input_changed"))
cv_params_scanrate_delay_entry.textChanged.connect(lambda: cv_reset_experiment_controller(mode="input_changed"))
cv_params_pot_window_delay_entry.textChanged.connect(lambda: cv_reset_experiment_controller(mode="input_changed"))
cv_params_pot_window_delay_OCP_checkbox.stateChanged.connect(lambda: cv_reset_experiment_controller(mode="input_changed"))
cv_file_entry.textChanged.connect(lambda: cv_reset_experiment_controller(mode="input_changed"))

# Start and stop buttons
cv_start_button = QtWidgets.QPushButton("Start cyclic voltammetries")
cv_start_button.clicked.connect(cv_initialise)
cv_stop_button = QtWidgets.QPushButton("Stop cyclic voltammetries")
cv_stop_button.clicked.connect(lambda: (cv_delay_timer.stop(), cv_stop(cv_current_exp_index, interrupted=True)))

cv_vbox.addWidget(cv_start_button)
cv_vbox.addWidget(cv_stop_button)

cv_vbox.addWidget(create_spacer(5))

# Experiment info box
cv_info_box = QtWidgets.QGroupBox(title="Experiment info", flat=False)
format_box_for_parameter_centered_title(cv_info_box)
cv_info_layout = QtWidgets.QVBoxLayout()
cv_info_box.setLayout(cv_info_layout)

# Current program state
cv_info_program_state_entry = make_label_entry(cv_info_layout,"Current program state:")
cv_info_program_state_entry.setText("No experiments running")
cv_info_program_state_entry.setReadOnly(True)

# Experiment and cycle numbers
cv_info_hlayout = QtWidgets.QHBoxLayout()
cv_info_expnum_label = QtWidgets.QLabel(text="Experiment number:")
cv_info_expnum_entry = QtWidgets.QLineEdit()
cv_info_expnum_entry.setText("-/-")
cv_info_expnum_entry.setReadOnly(True)
cv_info_expnum_entry.setAlignment(QtCore.Qt.AlignCenter)
cv_info_cyclenum_label = QtWidgets.QLabel(text="Cycle:")
cv_info_cyclenum_entry = QtWidgets.QLineEdit()
cv_info_cyclenum_entry.setText("-/-")
cv_info_cyclenum_entry.setReadOnly(True)
cv_info_cyclenum_entry.setAlignment(QtCore.Qt.AlignCenter)
cv_info_hlayout.addWidget(cv_info_expnum_label)
cv_info_hlayout.addWidget(cv_info_expnum_entry, stretch=1)
cv_info_hlayout.addWidget(cv_info_cyclenum_label)
cv_info_hlayout.addWidget(cv_info_cyclenum_entry, stretch=1)
cv_info_layout.addLayout(cv_info_hlayout)

cv_info_layout.setSpacing(5)
cv_info_layout.setContentsMargins(3, 10, 3, 3)
cv_vbox.addWidget(cv_info_box)

# Progress bar box
cv_progress_box = QtWidgets.QGroupBox(title="Estimated time remaining", flat=False)
format_box_for_parameter_centered_title(cv_progress_box)
cv_progress_layout = QtWidgets.QHBoxLayout()
cv_progress_box.setLayout(cv_progress_layout)

cv_progress_bar = TimeProgressBar()
cv_progress_layout.addWidget(cv_progress_bar)

cv_progress_layout.setSpacing(5)
cv_progress_layout.setContentsMargins(3, 10, 3, 3)
cv_vbox.addWidget(cv_progress_box)

# Plot options box
cv_plot_options_box = QtWidgets.QGroupBox(title="Plot options", flat=False)
format_box_for_parameter_centered_title(cv_plot_options_box)
cv_plot_options_layout = QtWidgets.QVBoxLayout()
cv_plot_options_box.setLayout(cv_plot_options_layout)

cv_plot_options_reverse_currents_checkbox = QtWidgets.QCheckBox("Show reverse currents for this experiment")
cv_plot_options_reverse_currents_checkbox.setChecked(False)

cv_plot_options_prev_cycles_checkbox = QtWidgets.QCheckBox("Show previous cycles from this experiment")
cv_plot_options_prev_cycles_checkbox.setChecked(False)

cv_plot_options_prev_experiments_hlayout = QtWidgets.QHBoxLayout()
cv_plot_options_prev_experiments_checkbox = QtWidgets.QCheckBox("Show final cycle from")
cv_plot_options_prev_experiments_checkbox.setChecked(False)
cv_plot_options_prev_experiments_dropdown = QtWidgets.QComboBox()
cv_plot_options_prev_experiments_dropdown.addItems(["all previous experiments", "same potential window", "same scan rate"])
cv_plot_options_prev_experiments_hlayout.addWidget(cv_plot_options_prev_experiments_checkbox)
cv_plot_options_prev_experiments_hlayout.addWidget(cv_plot_options_prev_experiments_dropdown)

cv_plot_options_layout.addWidget(cv_plot_options_reverse_currents_checkbox)
cv_plot_options_layout.addWidget(cv_plot_options_prev_cycles_checkbox)
cv_plot_options_layout.addWidget(create_line())
cv_plot_options_layout.addLayout(cv_plot_options_prev_experiments_hlayout)

cv_plot_options_layout.setAlignment(QtCore.Qt.AlignCenter)
cv_plot_options_layout.setSpacing(5)
cv_plot_options_layout.setContentsMargins(3, 10, 3, 3)
cv_vbox.addWidget(cv_plot_options_box)

cv_vbox.setSpacing(5)
cv_vbox.setContentsMargins(3, 3, 3, 3)

# Make scrollable area
cv_widget = QtWidgets.QWidget()
cv_widget.setLayout(cv_vbox)
cv_widget.setContentsMargins(0, 0, 0, 0)

cv_scroll_area = QtWidgets.QScrollArea()
cv_scroll_area.setWidgetResizable(True)
cv_scroll_area.setWidget(cv_widget)
cv_scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
cv_scroll_area.setContentsMargins(0, 0, 0, 0)

cv_layout = QtWidgets.QVBoxLayout()
cv_layout.setContentsMargins(0, 0, 0, 0)
cv_layout.addWidget(cv_scroll_area)
tabs[1].setLayout(cv_layout)


"""LINEAR SWEEP VOLTAMMETRY TAB"""

lsv_vbox = QtWidgets.QVBoxLayout()
lsv_vbox.setAlignment(QtCore.Qt.AlignTop)

# Parameters box
lsv_params_box = QtWidgets.QGroupBox(title="Linear sweep voltammetry parameters", flat=False)
format_box_for_parameter(lsv_params_box)
lsv_params_box_layout = QtWidgets.QVBoxLayout()
lsv_params_box.setLayout(lsv_params_box_layout)

# Measurement parameters
lsv_params_measurement_label = QtWidgets.QLabel("<u>Measurement parameters:</u>")
lsv_params_measurement_label.setToolTip(
	"Parameters to control the potential sweep during the LSV measurement."
)
lsv_params_box_layout.addWidget(lsv_params_measurement_label)


# Start potential sequence
lsv_params_startpot_hlayout = QtWidgets.QHBoxLayout()
lsv_params_startpot_label = QtWidgets.QLabel(text="Start potentials (csv, V)")
lsv_params_startpot_label.setToolTip(
	"Starting potentials for experiments to cycle through.\n\n"
	"Accepts numeric values or 'OCP'."
)
lsv_params_startpot_entry = QtWidgets.QLineEdit()

lsv_params_startpot_hlayout.addWidget(lsv_params_startpot_label)
lsv_params_startpot_hlayout.addWidget(lsv_params_startpot_entry)
lsv_params_box_layout.addLayout(lsv_params_startpot_hlayout)

# Stop potential sequence
lsv_params_stoppot_hlayout = QtWidgets.QHBoxLayout()
lsv_params_stoppot_label = QtWidgets.QLabel(text="Stop potentials (csv, V)")
lsv_params_stoppot_label.setToolTip(
	"Stop potentials for experiments to cycle through.\n\n"
	"Accepts numeric values or 'OCP'."
)
lsv_params_stoppot_entry = QtWidgets.QLineEdit()

lsv_params_stoppot_hlayout.addWidget(lsv_params_stoppot_label)
lsv_params_stoppot_hlayout.addWidget(lsv_params_stoppot_entry)
lsv_params_box_layout.addLayout(lsv_params_stoppot_hlayout)

# Sweep scanrate
lsv_params_scanrate_hlayout = QtWidgets.QHBoxLayout()
lsv_params_scanrate_label = QtWidgets.QLabel(text="Sweep scan rates (csv, mV/s)")
lsv_params_scanrate_label.setToolTip(
	"Scan rates to cycle through during LSV measurements.\n\n"
	"Experiments will cycle through each scan rate at each\n"
	"start/stop potential.\n\n"
	"Must be positive numeric values."
)
lsv_params_scanrate_entry = QtWidgets.QLineEdit()

lsv_params_scanrate_hlayout.addWidget(lsv_params_scanrate_label)
lsv_params_scanrate_hlayout.addWidget(lsv_params_scanrate_entry)
lsv_params_box_layout.addLayout(lsv_params_scanrate_hlayout)

# Negative current limits
lsv_params_current_limit_negative_hlayout = QtWidgets.QHBoxLayout()
lsv_params_current_limit_negative_label = QtWidgets.QLabel(text="Negative current limits (csv, µA)")
lsv_params_current_limit_negative_label.setToolTip(
	"Negative current limits for the experiments.\n"
	"Each current corresponds to a scan rate.\n\n"
	"Accepts numeric values or 'None'."
)
lsv_params_current_limit_negative_entry = QtWidgets.QLineEdit()
lsv_params_current_limit_negative_checkbox = QtWidgets.QCheckBox()
lsv_params_current_limit_negative_checkbox.setToolTip(
	"If checked, the given negative current limits are applied.\n\n"
	"If unchecked, the negative current limits are not applied."
)
lsv_params_current_limit_negative_checkbox.setChecked(False)

lsv_params_current_limit_negative_hlayout.addWidget(lsv_params_current_limit_negative_label)
lsv_params_current_limit_negative_hlayout.addWidget(lsv_params_current_limit_negative_entry)
lsv_params_current_limit_negative_hlayout.addWidget(lsv_params_current_limit_negative_checkbox)
lsv_params_box_layout.addLayout(lsv_params_current_limit_negative_hlayout)


# Positive reverse currents
lsv_params_current_limit_positive_hlayout = QtWidgets.QHBoxLayout()
lsv_params_current_limit_positive_label = QtWidgets.QLabel(text="Positive current limits (csv, µA)")
lsv_params_current_limit_positive_label.setToolTip(
	"Positive current limits for the experiments.\n"
	"Each current corresponds to a scan rate.\n\n"
	"Accepts numeric values or 'None'."
)
lsv_params_current_limit_positive_entry = QtWidgets.QLineEdit()
lsv_params_current_limit_positive_checkbox = QtWidgets.QCheckBox()
lsv_params_current_limit_positive_checkbox.setToolTip(
	"If checked, the given positive current limits are applied.\n\n"
	"If unchecked, the positive current limits are not applied."
)
lsv_params_current_limit_positive_checkbox.setChecked(False)

lsv_params_current_limit_positive_hlayout.addWidget(lsv_params_current_limit_positive_label)
lsv_params_current_limit_positive_hlayout.addWidget(lsv_params_current_limit_positive_entry)
lsv_params_current_limit_positive_hlayout.addWidget(lsv_params_current_limit_positive_checkbox)
lsv_params_box_layout.addLayout(lsv_params_current_limit_positive_hlayout)

lsv_params_box_layout.addWidget(create_line())

# Initialisation parameters
lsv_params_initialisation_label = QtWidgets.QLabel("<u>Initialisation parameters:</u>")
lsv_params_initialisation_label.setToolTip(
	"Parameters to control cell behaviour to initialise each LSV measurement."
)
lsv_params_box_layout.addWidget(lsv_params_initialisation_label)

# Initialisation scan rate
lsv_params_init_scanrate_hlayout = QtWidgets.QHBoxLayout()
lsv_params_init_scanrate_label = QtWidgets.QLabel(text="Scan rates (csv, mV/s)")
lsv_params_init_scanrate_label.setToolTip(
	"Initialisation scan rates to starting potentials.\n\n"
	"Experiments will cycle through these scan rates\n"
	"for its corresponding sweep scan rate.\n\n"
	"Must be positive numeric values or 'STEP'."
)
lsv_params_init_scanrate_entry = QtWidgets.QLineEdit()
lsv_params_init_scanrate_checkbox = QtWidgets.QCheckBox()
lsv_params_init_scanrate_checkbox.setToolTip(
	"If checked, the given initialisation scan rates are applied to\n"
	"reach the starting potential.\n\n"
	"If unchecked, the potential is stepped to the next starting potential."
)

lsv_params_init_scanrate_hlayout.addWidget(lsv_params_init_scanrate_checkbox)
lsv_params_init_scanrate_hlayout.addWidget(lsv_params_init_scanrate_label)
lsv_params_init_scanrate_hlayout.addWidget(lsv_params_init_scanrate_entry)
lsv_params_box_layout.addLayout(lsv_params_init_scanrate_hlayout)

# Initialisation hold time
lsv_params_init_holdtime_hlayout = QtWidgets.QHBoxLayout()
lsv_params_init_holdtime_label = QtWidgets.QLabel(text="Initialisation hold time (s)")
lsv_params_init_holdtime_label.setToolTip(
	"Duration to hold at the starting potential.\n"
	"before beginning each LSV experiment.\n\n"
	"Must be a non-negative numeric value."
)
lsv_params_init_holdtime_entry = QtWidgets.QLineEdit()

lsv_params_init_holdtime_hlayout.addWidget(lsv_params_init_holdtime_label)
lsv_params_init_holdtime_hlayout.addWidget(lsv_params_init_holdtime_entry)
lsv_params_box_layout.addLayout(lsv_params_init_holdtime_hlayout)

lsv_params_box_layout.addWidget(create_line())

# Number of samples to average
lsv_params_num_samples_hlayout = QtWidgets.QHBoxLayout()
lsv_params_num_samples_label = QtWidgets.QLabel(text="Number of samples to average per scan rate")
lsv_params_num_samples_label.setToolTip(
	"Number of data points to average for each scan rate.\n\n"
	"Must be a single positive integer value or a positive\n"
	"integer value for each scan rate.\n\n"
	"If a single integer is given but multiple scan rates exist,\n"
	"the number of samples to average will be scaled based on\n"
	"scan rate to maintain sampling resolution in the potential domain.\n"
	"This scaling uses the given integer as the reference for\n"
	"the lowest scan rate, adjusting as scan rate increases."
)
lsv_params_num_samples_entry = QtWidgets.QLineEdit()

lsv_params_num_samples_hlayout.addWidget(lsv_params_num_samples_label)
lsv_params_num_samples_hlayout.addWidget(lsv_params_num_samples_entry)
lsv_params_box_layout.addLayout(lsv_params_num_samples_hlayout)

# Pre-scan rate delay
lsv_params_scanrate_delay_hlayout = QtWidgets.QHBoxLayout()
lsv_params_scanrate_delay_label = QtWidgets.QLabel(text="Pre-scan rate delay (s)")
lsv_params_scanrate_delay_label.setToolTip(
	"Time delay before the experiment at the next scan rate begins to allow the system to settle.\n\n"
	"Must be a non-negative numeric value."
)
lsv_params_scanrate_delay_entry = QtWidgets.QLineEdit()

lsv_params_scanrate_delay_hlayout.addWidget(lsv_params_scanrate_delay_label)
lsv_params_scanrate_delay_hlayout.addWidget(lsv_params_scanrate_delay_entry)
lsv_params_box_layout.addLayout(lsv_params_scanrate_delay_hlayout)

# Pre-potential window delay
lsv_params_pot_window_delay_hlayout = QtWidgets.QHBoxLayout()
lsv_params_pot_window_delay_label = QtWidgets.QLabel(text="Pre-potential window delay (s)")
lsv_params_pot_window_delay_label.setToolTip(
	"Time delay before experiments at each potential window begin to allow the system to settle.\n\n"
	"Required if not waiting for OCP equilibration.\n\n"
	"Must be a non-negative numeric value."
)
lsv_params_pot_window_delay_entry = QtWidgets.QLineEdit()
lsv_params_pot_window_delay_OCP_checkbox = QtWidgets.QCheckBox("Wait for OCP equilibration")
lsv_params_pot_window_delay_OCP_checkbox.setToolTip(
	"<html><body>"
	"Wait for OCP equilibration before experiments at each potential window begin instead of using a fixed delay.<br><br>"
	"If checked, OCP equilibration is performed before every potential window to ensure consistent starting conditions.<br><br>"
	"<b>NOTE:</b> Mixing equilibration and fixed delays between potential windows within the same set of experiments is not supported. For users wishing to do this, please run separate sets of experiments.<br><br>"
	"Automatically checked if any start or stop potential set as 'OCP'."
	"</body></html>"
)

lsv_params_pot_window_delay_hlayout.addWidget(lsv_params_pot_window_delay_label)
lsv_params_pot_window_delay_hlayout.addWidget(lsv_params_pot_window_delay_entry)
lsv_params_pot_window_delay_hlayout.addWidget(lsv_params_pot_window_delay_OCP_checkbox)
lsv_params_box_layout.addLayout(lsv_params_pot_window_delay_hlayout)

lsv_params_box_layout.setSpacing(5)
lsv_params_box_layout.setContentsMargins(3, 10, 3, 3)
lsv_vbox.addWidget(lsv_params_box)

# Checkbutton box
lsv_checking_box = QtWidgets.QGroupBox(title="THIS BUTTON MUST BE PRESSED", flat=False)
format_box_for_parameter_centered_title(lsv_checking_box)
lsv_checking_layout = QtWidgets.QHBoxLayout()
lsv_checking_box.setLayout(lsv_checking_layout)

lsv_variables_checkbutton = QtWidgets.QPushButton("CHECK")
lsv_variables_checkbutton.clicked.connect(lsv_checkbutton_callback)
lsv_checking_layout.addWidget(lsv_variables_checkbutton)

lsv_checking_layout.setSpacing(5)
lsv_checking_layout.setContentsMargins(3, 10, 3, 3)
lsv_vbox.addWidget(lsv_checking_box)

# Autoranging box
lsv_range_box = QtWidgets.QGroupBox(title="Autoranging", flat=False)
lsv_range_box.setToolTip(
	"Enable autoranging to automatically select the optimal current range\n"
	"based on the measured current during the experiment.\n\n"
	"This feature allows the device to accurately measure currents across\n"
	"a wide dynamic range, from nanoamps up to 25 mA, by adapting to the\n"
	"current level in real time.\n\n"
	"If specific current ranges are undesirable for these experiments,\n"
	"untick their checkboxes to prevent them from being selected by autoranging."
)
format_box_for_parameter(lsv_range_box)
lsv_range_layout = QtWidgets.QHBoxLayout()
lsv_range_box.setLayout(lsv_range_layout)
lsv_range_layout.setAlignment(QtCore.Qt.AlignCenter)
lsv_range_checkboxes = []
for curr in current_range_list:
	checkbox = QtWidgets.QCheckBox(curr)
	lsv_range_checkboxes.append(checkbox)
	lsv_range_layout.addWidget(checkbox)
	checkbox.setChecked(True)

lsv_range_layout.setSpacing(50)
lsv_range_layout.setContentsMargins(3, 10, 3, 3)
lsv_vbox.addWidget(lsv_range_box)

# File box
lsv_file_box = QtWidgets.QGroupBox(title="Output filepath (experiment info will be appended)", flat=False)
format_box_for_parameter(lsv_file_box)
lsv_file_layout = QtWidgets.QVBoxLayout()
lsv_file_box.setLayout(lsv_file_layout)

# Filename
lsv_file_choose_hlayout = QtWidgets.QHBoxLayout()
lsv_file_entry = QtWidgets.QLineEdit()
lsv_file_entry.setToolTip(
	"Base output filename for these experiments.\n\n"
	"Each experiment's data file will append '_LSV' and details for\n"
	"the corresponding potential window and scan rate, saved as\n"
	"a .dat file.\n\n"
	"A summary file will also be created as a .txt file using the\n"
	"given base output filename."
)
lsv_file_choose_button = QtWidgets.QPushButton("...")
lsv_file_choose_button.setFixedWidth(32)
lsv_file_choose_button.clicked.connect(lambda: choose_file(lsv_file_entry, "Choose where to save the CV measurement data"))
lsv_file_choose_hlayout.addWidget(lsv_file_entry)
lsv_file_choose_hlayout.addWidget(lsv_file_choose_button)
lsv_file_layout.addLayout(lsv_file_choose_hlayout)

# Notes
lsv_file_notes_entry = QtWidgets.QTextEdit()
lsv_file_notes_entry.setPlaceholderText("*** Optional experiment notes to write in summary file ***")
lsv_file_notes_entry.setStyleSheet("""
	QTextEdit {
		color: black;
	}
	QTextEdit: empty {
		color:grey;
	}
""")
line_height = lsv_file_notes_entry.fontMetrics().lineSpacing()
num_lines = 5
lsv_file_notes_entry.setMaximumHeight(line_height * num_lines)
lsv_file_notes_entry.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
lsv_file_layout.addWidget(lsv_file_notes_entry)

lsv_file_layout.setSpacing(5)
lsv_file_layout.setContentsMargins(3, 10, 3, 3)
lsv_vbox.addWidget(lsv_file_box)

# Check button reset behaviour if parameter or file inputs change
lsv_params_startpot_entry.textChanged.connect(lambda: lsv_reset_experiment_controller(mode="input_changed"))
lsv_params_stoppot_entry.textChanged.connect(lambda: lsv_reset_experiment_controller(mode="input_changed"))
lsv_params_scanrate_entry.textChanged.connect(lambda: lsv_reset_experiment_controller(mode="input_changed"))
lsv_params_current_limit_negative_entry.textChanged.connect(lambda: lsv_reset_experiment_controller(mode="input_changed"))
lsv_params_current_limit_negative_checkbox.stateChanged.connect(lambda: lsv_reset_experiment_controller(mode="input_changed"))
lsv_params_current_limit_positive_entry.textChanged.connect(lambda: lsv_reset_experiment_controller(mode="input_changed"))
lsv_params_current_limit_positive_checkbox.stateChanged.connect(lambda: lsv_reset_experiment_controller(mode="input_changed"))
lsv_params_init_scanrate_entry.textChanged.connect(lambda: lsv_reset_experiment_controller(mode="input_changed"))
lsv_params_init_scanrate_checkbox.stateChanged.connect(lambda: lsv_reset_experiment_controller(mode="input_changed"))
lsv_params_init_holdtime_entry.textChanged.connect(lambda: lsv_reset_experiment_controller(mode="input_changed"))
lsv_params_num_samples_entry.textChanged.connect(lambda: lsv_reset_experiment_controller(mode="input_changed"))
lsv_params_scanrate_delay_entry.textChanged.connect(lambda: lsv_reset_experiment_controller(mode="input_changed"))
lsv_params_pot_window_delay_entry.textChanged.connect(lambda: lsv_reset_experiment_controller(mode="input_changed"))
lsv_params_pot_window_delay_OCP_checkbox.stateChanged.connect(lambda: lsv_reset_experiment_controller(mode="input_changed"))
lsv_file_entry.textChanged.connect(lambda: lsv_reset_experiment_controller(mode="input_changed"))

# Preview, start, and stop buttons
lsv_preview_button = QtWidgets.QPushButton("Preview linear sweep voltammetries")
lsv_preview_button.clicked.connect(lsv_preview)
lsv_start_button = QtWidgets.QPushButton("Start linear sweep voltammetries")
lsv_start_button.clicked.connect(lsv_initialise)
lsv_stop_button = QtWidgets.QPushButton("Stop linear sweep voltammetries")
lsv_stop_button.clicked.connect(lambda: (lsv_delay_timer.stop(), lsv_stop(lsv_current_exp_index, interrupted=True)))

lsv_vbox.addWidget(lsv_preview_button)
lsv_vbox.addWidget(lsv_start_button)
lsv_vbox.addWidget(lsv_stop_button)
lsv_vbox.addWidget(create_spacer(5))

# Experiment info box
lsv_info_box = QtWidgets.QGroupBox(title="Experiment info", flat=False)
format_box_for_parameter_centered_title(lsv_info_box)
lsv_info_layout = QtWidgets.QVBoxLayout()
lsv_info_box.setLayout(lsv_info_layout)

# Current program state
lsv_info_program_state_entry = make_label_entry(lsv_info_layout, "Current program state:")
lsv_info_program_state_entry.setText("No experiments running")
lsv_info_program_state_entry.setReadOnly(True)

# Experiment number and segment
lsv_info_hlayout = QtWidgets.QHBoxLayout()
lsv_info_expnum_label = QtWidgets.QLabel(text="Experiment number:")
lsv_info_expnum_entry = QtWidgets.QLineEdit()
lsv_info_expnum_entry.setText("-/-")
lsv_info_expnum_entry.setReadOnly(True)
lsv_info_expnum_entry.setAlignment(QtCore.Qt.AlignCenter)
lsv_info_current_segment_label = QtWidgets.QLabel(text="Segment:")
lsv_info_current_segment_entry = QtWidgets.QLineEdit()
lsv_info_current_segment_entry.setText("-")
lsv_info_current_segment_entry.setReadOnly(True)
lsv_info_current_segment_entry.setAlignment(QtCore.Qt.AlignCenter)

lsv_info_hlayout.addWidget(lsv_info_expnum_label)
lsv_info_hlayout.addWidget(lsv_info_expnum_entry, stretch=1)
lsv_info_hlayout.addWidget(lsv_info_current_segment_label)
lsv_info_hlayout.addWidget(lsv_info_current_segment_entry, stretch=2)
lsv_info_layout.addLayout(lsv_info_hlayout)

lsv_info_layout.setSpacing(5)
lsv_info_layout.setContentsMargins(3, 10, 3, 3)
lsv_vbox.addWidget(lsv_info_box)

# Progress bar box
lsv_progress_box = QtWidgets.QGroupBox(title="Estimated time remaining", flat=False)
format_box_for_parameter_centered_title(lsv_progress_box)
lsv_progress_layout = QtWidgets.QHBoxLayout()
lsv_progress_box.setLayout(lsv_progress_layout)

lsv_progress_bar = TimeProgressBar()
lsv_progress_layout.addWidget(lsv_progress_bar)

lsv_progress_layout.setSpacing(5)
lsv_progress_layout.setContentsMargins(3, 10, 3, 3)
lsv_vbox.addWidget(lsv_progress_box)

# Plot options box
lsv_plot_options_box = QtWidgets.QGroupBox(title="Plot options", flat=False)
format_box_for_parameter_centered_title(lsv_plot_options_box)
lsv_plot_options_layout = QtWidgets.QVBoxLayout()
lsv_plot_options_box.setLayout(lsv_plot_options_layout)

lsv_plot_options_current_limits_checkbox = QtWidgets.QCheckBox("Show current limits for this experiment")
lsv_plot_options_current_limits_checkbox.setChecked(False)

lsv_plot_options_prev_experiments_hlayout = QtWidgets.QHBoxLayout()
lsv_plot_options_prev_experiments_checkbox = QtWidgets.QCheckBox("Show plots from")
lsv_plot_options_prev_experiments_checkbox.setChecked(False)
lsv_plot_options_prev_experiments_dropdown = QtWidgets.QComboBox()
lsv_plot_options_prev_experiments_dropdown.addItems(["all previous experiments", "same potential window", "same scan rate"])
lsv_plot_options_prev_experiments_hlayout.addWidget(lsv_plot_options_prev_experiments_checkbox)
lsv_plot_options_prev_experiments_hlayout.addWidget(lsv_plot_options_prev_experiments_dropdown)

lsv_plot_options_init_segments_checkbox = QtWidgets.QCheckBox("Show initialisation segments")
lsv_plot_options_init_segments_checkbox.setChecked(False)

lsv_plot_options_layout.addWidget(lsv_plot_options_current_limits_checkbox)
lsv_plot_options_layout.addLayout(lsv_plot_options_prev_experiments_hlayout)
lsv_plot_options_layout.addWidget(lsv_plot_options_init_segments_checkbox)

lsv_plot_options_layout.setAlignment(QtCore.Qt.AlignCenter)
lsv_plot_options_layout.setSpacing(5)
lsv_plot_options_layout.setContentsMargins(3, 10, 3, 3)
lsv_vbox.addWidget(lsv_plot_options_box)

lsv_vbox.setSpacing(5)
lsv_vbox.setContentsMargins(3, 3, 3, 3)

# Make scrollable area
lsv_widget = QtWidgets.QWidget()
lsv_widget.setLayout(lsv_vbox)
lsv_widget.setContentsMargins(0, 0, 0, 0)

lsv_scroll_area = QtWidgets.QScrollArea()
lsv_scroll_area.setWidgetResizable(True)
lsv_scroll_area.setWidget(lsv_widget)
lsv_scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
lsv_scroll_area.setContentsMargins(0, 0, 0, 0)

lsv_layout = QtWidgets.QVBoxLayout()
lsv_layout.setContentsMargins(0, 0, 0, 0)
lsv_layout.addWidget(lsv_scroll_area)
tabs[2].setLayout(lsv_layout)


"""GALVANOSTATIC CHARGE/DISCHARGE CYCLING TAB"""

gcd_vbox = QtWidgets.QVBoxLayout()
gcd_vbox.setAlignment(QtCore.Qt.AlignTop)

# Parameters box
gcd_params_box = QtWidgets.QGroupBox(title="Galvanostatic charge/discharge parameters", flat=False)
format_box_for_parameter(gcd_params_box)
gcd_params_box_layout = QtWidgets.QVBoxLayout()
gcd_params_box.setLayout(gcd_params_box_layout)

# Lower potential bounds
gcd_params_lbound_hlayout = QtWidgets.QHBoxLayout()
gcd_params_lbound_label = QtWidgets.QLabel(text="Lower potential limits (csv, V)")
gcd_params_lbound_label.setToolTip(
	"Lower potential limits for experiments to cycle through.\n\n"
	"Accepts numeric values or 'OCP'."
)
gcd_params_lbound_entry = QtWidgets.QLineEdit()

gcd_params_lbound_hlayout.addWidget(gcd_params_lbound_label)
gcd_params_lbound_hlayout.addWidget(gcd_params_lbound_entry)
gcd_params_box_layout.addLayout(gcd_params_lbound_hlayout)

# Upper potential bounds
gcd_params_ubound_hlayout = QtWidgets.QHBoxLayout()
gcd_params_ubound_label = QtWidgets.QLabel(text="Upper potential limits (csv, V)")
gcd_params_ubound_label.setToolTip(
	"Upper potential limits for experiments to cycle through.\n\n"
	"Accepts numeric values or 'OCP'."
)
gcd_params_ubound_entry = QtWidgets.QLineEdit()

gcd_params_ubound_hlayout.addWidget(gcd_params_ubound_label)
gcd_params_ubound_hlayout.addWidget(gcd_params_ubound_entry)
gcd_params_box_layout.addLayout(gcd_params_ubound_hlayout)

# Charge currents
gcd_params_chargecurrent_hlayout = QtWidgets.QHBoxLayout()
gcd_params_chargecurrent_label = QtWidgets.QLabel(text="Charge currents (csv, µA)")
gcd_params_chargecurrent_label.setToolTip(
	"Charge currents to cycle through.\n\n"
	"Experiments will cycle through each charge/discharge\n"
	"current at each potential window.\n\n"
	"Must be positive numeric values."
)
gcd_params_chargecurrent_entry = QtWidgets.QLineEdit()

gcd_params_chargecurrent_hlayout.addWidget(gcd_params_chargecurrent_label)
gcd_params_chargecurrent_hlayout.addWidget(gcd_params_chargecurrent_entry)
gcd_params_box_layout.addLayout(gcd_params_chargecurrent_hlayout)

# Discharge currents
gcd_params_dischargecurrent_hlayout = QtWidgets.QHBoxLayout()
gcd_params_dischargecurrent_label = QtWidgets.QLabel(text="Discharge currents (csv, µA)")
gcd_params_dischargecurrent_label.setToolTip(
	"Discharge currents to cycle through.\n\n"
	"Experiments will cycle through each charge/discharge\n"
	"current at each potential window.\n\n"
	"Must be negative numeric values."
)
gcd_params_dischargecurrent_entry = QtWidgets.QLineEdit()

gcd_params_dischargecurrent_hlayout.addWidget(gcd_params_dischargecurrent_label)
gcd_params_dischargecurrent_hlayout.addWidget(gcd_params_dischargecurrent_entry)
gcd_params_box_layout.addLayout(gcd_params_dischargecurrent_hlayout)

# Number of half cycles per charge/discharge current
gcd_params_num_halfcycles_hlayout = QtWidgets.QHBoxLayout()
gcd_params_num_halfcycles_label = QtWidgets.QLabel(text="Number of half cycles per experiment")
gcd_params_num_halfcycles_label.setToolTip(
	"Number of half cycles per experiment, where each\n"
	"charge or discharge stage counts as a half cycle.\n\n"
	"Can be updated mid-experiment using the\n"
	"'Update no. of half cycles' box below.\n\n"
	"Must be a positive integer value."
)
gcd_params_num_halfcycles_entry = QtWidgets.QLineEdit()

gcd_params_num_halfcycles_hlayout.addWidget(gcd_params_num_halfcycles_label)
gcd_params_num_halfcycles_hlayout.addWidget(gcd_params_num_halfcycles_entry)
gcd_params_box_layout.addLayout(gcd_params_num_halfcycles_hlayout)

# Number of samples to average
gcd_params_num_samples_hlayout = QtWidgets.QHBoxLayout()
gcd_params_num_samples_label = QtWidgets.QLabel(text="Number of samples to average")
gcd_params_num_samples_label.setToolTip(
	"Number of data points to average for each charge/discharge current.\n\n"
	"Must be a single positive integer value or a positive integer value\n"
	"for each charge/discharge current.\n\n"
	"If a single integer is given but multiple charge/discharge currents\n"
	"are given, this value is applied to all charge/discharge currents."
)
gcd_params_num_samples_entry = QtWidgets.QLineEdit()
gcd_params_num_samples_hlayout.addWidget(gcd_params_num_samples_label)
gcd_params_num_samples_hlayout.addWidget(gcd_params_num_samples_entry)
gcd_params_box_layout.addLayout(gcd_params_num_samples_hlayout)

# Pre-charge/discharge current delay
gcd_params_current_delay_hlayout = QtWidgets.QHBoxLayout()
gcd_params_current_delay_label = QtWidgets.QLabel(text="Pre-current delay (s)")
gcd_params_current_delay_label.setToolTip(
	"Time delay before the experiment at the next charge/discharge current begins to allow the system to settle.\n\n"
	"Must be a non-negative numeric value."
)
gcd_params_current_delay_entry = QtWidgets.QLineEdit()

gcd_params_current_delay_hlayout.addWidget(gcd_params_current_delay_label)
gcd_params_current_delay_hlayout.addWidget(gcd_params_current_delay_entry)
gcd_params_box_layout.addLayout(gcd_params_current_delay_hlayout)

# Pre-potential window delay
gcd_params_pot_window_delay_hlayout = QtWidgets.QHBoxLayout()
gcd_params_pot_window_delay_label = QtWidgets.QLabel(text="Pre-potential window delay (s)")
gcd_params_pot_window_delay_label.setToolTip(
	"Time delay before experiments at each potential window begin to allow the system to settle.\n\n"
	"Required if not waiting for OCP equilibration.\n\n"
	"Must be a non-negative numeric value."
)
gcd_params_pot_window_delay_entry = QtWidgets.QLineEdit()
gcd_params_pot_window_delay_OCP_checkbox = QtWidgets.QCheckBox("Wait for OCP equilibration")
gcd_params_pot_window_delay_OCP_checkbox.setToolTip(
	"<html><body>"
	"Wait for OCP equilibration before experiments at each potential window begin instead of using a fixed delay.<br><br>"
	"If checked, OCP equilibration is performed before every potential window to ensure consistent starting conditions.<br><br>"
	"<b>NOTE:</b> Mixing equilibration and fixed delays between potential windows within the same set of experiments is not supported. For users wishing to do this, please run separate sets of experiments.<br><br>"
	"Automatically checked if any potential limit set as 'OCP'."
	"</body></html>"
)

gcd_params_pot_window_delay_hlayout.addWidget(gcd_params_pot_window_delay_label)
gcd_params_pot_window_delay_hlayout.addWidget(gcd_params_pot_window_delay_entry)
gcd_params_pot_window_delay_hlayout.addWidget(gcd_params_pot_window_delay_OCP_checkbox)
gcd_params_box_layout.addLayout(gcd_params_pot_window_delay_hlayout)

gcd_params_box_layout.setSpacing(5)
gcd_params_box_layout.setContentsMargins(3, 10, 3, 3)
gcd_vbox.addWidget(gcd_params_box)

# Checkbutton box
gcd_checking_box = QtWidgets.QGroupBox(title="THIS BUTTON MUST BE PRESSED", flat=False)
format_box_for_parameter_centered_title(gcd_checking_box)
gcd_checking_layout = QtWidgets.QHBoxLayout()
gcd_checking_box.setLayout(gcd_checking_layout)

gcd_variables_checkbutton = QtWidgets.QPushButton("CHECK")
gcd_variables_checkbutton.clicked.connect(gcd_checkbutton_callback)
gcd_checking_layout.addWidget(gcd_variables_checkbutton)

gcd_checking_layout.setSpacing(5)
gcd_checking_layout.setContentsMargins(3, 10, 3, 3)
gcd_vbox.addWidget(gcd_checking_box)

# File box
gcd_file_box = QtWidgets.QGroupBox(title="Output filepath (experiment info will be appended)", flat=False)
format_box_for_parameter(gcd_file_box)
gcd_file_layout = QtWidgets.QVBoxLayout()
gcd_file_box.setLayout(gcd_file_layout)

# Filename
gcd_file_choose_hlayout = QtWidgets.QHBoxLayout()
gcd_file_entry = QtWidgets.QLineEdit()
gcd_file_entry.setToolTip(
	"Base output filename for these experiments.\n\n"
	"Each experiment will generate raw data and measured capacity per half cycle files,\n"
	"appending '_GCD' with the corresponding potential window and charge/discharge current details\n"
	"and saved as .dat files.\n\n"
	"A summary file will also be created as a .txt file using the given base output filename."
)
gcd_file_choose_button = QtWidgets.QPushButton("...")
gcd_file_choose_button.setFixedWidth(32)
gcd_file_choose_button.clicked.connect(lambda: choose_file(gcd_file_entry,"Choose where to save the charge/discharge measurement data"))

gcd_file_choose_hlayout.addWidget(gcd_file_entry)
gcd_file_choose_hlayout.addWidget(gcd_file_choose_button)
gcd_file_layout.addLayout(gcd_file_choose_hlayout)

# Notes
gcd_file_notes_entry = QtWidgets.QTextEdit()
gcd_file_notes_entry.setPlaceholderText("*** Optional experiment notes to write in summary file ***")
gcd_file_notes_entry.setStyleSheet("""
	QTextEdit {
		color: black;
	}
	QTextEdit: empty {
		color:grey;
	}
""")
line_height = gcd_file_notes_entry.fontMetrics().lineSpacing()
num_lines = 5
gcd_file_notes_entry.setMaximumHeight(line_height * num_lines)
gcd_file_notes_entry.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
gcd_file_layout.addWidget(gcd_file_notes_entry)

gcd_file_layout.setSpacing(5)
gcd_file_layout.setContentsMargins(3, 10, 3, 3)
gcd_vbox.addWidget(gcd_file_box)

# Check button reset behaviour if parameter or file inputs change
gcd_params_lbound_entry.textChanged.connect(lambda: gcd_reset_experiment_controller(mode="input_changed"))
gcd_params_ubound_entry.textChanged.connect(lambda: gcd_reset_experiment_controller(mode="input_changed"))
gcd_params_chargecurrent_entry.textChanged.connect(lambda: gcd_reset_experiment_controller(mode="input_changed"))
gcd_params_dischargecurrent_entry.textChanged.connect(lambda: gcd_reset_experiment_controller(mode="input_changed"))
gcd_params_num_halfcycles_entry.textChanged.connect(lambda: gcd_reset_experiment_controller(mode="input_changed"))
gcd_params_num_samples_entry.textChanged.connect(lambda: gcd_reset_experiment_controller(mode="input_changed"))
gcd_params_current_delay_entry.textChanged.connect(lambda: gcd_reset_experiment_controller(mode="input_changed"))
gcd_params_pot_window_delay_entry.textChanged.connect(lambda: gcd_reset_experiment_controller(mode="input_changed"))
gcd_params_pot_window_delay_OCP_checkbox.stateChanged.connect(lambda: gcd_reset_experiment_controller(mode="input_changed"))
gcd_file_entry.textChanged.connect(lambda: gcd_reset_experiment_controller(mode="input_changed"))

# Start and stop buttons
gcd_start_button = QtWidgets.QPushButton("Start charge/discharge experiments")
gcd_start_button.clicked.connect(gcd_initialise)
gcd_stop_button = QtWidgets.QPushButton("Stop charge/discharge experiments")
gcd_stop_button.clicked.connect(lambda: (gcd_delay_timer.stop(), gcd_stop(gcd_current_exp_index, interrupted=True)))

gcd_vbox.addWidget(gcd_start_button)
gcd_vbox.addWidget(gcd_stop_button)
gcd_vbox.addWidget(create_spacer(5))

# Experiment info box
gcd_info_box = QtWidgets.QGroupBox(title="Experiment info", flat=False)
format_box_for_parameter_centered_title(gcd_info_box)
gcd_info_layout = QtWidgets.QVBoxLayout()
gcd_info_box.setLayout(gcd_info_layout)

# Current program state
gcd_info_program_state_entry = make_label_entry(gcd_info_layout, "Current program state:")
gcd_info_program_state_entry.setText("No experiments running")
gcd_info_program_state_entry.setReadOnly(True)

# Experiment and half cycle numbers
gcd_info_hlayout = QtWidgets.QHBoxLayout()
gcd_info_expnum_label = QtWidgets.QLabel(text="Experiment number:")
gcd_info_expnum_entry = QtWidgets.QLineEdit()
gcd_info_expnum_entry.setText("-/-")
gcd_info_expnum_entry.setReadOnly(True)
gcd_info_expnum_entry.setAlignment(QtCore.Qt.AlignCenter)
gcd_info_halfcyclenum_label = QtWidgets.QLabel(text="Half cycle no.:")
gcd_info_halfcyclenum_entry = QtWidgets.QLineEdit()
gcd_info_halfcyclenum_entry.setText("-/-")
gcd_info_halfcyclenum_entry.setReadOnly(True)
gcd_info_halfcyclenum_entry.setAlignment(QtCore.Qt.AlignCenter)

gcd_info_hlayout.addWidget(gcd_info_expnum_label)
gcd_info_hlayout.addWidget(gcd_info_expnum_entry, stretch=1)
gcd_info_hlayout.addWidget(gcd_info_halfcyclenum_label)
gcd_info_hlayout.addWidget(gcd_info_halfcyclenum_entry, stretch=1)
gcd_info_layout.addLayout(gcd_info_hlayout)

gcd_info_layout.setSpacing(5)
gcd_info_layout.setContentsMargins(3, 10, 3, 3)
gcd_vbox.addWidget(gcd_info_box)

# Progress bar box
gcd_progress_box = QtWidgets.QGroupBox(title="Experiment progress", flat=False)
format_box_for_parameter_centered_title(gcd_progress_box)
gcd_progress_layout = QtWidgets.QHBoxLayout()
gcd_progress_box.setLayout(gcd_progress_layout)

gcd_progress_bar = LabeledProgressBar(label="Cycle")
gcd_progress_layout.addWidget(gcd_progress_bar)

gcd_progress_layout.setSpacing(5)
gcd_progress_layout.setContentsMargins(3, 10, 3, 3)
gcd_vbox.addWidget(gcd_progress_box)

# Plot options box
gcd_plot_options_box = QtWidgets.QGroupBox(title="Plot options", flat=False)
format_box_for_parameter_centered_title(gcd_plot_options_box)
gcd_plot_options_layout = QtWidgets.QVBoxLayout()
gcd_plot_options_box.setLayout(gcd_plot_options_layout)

# Display time or charge radio buttons
gcd_plot_options_timecharge_hlayout = QtWidgets.QHBoxLayout()
gcd_plot_options_timecharge_hlayout.setSpacing(10)
gcd_plot_options_x_time_radiobutton = QtWidgets.QRadioButton("Time (s)")
gcd_plot_options_x_time_radiobutton.setChecked(True)
gcd_plot_options_x_charge_radiobutton = QtWidgets.QRadioButton("Charge (Ah)")

gcd_plot_options_timecharge_hlayout.addWidget(QtWidgets.QLabel("Units:"))
gcd_plot_options_timecharge_hlayout.addWidget(gcd_plot_options_x_time_radiobutton)
gcd_plot_options_timecharge_hlayout.addWidget(gcd_plot_options_x_charge_radiobutton)
gcd_plot_options_timecharge_hlayout.setAlignment(QtCore.Qt.AlignLeft)
gcd_plot_options_layout.addLayout(gcd_plot_options_timecharge_hlayout)

gcd_plot_options_layout.addWidget(create_line())

# This cycle and previous 10 cycles radiobuttons
gcd_plot_options_this_cycle_radiobutton = QtWidgets.QRadioButton("Show this cycle only")
gcd_plot_options_this_cycle_radiobutton.setChecked(True)
gcd_plot_options_prev10_cycles_radiobutton = QtWidgets.QRadioButton("Show previous 10 full cycles from this experiment")

# Nth cycle radiobutton
gcd_plot_options_nth_cycles_hlayout = QtWidgets.QHBoxLayout()
gcd_plot_options_nth_cycles_hlayout.setSpacing(0)
gcd_plot_options_nth_cycles_radiobutton = QtWidgets.QRadioButton("Show every")
gcd_plot_options_nth_cycles_dropdown = QtWidgets.QComboBox()
gcd_plot_options_nth_cycles_dropdown.addItems([])

gcd_plot_options_nth_cycles_hlayout.addWidget(gcd_plot_options_nth_cycles_radiobutton)
gcd_plot_options_nth_cycles_hlayout.addWidget(gcd_plot_options_nth_cycles_dropdown)
gcd_plot_options_nth_cycles_hlayout.addWidget(QtWidgets.QLabel(" th full cycle from this experiment"))

gcd_plot_options_layout.addWidget(gcd_plot_options_this_cycle_radiobutton)
gcd_plot_options_layout.addWidget(gcd_plot_options_prev10_cycles_radiobutton)
gcd_plot_options_layout.addLayout(gcd_plot_options_nth_cycles_hlayout)

gcd_plot_options_layout.addWidget(create_line())

# Previous experiments checkbox
gcd_plot_options_prev_experiments_hlayout = QtWidgets.QHBoxLayout()
gcd_plot_options_prev_experiments_hlayout.setSpacing(0)
gcd_plot_options_prev_experiments_checkbox = QtWidgets.QCheckBox("Show final full cycle from")
gcd_plot_options_prev_experiments_checkbox.setChecked(False)
gcd_plot_options_prev_experiments_dropdown = QtWidgets.QComboBox()
gcd_plot_options_prev_experiments_dropdown.addItems(["all previous experiments", "same potential window", "same charge/discharge current"])

gcd_plot_options_prev_experiments_hlayout.addWidget(gcd_plot_options_prev_experiments_checkbox)
gcd_plot_options_prev_experiments_hlayout.addWidget(gcd_plot_options_prev_experiments_dropdown)
gcd_plot_options_layout.addLayout(gcd_plot_options_prev_experiments_hlayout)

# Make time/charge button group
gcd_plot_options_timecharge_group = QtWidgets.QButtonGroup()
gcd_plot_options_timecharge_group.addButton(gcd_plot_options_x_time_radiobutton)
gcd_plot_options_timecharge_group.addButton(gcd_plot_options_x_charge_radiobutton)

# Make this cycle/previous 10 cycles/nth cycles button group
gcd_plot_options_current_experiment_group = QtWidgets.QButtonGroup()
gcd_plot_options_current_experiment_group.addButton(gcd_plot_options_this_cycle_radiobutton)
gcd_plot_options_current_experiment_group.addButton(gcd_plot_options_prev10_cycles_radiobutton)
gcd_plot_options_current_experiment_group.addButton(gcd_plot_options_nth_cycles_radiobutton)

gcd_plot_options_layout.setAlignment(QtCore.Qt.AlignCenter)
gcd_plot_options_layout.setSpacing(5)
gcd_plot_options_layout.setContentsMargins(3, 10, 3, 3)
gcd_vbox.addWidget(gcd_plot_options_box)

gcd_vbox.addWidget(create_spacer(5))

# Update half cycles box
gcd_update_num_halfcycles_input_box = QtWidgets.QGroupBox(title="Update no. of half cycles (mid-experiment)", flat=False)
gcd_update_num_halfcycles_input_box.setToolTip(
	"Update the number of half cycles during ongoing experiments.\n\n"
	"Applies the new number of half cycles to this and all\n"
	"remaining experiments.\n\n"
	"Must be a positive integer value."
)
format_box_for_parameter_centered_title(gcd_update_num_halfcycles_input_box)
gcd_update_num_halfcycles_input_layout = QtWidgets.QVBoxLayout()
gcd_update_num_halfcycles_input_box.setLayout(gcd_update_num_halfcycles_input_layout)

gcd_update_hlayout = QtWidgets.QHBoxLayout()
gcd_update_num_halfcycles_input_label = QtWidgets.QLabel(text="No. of half cycles:")
gcd_update_num_halfcycles_input_entry = QtWidgets.QLineEdit()
gcd_update_num_halfcycles_input_button = QtWidgets.QPushButton("UPDATE")
gcd_update_num_halfcycles_input_button_timer = None
gcd_update_num_halfcycles_input_button.clicked.connect(gcd_update_num_halfcycles_input)
gcd_update_num_halfcycles_input_button.setEnabled(False)

gcd_update_hlayout.addWidget(gcd_update_num_halfcycles_input_label)
gcd_update_hlayout.addWidget(gcd_update_num_halfcycles_input_entry)
gcd_update_hlayout.addWidget(gcd_update_num_halfcycles_input_button)

gcd_update_num_halfcycles_input_layout.addLayout(gcd_update_hlayout)
gcd_update_num_halfcycles_input_layout.setSpacing(5)
gcd_update_num_halfcycles_input_layout.setContentsMargins(3, 10, 3, 3)
gcd_vbox.addWidget(gcd_update_num_halfcycles_input_box)

gcd_vbox.setSpacing(5)
gcd_vbox.setContentsMargins(3, 3, 3, 3)

# Make scrollable area
gcd_widget = QtWidgets.QWidget()
gcd_widget.setLayout(gcd_vbox)
gcd_widget.setContentsMargins(0, 0, 0, 0)

gcd_scroll_area = QtWidgets.QScrollArea()
gcd_scroll_area.setWidgetResizable(True)
gcd_scroll_area.setWidget(gcd_widget)
gcd_scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
gcd_scroll_area.setContentsMargins(0, 0, 0, 0)

gcd_layout = QtWidgets.QVBoxLayout()
gcd_layout.setContentsMargins(0, 0, 0, 0)
gcd_layout.addWidget(gcd_scroll_area)
tabs[3].setLayout(gcd_layout)


"""CHRONOAMPEROMETRY TAB"""

ca_vbox = QtWidgets.QVBoxLayout()
ca_vbox.setAlignment(QtCore.Qt.AlignTop)

# Parameters box
ca_params_box = QtWidgets.QGroupBox(title="Chronoamperometry parameters", flat=False)
format_box_for_parameter(ca_params_box)
ca_params_box_layout = QtWidgets.QVBoxLayout()
ca_params_box.setLayout(ca_params_box_layout)

# Potential A sequence
ca_params_A_potential_hlayout = QtWidgets.QHBoxLayout()
ca_params_A_potential_label = QtWidgets.QLabel("Potential sequence (csv, V)")
ca_params_A_potential_label.setToolTip(
	"Potentials to cycle through.\n\n"
	"Accepts numeric values or 'OCP'."
)
ca_params_A_potential_entry = QtWidgets.QLineEdit()
ca_params_A_potential_hlayout.addWidget(ca_params_A_potential_label)
ca_params_A_potential_hlayout.addWidget(ca_params_A_potential_entry)
ca_params_box_layout.addLayout(ca_params_A_potential_hlayout)

# Hold time A sequence
ca_params_A_hold_time_hlayout = QtWidgets.QHBoxLayout()
ca_params_A_hold_time_label = QtWidgets.QLabel("Hold time per potential (csv, s)")
ca_params_A_hold_time_label.setToolTip(
	"Hold time for each potential.\n\n"
	"Must be non-negative numeric values."
)
ca_params_A_hold_time_entry = QtWidgets.QLineEdit()
ca_params_A_hold_time_hlayout.addWidget(ca_params_A_hold_time_label)
ca_params_A_hold_time_hlayout.addWidget(ca_params_A_hold_time_entry)
ca_params_box_layout.addLayout(ca_params_A_hold_time_hlayout)

# Ramp rate A sequence
ca_params_A_ramp_rate_hlayout = QtWidgets.QHBoxLayout()
ca_params_A_ramp_rate_checkbox = QtWidgets.QCheckBox()
ca_params_A_ramp_rate_checkbox.setToolTip(
	"If checked, the given ramp rates are applied to reach the potential.\n\n"
	"If unchecked, the potential is stepped."
)
ca_params_A_ramp_rate_label = QtWidgets.QLabel("Potential ramp rates (csv, mV/s)")
ca_params_A_ramp_rate_label.setToolTip(
	"Ramp rate to each potential.\n\n"
	"Accepts positive numeric values or 'STEP'."
)
ca_params_A_ramp_rate_entry = QtWidgets.QLineEdit()
ca_params_A_ramp_rate_hlayout.addWidget(ca_params_A_ramp_rate_checkbox)
ca_params_A_ramp_rate_hlayout.addWidget(ca_params_A_ramp_rate_label)
ca_params_A_ramp_rate_hlayout.addWidget(ca_params_A_ramp_rate_entry)
ca_params_box_layout.addLayout(ca_params_A_ramp_rate_hlayout)

# Potential equilibration A
ca_params_A_equilibration_tolerance_hlayout = QtWidgets.QHBoxLayout()
ca_params_A_equilibration_checkbox = QtWidgets.QCheckBox()
ca_params_A_equilibration_checkbox.setToolTip(
	"If checked, the given equilibration parameters are applied.\n\n"
	"If unchecked, the equilibration parameters are not applied."
)
ca_params_A_equilibration_tolerance_label = QtWidgets.QLabel("Equilibration tolerance (µA)")
ca_params_A_equilibration_tolerance_label.setToolTip(
	"The change in current below which the system considers\n"
	"the current to be equilibrated.\n\n"
	"Must be a non-negative numeric value."
)
ca_params_A_equilibration_tolerance_entry = QtWidgets.QLineEdit()
ca_params_A_equilibration_timescale_label = QtWidgets.QLabel("Timescale (s)")
ca_params_A_equilibration_timescale_label.setToolTip(
	"The timescale over which the change in current is measured\n"
	"to determine equilibration.\n\n"
	"Must be a positive numeric value."
)
ca_params_A_equilibration_timescale_entry = QtWidgets.QLineEdit()
ca_params_A_equilibration_tolerance_hlayout.addWidget(ca_params_A_equilibration_checkbox)
ca_params_A_equilibration_tolerance_hlayout.addWidget(ca_params_A_equilibration_tolerance_label)
ca_params_A_equilibration_tolerance_hlayout.addWidget(ca_params_A_equilibration_tolerance_entry)
ca_params_A_equilibration_tolerance_hlayout.addWidget(ca_params_A_equilibration_timescale_label)
ca_params_A_equilibration_tolerance_hlayout.addWidget(ca_params_A_equilibration_timescale_entry)
ca_params_box_layout.addLayout(ca_params_A_equilibration_tolerance_hlayout)

class CA_DropdownArea(QtWidgets.QWidget):
	def __init__(self):
		super().__init__()
		self.initUI()

	def initUI(self):
		self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

		# Button to toggle the dropdown
		self.toggle_button = QtWidgets.QPushButton("Use Alternating Parameters", self)
		self.toggle_button.clicked.connect(self.toggleDropdown)

		# Dropdown area
		self.dropdown_frame = QtWidgets.QFrame(self)
		self.dropdown_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
		self.dropdown_frame.setHidden(True)  # Initially hidden

		# Set dynamic resizing policies
		self.dropdown_frame.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

		# Layout for dropdown contents
		ca_params_dropdown_layout = QtWidgets.QVBoxLayout(self.dropdown_frame)

		# Add custom entries to the dropdown area
		ca_params_dropdown_label = QtWidgets.QLabel("<b>Potential will alternate between A (above) and B (below)</b>")
		ca_params_dropdown_label.setToolTip(
			"Experiments will follow the sequence A-B-A-B-A-...\n\n"
			"A-sequence parameters must be the same length as\n"
			"or 1 input longer than the B-sequence parameters."
		)
		ca_params_dropdown_layout.addWidget(ca_params_dropdown_label)

		# Potential B sequence
		ca_params_B_potential_hlayout = QtWidgets.QHBoxLayout()
		self.ca_params_B_potential_label = QtWidgets.QLabel("Potential B sequence (csv, V)")
		self.ca_params_B_potential_label.setToolTip(
			"B-sequence potentials to cycle through.\n\n"
			"Accepts numeric values or 'OCP'."
		)
		self.ca_params_B_potential_entry = QtWidgets.QLineEdit()
		ca_params_B_potential_hlayout.addWidget(self.ca_params_B_potential_label)
		ca_params_B_potential_hlayout.addWidget(self.ca_params_B_potential_entry)
		ca_params_dropdown_layout.addLayout(ca_params_B_potential_hlayout)

		# Hold time B sequence
		ca_params_B_hold_time_hlayout = QtWidgets.QHBoxLayout()
		self.ca_params_B_hold_time_label = QtWidgets.QLabel("Potential B hold times (csv, s)")
		self.ca_params_B_hold_time_label.setToolTip(
			"Hold time for each B-sequence potential.\n\n"
			"Must be non-negative numeric values."
		)
		self.ca_params_B_hold_time_entry = QtWidgets.QLineEdit()
		ca_params_B_hold_time_hlayout.addWidget(self.ca_params_B_hold_time_label)
		ca_params_B_hold_time_hlayout.addWidget(self.ca_params_B_hold_time_entry)
		ca_params_dropdown_layout.addLayout(ca_params_B_hold_time_hlayout)

		# Ramp rate B sequence
		ca_params_B_ramp_rate_hlayout = QtWidgets.QHBoxLayout()
		self.ca_params_B_ramp_rate_checkbox = QtWidgets.QCheckBox()
		self.ca_params_B_ramp_rate_checkbox.setToolTip(
			"If checked, the given ramp rates are applied to reach the B-sequence potentials.\n\n"
			"If unchecked, the potential is stepped."
		)
		self.ca_params_B_ramp_rate_label = QtWidgets.QLabel("Potential B ramp rates (csv, mV/s)")
		self.ca_params_B_ramp_rate_label.setToolTip(
			"Ramp rate to each B-sequence potential.\n\n"
			"Accepts positive numeric values or 'STEP'."
		)
		self.ca_params_B_ramp_rate_entry = QtWidgets.QLineEdit()
		ca_params_B_ramp_rate_hlayout.addWidget(self.ca_params_B_ramp_rate_checkbox)
		ca_params_B_ramp_rate_hlayout.addWidget(self.ca_params_B_ramp_rate_label)
		ca_params_B_ramp_rate_hlayout.addWidget(self.ca_params_B_ramp_rate_entry)
		ca_params_dropdown_layout.addLayout(ca_params_B_ramp_rate_hlayout)

		# Equilibration tolerance section
		ca_params_B_equilibration_tolerance_hlayout = QtWidgets.QHBoxLayout()
		self.ca_params_B_equilibration_checkbox = QtWidgets.QCheckBox()
		self.ca_params_B_equilibration_checkbox.setToolTip(
			"If checked, the given equilibration parameters are applied to the B-sequence potentials.\n\n"
			"If unchecked, the equilibration parameters are not applied to the B-sequence potentials."
		)
		self.ca_params_B_equilibration_tolerance_label = QtWidgets.QLabel("Equilibration B tolerance (µA)")
		self.ca_params_B_equilibration_tolerance_label.setToolTip(
			"The B-sequence change in current below which the system considers\n"
			"the current to be equilibrated.\n\n"
			"Must be a non-negative numeric value."
		)
		self.ca_params_B_equilibration_tolerance_entry = QtWidgets.QLineEdit()
		self.ca_params_B_equilibration_timescale_label = QtWidgets.QLabel("Timescale (s)")
		self.ca_params_B_equilibration_timescale_label.setToolTip(
			"The B-sequence timescale over which the change in current is measured\n"
			"to determine equilibration.\n\n"
			"Must be a positive numeric value."
		)
		self.ca_params_B_equilibration_timescale_entry = QtWidgets.QLineEdit()
		ca_params_B_equilibration_tolerance_hlayout.addWidget(self.ca_params_B_equilibration_checkbox)
		ca_params_B_equilibration_tolerance_hlayout.addWidget(self.ca_params_B_equilibration_tolerance_label)
		ca_params_B_equilibration_tolerance_hlayout.addWidget(self.ca_params_B_equilibration_tolerance_entry)
		ca_params_B_equilibration_tolerance_hlayout.addWidget(self.ca_params_B_equilibration_timescale_label)
		ca_params_B_equilibration_tolerance_hlayout.addWidget(self.ca_params_B_equilibration_timescale_entry)
		ca_params_dropdown_layout.addLayout(ca_params_B_equilibration_tolerance_hlayout)

		# Set spacing and margins for dropdown content
		ca_params_dropdown_layout.setSpacing(5)
		ca_params_dropdown_layout.setContentsMargins(3, 10, 3, 3)

		# Main layout
		main_layout = QtWidgets.QVBoxLayout(self)
		main_layout.addWidget(self.toggle_button)
		main_layout.addWidget(self.dropdown_frame)
		main_layout.setSpacing(5)
		main_layout.setContentsMargins(0, 0, 0, 0)

	def toggleDropdown(self):
		if self.dropdown_frame.isHidden():
			self.dropdown_frame.setVisible(True)
			self.toggle_button.setText("Remove Alternating Parameters")

			# Rename to Potential A sequence, as this is now alternating with the Potential B sequence
			ca_params_A_potential_label.setText("Potential A sequence (csv, V)")
			ca_params_A_potential_label.setToolTip(
				"A-sequence potentials to cycle through.\n\n"
				"Accepts numeric values or 'OCP'."
			)
			ca_params_A_hold_time_label.setText("Potential A hold times (csv, s)")
			ca_params_A_hold_time_label.setToolTip(
				"Hold time for each A-sequence potential.\n\n"
				"Must be non-negative numeric values."
			)
			ca_params_A_ramp_rate_checkbox.setToolTip(
				"If checked, the given ramp rates are applied to reach the A-sequence potentials.\n\n"
				"If unchecked, the potential is stepped."
			)
			ca_params_A_ramp_rate_label.setText("Potential A ramp rates (csv, mV/s)")
			ca_params_A_ramp_rate_label.setToolTip(
				"Ramp rate to each A-sequence potential.\n\n"
				"Accepts positive numeric values or 'STEP'."
			)
			ca_params_A_equilibration_checkbox.setToolTip(
				"If checked, the given equilibration parameters are applied to the A-sequence potentials.\n\n"
				"If unchecked, the equilibration parameters are not applied to the A-sequence potentials."
			)
			ca_params_A_equilibration_tolerance_label.setText("Equilibration A tolerance (µA)")
			ca_params_A_equilibration_tolerance_label.setToolTip(
				"The A-sequence change in current below which the system considers\n"
				"the current to be equilibrated.\n\n"
				"Must be a non-negative numeric value."
			)
			ca_params_A_equilibration_timescale_label.setToolTip(
				"The A-sequence timescale over which the change in current is measured\n"
				"to determine equilibration.\n\n"
				"Must be a positive numeric value."
			)

		else:
			self.dropdown_frame.setVisible(False)
			self.toggle_button.setText("Use Alternating Parameters")

			# Rename Potential A sequence, since this is now the only potential sequence
			ca_params_A_potential_label.setText("Potential sequence (csv, V)")
			ca_params_A_potential_label.setToolTip(
				"Potentials to cycle through.\n\n"
				"Accepts numeric values or 'OCP'."
			)
			ca_params_A_hold_time_label.setText("Hold times per potential (csv, s)")
			ca_params_A_hold_time_label.setToolTip(
				"Hold time for each potential.\n\n"
				"Must be non-negative numeric values."
			)
			ca_params_A_ramp_rate_checkbox.setToolTip(
				"If checked, the given ramp rates are applied to reach the potential.\n\n"
				"If unchecked, the potential is stepped."
			)
			ca_params_A_ramp_rate_label.setText("Potential ramp rates (csv, mV/s)")
			ca_params_A_ramp_rate_label.setToolTip(
				"Ramp rate to each potential.\n\n"
				"Accepts positive numeric values or 'STEP'."
			)
			ca_params_A_equilibration_checkbox.setToolTip(
				"If checked, the given equilibration parameters are applied.\n\n"
				"If unchecked, the equilibration parameters are not applied."
			)
			ca_params_A_equilibration_tolerance_label.setText("Equilibration tolerance (µA)")
			ca_params_A_equilibration_tolerance_label.setToolTip(
				"The change in current below which the system considers\n"
				"the current to be equilibrated.\n\n"
				"Must be a non-negative numeric value."
			)
			ca_params_A_equilibration_timescale_label.setToolTip(
				"The timescale over which the change in current is measured\n"
				"to determine equilibration.\n\n"
				"Must be a positive numeric value."
			)

			# Remove inputs and uncheck boxes from the dropdown menu
			self.ca_params_B_potential_entry.setText("")
			self.ca_params_B_hold_time_entry.setText("")
			self.ca_params_B_ramp_rate_checkbox.setChecked(False)
			self.ca_params_B_ramp_rate_entry.setText("")
			self.ca_params_B_equilibration_checkbox.setChecked(False)
			self.ca_params_B_equilibration_tolerance_entry.setText("")
			self.ca_params_B_equilibration_timescale_entry.setText("")

	def freezeInputs(self, freeze):
		if freeze:
			self.toggle_button.setEnabled(False)
			self.ca_params_B_potential_entry.setEnabled(False)
			self.ca_params_B_hold_time_entry.setEnabled(False)
			self.ca_params_B_ramp_rate_checkbox.setEnabled(False)
			self.ca_params_B_ramp_rate_entry.setEnabled(False)
			self.ca_params_B_equilibration_checkbox.setEnabled(False)
			self.ca_params_B_equilibration_tolerance_entry.setEnabled(False)
			self.ca_params_B_equilibration_timescale_entry.setEnabled(False)

		elif not freeze:
			self.toggle_button.setEnabled(True)
			self.ca_params_B_potential_entry.setEnabled(True)
			self.ca_params_B_hold_time_entry.setEnabled(True)
			self.ca_params_B_ramp_rate_checkbox.setEnabled(True)
			self.ca_params_B_ramp_rate_entry.setEnabled(True)
			self.ca_params_B_equilibration_checkbox.setEnabled(True)
			self.ca_params_B_equilibration_tolerance_entry.setEnabled(True)
			self.ca_params_B_equilibration_timescale_entry.setEnabled(True)


ca_alternating_parameters_dropdown = CA_DropdownArea()
ca_params_box_layout.addWidget(ca_alternating_parameters_dropdown)

# Current limits
ca_params_curr_limits_hlayout = QtWidgets.QHBoxLayout()
ca_params_curr_limits_checkbox = QtWidgets.QCheckBox()
ca_params_curr_limits_checkbox.setToolTip(
	"If checked, the given current limits are applied.\n\n"
	"If unchecked, the given current limits are not applied."
)
ca_params_curr_limits_label = QtWidgets.QLabel("Current limits (mA):")
ca_params_curr_limits_label.setToolTip(
	"The current limits which, if surpassed, interrupt the experiment\n"
	"and switch off the cell."
)
ca_params_curr_limits_lower_label = QtWidgets.QLabel("Lower")
ca_params_curr_limits_lower_label.setToolTip(
	"If the measured cell current drops below this value, the experiment\n"
	"is interrupted and the cell is switched off.\n\n"
	"Accepts a numeric value or 'None'."
)
ca_params_curr_limits_lower_entry = QtWidgets.QLineEdit()
ca_params_curr_limits_upper_label = QtWidgets.QLabel("Upper")
ca_params_curr_limits_upper_label.setToolTip(
	"If the measured cell current increases above this value, the experiment\n"
	"is interrupted and the cell is switched off.\n\n"
	"Accepts a numeric value or 'None'."
)
ca_params_curr_limits_upper_entry = QtWidgets.QLineEdit()

ca_params_curr_limits_hlayout.addWidget(ca_params_curr_limits_checkbox)
ca_params_curr_limits_hlayout.addWidget(ca_params_curr_limits_label)
ca_params_curr_limits_hlayout.addWidget(ca_params_curr_limits_lower_label)
ca_params_curr_limits_hlayout.addWidget(ca_params_curr_limits_lower_entry)
ca_params_curr_limits_hlayout.addWidget(ca_params_curr_limits_upper_label)
ca_params_curr_limits_hlayout.addWidget(ca_params_curr_limits_upper_entry)
ca_params_box_layout.addLayout(ca_params_curr_limits_hlayout)

# Number of samples to average
ca_params_num_samples_hlayout = QtWidgets.QHBoxLayout()
ca_params_num_samples_label = QtWidgets.QLabel("Number of samples to average")
ca_params_num_samples_label.setToolTip(
	"Number of data points to average.\n\n"
	"Must be a positive integer value."
)
ca_params_num_samples_entry = QtWidgets.QLineEdit()

ca_params_num_samples_hlayout.addWidget(ca_params_num_samples_label)
ca_params_num_samples_hlayout.addWidget(ca_params_num_samples_entry)
ca_params_box_layout.addLayout(ca_params_num_samples_hlayout)

# Pre-experiment delay
ca_params_delay_hlayout = QtWidgets.QHBoxLayout()
ca_params_delay_label = QtWidgets.QLabel(text="Pre-experiment delay (s)")
ca_params_delay_label.setToolTip(
	"Time delay before the experiment begins to allow the system to settle.\n\n"
	"Required if not waiting for OCP equilibration.\n\n"
	"Must be a non-negative numeric value."
)
ca_params_delay_entry = QtWidgets.QLineEdit()
ca_params_delay_OCP_checkbox = QtWidgets.QCheckBox("Wait for OCP equilibration")
ca_params_delay_OCP_checkbox.setToolTip(
	"Wait for OCP equilibration before the experiment begins instead of using a fixed delay.\n\n"
	"If checked, OCP equilibration is performed.\n\n"
	"Automatically checked if any potential in the sequence set as 'OCP'."
)

ca_params_delay_hlayout.addWidget(ca_params_delay_label)
ca_params_delay_hlayout.addWidget(ca_params_delay_entry)
ca_params_delay_hlayout.addWidget(ca_params_delay_OCP_checkbox)
ca_params_box_layout.addLayout(ca_params_delay_hlayout)

ca_params_box_layout.setSpacing(5)
ca_params_box_layout.setContentsMargins(3, 10, 3, 3)
ca_vbox.addWidget(ca_params_box)

# Checkbutton box
ca_checking_box = QtWidgets.QGroupBox(title="THIS BUTTON MUST BE PRESSED", flat=False)
format_box_for_parameter_centered_title(ca_checking_box)
ca_checking_layout = QtWidgets.QHBoxLayout()
ca_checking_box.setLayout(ca_checking_layout)

ca_variables_checkbutton = QtWidgets.QPushButton("CHECK")
ca_variables_checkbutton.clicked.connect(ca_checkbutton_callback)
ca_checking_layout.addWidget(ca_variables_checkbutton)

ca_checking_layout.setSpacing(5)
ca_checking_layout.setContentsMargins(3, 10, 3, 3)
ca_vbox.addWidget(ca_checking_box)

# Autoranging box
ca_range_box = QtWidgets.QGroupBox(title="Autoranging", flat=False)
ca_range_box.setToolTip(
	"<html><body>"
	"Enable autoranging to automatically select the optimal current range<br>"
	"based on the measured current during the experiment.<br><br>"
	"This feature allows the device to accurately measure currents across<br>"
	"a wide dynamic range, from nanoamps up to 25 mA, by adapting to the<br>"
	"current level in real time.<br><br>"
	"If specific current ranges are undesirable for these experiments,<br>"
	"untick their checkboxes to prevent them from being selected by autoranging.<br><br>"
	"<b>NOTE:</b> Chronoamperometry experiments may experience difficulty<br>"
	"stepping and holding at a fixed potential for measurement if multiple<br>"
	"current ranges are selected. <b>This can impact the stability and<br>"
	"quality of the recorded data.</b>"
	"</body></html>"
)
format_box_for_parameter(ca_range_box)
ca_range_layout = QtWidgets.QHBoxLayout()
ca_range_box.setLayout(ca_range_layout)
ca_range_layout.setAlignment(QtCore.Qt.AlignCenter)
ca_range_checkboxes = []
for curr in current_range_list:
	checkbox = QtWidgets.QCheckBox(curr)
	ca_range_checkboxes.append(checkbox)
	ca_range_layout.addWidget(checkbox)
	checkbox.setChecked(False)  # Set checked to False as multiple can cause issues
ca_range_checkboxes[0].setChecked(True)  # Check 20 mA as this gives best results for CA

ca_range_layout.setSpacing(50)
ca_range_layout.setContentsMargins(3, 10, 3, 3)
ca_vbox.addWidget(ca_range_box)

# File box
ca_file_box = QtWidgets.QGroupBox(title="Output filepath (experiment info will be appended)", flat=False)
format_box_for_parameter(ca_file_box)
ca_file_layout = QtWidgets.QVBoxLayout()
ca_file_box.setLayout(ca_file_layout)

# Filename
ca_file_choose_hlayout = QtWidgets.QHBoxLayout()
ca_file_entry = QtWidgets.QLineEdit()
ca_file_entry.setToolTip(
	"Base output filename for this experiment.\n\n"
	"The experiment will generate a raw data file, appending '_CA' with the\n"
	"potential sequence to the filename and saved as a .dat file.\n\n"
	"A summary file will also be created as a .txt file using the given base output filename."
)
ca_file_choose_button = QtWidgets.QPushButton("...")
ca_file_choose_button.setFixedWidth(32)
ca_file_choose_button.clicked.connect(lambda: choose_file(ca_file_entry,"Choose where to save the chronoamperometry measurement data"))
ca_file_choose_hlayout.addWidget(ca_file_entry)
ca_file_choose_hlayout.addWidget(ca_file_choose_button)
ca_file_layout.addLayout(ca_file_choose_hlayout)

# Notes box
ca_file_notes_entry = QtWidgets.QTextEdit()
ca_file_notes_entry.setPlaceholderText("*** Optional experiment notes to write in summary file ***")
ca_file_notes_entry.setStyleSheet("""
	QTextEdit {
		color: black;
	}
	QTextEdit: empty {
		color:grey;
	}
""")
line_height = ca_file_notes_entry.fontMetrics().lineSpacing()
num_lines = 5
ca_file_notes_entry.setMaximumHeight(line_height * num_lines)
ca_file_notes_entry.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
ca_file_layout.addWidget(ca_file_notes_entry)

ca_file_layout.setSpacing(5)
ca_file_layout.setContentsMargins(3,10,3,3)
ca_vbox.addWidget(ca_file_box)

# Check button reset behaviour if parameter or file inputs change
ca_params_A_potential_entry.textChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_params_A_hold_time_entry.textChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_params_A_ramp_rate_checkbox.stateChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_params_A_ramp_rate_entry.textChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_params_A_equilibration_checkbox.stateChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_params_A_equilibration_tolerance_entry.textChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_params_A_equilibration_timescale_entry.textChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_params_curr_limits_checkbox.stateChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_params_curr_limits_lower_entry.textChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_params_curr_limits_upper_entry.textChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_params_num_samples_entry.textChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_params_delay_entry.textChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_params_delay_OCP_checkbox.stateChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_file_entry.textChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))

ca_alternating_parameters_dropdown.toggle_button.clicked.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_alternating_parameters_dropdown.ca_params_B_potential_entry.textChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_alternating_parameters_dropdown.ca_params_B_hold_time_entry.textChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_alternating_parameters_dropdown.ca_params_B_ramp_rate_checkbox.stateChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_alternating_parameters_dropdown.ca_params_B_ramp_rate_entry.textChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_alternating_parameters_dropdown.ca_params_B_equilibration_checkbox.stateChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_alternating_parameters_dropdown.ca_params_B_equilibration_tolerance_entry.textChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))
ca_alternating_parameters_dropdown.ca_params_B_equilibration_timescale_entry.textChanged.connect(lambda: ca_reset_experiment_controller(mode="input_changed"))

# Preview, start, and stop buttons
ca_preview_button = QtWidgets.QPushButton("Preview chronoamperometry experiment")
ca_preview_button.clicked.connect(ca_preview)
ca_start_button = QtWidgets.QPushButton("Start chronoamperometry experiment")
ca_start_button.clicked.connect(ca_initialise)
ca_stop_button = QtWidgets.QPushButton("Stop chronoamperometry experiment")
ca_stop_button.clicked.connect(lambda: (ca_delay_timer.stop(), ca_stop(ca_current_segment_index, interrupted=True)))

ca_vbox.addWidget(ca_preview_button)
ca_vbox.addWidget(ca_start_button)
ca_vbox.addWidget(ca_stop_button)

ca_vbox.addWidget(create_spacer(5))

# Experiment info box
ca_info_box = QtWidgets.QGroupBox(title="Experiment info", flat=False)
format_box_for_parameter_centered_title(ca_info_box)
ca_info_layout = QtWidgets.QVBoxLayout()
ca_info_box.setLayout(ca_info_layout)

# Current program state
ca_info_program_state_entry = make_label_entry(ca_info_layout, "Current program state:")
ca_info_program_state_entry.setText("No experiments running")
ca_info_program_state_entry.setReadOnly(True)

# Segment number
ca_info_segmentnum_entry = make_label_entry(ca_info_layout, "Current segment number:")
ca_info_segmentnum_entry.setText("-/-")
ca_info_segmentnum_entry.setReadOnly(True)
ca_info_segmentnum_entry.setAlignment(QtCore.Qt.AlignCenter)

ca_info_layout.setSpacing(5)
ca_info_layout.setContentsMargins(3, 10, 3, 3)
ca_vbox.addWidget(ca_info_box)

# Progress bar box
ca_progress_box = QtWidgets.QGroupBox(title="Experiment progress", flat=False)
format_box_for_parameter_centered_title(ca_progress_box)
ca_progress_layout = QtWidgets.QHBoxLayout()
ca_progress_box.setLayout(ca_progress_layout)

ca_progress_bar = LabeledProgressBar(label="Segment", offset=1)
ca_progress_layout.addWidget(ca_progress_bar)

ca_progress_layout.setSpacing(5)
ca_progress_layout.setContentsMargins(3, 10, 3, 3)
ca_vbox.addWidget(ca_progress_box)

# Plot options box
ca_plot_options_box = QtWidgets.QGroupBox(title="Plot options", flat=False)
format_box_for_parameter_centered_title(ca_plot_options_box)
ca_plot_options_layout = QtWidgets.QVBoxLayout()
ca_plot_options_box.setLayout(ca_plot_options_layout)

# Show all segments and current segment only
ca_plot_options_all_segments_radiobutton = QtWidgets.QRadioButton("Show all segments")
ca_plot_options_all_segments_radiobutton.setChecked(True)
ca_plot_options_current_segment_only_radiobutton = QtWidgets.QRadioButton("Show current segment only")

# Show selected segments
def ca_plot_options_update_segment_selector():

	if ca_current_segment_index is None:
		return

	# Get the current selected index in segment2 before clearing
	current_segment2_value = ca_plot_options_segment2_dropdown.currentText()

	# Get the selected index of segment1
	selected_segment1_index = ca_plot_options_segment1_dropdown.currentIndex()

	# Clear the segment2 dropdown
	ca_plot_options_segment2_dropdown.clear()

	# Populate segment2 with values from selected_segment1_index to current_segment_index
	segment_2_options = [str(i + 1) for i in range(selected_segment1_index, ca_current_segment_index + 1)]
	ca_plot_options_segment2_dropdown.addItems(segment_2_options)

	# Check if the previous selected value is still in the new list of segment2 options
	if current_segment2_value in segment_2_options:
		# If it's still valid, re-select that value
		index_to_select = segment_2_options.index(current_segment2_value)
		ca_plot_options_segment2_dropdown.setCurrentIndex(index_to_select)
	else:
		# If it's no longer valid, select the first item (default behavior)
		ca_plot_options_segment2_dropdown.setCurrentIndex(0)

	if not ca_plot_options_segment_select_radiobutton.isChecked():
		ca_plot_options_segment2_dropdown.setCurrentIndex(len(segment_2_options) - 1)


ca_plot_options_segment_select_radiobutton = QtWidgets.QRadioButton("Show segment")
ca_plot_options_segment1_dropdown = QtWidgets.QComboBox()
ca_plot_options_segment1_dropdown.addItems([])
ca_plot_options_segment1_dropdown.currentIndexChanged.connect(ca_plot_options_update_segment_selector)
ca_plot_options_segment2_dropdown = QtWidgets.QComboBox()
ca_plot_options_segment2_dropdown.addItems([])
ca_plot_options_segment_select_hlayout = QtWidgets.QHBoxLayout()
ca_plot_options_segment_select_hlayout.addWidget(ca_plot_options_segment_select_radiobutton)
ca_plot_options_segment_select_hlayout.addWidget(ca_plot_options_segment1_dropdown)
ca_plot_options_segment_select_hlayout.addWidget(QtWidgets.QLabel("to"))
ca_plot_options_segment_select_hlayout.addWidget(ca_plot_options_segment2_dropdown)

ca_plot_options_layout.addWidget(ca_plot_options_all_segments_radiobutton)
ca_plot_options_layout.addWidget(ca_plot_options_current_segment_only_radiobutton)
ca_plot_options_layout.addLayout(ca_plot_options_segment_select_hlayout)

# Make all segments/current and previous segment/current segment only button group
ca_plot_options_segment_group = QtWidgets.QButtonGroup()
ca_plot_options_segment_group.addButton(ca_plot_options_all_segments_radiobutton)
ca_plot_options_segment_group.addButton(ca_plot_options_current_segment_only_radiobutton)
ca_plot_options_segment_group.addButton(ca_plot_options_segment_select_radiobutton)

ca_plot_options_layout.setAlignment(QtCore.Qt.AlignCenter)
ca_plot_options_layout.setSpacing(5)
ca_plot_options_layout.setContentsMargins(3, 10, 3, 3)
ca_vbox.addWidget(ca_plot_options_box)

ca_vbox.setSpacing(5)
ca_vbox.setContentsMargins(3, 3, 3, 3)

# Make scrollable area
ca_widget = QtWidgets.QWidget()
ca_widget.setLayout(ca_vbox)
ca_widget.setContentsMargins(0, 0, 0, 0)

ca_scroll_area = QtWidgets.QScrollArea()
ca_scroll_area.setWidgetResizable(True)
ca_scroll_area.setWidget(ca_widget)
ca_scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
ca_scroll_area.setContentsMargins(0, 0, 0, 0)

ca_layout = QtWidgets.QVBoxLayout()
ca_layout.setContentsMargins(0, 0, 0, 0)
ca_layout.addWidget(ca_scroll_area)
tabs[4].setLayout(ca_layout)


"""CHRONOPOTENTIOMETRY TAB"""

cp_vbox = QtWidgets.QVBoxLayout()
cp_vbox.setAlignment(QtCore.Qt.AlignTop)

# Parameters box
cp_params_box = QtWidgets.QGroupBox(title="Chronopotentiometry parameters", flat=False)
format_box_for_parameter(cp_params_box)
cp_params_box_layout = QtWidgets.QVBoxLayout()
cp_params_box.setLayout(cp_params_box_layout)

# Current A sequence
cp_params_A_current_hlayout = QtWidgets.QHBoxLayout()
cp_params_A_current_label = QtWidgets.QLabel("Current sequence (csv, µA)")
cp_params_A_current_label.setToolTip(
	"Currents to cycle through.\n\n"
	"Must be numeric values."
)
cp_params_A_current_entry = QtWidgets.QLineEdit()
cp_params_A_current_hlayout.addWidget(cp_params_A_current_label)
cp_params_A_current_hlayout.addWidget(cp_params_A_current_entry)
cp_params_box_layout.addLayout(cp_params_A_current_hlayout)

# Hold time A sequence
cp_params_A_hold_time_hlayout = QtWidgets.QHBoxLayout()
cp_params_A_hold_time_label = QtWidgets.QLabel("Hold time per current (csv, s)")
cp_params_A_hold_time_label.setToolTip(
	"Hold time for each current.\n\n"
	"Must be non-negative numeric values."
)
cp_params_A_hold_time_entry = QtWidgets.QLineEdit()
cp_params_A_hold_time_hlayout.addWidget(cp_params_A_hold_time_label)
cp_params_A_hold_time_hlayout.addWidget(cp_params_A_hold_time_entry)
cp_params_box_layout.addLayout(cp_params_A_hold_time_hlayout)

# Ramp rate A sequence
cp_params_A_ramp_rate_hlayout = QtWidgets.QHBoxLayout()
cp_params_A_ramp_rate_checkbox = QtWidgets.QCheckBox()
cp_params_A_ramp_rate_checkbox.setToolTip(
	"If checked, the given ramp rates are applied to reach the current.\n\n"
	"If unchecked, the current is stepped."
)
cp_params_A_ramp_rate_label = QtWidgets.QLabel("Current ramp rates (csv, µA/s)")
cp_params_A_ramp_rate_label.setToolTip(
	"Ramp rate to each current.\n\n"
	"Accepts positive numeric values or 'STEP'."
)
cp_params_A_ramp_rate_entry = QtWidgets.QLineEdit()
cp_params_A_ramp_rate_hlayout.addWidget(cp_params_A_ramp_rate_checkbox)
cp_params_A_ramp_rate_hlayout.addWidget(cp_params_A_ramp_rate_label)
cp_params_A_ramp_rate_hlayout.addWidget(cp_params_A_ramp_rate_entry)
cp_params_box_layout.addLayout(cp_params_A_ramp_rate_hlayout)

# Potential equilibration A
cp_params_A_equilibration_tolerance_hlayout = QtWidgets.QHBoxLayout()
cp_params_A_equilibration_checkbox = QtWidgets.QCheckBox()
cp_params_A_equilibration_checkbox.setToolTip(
	"If checked, the given equilibration parameters are applied.\n\n"
	"If unchecked, the equilibration parameters are not applied."
)
cp_params_A_equilibration_tolerance_label = QtWidgets.QLabel("Equilibration tolerance (mV)")
cp_params_A_equilibration_tolerance_label.setToolTip(
	"The change in potential below which the system considers\n"
	"the potential to be equilibrated.\n\n"
	"Must be a non-negative numeric value."
)
cp_params_A_equilibration_tolerance_entry = QtWidgets.QLineEdit()
cp_params_A_equilibration_timescale_label = QtWidgets.QLabel("Timescale (s)")
cp_params_A_equilibration_timescale_label.setToolTip(
	"The timescale over which the change in potential is measured\n"
	"to determine equilibration.\n\n"
	"Must be a positive numeric value."
)
cp_params_A_equilibration_timescale_entry = QtWidgets.QLineEdit()
cp_params_A_equilibration_tolerance_hlayout.addWidget(cp_params_A_equilibration_checkbox)
cp_params_A_equilibration_tolerance_hlayout.addWidget(cp_params_A_equilibration_tolerance_label)
cp_params_A_equilibration_tolerance_hlayout.addWidget(cp_params_A_equilibration_tolerance_entry)
cp_params_A_equilibration_tolerance_hlayout.addWidget(cp_params_A_equilibration_timescale_label)
cp_params_A_equilibration_tolerance_hlayout.addWidget(cp_params_A_equilibration_timescale_entry)
cp_params_box_layout.addLayout(cp_params_A_equilibration_tolerance_hlayout)

class CP_DropdownArea(QtWidgets.QWidget):
	def __init__(self):
		super().__init__()
		self.initUI()

	def initUI(self):
		self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

		# Button to toggle the dropdown
		self.toggle_button = QtWidgets.QPushButton("Use Alternating Parameters", self)
		self.toggle_button.clicked.connect(self.toggleDropdown)

		# Dropdown area
		self.dropdown_frame = QtWidgets.QFrame(self)
		self.dropdown_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
		self.dropdown_frame.setHidden(True)  # Initially hidden

		# Set dynamic resizing policies
		self.dropdown_frame.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

		# Layout for dropdown contents
		cp_params_dropdown_layout = QtWidgets.QVBoxLayout(self.dropdown_frame)

		# Add custom entries to the dropdown area
		cp_params_dropdown_label = QtWidgets.QLabel("<b>Current will alternate between A (above) and B (below)</b>")
		cp_params_dropdown_label.setToolTip(
			"Experiments will follow the sequence A-B-A-B-A-...\n\n"
			"A-sequence parameters must be the same length as\n"
			"or 1 input longer than the B-sequence parameters."
		)
		cp_params_dropdown_layout.addWidget(cp_params_dropdown_label)

		# Current B sequence
		cp_params_B_current_hlayout = QtWidgets.QHBoxLayout()
		self.cp_params_B_current_label = QtWidgets.QLabel("Current B sequence (csv, µA)")
		self.cp_params_B_current_label.setToolTip(
			"B-sequence currents to cycle through.\n\n"
			"Must be numeric values."
		)
		self.cp_params_B_current_entry = QtWidgets.QLineEdit()
		cp_params_B_current_hlayout.addWidget(self.cp_params_B_current_label)
		cp_params_B_current_hlayout.addWidget(self.cp_params_B_current_entry)
		cp_params_dropdown_layout.addLayout(cp_params_B_current_hlayout)

		# Hold time B sequence
		cp_params_B_hold_time_hlayout = QtWidgets.QHBoxLayout()
		self.cp_params_B_hold_time_label = QtWidgets.QLabel("Current B hold times (csv, s)")
		self.cp_params_B_hold_time_label.setToolTip(
			"Hold time for each B-sequence current.\n\n"
			"Must be non-negative numeric values."
		)
		self.cp_params_B_hold_time_entry = QtWidgets.QLineEdit()
		cp_params_B_hold_time_hlayout.addWidget(self.cp_params_B_hold_time_label)
		cp_params_B_hold_time_hlayout.addWidget(self.cp_params_B_hold_time_entry)
		cp_params_dropdown_layout.addLayout(cp_params_B_hold_time_hlayout)

		# Ramp rate B sequence
		cp_params_B_ramp_rate_hlayout = QtWidgets.QHBoxLayout()
		self.cp_params_B_ramp_rate_checkbox = QtWidgets.QCheckBox()
		self.cp_params_B_ramp_rate_checkbox.setToolTip(
			"If checked, the given ramp rates are applied to reach the B-sequence currents.\n\n"
			"If unchecked, the current is stepped."
		)
		self.cp_params_B_ramp_rate_label = QtWidgets.QLabel("Current B ramp rates (csv, µA/s)")
		self.cp_params_B_ramp_rate_label.setToolTip(
			"Ramp rate to each B-sequence current.\n\n"
			"Accepts positive numeric values or 'STEP'."
		)
		self.cp_params_B_ramp_rate_entry = QtWidgets.QLineEdit()
		cp_params_B_ramp_rate_hlayout.addWidget(self.cp_params_B_ramp_rate_checkbox)
		cp_params_B_ramp_rate_hlayout.addWidget(self.cp_params_B_ramp_rate_label)
		cp_params_B_ramp_rate_hlayout.addWidget(self.cp_params_B_ramp_rate_entry)
		cp_params_dropdown_layout.addLayout(cp_params_B_ramp_rate_hlayout)

		# Equilibration tolerance section
		cp_params_B_equilibration_tolerance_hlayout = QtWidgets.QHBoxLayout()
		self.cp_params_B_equilibration_checkbox = QtWidgets.QCheckBox()
		self.cp_params_B_equilibration_checkbox.setToolTip(
			"If checked, the given equilibration parameters are applied to the B-sequence currents.\n\n"
			"If unchecked, the equilibration parameters are not applied to the B-sequence currents."
		)
		self.cp_params_B_equilibration_tolerance_label = QtWidgets.QLabel("Equilibration B tolerance (mV)")
		self.cp_params_B_equilibration_tolerance_label.setToolTip(
			"The B-sequence change in potential below which the system considers\n"
			"the potential to be equilibrated.\n\n"
			"Must be a non-negative numeric value."
		)
		self.cp_params_B_equilibration_tolerance_entry = QtWidgets.QLineEdit()
		self.cp_params_B_equilibration_timescale_label = QtWidgets.QLabel("Timescale (s)")
		self.cp_params_B_equilibration_timescale_label.setToolTip(
			"The B-sequence timescale over which the change in potential is measured\n"
			"to determine equilibration.\n\n"
			"Must be a positive numeric value."
		)
		self.cp_params_B_equilibration_timescale_entry = QtWidgets.QLineEdit()
		cp_params_B_equilibration_tolerance_hlayout.addWidget(self.cp_params_B_equilibration_checkbox)
		cp_params_B_equilibration_tolerance_hlayout.addWidget(self.cp_params_B_equilibration_tolerance_label)
		cp_params_B_equilibration_tolerance_hlayout.addWidget(self.cp_params_B_equilibration_tolerance_entry)
		cp_params_B_equilibration_tolerance_hlayout.addWidget(self.cp_params_B_equilibration_timescale_label)
		cp_params_B_equilibration_tolerance_hlayout.addWidget(self.cp_params_B_equilibration_timescale_entry)
		cp_params_dropdown_layout.addLayout(cp_params_B_equilibration_tolerance_hlayout)

		# Set spacing and margins for dropdown content
		cp_params_dropdown_layout.setSpacing(5)
		cp_params_dropdown_layout.setContentsMargins(3, 10, 3, 3)

		# Main layout
		main_layout = QtWidgets.QVBoxLayout(self)
		main_layout.addWidget(self.toggle_button)
		main_layout.addWidget(self.dropdown_frame)
		main_layout.setSpacing(5)
		main_layout.setContentsMargins(0, 0, 0, 0)

	def toggleDropdown(self):
		if self.dropdown_frame.isHidden():
			self.dropdown_frame.setVisible(True)
			self.toggle_button.setText("Remove Alternating Parameters")

			# Rename to Current A sequence, as this is now alternating with the Current B sequence
			cp_params_A_current_label.setText("Current A sequence (csv, µA)")
			cp_params_A_current_label.setToolTip(
				"A-sequence currents to cycle through.\n\n"
				"Must be numeric values."
			)
			cp_params_A_hold_time_label.setText("Current A hold times (csv, s)")
			cp_params_A_hold_time_label.setToolTip(
				"Hold time for each A-sequence current.\n\n"
				"Must be non-negative numeric values."
			)
			cp_params_A_ramp_rate_checkbox.setToolTip(
				"If checked, the given ramp rates are applied to reach the A-sequence currents.\n\n"
				"If unchecked, the current is stepped."
			)
			cp_params_A_ramp_rate_label.setText("Current A ramp rates (csv, µA/s)")
			cp_params_A_ramp_rate_label.setToolTip(
				"Ramp rate to each A-sequence current.\n\n"
				"Accepts positive numeric values or 'STEP'."
			)
			cp_params_A_equilibration_checkbox.setToolTip(
				"If checked, the given equilibration parameters are applied to the A-sequence currents.\n\n"
				"If unchecked, the equilibration parameters are not applied to the A-sequence currents."
			)
			cp_params_A_equilibration_tolerance_label.setText("Equilibration A tolerance (mV)")
			cp_params_A_equilibration_tolerance_label.setToolTip(
				"The A-sequence change in potential below which the system considers\n"
				"the potential to be equilibrated.\n\n"
				"Must be a non-negative numeric value."
			)
			cp_params_A_equilibration_timescale_label.setToolTip(
				"The A-sequence timescale over which the change in potential is measured\n"
				"to determine equilibration.\n\n"
				"Must be a positive numeric value."
			)

		else:
			self.dropdown_frame.setVisible(False)
			self.toggle_button.setText("Use Alternating Parameters")

			# Rename Current A sequence, since this is now the only potential sequence
			cp_params_A_current_label.setText("Current sequence (csv, µA)")
			cp_params_A_current_label.setToolTip(
				"Currents to cycle through.\n\n"
				"Must be numeric values."
			)
			cp_params_A_hold_time_label.setText("Hold times per current (csv, s)")
			cp_params_A_hold_time_label.setToolTip(
				"Hold time for each current.\n\n"
				"Must be non-negative numeric values."
			)
			cp_params_A_ramp_rate_checkbox.setToolTip(
				"If checked, the given ramp rates are applied to reach the current.\n\n"
				"If unchecked, the current is stepped."
			)
			cp_params_A_ramp_rate_label.setText("Current ramp rates (csv, µA/s)")
			cp_params_A_ramp_rate_label.setToolTip(
				"Ramp rate to each current.\n\n"
				"Accepts positive numeric values or 'STEP'."
			)
			cp_params_A_equilibration_checkbox.setToolTip(
				"If checked, the given equilibration parameters are applied.\n\n"
				"If unchecked, the potential equilibration parameters are not applied."
			)
			cp_params_A_equilibration_tolerance_label.setText("Equilibration tolerance (mV)")
			cp_params_A_equilibration_tolerance_label.setToolTip(
				"The change in potential below which the system considers\n"
				"the potential to be equilibrated.\n\n"
				"Must be a non-negative numeric value."
			)
			cp_params_A_equilibration_timescale_label.setToolTip(
				"The timescale over which the change in potential is measured\n"
				"to determine equilibration.\n\n"
				"Must be a positive numeric value."
			)

			# Remove inputs and uncheck boxes from the dropdown menu
			self.cp_params_B_current_entry.setText("")
			self.cp_params_B_hold_time_entry.setText("")
			self.cp_params_B_ramp_rate_checkbox.setChecked(False)
			self.cp_params_B_ramp_rate_entry.setText("")
			self.cp_params_B_equilibration_checkbox.setChecked(False)
			self.cp_params_B_equilibration_tolerance_entry.setText("")
			self.cp_params_B_equilibration_timescale_entry.setText("")

	def freezeInputs(self, freeze):
		if freeze:
			self.toggle_button.setEnabled(False)
			self.cp_params_B_current_entry.setEnabled(False)
			self.cp_params_B_hold_time_entry.setEnabled(False)
			self.cp_params_B_ramp_rate_checkbox.setEnabled(False)
			self.cp_params_B_ramp_rate_entry.setEnabled(False)
			self.cp_params_B_equilibration_checkbox.setEnabled(False)
			self.cp_params_B_equilibration_tolerance_entry.setEnabled(False)
			self.cp_params_B_equilibration_timescale_entry.setEnabled(False)

		elif not freeze:
			self.toggle_button.setEnabled(True)
			self.cp_params_B_current_entry.setEnabled(True)
			self.cp_params_B_hold_time_entry.setEnabled(True)
			self.cp_params_B_ramp_rate_checkbox.setEnabled(True)
			self.cp_params_B_ramp_rate_entry.setEnabled(True)
			self.cp_params_B_equilibration_checkbox.setEnabled(True)
			self.cp_params_B_equilibration_tolerance_entry.setEnabled(True)
			self.cp_params_B_equilibration_timescale_entry.setEnabled(True)


cp_alternating_parameters_dropdown = CP_DropdownArea()
cp_params_box_layout.addWidget(cp_alternating_parameters_dropdown)

# Potential limits
cp_params_pot_limits_hlayout = QtWidgets.QHBoxLayout()
cp_params_pot_limits_checkbox = QtWidgets.QCheckBox()
cp_params_pot_limits_checkbox.setToolTip(
	"If checked, the given potential limits are applied.\n\n"
	"If unchecked, the given potential limits are not applied."
)
cp_params_pot_limits_label = QtWidgets.QLabel("Potential limits (V):")
cp_params_pot_limits_label.setToolTip(
	"The potential limits which, if surpassed, interrupt the experiment\n"
	"and switch off the cell."
)
cp_params_pot_limits_lower_label = QtWidgets.QLabel("Lower")
cp_params_pot_limits_lower_label.setToolTip(
	"If the measured cell potential drops below this value, the experiment\n"
	"is interrupted and the cell is switched off.\n\n"
	"Accepts a numeric value, 'OCP', or 'None'."
)
cp_params_pot_limits_lower_entry = QtWidgets.QLineEdit()
cp_params_pot_limits_upper_label = QtWidgets.QLabel("Upper")
cp_params_pot_limits_upper_label.setToolTip(
	"If the measured cell potential increases above this value, the experiment\n"
	"is interrupted and the cell is switched off.\n\n"
	"Accepts a numeric value, 'OCP', or 'None'."
)
cp_params_pot_limits_upper_entry = QtWidgets.QLineEdit()

cp_params_pot_limits_hlayout.addWidget(cp_params_pot_limits_checkbox)
cp_params_pot_limits_hlayout.addWidget(cp_params_pot_limits_label)
cp_params_pot_limits_hlayout.addWidget(cp_params_pot_limits_lower_label)
cp_params_pot_limits_hlayout.addWidget(cp_params_pot_limits_lower_entry)
cp_params_pot_limits_hlayout.addWidget(cp_params_pot_limits_upper_label)
cp_params_pot_limits_hlayout.addWidget(cp_params_pot_limits_upper_entry)
cp_params_box_layout.addLayout(cp_params_pot_limits_hlayout)

# Number of samples to average
cp_params_num_samples_hlayout = QtWidgets.QHBoxLayout()
cp_params_num_samples_label = QtWidgets.QLabel("Number of samples to average")
cp_params_num_samples_label.setToolTip(
	"Number of data points to average.\n\n"
	"Must be a positive integer value."
)
cp_params_num_samples_entry = QtWidgets.QLineEdit()

cp_params_num_samples_hlayout.addWidget(cp_params_num_samples_label)
cp_params_num_samples_hlayout.addWidget(cp_params_num_samples_entry)
cp_params_box_layout.addLayout(cp_params_num_samples_hlayout)

# Pre-experiment delay
cp_params_delay_hlayout = QtWidgets.QHBoxLayout()
cp_params_delay_label = QtWidgets.QLabel(text="Pre-experiment delay (s)")
cp_params_delay_label.setToolTip(
	"Time delay before the experiment begins to allow the system to settle.\n\n"
	"Required if not waiting for OCP equilibration.\n\n"
	"Must be a non-negative numeric value."
)
cp_params_delay_entry = QtWidgets.QLineEdit()
cp_params_delay_OCP_checkbox = QtWidgets.QCheckBox("Wait for OCP equilibration")
cp_params_delay_OCP_checkbox.setToolTip(
	"Wait for OCP equilibration before the experiment begins instead of using a fixed delay.\n\n"
	"If checked, OCP equilibration is performed.\n\n"
	"Automatically checked if either potential limit set as 'OCP'."
)

cp_params_delay_hlayout.addWidget(cp_params_delay_label)
cp_params_delay_hlayout.addWidget(cp_params_delay_entry)
cp_params_delay_hlayout.addWidget(cp_params_delay_OCP_checkbox)
cp_params_box_layout.addLayout(cp_params_delay_hlayout)

cp_params_box_layout.setSpacing(5)
cp_params_box_layout.setContentsMargins(3, 10, 3, 3)
cp_vbox.addWidget(cp_params_box)

# Checkbutton box
cp_checking_box = QtWidgets.QGroupBox(title="THIS BUTTON MUST BE PRESSED", flat=False)
format_box_for_parameter_centered_title(cp_checking_box)
cp_checking_layout = QtWidgets.QHBoxLayout()
cp_checking_box.setLayout(cp_checking_layout)

cp_variables_checkbutton = QtWidgets.QPushButton("CHECK")
cp_variables_checkbutton.clicked.connect(cp_checkbutton_callback)
cp_checking_layout.addWidget(cp_variables_checkbutton)

cp_checking_layout.setSpacing(5)
cp_checking_layout.setContentsMargins(3, 10, 3, 3)
cp_vbox.addWidget(cp_checking_box)

# File box
cp_file_box = QtWidgets.QGroupBox(title="Output filepath (experiment info will be appended)", flat=False)
format_box_for_parameter(cp_file_box)
cp_file_layout = QtWidgets.QVBoxLayout()
cp_file_box.setLayout(cp_file_layout)

# Filename
cp_file_choose_hlayout = QtWidgets.QHBoxLayout()
cp_file_entry = QtWidgets.QLineEdit()
cp_file_entry.setToolTip(
	"Base output filename for this experiment.\n\n"
	"The experiment will generate a raw data file, appending '_CP' and the\n"
	"current sequence to the filename and saved as a .dat file.\n\n"
	"A summary file will also be created as a .txt file using the given base output filename."
)
cp_file_choose_button = QtWidgets.QPushButton("...")
cp_file_choose_button.setFixedWidth(32)
cp_file_choose_button.clicked.connect(lambda: choose_file(cp_file_entry,"Choose where to save the chronopotentiometry measurement data"))
cp_file_choose_hlayout.addWidget(cp_file_entry)
cp_file_choose_hlayout.addWidget(cp_file_choose_button)
cp_file_layout.addLayout(cp_file_choose_hlayout)

# Notes box
cp_file_notes_entry = QtWidgets.QTextEdit()
cp_file_notes_entry.setPlaceholderText("*** Optional experiment notes to write in summary file ***")
cp_file_notes_entry.setStyleSheet("""
	QTextEdit {
		color: black;
	}
	QTextEdit: empty {
		color:grey;
	}
""")
line_height = cp_file_notes_entry.fontMetrics().lineSpacing()
num_lines = 5
cp_file_notes_entry.setMaximumHeight(line_height * num_lines)
cp_file_notes_entry.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
cp_file_layout.addWidget(cp_file_notes_entry)

cp_file_layout.setSpacing(5)
cp_file_layout.setContentsMargins(3, 10, 3, 3)
cp_vbox.addWidget(cp_file_box)

# Check button reset behaviour if parameter or file inputs change
cp_params_A_current_entry.textChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_params_A_hold_time_entry.textChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_params_A_ramp_rate_checkbox.stateChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_params_A_ramp_rate_entry.textChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_params_A_equilibration_checkbox.stateChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_params_A_equilibration_tolerance_entry.textChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_params_A_equilibration_timescale_entry.textChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_params_pot_limits_checkbox.stateChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_params_pot_limits_lower_entry.textChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_params_pot_limits_upper_entry.textChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_params_num_samples_entry.textChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_params_delay_entry.textChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_params_delay_OCP_checkbox.stateChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_file_entry.textChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))

cp_alternating_parameters_dropdown.toggle_button.clicked.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_alternating_parameters_dropdown.cp_params_B_current_entry.textChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_alternating_parameters_dropdown.cp_params_B_hold_time_entry.textChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_alternating_parameters_dropdown.cp_params_B_ramp_rate_checkbox.stateChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_alternating_parameters_dropdown.cp_params_B_ramp_rate_entry.textChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_alternating_parameters_dropdown.cp_params_B_equilibration_checkbox.stateChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_alternating_parameters_dropdown.cp_params_B_equilibration_tolerance_entry.textChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))
cp_alternating_parameters_dropdown.cp_params_B_equilibration_timescale_entry.textChanged.connect(lambda: cp_reset_experiment_controller(mode="input_changed"))

# Preview, start, and stop buttons
cp_preview_button = QtWidgets.QPushButton("Preview chronopotentiometry experiment")
cp_preview_button.clicked.connect(cp_preview)
cp_start_button = QtWidgets.QPushButton("Start chronopotentiometry experiment")
cp_start_button.clicked.connect(cp_initialise)
cp_stop_button = QtWidgets.QPushButton("Stop chronopotentiometry experiment")
cp_stop_button.clicked.connect(lambda: (cp_delay_timer.stop(), cp_stop(cp_current_segment_index, interrupted=True)))

cp_vbox.addWidget(cp_preview_button)
cp_vbox.addWidget(cp_start_button)
cp_vbox.addWidget(cp_stop_button)

cp_vbox.addWidget(create_spacer(5))

# Experiment info box
cp_info_box = QtWidgets.QGroupBox(title="Experiment info", flat=False)
format_box_for_parameter_centered_title(cp_info_box)
cp_info_layout = QtWidgets.QVBoxLayout()
cp_info_box.setLayout(cp_info_layout)

# Current program state
cp_info_program_state_entry = make_label_entry(cp_info_layout, "Current program state:")
cp_info_program_state_entry.setText("No experiments running")
cp_info_program_state_entry.setReadOnly(True)

# Segment number
cp_info_segmentnum_entry = make_label_entry(cp_info_layout, "Current segment number:")
cp_info_segmentnum_entry.setText("-/-")
cp_info_segmentnum_entry.setReadOnly(True)
cp_info_segmentnum_entry.setAlignment(QtCore.Qt.AlignCenter)

cp_info_layout.setSpacing(5)
cp_info_layout.setContentsMargins(3, 10, 3, 3)
cp_vbox.addWidget(cp_info_box)

# Progress bar box
cp_progress_box = QtWidgets.QGroupBox(title="Experiment progress", flat=False)
format_box_for_parameter_centered_title(cp_progress_box)
cp_progress_layout = QtWidgets.QHBoxLayout()
cp_progress_box.setLayout(cp_progress_layout)

cp_progress_bar = LabeledProgressBar(label="Segment", offset=1)
cp_progress_layout.addWidget(cp_progress_bar)

cp_progress_layout.setSpacing(5)
cp_progress_layout.setContentsMargins(3, 10, 3, 3)
cp_vbox.addWidget(cp_progress_box)

# Plot options box
cp_plot_options_box = QtWidgets.QGroupBox(title="Plot options", flat=False)
format_box_for_parameter_centered_title(cp_plot_options_box)
cp_plot_options_layout = QtWidgets.QVBoxLayout()
cp_plot_options_box.setLayout(cp_plot_options_layout)

# Show all segments and current segment only
cp_plot_options_all_segments_radiobutton = QtWidgets.QRadioButton("Show all segments")
cp_plot_options_all_segments_radiobutton.setChecked(True)
cp_plot_options_current_segment_only_radiobutton = QtWidgets.QRadioButton("Show current segment only")

# Show selected segments
def cp_plot_options_update_segment_selector():

	if cp_current_segment_index is None:
		return

	# Get the current selected index in segment2 before clearing
	current_segment2_value = cp_plot_options_segment2_dropdown.currentText()

	# Get the selected index of segment1
	selected_segment1_index = cp_plot_options_segment1_dropdown.currentIndex()

	# Clear the segment2 dropdown
	cp_plot_options_segment2_dropdown.clear()

	# Populate segment2 with values from selected_segment1_index to current_segment_index
	segment_2_options = [str(i + 1) for i in range(selected_segment1_index, cp_current_segment_index + 1)]
	cp_plot_options_segment2_dropdown.addItems(segment_2_options)

	# Check if the previous selected value is still in the new list of segment2 options
	if current_segment2_value in segment_2_options:
		# If it's still valid, re-select that value
		index_to_select = segment_2_options.index(current_segment2_value)
		cp_plot_options_segment2_dropdown.setCurrentIndex(index_to_select)
	else:
		# If it's no longer valid, select the first item (default behavior)
		cp_plot_options_segment2_dropdown.setCurrentIndex(0)

	if not cp_plot_options_segment_select_radiobutton.isChecked():
		cp_plot_options_segment2_dropdown.setCurrentIndex(len(segment_2_options) - 1)


cp_plot_options_segment_select_radiobutton = QtWidgets.QRadioButton("Show segment")
cp_plot_options_segment1_dropdown = QtWidgets.QComboBox()
cp_plot_options_segment1_dropdown.addItems([])
cp_plot_options_segment1_dropdown.currentIndexChanged.connect(cp_plot_options_update_segment_selector)
cp_plot_options_segment2_dropdown = QtWidgets.QComboBox()
cp_plot_options_segment2_dropdown.addItems([])
cp_plot_options_segment_select_hlayout = QtWidgets.QHBoxLayout()
cp_plot_options_segment_select_hlayout.addWidget(cp_plot_options_segment_select_radiobutton)
cp_plot_options_segment_select_hlayout.addWidget(cp_plot_options_segment1_dropdown)
cp_plot_options_segment_select_hlayout.addWidget(QtWidgets.QLabel("to"))
cp_plot_options_segment_select_hlayout.addWidget(cp_plot_options_segment2_dropdown)

cp_plot_options_layout.addWidget(cp_plot_options_all_segments_radiobutton)
cp_plot_options_layout.addWidget(cp_plot_options_current_segment_only_radiobutton)
cp_plot_options_layout.addLayout(cp_plot_options_segment_select_hlayout)

# Make all segments/current and previous segment/current segment only button group
cp_plot_options_segment_group = QtWidgets.QButtonGroup()
cp_plot_options_segment_group.addButton(cp_plot_options_all_segments_radiobutton)
cp_plot_options_segment_group.addButton(cp_plot_options_current_segment_only_radiobutton)
cp_plot_options_segment_group.addButton(cp_plot_options_segment_select_radiobutton)

cp_plot_options_layout.setAlignment(QtCore.Qt.AlignCenter)
cp_plot_options_layout.setSpacing(5)
cp_plot_options_layout.setContentsMargins(3, 10, 3, 3)
cp_vbox.addWidget(cp_plot_options_box)

cp_vbox.setSpacing(5)
cp_vbox.setContentsMargins(3, 3, 3, 3)

# Make scrollable area
cp_widget = QtWidgets.QWidget()
cp_widget.setLayout(cp_vbox)
cp_widget.setContentsMargins(0, 0, 0, 0)

cp_scroll_area = QtWidgets.QScrollArea()
cp_scroll_area.setWidgetResizable(True)
cp_scroll_area.setWidget(cp_widget)
cp_scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
cp_scroll_area.setContentsMargins(0, 0, 0, 0)

cp_layout = QtWidgets.QVBoxLayout()
cp_layout.setContentsMargins(0, 0, 0, 0)
cp_layout.addWidget(cp_scroll_area)
tabs[5].setLayout(cp_layout)


"""SELF-DISCHARGE TAB"""

sd_vbox = QtWidgets.QVBoxLayout()
sd_vbox.setAlignment(QtCore.Qt.AlignTop)

# Parameters box
sd_params_box = QtWidgets.QGroupBox(title="Self-discharge parameters", flat=False)
format_box_for_parameter(sd_params_box)
sd_params_box_layout = QtWidgets.QVBoxLayout()
sd_params_box.setLayout(sd_params_box_layout)

# Charging parameters
sd_params_charge_label = QtWidgets.QLabel("<u>Charging parameters:</u>")
sd_params_charge_label.setToolTip(
	"Parameters to control cell charging before each\n"
	"self-discharge measurement."
)
sd_params_box_layout.addWidget(sd_params_charge_label)

# Potential sequence
sd_params_charge_potential_hlayout = QtWidgets.QHBoxLayout()
sd_params_charge_potential_label = QtWidgets.QLabel("Charge potentials (csv, V)")
sd_params_charge_potential_label.setToolTip(
	"Charge potentials to cycle through.\n\n"
	"Accepts numeric values or 'OCP'."
)
sd_params_charge_potential_entry = QtWidgets.QLineEdit()
sd_params_charge_potential_hlayout.addWidget(sd_params_charge_potential_label)
sd_params_charge_potential_hlayout.addWidget(sd_params_charge_potential_entry)
sd_params_box_layout.addLayout(sd_params_charge_potential_hlayout)

# Hold time sequence
sd_params_hold_time_hlayout = QtWidgets.QHBoxLayout()
sd_params_hold_time_label = QtWidgets.QLabel("Charge hold times (csv, s)")
sd_params_hold_time_label.setToolTip(
	"Hold time for each charge potential.\n\n"
	"Must be non-negative numeric values."
)
sd_params_hold_time_entry = QtWidgets.QLineEdit()
sd_params_hold_time_hlayout.addWidget(sd_params_hold_time_label)
sd_params_hold_time_hlayout.addWidget(sd_params_hold_time_entry)
sd_params_box_layout.addLayout(sd_params_hold_time_hlayout)

# Ramp rate sequence
sd_params_ramp_rate_hlayout = QtWidgets.QHBoxLayout()
sd_params_ramp_rate_checkbox = QtWidgets.QCheckBox()
sd_params_ramp_rate_checkbox.setToolTip(
	"If checked, the given ramp rates are applied to reach the charge potential.\n\n"
	"If unchecked, the potential is stepped."
)
sd_params_ramp_rate_label = QtWidgets.QLabel("Charging ramp rates (csv, mV/s)")
sd_params_ramp_rate_label.setToolTip(
	"Ramp rate for each charge potential.\n\n"
	"Accepts positive numeric values or 'STEP'."
)
sd_params_ramp_rate_entry = QtWidgets.QLineEdit()
sd_params_ramp_rate_hlayout.addWidget(sd_params_ramp_rate_checkbox)
sd_params_ramp_rate_hlayout.addWidget(sd_params_ramp_rate_label)
sd_params_ramp_rate_hlayout.addWidget(sd_params_ramp_rate_entry)
sd_params_box_layout.addLayout(sd_params_ramp_rate_hlayout)

sd_params_box_layout.addWidget(create_line())

# Acquisition parameters
sd_params_acquisition_label = QtWidgets.QLabel("<u>Self-discharge acquisition parameters:</u>")
sd_params_acquisition_label.setToolTip(
	"Parameters to control data acquisition during the\n"
	"self-discharge measurement."
)
sd_params_box_layout.addWidget(sd_params_acquisition_label)

# Acquisition time sequence
sd_params_acquisition_time_hlayout = QtWidgets.QHBoxLayout()
sd_params_acquisition_time_label = QtWidgets.QLabel("Acquisition times (csv, s)")
sd_params_acquisition_time_label.setToolTip(
	"Acquisition time for each charge potential self-discharge.\n\n"
	"Must be positive numeric values."
)
sd_params_acquisition_time_entry = QtWidgets.QLineEdit()
sd_params_acquisition_time_hlayout.addWidget(sd_params_acquisition_time_label)
sd_params_acquisition_time_hlayout.addWidget(sd_params_acquisition_time_entry)
sd_params_box_layout.addLayout(sd_params_acquisition_time_hlayout)

# Potential cutoff threshold sequence
sd_params_acquisition_cutoff_hlayout = QtWidgets.QHBoxLayout()
sd_params_acquisition_cutoff_checkbox = QtWidgets.QCheckBox()
sd_params_acquisition_cutoff_checkbox.setToolTip(
	"If checked, the given potential cutoff thresholds are applied.\n\n"
	"If unchecked, the potential cutoff thresholds are not applied."
)
sd_params_acquisition_cutoff_label = QtWidgets.QLabel("Potential cutoff thresholds (csv, V)")
sd_params_acquisition_cutoff_label.setToolTip(
	"The potential cutoff for self-discharge at each charge potential at which\n"
	"the system progresses to the next charge potential.\n\n"
	"Accepts numeric values, 'OCP', or 'None'."
)
sd_params_acquisition_cutoff_entry = QtWidgets.QLineEdit()
sd_params_acquisition_cutoff_hlayout.addWidget(sd_params_acquisition_cutoff_checkbox)
sd_params_acquisition_cutoff_hlayout.addWidget(sd_params_acquisition_cutoff_label)
sd_params_acquisition_cutoff_hlayout.addWidget(sd_params_acquisition_cutoff_entry)
sd_params_box_layout.addLayout(sd_params_acquisition_cutoff_hlayout)

# Potential equilibration
sd_params_acquisition_equilibration_hlayout = QtWidgets.QHBoxLayout()
sd_params_acquisition_equilibration_checkbox = QtWidgets.QCheckBox()
sd_params_acquisition_equilibration_checkbox.setToolTip(
	"If checked, the given equilibration parameters are applied.\n\n"
	"If unchecked, the equilibration parameters are not applied."
)
sd_params_acquisition_equilibration_tolerance_label = QtWidgets.QLabel("Equilibration tolerance (mV)")
sd_params_acquisition_equilibration_tolerance_label.setToolTip(
	"The change in potential below which the system considers\n"
	"the potential to be equilibrated.\n\n"
	"Must be a non-negative numeric value."
)
sd_params_acquisition_equilibration_tolerance_entry = QtWidgets.QLineEdit()
sd_params_acquisition_equilibration_timescale_label = QtWidgets.QLabel("Timescale (s)")
sd_params_acquisition_equilibration_timescale_label.setToolTip(
	"The timescale over which the change in potential is measured\n"
	"to determine equilibration.\n\n"
	"Must be a positive numeric value."
)
sd_params_acquisition_equilibration_timescale_entry = QtWidgets.QLineEdit()
sd_params_acquisition_equilibration_hlayout.addWidget(sd_params_acquisition_equilibration_checkbox)
sd_params_acquisition_equilibration_hlayout.addWidget(sd_params_acquisition_equilibration_tolerance_label)
sd_params_acquisition_equilibration_hlayout.addWidget(sd_params_acquisition_equilibration_tolerance_entry)
sd_params_acquisition_equilibration_hlayout.addWidget(sd_params_acquisition_equilibration_timescale_label)
sd_params_acquisition_equilibration_hlayout.addWidget(sd_params_acquisition_equilibration_timescale_entry)
sd_params_box_layout.addLayout(sd_params_acquisition_equilibration_hlayout)

sd_params_box_layout.addWidget(create_line())

# Number of samples to average
sd_params_num_samples_hlayout = QtWidgets.QHBoxLayout()
sd_params_num_samples_label = QtWidgets.QLabel(text="Number of samples to average")
sd_params_num_samples_label.setToolTip(
	"Number of data points to average.\n\n"
	"Must be a positive integer value."
)
sd_params_num_samples_entry = QtWidgets.QLineEdit()
sd_params_num_samples_hlayout.addWidget(sd_params_num_samples_label)
sd_params_num_samples_hlayout.addWidget(sd_params_num_samples_entry)
sd_params_box_layout.addLayout(sd_params_num_samples_hlayout)

# Pre-experiment delay
sd_params_delay_hlayout = QtWidgets.QHBoxLayout()
sd_params_delay_label = QtWidgets.QLabel(text="Pre-experiment delay (s)")
sd_params_delay_label.setToolTip(
	"Time delay before the experiment begins to allow the system to settle.\n\n"
	"Required if not waiting for OCP equilibration.\n\n"
	"Must be a non-negative numeric value."
)
sd_params_delay_entry = QtWidgets.QLineEdit()
sd_params_delay_OCP_checkbox = QtWidgets.QCheckBox("Wait for OCP equilibration")
sd_params_delay_OCP_checkbox.setToolTip(
	"Wait for OCP equilibration before the experiment begins instead of using a fixed delay.\n\n"
	"If checked, OCP equilibration is performed.\n\n"
	"Automatically checked if any charge potential or potential cutoff is set as 'OCP'."
)

sd_params_delay_hlayout.addWidget(sd_params_delay_label)
sd_params_delay_hlayout.addWidget(sd_params_delay_entry)
sd_params_delay_hlayout.addWidget(sd_params_delay_OCP_checkbox)
sd_params_box_layout.addLayout(sd_params_delay_hlayout)

sd_params_box_layout.setSpacing(5)
sd_params_box_layout.setContentsMargins(3, 10, 3, 3)
sd_vbox.addWidget(sd_params_box)

# Checkbutton box
sd_checking_box = QtWidgets.QGroupBox(title="THIS BUTTON MUST BE PRESSED", flat=False)
format_box_for_parameter_centered_title(sd_checking_box)
sd_checking_layout = QtWidgets.QHBoxLayout()
sd_checking_box.setLayout(sd_checking_layout)

sd_variables_checkbutton = QtWidgets.QPushButton("CHECK")
sd_variables_checkbutton.clicked.connect(sd_checkbutton_callback)
sd_checking_layout.addWidget(sd_variables_checkbutton)

sd_checking_layout.setSpacing(5)
sd_checking_layout.setContentsMargins(3, 10, 3, 3)
sd_vbox.addWidget(sd_checking_box)

# File box
sd_file_box = QtWidgets.QGroupBox(title="Output filepath (experiment info will be appended)", flat=False)
format_box_for_parameter(sd_file_box)
sd_file_layout = QtWidgets.QVBoxLayout()
sd_file_box.setLayout(sd_file_layout)

# Filename
sd_file_choose_hlayout = QtWidgets.QHBoxLayout()
sd_file_entry = QtWidgets.QLineEdit()
sd_file_entry.setToolTip(
	"Base output filename for this experiment.\n\n"
	"The experiment will generate a raw data file, appending '_SD' and the\n"
	"charge potential sequence to the filename and saved as a .dat file.\n\n"
	"A summary file will also be created as a .txt file using the given base output filename."
)
sd_file_choose_button = QtWidgets.QPushButton("...")
sd_file_choose_button.setFixedWidth(32)
sd_file_choose_button.clicked.connect(lambda: choose_file(sd_file_entry,"Choose where to save the self-discharge measurement data"))
sd_file_choose_hlayout.addWidget(sd_file_entry)
sd_file_choose_hlayout.addWidget(sd_file_choose_button)
sd_file_layout.addLayout(sd_file_choose_hlayout)

# Notes
sd_file_notes_entry = QtWidgets.QTextEdit()
sd_file_notes_entry.setPlaceholderText("*** Optional experiment notes to write in summary file ***")
sd_file_notes_entry.setStyleSheet("""
	QTextEdit {
		color: black;
	}
	QTextEdit: empty {
		color:grey;
	}
""")
line_height = sd_file_notes_entry.fontMetrics().lineSpacing()
num_lines = 5
sd_file_notes_entry.setMaximumHeight(line_height * num_lines)
sd_file_notes_entry.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
sd_file_layout.addWidget(sd_file_notes_entry)

sd_file_layout.setSpacing(5)
sd_file_layout.setContentsMargins(3, 10, 3, 3)
sd_vbox.addWidget(sd_file_box)

# Check button reset behaviour if parameter or file inputs change
sd_params_charge_potential_entry.textChanged.connect(lambda: sd_reset_experiment_controller(mode="input_changed"))
sd_params_hold_time_entry.textChanged.connect(lambda: sd_reset_experiment_controller(mode="input_changed"))
sd_params_ramp_rate_checkbox.stateChanged.connect(lambda: sd_reset_experiment_controller(mode="input_changed"))
sd_params_ramp_rate_entry.textChanged.connect(lambda: sd_reset_experiment_controller(mode="input_changed"))
sd_params_acquisition_time_entry.textChanged.connect(lambda: sd_reset_experiment_controller(mode="input_changed"))
sd_params_acquisition_cutoff_checkbox.stateChanged.connect(lambda: sd_reset_experiment_controller(mode="input_changed"))
sd_params_acquisition_cutoff_entry.textChanged.connect(lambda: sd_reset_experiment_controller(mode="input_changed"))
sd_params_acquisition_equilibration_checkbox.stateChanged.connect(lambda: sd_reset_experiment_controller(mode="input_changed"))
sd_params_acquisition_equilibration_tolerance_entry.textChanged.connect(lambda: sd_reset_experiment_controller(mode="input_changed"))
sd_params_acquisition_equilibration_timescale_entry.textChanged.connect(lambda: sd_reset_experiment_controller(mode="input_changed"))
sd_params_num_samples_entry.textChanged.connect(lambda: sd_reset_experiment_controller(mode="input_changed"))
sd_params_delay_entry.textChanged.connect(lambda: sd_reset_experiment_controller(mode="input_changed"))
sd_params_delay_OCP_checkbox.stateChanged.connect(lambda: sd_reset_experiment_controller(mode="input_changed"))
sd_file_entry.textChanged.connect(lambda: sd_reset_experiment_controller(mode="input_changed"))

# Start and stop buttons
sd_start_button = QtWidgets.QPushButton("Start self-discharge experiment")
sd_start_button.clicked.connect(sd_initialise)
sd_stop_button = QtWidgets.QPushButton("Stop self-discharge experiment")
sd_stop_button.clicked.connect(lambda: (sd_delay_timer.stop(), sd_stop(sd_current_segment_index, interrupted=True)))

sd_vbox.addWidget(sd_start_button)
sd_vbox.addWidget(sd_stop_button)

sd_vbox.addWidget(create_spacer(5))

# Experiment info box
sd_info_box = QtWidgets.QGroupBox(title="Experiment info", flat=False)
format_box_for_parameter_centered_title(sd_info_box)
sd_info_layout = QtWidgets.QVBoxLayout()
sd_info_box.setLayout(sd_info_layout)

# Current program state
sd_info_program_state_entry = make_label_entry(sd_info_layout, "Current program state:")
sd_info_program_state_entry.setText("No experiments running")
sd_info_program_state_entry.setReadOnly(True)

# Experiment and cycle numbers
sd_info_hlayout = QtWidgets.QHBoxLayout()
sd_info_segmentnum_label = QtWidgets.QLabel(text="Segment number:")
sd_info_segmentnum_entry = QtWidgets.QLineEdit()
sd_info_segmentnum_entry.setText("-/-")
sd_info_segmentnum_entry.setReadOnly(True)
sd_info_segmentnum_entry.setAlignment(QtCore.Qt.AlignCenter)
sd_info_charge_self_discharge_label = QtWidgets.QLabel(text="Mode:")
sd_info_charge_self_discharge_entry = QtWidgets.QLineEdit()
sd_info_charge_self_discharge_entry.setText("-")
sd_info_charge_self_discharge_entry.setReadOnly(True)
sd_info_charge_self_discharge_entry.setAlignment(QtCore.Qt.AlignCenter)
sd_info_hlayout.addWidget(sd_info_segmentnum_label)
sd_info_hlayout.addWidget(sd_info_segmentnum_entry, stretch=1)
sd_info_hlayout.addWidget(sd_info_charge_self_discharge_label)
sd_info_hlayout.addWidget(sd_info_charge_self_discharge_entry, stretch=2)
sd_info_layout.addLayout(sd_info_hlayout)

sd_info_layout.setSpacing(5)
sd_info_layout.setContentsMargins(3, 10, 3, 3)
sd_vbox.addWidget(sd_info_box)

# Progress bar box
sd_progress_box = QtWidgets.QGroupBox(title="Experiment progress", flat=False)
format_box_for_parameter_centered_title(sd_progress_box)
sd_progress_layout = QtWidgets.QHBoxLayout()
sd_progress_box.setLayout(sd_progress_layout)

sd_progress_bar = LabeledProgressBar(label="Segment", offset=1)
sd_progress_layout.addWidget(sd_progress_bar)

sd_progress_layout.setSpacing(5)
sd_progress_layout.setContentsMargins(3, 10, 3, 3)
sd_vbox.addWidget(sd_progress_box)

# Plot options box
sd_plot_options_box = QtWidgets.QGroupBox(title="Plot options", flat=False)
format_box_for_parameter_centered_title(sd_plot_options_box)
sd_plot_options_layout = QtWidgets.QVBoxLayout()
sd_plot_options_box.setLayout(sd_plot_options_layout)

# Current segment and all segments radiobuttons
sd_plot_options_current_segment_only_radiobutton = QtWidgets.QRadioButton("Show current segment only")
sd_plot_options_current_segment_only_radiobutton.setChecked(True)
sd_plot_options_all_segments_radiobutton = QtWidgets.QRadioButton("Show all segments")

# Potential cutoff checkbox
sd_plot_options_pot_cutoff_checkbox = QtWidgets.QCheckBox("Show potential cutoff for this segment")
sd_plot_options_pot_cutoff_checkbox.setChecked(False)

# Show charging checkbox
sd_plot_options_show_charging_checkbox = QtWidgets.QCheckBox("Show charging stages")
sd_plot_options_show_charging_checkbox.setChecked(True)

sd_plot_options_layout.addWidget(sd_plot_options_current_segment_only_radiobutton)
sd_plot_options_layout.addWidget(sd_plot_options_all_segments_radiobutton)
sd_plot_options_layout.addWidget(create_line())
sd_plot_options_layout.addWidget(sd_plot_options_pot_cutoff_checkbox)
sd_plot_options_layout.addWidget(sd_plot_options_show_charging_checkbox)

# Make all segments / current segment only button group
sd_plot_options_segment_group = QtWidgets.QButtonGroup()
sd_plot_options_segment_group.addButton(sd_plot_options_current_segment_only_radiobutton)
sd_plot_options_segment_group.addButton(sd_plot_options_all_segments_radiobutton)

sd_plot_options_layout.setAlignment(QtCore.Qt.AlignCenter)
sd_plot_options_layout.setSpacing(5)
sd_plot_options_layout.setContentsMargins(3, 10, 3, 3)
sd_vbox.addWidget(sd_plot_options_box)

sd_vbox.setSpacing(5)
sd_vbox.setContentsMargins(3, 3, 3, 3)

# Make scrollable area
sd_widget = QtWidgets.QWidget()
sd_widget.setLayout(sd_vbox)
sd_widget.setContentsMargins(0, 0, 0, 0)

sd_scroll_area = QtWidgets.QScrollArea()
sd_scroll_area.setWidgetResizable(True)
sd_scroll_area.setWidget(sd_widget)
sd_scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
sd_scroll_area.setContentsMargins(0, 0, 0, 0)

sd_layout = QtWidgets.QVBoxLayout()
sd_layout.setContentsMargins(0, 0, 0, 0)
sd_layout.addWidget(sd_scroll_area)
tabs[6].setLayout(sd_layout)


"""RATE-TESTING TAB"""

rate_vbox = QtWidgets.QVBoxLayout()
rate_vbox.setAlignment(QtCore.Qt.AlignTop)

# Parameters box
rate_params_box = QtWidgets.QGroupBox(title="Rate-testing parameters", flat=False)
format_box_for_parameter(rate_params_box)
rate_params_box_layout = QtWidgets.QVBoxLayout()
rate_params_box.setLayout(rate_params_box_layout)

# Lower potential bounds
rate_params_lbound_hlayout = QtWidgets.QHBoxLayout()
rate_params_lbound_label = QtWidgets.QLabel(text="Lower potential limits (csv, V)")
rate_params_lbound_label.setToolTip(
	"Lower potential limits for experiments to cycle through.\n\n"
	"Accepts numeric values or 'OCP'."
)
rate_params_lbound_entry = QtWidgets.QLineEdit()

rate_params_lbound_hlayout.addWidget(rate_params_lbound_label)
rate_params_lbound_hlayout.addWidget(rate_params_lbound_entry)
rate_params_box_layout.addLayout(rate_params_lbound_hlayout)

# Upper potential bounds
rate_params_ubound_hlayout = QtWidgets.QHBoxLayout()
rate_params_ubound_label = QtWidgets.QLabel(text="Upper potential limits (csv, V)")
rate_params_ubound_label.setToolTip(
	"Upper potential limits for experiments to cycle through.\n\n"
	"Accepts numeric values or 'OCP'."
)
rate_params_ubound_entry = QtWidgets.QLineEdit()

rate_params_ubound_hlayout.addWidget(rate_params_ubound_label)
rate_params_ubound_hlayout.addWidget(rate_params_ubound_entry)
rate_params_box_layout.addLayout(rate_params_ubound_hlayout)

# 1C current
rate_params_one_c_hlayout = QtWidgets.QHBoxLayout()
rate_params_one_c_label = QtWidgets.QLabel(text="1C values (csv, µAh)")
rate_params_one_c_label.setToolTip(
	"User-defined 1C values for each potential window.\n\n"
	"Must be positive numeric values."
)
rate_params_one_c_entry = QtWidgets.QLineEdit()
rate_params_one_c_calc_checkbox = QtWidgets.QCheckBox("Auto-calculate")
rate_params_one_c_calc_checkbox.setToolTip(
	"If checked, 1C values are automatically calculated from\n"
	"galvanostatic charge/discharge cycles performed before each\n"
	"potential window using the dropdown box parameters.\n\n"
	"If unchecked, the user-defined 1C values are used."
)

rate_params_one_c_hlayout.addWidget(rate_params_one_c_label)
rate_params_one_c_hlayout.addWidget(rate_params_one_c_entry)
rate_params_one_c_hlayout.addWidget(rate_params_one_c_calc_checkbox)
rate_params_box_layout.addLayout(rate_params_one_c_hlayout)

# 1C calculation dropdown
class Rate_1C_Calc_DropdownArea(QtWidgets.QWidget):
	def __init__(self):
		super().__init__()
		self.initUI()

	def initUI(self):
		self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

		# Dropdown area
		self.dropdown_frame = QtWidgets.QFrame(self)
		self.dropdown_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
		self.dropdown_frame.setHidden(True)  # Initially hidden

		# Set dynamic resizing policies
		self.dropdown_frame.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

		# Layout for dropdown contents
		rate_params_one_c_calc_dropdown_layout = QtWidgets.QVBoxLayout(self.dropdown_frame)

		# Add entries to the dropdown area
		rate_params_one_c_calc_dropdown_label = QtWidgets.QLabel("<u>1C calculation parameters:</u>")
		rate_params_one_c_calc_dropdown_label.setToolTip(
			"Before C-rate experiments begin at each potential window,\n"
			"galvanostatic charge/discharge cycles will be performed using the\n"
			"parameters below to determine the 1C value within that window.\n\n"
			"1C is defined as the capacity of the final discharge half-cycle in µAh."
		)
		rate_params_one_c_calc_dropdown_layout.addWidget(rate_params_one_c_calc_dropdown_label)

		# Current for calculation
		rate_params_one_c_calc_current_hlayout = QtWidgets.QHBoxLayout()
		self.rate_params_one_c_calc_current_label = QtWidgets.QLabel("Current (µA)")
		self.rate_params_one_c_calc_current_label.setToolTip(
			"<html><body>"
			"Current used for galvanostatic cycling to determine the 1C rate.<br><br>"
			"<b>NOTE:</b> Choose a low current to minimise kinetic limitations and ensure an accurate capacity calculation.<br><br>"
			"Must be a positive numeric value."
			"</body></html>"
		)
		self.rate_params_one_c_calc_current_entry = QtWidgets.QLineEdit()
		rate_params_one_c_calc_current_hlayout.addWidget(self.rate_params_one_c_calc_current_label)
		rate_params_one_c_calc_current_hlayout.addWidget(self.rate_params_one_c_calc_current_entry)
		rate_params_one_c_calc_dropdown_layout.addLayout(rate_params_one_c_calc_current_hlayout)

		# Number of cycles for calculation
		rate_params_one_c_calc_num_cycles_hlayout = QtWidgets.QHBoxLayout()
		self.rate_params_one_c_calc_num_cycles_label = QtWidgets.QLabel("Number of cycles")
		self.rate_params_one_c_calc_num_cycles_label.setToolTip(
			"Number of full charge-discharge cycles to perform. The 1C value\n"
			"is determined by the capacity of the final discharge half-cycle.\n\n"
			"Must be a positive integer value."
		)
		self.rate_params_one_c_calc_num_cycles_entry = QtWidgets.QLineEdit()
		rate_params_one_c_calc_num_cycles_hlayout.addWidget(self.rate_params_one_c_calc_num_cycles_label)
		rate_params_one_c_calc_num_cycles_hlayout.addWidget(self.rate_params_one_c_calc_num_cycles_entry)
		rate_params_one_c_calc_dropdown_layout.addLayout(rate_params_one_c_calc_num_cycles_hlayout)

		# Number of samples to average
		rate_params_one_c_calc_num_samples_hlayout = QtWidgets.QHBoxLayout()
		self.rate_params_one_c_calc_num_samples_label = QtWidgets.QLabel("Number of samples to average")
		self.rate_params_one_c_calc_num_samples_label.setToolTip(
			"Number of data points to average during the 1C calculation.\n\n"
			"Must be a positive integer value."
		)
		self.rate_params_one_c_calc_num_samples_entry = QtWidgets.QLineEdit()
		rate_params_one_c_calc_num_samples_hlayout.addWidget(self.rate_params_one_c_calc_num_samples_label)
		rate_params_one_c_calc_num_samples_hlayout.addWidget(self.rate_params_one_c_calc_num_samples_entry)
		rate_params_one_c_calc_dropdown_layout.addLayout(rate_params_one_c_calc_num_samples_hlayout)

		# Post-calculation delay
		rate_params_one_c_calc_post_delay_hlayout = QtWidgets.QHBoxLayout()
		self.rate_params_one_c_calc_post_delay_label = QtWidgets.QLabel("Post-calculation delay (s)")
		self.rate_params_one_c_calc_post_delay_label.setToolTip(
			"Time delay after the 1C calculation finishes to allow the system to settle\n"
			"before proceeding with C-rate cycling.\n\n"
			"Required if not waiting for OCP equilibration.\n\n"
			"Must be a non-negative numeric value."
		)
		self.rate_params_one_c_calc_post_delay_entry = QtWidgets.QLineEdit()
		self.rate_params_one_c_calc_post_delay_OCP_checkbox = QtWidgets.QCheckBox("Wait for OCP equilibration")
		self.rate_params_one_c_calc_post_delay_OCP_checkbox.setToolTip(
			"<html><body>"
			"Wait for OCP equilibration after the 1C calculation instead of using a fixed delay.<br><br>"
			"<b>NOTE:</b> If 'OCP' is set as a potential limit, the OCP value equilibrated <b>before</b> the 1C calculation is used. This equilibration is only used to allow the system to settle."
			"</body></html>"
		)
		rate_params_one_c_calc_post_delay_hlayout.addWidget(self.rate_params_one_c_calc_post_delay_label)
		rate_params_one_c_calc_post_delay_hlayout.addWidget(self.rate_params_one_c_calc_post_delay_entry)
		rate_params_one_c_calc_post_delay_hlayout.addWidget(self.rate_params_one_c_calc_post_delay_OCP_checkbox)
		rate_params_one_c_calc_dropdown_layout.addLayout(rate_params_one_c_calc_post_delay_hlayout)

		# Set spacing and margins for dropdown
		rate_params_one_c_calc_dropdown_layout.setSpacing(5)
		rate_params_one_c_calc_dropdown_layout.setContentsMargins(3, 10, 3, 3)

		# Main layout
		main_layout = QtWidgets.QVBoxLayout(self)
		main_layout.addWidget(self.dropdown_frame)
		main_layout.setSpacing(5)
		main_layout.setContentsMargins(0, 0, 0, 0)

	def toggleDropdown(self):
		if rate_params_one_c_calc_checkbox.isChecked():
			self.dropdown_frame.setVisible(True)
			rate_params_one_c_entry.setEnabled(False)
		else:
			self.dropdown_frame.setVisible(False)
			rate_params_one_c_entry.setEnabled(True)


	def freezeInputs(self, freeze):
		if freeze:
			self.rate_params_one_c_calc_current_entry.setEnabled(False)
			self.rate_params_one_c_calc_num_cycles_entry.setEnabled(False)
			self.rate_params_one_c_calc_num_samples_entry.setEnabled(False)
			self.rate_params_one_c_calc_post_delay_entry.setEnabled(False)
			self.rate_params_one_c_calc_post_delay_OCP_checkbox.setEnabled(False)

		elif not freeze:
			self.rate_params_one_c_calc_current_entry.setEnabled(True)
			self.rate_params_one_c_calc_num_cycles_entry.setEnabled(True)
			self.rate_params_one_c_calc_num_samples_entry.setEnabled(True)
			self.rate_params_one_c_calc_post_delay_entry.setEnabled(True)
			self.rate_params_one_c_calc_post_delay_OCP_checkbox.setEnabled(True)


rate_one_c_calc_dropdown = Rate_1C_Calc_DropdownArea()
rate_params_box_layout.addWidget(rate_one_c_calc_dropdown)

# C-rates entry
rate_params_c_rate_hlayout = QtWidgets.QHBoxLayout()
rate_params_c_rate_label = QtWidgets.QLabel(text="C-rates (csv)")
rate_params_c_rate_label.setToolTip(
	"C-rates to cycle through per potential window.\n\n"
	"Must be positive numeric values."
)
rate_params_c_rate_entry = QtWidgets.QLineEdit()

rate_params_c_rate_hlayout.addWidget(rate_params_c_rate_label)
rate_params_c_rate_hlayout.addWidget(rate_params_c_rate_entry)
rate_params_box_layout.addLayout(rate_params_c_rate_hlayout)

# Number of cycles per C-rate and pre-C-rate delay
rate_params_c_rate_num_cycles_delay_hlayout = QtWidgets.QHBoxLayout()
rate_params_c_rate_num_cycles_label = QtWidgets.QLabel(text="Cycles per C-rate")
rate_params_c_rate_num_cycles_label.setToolTip(
	"Number of full charge-discharge cycles per C-rate.\n\n"
	"Must be a positive integer value."
)
rate_params_c_rate_num_cycles_entry = QtWidgets.QLineEdit()
rate_params_c_rate_delay_label = QtWidgets.QLabel(text="Pre-C-rate delay (s)")
rate_params_c_rate_delay_label.setToolTip(
	"Time delay before progressing to the next C-rate to allow the system to settle.\n\n"
	"Must be a non-negative numeric value."
)
rate_params_c_rate_delay_entry = QtWidgets.QLineEdit()

rate_params_c_rate_num_cycles_delay_hlayout.addWidget(rate_params_c_rate_num_cycles_label)
rate_params_c_rate_num_cycles_delay_hlayout.addWidget(rate_params_c_rate_num_cycles_entry)
rate_params_c_rate_num_cycles_delay_hlayout.addWidget(rate_params_c_rate_delay_label)
rate_params_c_rate_num_cycles_delay_hlayout.addWidget(rate_params_c_rate_delay_entry)
rate_params_box_layout.addLayout(rate_params_c_rate_num_cycles_delay_hlayout)

# Pre-experiment delay
rate_params_delay_hlayout = QtWidgets.QHBoxLayout()
rate_params_delay_label = QtWidgets.QLabel(text="Pre-potential window delay (s)")
rate_params_delay_label.setToolTip(
	"Time delay before experiment at each potential window begins to allow the system to settle.\n\n"
	"Required if not waiting for OCP equilibration.\n\n"
	"Must be a non-negative numeric value."
)
rate_params_delay_entry = QtWidgets.QLineEdit()
rate_params_delay_OCP_checkbox = QtWidgets.QCheckBox("Wait for OCP equilibration")
rate_params_delay_OCP_checkbox.setToolTip(
	"<html><body>"
	"Wait for OCP equilibration before each potential window instead of using a fixed delay.<br><br>"
	"If checked, OCP equilibration is performed before every potential window to ensure consistent starting conditions.<br><br>"
	"<b>NOTE:</b> Mixing equilibration and fixed delays within the same set of experiments is not supported. For users wishing to do this, please run separate sets of experiments.<br><br>"
	"Automatically checked if any potential limit set as 'OCP'."
	"</body></html>"
)

rate_params_delay_hlayout.addWidget(rate_params_delay_label)
rate_params_delay_hlayout.addWidget(rate_params_delay_entry)
rate_params_delay_hlayout.addWidget(rate_params_delay_OCP_checkbox)
rate_params_box_layout.addLayout(rate_params_delay_hlayout)

rate_params_box_layout.setSpacing(5)
rate_params_box_layout.setContentsMargins(3, 10, 3, 3)
rate_vbox.addWidget(rate_params_box)

# Checkbutton box
rate_checking_box = QtWidgets.QGroupBox(title="THIS BUTTON MUST BE PRESSED", flat=False)
format_box_for_parameter_centered_title(rate_checking_box)
rate_checking_layout = QtWidgets.QHBoxLayout()
rate_checking_box.setLayout(rate_checking_layout)

rate_variables_checkbutton = QtWidgets.QPushButton("CHECK")
rate_variables_checkbutton.clicked.connect(rate_checkbutton_callback)
rate_checking_layout.addWidget(rate_variables_checkbutton)

rate_checking_layout.setSpacing(5)
rate_checking_layout.setContentsMargins(3, 10, 3, 3)
rate_vbox.addWidget(rate_checking_box)

# File box
rate_file_box = QtWidgets.QGroupBox(title="Output filepath (experiment info will be appended)", flat=False)
format_box_for_parameter(rate_file_box)
rate_file_layout = QtWidgets.QVBoxLayout()
rate_file_box.setLayout(rate_file_layout)

# Filename
rate_file_choose_hlayout = QtWidgets.QHBoxLayout()
rate_file_entry = QtWidgets.QLineEdit()
rate_file_entry.setToolTip(
	"Base output filename for these experiments.\n\n"
	"Each experiment will generate raw data and measured capacity per half cycle files,\n"
	"appending '_RT' with the corresponding potential window and C-rate details\n"
	"and saved as .dat files.\n\n"
	"If auto-calculating 1C currents, further raw data and measured capacity per half cycle files\n"
	"will be generated for each experiment, appending '_RT_1C_calc' with the corresponding potential window\n"
	"and saved as .dat files.\n\n"
	"A summary file will also be created as a .txt file using the given base output filename."
)
rate_file_choose_button = QtWidgets.QPushButton("...")
rate_file_choose_button.setFixedWidth(32)
rate_file_choose_button.clicked.connect(lambda: choose_file(rate_file_entry,"Choose where to save the rate-testing measurement data"))

rate_file_choose_hlayout.addWidget(rate_file_entry)
rate_file_choose_hlayout.addWidget(rate_file_choose_button)
rate_file_layout.addLayout(rate_file_choose_hlayout)

# Notes
rate_file_notes_entry = QtWidgets.QTextEdit()
rate_file_notes_entry.setPlaceholderText("*** Optional experiment notes to write in summary file ***")
rate_file_notes_entry.setStyleSheet("""
	QTextEdit {
		color: black;
	}
	QTextEdit: empty {
		color:grey;
	}
""")
line_height = rate_file_notes_entry.fontMetrics().lineSpacing()
num_lines = 5
rate_file_notes_entry.setMaximumHeight(line_height * num_lines)
rate_file_notes_entry.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
rate_file_layout.addWidget(rate_file_notes_entry)

rate_file_layout.setSpacing(5)
rate_file_layout.setContentsMargins(3, 10, 3, 3)
rate_vbox.addWidget(rate_file_box)

# Check button reset behaviour if parameter or file inputs change
rate_params_lbound_entry.textChanged.connect(lambda: rate_reset_experiment_controller(mode="input_changed"))
rate_params_ubound_entry.textChanged.connect(lambda: rate_reset_experiment_controller(mode="input_changed"))
rate_params_one_c_entry.textChanged.connect(lambda: rate_reset_experiment_controller(mode="input_changed"))
rate_params_one_c_calc_checkbox.stateChanged.connect(lambda: (rate_one_c_calc_dropdown.toggleDropdown(), rate_reset_experiment_controller(mode="input_changed")))
rate_params_c_rate_entry.textChanged.connect(lambda: rate_reset_experiment_controller(mode="input_changed"))
rate_params_c_rate_num_cycles_entry.textChanged.connect(lambda: rate_reset_experiment_controller(mode="input_changed"))
rate_params_c_rate_delay_entry.textChanged.connect(lambda: rate_reset_experiment_controller(mode="input_changed"))
rate_params_delay_entry.textChanged.connect(lambda: rate_reset_experiment_controller(mode="input_changed"))
rate_params_delay_OCP_checkbox.stateChanged.connect(lambda: rate_reset_experiment_controller(mode="input_changed"))
rate_file_entry.textChanged.connect(lambda: rate_reset_experiment_controller(mode="input_changed"))

rate_one_c_calc_dropdown.rate_params_one_c_calc_current_entry.textChanged.connect(lambda: rate_reset_experiment_controller(mode="input_changed"))
rate_one_c_calc_dropdown.rate_params_one_c_calc_num_cycles_entry.textChanged.connect(lambda: rate_reset_experiment_controller(mode="input_changed"))
rate_one_c_calc_dropdown.rate_params_one_c_calc_num_samples_entry.textChanged.connect(lambda: rate_reset_experiment_controller(mode="input_changed"))
rate_one_c_calc_dropdown.rate_params_one_c_calc_post_delay_entry.textChanged.connect(lambda: rate_reset_experiment_controller(mode="input_changed"))
rate_one_c_calc_dropdown.rate_params_one_c_calc_post_delay_OCP_checkbox.stateChanged.connect(lambda: rate_reset_experiment_controller(mode="input_changed"))

# Start and stop buttons
rate_start_button = QtWidgets.QPushButton("Start rate-testing experiments")
rate_start_button.clicked.connect(rate_initialise)
rate_stop_button = QtWidgets.QPushButton("Stop rate-testing experiments")
rate_stop_button.clicked.connect(lambda: (rate_delay_timer.stop(), rate_stop(rate_current_exp_index, interrupted=True)))

rate_vbox.addWidget(rate_start_button)
rate_vbox.addWidget(rate_stop_button)
rate_vbox.addWidget(create_spacer(5))

# Experiment info box
rate_info_box = QtWidgets.QGroupBox(title="Experiment info", flat=False)
format_box_for_parameter_centered_title(rate_info_box)
rate_info_layout = QtWidgets.QVBoxLayout()
rate_info_box.setLayout(rate_info_layout)

# Current program state
rate_info_program_state_entry = make_label_entry(rate_info_layout, "Current program state:")
rate_info_program_state_entry.setText("No experiments running")
rate_info_program_state_entry.setReadOnly(True)

# Experiment and half cycle numbers
rate_info_hlayout = QtWidgets.QHBoxLayout()
rate_info_expnum_label = QtWidgets.QLabel(text="Experiment number:")
rate_info_expnum_entry = QtWidgets.QLineEdit()
rate_info_expnum_entry.setText("-/-")
rate_info_expnum_entry.setReadOnly(True)
rate_info_expnum_entry.setAlignment(QtCore.Qt.AlignCenter)
rate_info_c_rate_label = QtWidgets.QLabel(text="C-rate number:")
rate_info_c_rate_entry = QtWidgets.QLineEdit()
rate_info_c_rate_entry.setText("-/-")
rate_info_c_rate_entry.setReadOnly(True)
rate_info_c_rate_entry.setAlignment(QtCore.Qt.AlignCenter)

rate_info_hlayout.addWidget(rate_info_expnum_label)
rate_info_hlayout.addWidget(rate_info_expnum_entry, stretch=1)
rate_info_hlayout.addWidget(rate_info_c_rate_label)
rate_info_hlayout.addWidget(rate_info_c_rate_entry, stretch=1)
rate_info_layout.addLayout(rate_info_hlayout)

rate_info_layout.setSpacing(5)
rate_info_layout.setContentsMargins(3, 10, 3, 3)
rate_vbox.addWidget(rate_info_box)

# Progress bar box
rate_progress_box = QtWidgets.QGroupBox(title="Experiment progress", flat=False)
format_box_for_parameter_centered_title(rate_progress_box)
rate_progress_layout = QtWidgets.QHBoxLayout()
rate_progress_box.setLayout(rate_progress_layout)

rate_progress_bar = LabeledProgressBar(label="C-rate", offset=1)
rate_progress_layout.addWidget(rate_progress_bar)

rate_progress_layout.setSpacing(5)
rate_progress_layout.setContentsMargins(3, 10, 3, 3)
rate_vbox.addWidget(rate_progress_box)

# Plot options box
rate_plot_options_box = QtWidgets.QGroupBox(title="Plot options", flat=False)
format_box_for_parameter_centered_title(rate_plot_options_box)
rate_plot_options_layout = QtWidgets.QVBoxLayout()
rate_plot_options_box.setLayout(rate_plot_options_layout)

# Display C-rate scale radio buttons
rate_plot_options_c_rate_scale_hlayout = QtWidgets.QHBoxLayout()
rate_plot_options_c_rate_scale_hlayout.setSpacing(10)
rate_plot_options_c_rate_scale_linear_radiobutton = QtWidgets.QRadioButton("Linear")
rate_plot_options_c_rate_scale_linear_radiobutton.setChecked(True)
rate_plot_options_c_rate_scale_log_radiobutton = QtWidgets.QRadioButton("Log")

rate_plot_options_c_rate_scale_hlayout.addWidget(QtWidgets.QLabel("C-rate scale:"))
rate_plot_options_c_rate_scale_hlayout.addWidget(rate_plot_options_c_rate_scale_linear_radiobutton)
rate_plot_options_c_rate_scale_hlayout.addWidget(rate_plot_options_c_rate_scale_log_radiobutton)
rate_plot_options_c_rate_scale_hlayout.setAlignment(QtCore.Qt.AlignLeft)
rate_plot_options_layout.addLayout(rate_plot_options_c_rate_scale_hlayout)

rate_plot_options_layout.addWidget(create_line())

# Display C-rates from this experiment or all previous experiments radio buttons
rate_plot_options_this_experiment_radiobutton = QtWidgets.QRadioButton("This experiment only")
rate_plot_options_this_experiment_radiobutton.setChecked(True)
rate_plot_options_all_prev_experiments_radiobutton = QtWidgets.QRadioButton("All previous experiments")

rate_plot_options_layout.addWidget(QtWidgets.QLabel("Show inserted/extracted charge per C-rate from:"))
rate_plot_options_layout.addWidget(rate_plot_options_this_experiment_radiobutton)
rate_plot_options_layout.addWidget(rate_plot_options_all_prev_experiments_radiobutton)

# Make C-rate scale button group
rate_plot_options_c_rate_scale_group = QtWidgets.QButtonGroup()
rate_plot_options_c_rate_scale_group.addButton(rate_plot_options_c_rate_scale_linear_radiobutton)
rate_plot_options_c_rate_scale_group.addButton(rate_plot_options_c_rate_scale_log_radiobutton)

# Make this experiment only and all previous experiments button group
rate_plot_options_experiment_group = QtWidgets.QButtonGroup()
rate_plot_options_experiment_group.addButton(rate_plot_options_this_experiment_radiobutton)
rate_plot_options_experiment_group.addButton(rate_plot_options_all_prev_experiments_radiobutton)

rate_plot_options_layout.setAlignment(QtCore.Qt.AlignCenter)
rate_plot_options_layout.setSpacing(5)
rate_plot_options_layout.setContentsMargins(3, 10, 3, 3)
rate_vbox.addWidget(rate_plot_options_box)

def rate_plot_update_handler():
	if state == States.Measuring_Rate:
		rate_update_plot(rate_current_exp_index)

rate_plot_options_this_experiment_radiobutton.toggled.connect(rate_plot_update_handler)
rate_plot_options_all_prev_experiments_radiobutton.toggled.connect(rate_plot_update_handler)
rate_plot_options_c_rate_scale_linear_radiobutton.toggled.connect(rate_plot_update_handler)
rate_plot_options_c_rate_scale_log_radiobutton.toggled.connect(rate_plot_update_handler)

rate_vbox.setSpacing(5)
rate_vbox.setContentsMargins(3, 3, 3, 3)

# Make scrollable area
rate_widget = QtWidgets.QWidget()
rate_widget.setLayout(rate_vbox)
rate_widget.setContentsMargins(0, 0, 0, 0)

rate_scroll_area = QtWidgets.QScrollArea()
rate_scroll_area.setWidgetResizable(True)
rate_scroll_area.setWidget(rate_widget)
rate_scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
rate_scroll_area.setContentsMargins(0, 0, 0, 0)

rate_layout = QtWidgets.QVBoxLayout()
rate_layout.setContentsMargins(0, 0, 0, 0)
rate_layout.addWidget(rate_scroll_area)
tabs[7].setLayout(rate_layout)


"""PLOTTING TAB"""

plotter_vbox = QtWidgets.QVBoxLayout()
plotter_vbox.setAlignment(QtCore.Qt.AlignTop)

# File box
plotter_file_box = QtWidgets.QGroupBox(title="Summary file to plot", flat=False)
format_box_for_parameter(plotter_file_box)
plotter_file_layout = QtWidgets.QVBoxLayout()
plotter_file_box.setLayout(plotter_file_layout)

# File entry
plotter_file_choose_hlayout = QtWidgets.QHBoxLayout()
plotter_file_entry = QtWidgets.QLineEdit()
plotter_file_choose_button = QtWidgets.QPushButton("...")
plotter_file_choose_button.setFixedWidth(32)
plotter_file_choose_button.clicked.connect(lambda: choose_file(plotter_file_entry,"Choose the summary file to load data from"))
plotter_file_choose_hlayout.addWidget(plotter_file_entry)
plotter_file_choose_hlayout.addWidget(plotter_file_choose_button)
plotter_file_layout.addLayout(plotter_file_choose_hlayout)

plotter_vbox.addWidget(plotter_file_box)

# Params box
plotter_params_box = QtWidgets.QGroupBox(title="Plotting options", flat=False)
format_box_for_parameter(plotter_params_box)
plotter_params_layout = QtWidgets.QVBoxLayout()
plotter_params_box.setLayout(plotter_params_layout)

class CheckableComboBox(QtWidgets.QWidget):
	def __init__(self, parent=None, max_height=150):
		super().__init__(parent)

		self.layout = QtWidgets.QHBoxLayout(self)
		self.layout.setContentsMargins(0, 0, 0, 0)

		# Button to show the dropdown
		self.button = QtWidgets.QToolButton(self)
		self.button.setPopupMode(QtWidgets.QToolButton.InstantPopup)
		self.button.clicked.connect(self.toggleDropdown)
		self.layout.addWidget(self.button)

		# Floating dropdown
		self.dropdown = QtWidgets.QFrame(self.parent(), QtCore.Qt.Popup)
		self.dropdown.setFrameShape(QtWidgets.QFrame.StyledPanel)

		# Scroll area inside the dropdown
		self.scroll_area = QtWidgets.QScrollArea(self.dropdown)
		self.scroll_area.setWidgetResizable(True)
		self.scroll_area.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOn)
		self.scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
		self.scroll_area.setFixedHeight(200)

		# Scroll area content widget
		self.scroll_widget = QtWidgets.QWidget()
		self.scroll_layout = QtWidgets.QVBoxLayout(self.scroll_widget)
		self.scroll_widget.setLayout(self.scroll_layout)
		self.scroll_area.setWidget(self.scroll_widget)

		# Add scroll area to dropdown
		dropdown_layout = QtWidgets.QVBoxLayout(self.dropdown)
		dropdown_layout.setContentsMargins(5, 5, 5, 5)
		dropdown_layout.addWidget(self.scroll_area)

		# Dictionary to store checkboxes
		self.checkboxes = {}
		self.top_checkbox_label = None  # Label of the top checkbox

	def addItem(self, label):
		"""Add a checkbox for the given label."""
		checkbox = QtWidgets.QCheckBox(label, self.scroll_widget)
		checkbox.stateChanged.connect(lambda checkbox_state, lbl=label: self.handleCheckboxStateChange(lbl, checkbox_state))
		self.scroll_layout.addWidget(checkbox)
		self.checkboxes[label] = checkbox

		# Set the top checkbox (first one added)
		if self.top_checkbox_label is None:
			self.top_checkbox_label = label

		# Dynamically adjust the width of the dropdown
		self.adjustDropdownWidth()

	def adjustDropdownWidth(self):
		"""Adjust the width of the dropdown to fit the widest checkbox option."""
		font_metrics = QtGui.QFontMetrics(self.scroll_widget.font())
		max_width = 0

		for label in self.checkboxes.keys():
			width = font_metrics.horizontalAdvance(label) + 40  # Add padding for checkbox and margins
			if width > max_width:
				max_width = width

		# Set the new width for the dropdown and scroll area
		self.scroll_area.setFixedWidth(max_width + 20)  # Add extra padding for the scroll bar
		self.dropdown.setFixedWidth(max_width + 30)  # Include some extra margin for the frame

	def handleCheckboxStateChange(self, label, checkbox_state):
		"""Handle the behavior of checkboxes when one is changed."""
		if label == self.top_checkbox_label and checkbox_state == QtCore.Qt.Checked:
			# Uncheck all other checkboxes
			for lbl, chk in self.checkboxes.items():
				if lbl != self.top_checkbox_label:
					chk.setChecked(False)
		elif label != self.top_checkbox_label and checkbox_state == QtCore.Qt.Checked:
			# Uncheck the top checkbox if another checkbox is checked
			self.checkboxes[self.top_checkbox_label].setChecked(False)

	def toggleDropdown(self):
		"""Show or hide the dropdown."""
		if self.dropdown.isVisible():
			self.dropdown.hide()
		else:
			# Position the dropdown just below the button
			button_rect = self.button.geometry()
			global_pos = self.button.mapToGlobal(button_rect.bottomLeft())
			self.dropdown.setGeometry(global_pos.x(), global_pos.y(), button_rect.width(), self.scroll_area.height())
			self.dropdown.show()

	def getSelectedItems(self):
		"""Return a list of selected (checked) items."""
		return [label for label, checkbox in self.checkboxes.items() if checkbox.isChecked()]


class Plotter_DropdownArea(QtWidgets.QWidget):
	def __init__(self):
		super().__init__()
		self.initUI()
		self.summary_filepath = None  # Variable to store the file path of the summary files
		self.experiment_type = None  # Variable to store the type of experiment
		self.data = {}  # Dictionary to store all extracted data from experiments
		self.plot_data = {}  # Dictionary to store extracted data for plotting
		self.experiments_complete_bool = None  # Boolean to store the completion-state of the experiments chosen
		self.all_experiments_loaded_bool = None  # Boolean to store the load-state of experiments listed in the summary file
		self.unloaded_filepaths = []

		# Experiment-specific
		self.file_checkboxes = {}  # Store references to file checkboxes
		self.init_checkboxes = {}  # Store references to initialisation checkboxes
		self.cycle_dropdowns = {}  # Store references to cycle dropdowns


	def initUI(self):
		self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

		# Button to load data and toggle the dropdown
		self.load_button = QtWidgets.QPushButton("Load data from file", self)
		self.load_button.clicked.connect(self.handleLoadButtonClick)

		# Dropdown area
		self.dropdown_frame = QtWidgets.QFrame(self)
		self.dropdown_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
		self.dropdown_frame.setHidden(True)  # Initially hidden

		# Set dynamic resizing policies
		self.dropdown_frame.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

		# Layout for dropdown contents
		self.plotter_params_dropdown_layout = QtWidgets.QVBoxLayout(self.dropdown_frame)

		# Set spacing and margins for dropdown content
		self.plotter_params_dropdown_layout.setSpacing(5)
		self.plotter_params_dropdown_layout.setContentsMargins(3, 10, 3, 3)

		# Main layout
		main_layout = QtWidgets.QVBoxLayout(self)
		main_layout.addWidget(self.load_button)
		main_layout.addWidget(self.dropdown_frame)
		main_layout.setSpacing(5)
		main_layout.setContentsMargins(0, 0, 0, 0)

	def handleLoadButtonClick(self):
		if self.dropdown_frame.isHidden():
			if self.parseFile():  # Load data and set the experiment type
				self.updateDropdownWidgets()  # Adjust dropdown widgets
				self.toggleDropdown()  # Show dropdown
		else:
			self.clearData()  # Clear loaded data
			self.clearWidgets()  # Clear existing widgets
			self.toggleDropdown()  # Hide dropdown

	def handleStateChange(self):
		self.updateDropdownWidgets()  # Update dropdown widgets

	def parseFile(self):
		self.all_experiments_loaded_bool = True  # Initialise as True
		self.summary_filepath = plotter_file_entry.text().strip()
		if self.summary_filepath == "":
			self.experiment_type = "filepath_not_given"
			QtWidgets.QMessageBox.critical(
				mainwidget,
				"Filepath not given",
				"Please provide the filepath to a summary file to extract data from."
			)
			return False
		else:
			if not os.path.isfile(self.summary_filepath):
				QtWidgets.QMessageBox.critical(
					mainwidget,
					"Filepath invalid",
					"File does not exist. Please provide the filepath to a summary file to extract data from."
				)
				return False

			with open(self.summary_filepath, 'r') as file:
				lines = file.readlines()

			data = {}

			if lines[0].strip() == "CV EXPERIMENTS INFORMATION FILE":
				self.experiment_type = "CV"

				# Start processing after "Experiment information file for the experiments stored in:"
				for i in range(len(lines)):
					if lines[i].strip() == "Experiment information file for the experiments stored in:":
						for file_line in lines[i + 1:]:
							# Stop at the first empty line
							if file_line.strip() == "":
								break

							match = re.search(r'.*?([-\w.]+)_([-\w.]+)_([-\w.]+)_V_([-\d.]+)_mV_s\.dat', file_line)
							if match:
								lbound = match.group(2)
								ubound = match.group(3)
								scanrate_mV = match.group(4)

								data_filepath = file_line.strip()
								file_data = {}

								# Attempt to read data file
								if os.path.exists(data_filepath):
									with open(data_filepath, 'r') as data_file:
										try:
											header = next(data_file)  # Skip header row
										except StopIteration:
											log_message(f"WARNING: {data_filepath} is empty")
											self.all_experiments_loaded_bool = False
											continue  # Skip to the next file

										for data_line in data_file:
											parts = data_line.strip().split('\t')
											cycle_number = int(parts[0])
											entry = {
												'elapsed_time': float(parts[1]),
												'potential': float(parts[2]),
												'current': float(parts[3])
											}

											# Group data by cycle number
											if cycle_number not in file_data:
												file_data[cycle_number] = []
											file_data[cycle_number].append(entry)

									# Store the data
									data[data_filepath] = {
										'lbound': lbound,
										'ubound': ubound,
										'scanrate_mV': scanrate_mV,
										'data': file_data
									}

								else:  # If the data filepath cannot be located
									self.unloaded_filepaths.append(data_filepath)
									self.all_experiments_loaded_bool = False

			elif lines[0].strip() == "LSV EXPERIMENTS INFORMATION FILE":
				self.experiment_type = "LSV"

				# Start processing after "Experiment information file for the experiments stored in:"
				for i in range(len(lines)):
					if lines[i].strip() == "Experiment information file for the experiments stored in:":
						for file_line in lines[i + 1:]:
							# Stop at the first empty line
							if file_line.strip() == "":
								break

							match = re.search(r'.*?([-\w.]+)_([-\w.]+)_([-\w.]+)_V_([-\d.]+)_mV_s\.dat', file_line)
							if match:
								lbound = match.group(2)
								ubound = match.group(3)
								scanrate_mV = match.group(4)

								data_filepath = file_line.strip()
								file_data = {}

								# Attempt to read data file
								if os.path.exists(data_filepath):
									with open(data_filepath, 'r') as data_file:
										try:
											header = next(data_file)  # Skip header row
										except StopIteration:
											log_message(f"WARNING: {data_filepath} is empty")
											self.all_experiments_loaded_bool = False
											continue  # Skip to the next file

										for data_line in data_file:
											parts = data_line.strip().split('\t')
											experiment_segment = str(parts[0])
											entry = {
												'elapsed_time': float(parts[1]),
												'potential': float(parts[2]),
												'current': float(parts[3])
											}

											# Group data by experiment segment
											if experiment_segment not in file_data:
												file_data[experiment_segment] = []
											file_data[experiment_segment].append(entry)

									# Store the data
									data[data_filepath] = {
										'lbound': lbound,
										'ubound': ubound,
										'scanrate_mV': scanrate_mV,
										'data': file_data
									}

								else:  # If the data filepath cannot be located
									self.unloaded_filepaths.append(data_filepath)
									self.all_experiments_loaded_bool = False

			elif lines[0].strip() == "GCD EXPERIMENTS INFORMATION FILE":
				self.experiment_type = "GCD"

				# Start processing after "Experiment information file for the experiments stored in:"
				for i in range(len(lines)):
					if lines[i].strip() == "Experiment information file for the experiments stored in:":
						for file_line in lines[i + 1:]:
							# Stop at the first empty line
							if file_line.strip() == "":
								break

							match = re.search(r'.*?([-\w.]+)_([-\w.]+)_([-\w.]+)_V_([-\d.]+)_([-\d.]+)_uA\.dat', file_line)
							if match:
								lbound = match.group(2)
								ubound = match.group(3)
								charge_current = float(match.group(4))
								discharge_current = float(match.group(5))

								data_filepath = file_line.strip()
								file_data = {}

								# Attempt to read data file
								if os.path.exists(data_filepath):
									with open(data_filepath, 'r') as data_file:
										try:
											header = next(data_file)  # Skip header row
										except StopIteration:
											log_message(f"WARNING: {data_filepath} is empty")
											self.all_experiments_loaded_bool = False
											continue  # Skip to the next file

										for data_line in data_file:
											parts = data_line.strip().split('\t')
											cycle_number = int(parts[0])
											halfcycle_number = int(parts[1])
											entry = {
												'elapsed_time': float(parts[2]),
												'potential': float(parts[3]),
												'current': float(parts[4])
											}

											# Group data by cycle number and halfcycle number
											if cycle_number not in file_data:
												file_data[cycle_number] = {}  # Initialise a dictionary for each cycle number
											if halfcycle_number not in file_data[cycle_number]:
												file_data[cycle_number][halfcycle_number] = []  # Initialise list for each halfcycle
											file_data[cycle_number][halfcycle_number].append(entry)

									# Store the data
									data[data_filepath] = {
										'lbound': lbound,
										'ubound': ubound,
										'charge_current': charge_current,
										'discharge_current': discharge_current,
										'data': file_data
									}

								else:  # If the data filepath cannot be located
									self.unloaded_filepaths.append(data_filepath)
									self.all_experiments_loaded_bool = False

			elif lines[0].strip() == "CA EXPERIMENTS INFORMATION FILE":
				self.experiment_type = "CA"

				# Extract filepath after "Experiment information file for the experiments stored in:"
				for i in range(len(lines)):
					if lines[i].strip() == "Experiment information file for the experiments stored in:":
						file_line = lines[i+1]
						break

				match = re.search(r'.*?_((?:-?\d+(?:\.\d+)?|OCP)(?:_(?:-?\d+(?:\.\d+)?|OCP))*)_V\.dat', file_line)
				if match:
					hold_potentials = [pot for pot in match.group(1).split('_') if pot]

					data_filepath = file_line.strip()
					file_data = {}

					# Attempt to read the file
					if os.path.exists(data_filepath):
						with open(data_filepath, 'r') as data_file:
							next(data_file)  # Skip header row
							for data_line in data_file:
								parts = data_line.strip().split('\t')
								segment_number = int(parts[0])
								ramp_or_hold = str(parts[1])
								entry = {
									'elapsed_time': float(parts[2]),
									'segment_time': float(parts[3]),
									'potential': float(parts[4]),
									'current': float(parts[5])
								}

								# Group data by segment number and ramping/holding
								if segment_number not in file_data:
									file_data[segment_number] = {}  # Initialise a dictionary for each segment number
								if ramp_or_hold not in file_data[segment_number]:
									file_data[segment_number][ramp_or_hold] = []  # Initialise a list for each ramp/hold
								file_data[segment_number][ramp_or_hold].append(entry)

						# Store the data
						for i, segment_number in enumerate(file_data.keys()):
							data[segment_number] = {
								'hold_potential': hold_potentials[i],
								'data': file_data[segment_number]
							}

					else:  # If the filepath cannot be located
						self.unloaded_filepaths.append(data_filepath)
						self.all_experiments_loaded_bool = False

			elif lines[0].strip() == "CP EXPERIMENTS INFORMATION FILE":
				self.experiment_type = "CP"

				# Extract filepath after "Experiment information file for the experiments stored in:"
				for i in range(len(lines)):
					if lines[i].strip() == "Experiment information file for the experiments stored in:":
						file_line = lines[i+1]
						break

				match = re.search(r'.*?_(-?[\d.]+(?:_-?[\d.]+)*)_uA\.dat', file_line)
				if match:
					hold_currents = [curr for curr in match.group(1).split('_') if curr]

					data_filepath = file_line.strip()
					file_data = {}

					# Attempt to read the file
					if os.path.exists(data_filepath):
						with open(data_filepath, 'r') as data_file:
							next(data_file)  # Skip header row
							for data_line in data_file:
								parts = data_line.strip().split('\t')
								segment_number = int(parts[0])
								ramp_or_hold = str(parts[1])
								entry = {
									'elapsed_time': float(parts[2]),
									'segment_time': float(parts[3]),
									'potential': float(parts[4]),
									'current': float(parts[5])
								}

								# Group data by segment number and ramping/holding
								if segment_number not in file_data:
									file_data[segment_number] = {}  # Initialise a dictionary for each segment number
								if ramp_or_hold not in file_data[segment_number]:
									file_data[segment_number][ramp_or_hold] = []  # Initialise a list for each ramp/hold
								file_data[segment_number][ramp_or_hold].append(entry)

						# Store the data
						for i, segment_number in enumerate(file_data.keys()):
							data[segment_number] = {
								'hold_current': hold_currents[i],
								'data': file_data[segment_number]
							}

					else:  # If the filepath cannot be located
						self.unloaded_filepaths.append(data_filepath)
						self.all_experiments_loaded_bool = False

			elif lines[0].strip() == "SD EXPERIMENTS INFORMATION FILE":
				self.experiment_type = "SD"

				# Extract filepath after "Experiment information file for the experiments stored in:"
				for i in range(len(lines)):
					if lines[i].strip() == "Experiment information file for the experiments stored in:":
						file_line = lines[i + 1]
						break

				match = re.search(r'.*?_((?:-?\d+(?:\.\d+)?|OCP)(?:_(?:-?\d+(?:\.\d+)?|OCP))*)_V\.dat', file_line)
				if match:
					hold_potentials = [pot for pot in match.group(1).split('_') if pot]

					data_filepath = file_line.strip()
					file_data = {}

					# Attempt to read the file
					if os.path.exists(data_filepath):
						with open(data_filepath, 'r') as data_file:
							next(data_file)  # Skip header row
							for data_line in data_file:
								parts = data_line.strip().split('\t')
								segment_number = int(parts[0])
								charge_selfdischarge = str(parts[1])
								entry = {
									'elapsed_time': float(parts[2]),
									'segment_time': float(parts[3]),
									'potential': float(parts[4]),
									'current': float(parts[5])
								}

								# Group data by segment number and charge/self-discharge
								if segment_number not in file_data:
									file_data[segment_number] = {}  # Initialise a dictionary for each segment number
								if charge_selfdischarge not in file_data[segment_number]:
									file_data[segment_number][charge_selfdischarge] = []  # Initialise list for each charge/self-discharge
								file_data[segment_number][charge_selfdischarge].append(entry)

						# Store the data
						for i, segment_number in enumerate(file_data.keys()):
							data[segment_number] = {
								'hold_potential': hold_potentials[i],
								'data': file_data[segment_number]
							}

					else:  # If the filepath cannot be located
						self.unloaded_filepaths.append(data_filepath)
						self.all_experiments_loaded_bool = False

			elif lines[0].strip() == "RATE-TESTING EXPERIMENTS INFORMATION FILE":
				self.experiment_type = "Rate"

				# Start processing after "Calculated charge/discharge capacities for each full cycle are stored in:"
				for i in range(len(lines)):
					if lines[i].strip() == "Calculated charge/discharge capacities for each full cycle are stored in:":
						for file_line in lines[i + 1:]:
							# Stop at the first empty line
							if file_line.strip() == "":
								break

							match = re.search(r'.*?([-\w.]+)_([-\w.]+)_([-\w.]+)_V_((?:[-.\d]+_?)*)_C_capacities\.dat', file_line)
							if match:
								lbound = match.group(2)
								ubound = match.group(3)
								c_rates = [float(c_rate) for c_rate in match.group(4).split('_') if c_rate]

								data_filepath = file_line.strip()
								file_data = {}

								# Attempt to read the file
								if os.path.exists(data_filepath):
									with open(data_filepath, 'r') as data_file:
										try:
											header = next(data_file)  # Skip header row
										except StopIteration:
											log_message(f"WARNING: {data_filepath} is empty")
											self.all_experiments_loaded_bool = False
											continue  # Skip to the next file

										for data_line in data_file:
											parts = data_line.strip().split('\t')
											c_rate = float(parts[0])
											cycle_num = int(parts[1])
											entry = {
												'charge_capacity': float(parts[2]),
												'discharge_capacity': float(parts[3])
											}

											# Group data by C-rate and cycle number
											if c_rate not in file_data:
												file_data[c_rate] = {}  # Initialise a dictionary for each C-rate
											file_data[c_rate][cycle_num] = entry

									# Store the data
									data[data_filepath] = {
										'lbound': lbound,
										'ubound': ubound,
										'c_rates': c_rates,
										'data': file_data
									}

								else:  # If the data filepath cannot be located
									self.unloaded_filepaths.append(data_filepath)
									self.all_experiments_loaded_bool = False

			if lines[-1].strip() == "EXPERIMENTS COMPLETE":
				self.experiments_complete_bool = True
			else:
				self.experiments_complete_bool = False

			self.data = data

			return True

	def updateDropdownWidgets(self):

		# Ensure existing widgets are cleared
		self.clearWidgets()

		# Check data has been stored successfully
		if not self.data:
			data_empty_warning = QtWidgets.QLabel("No data could be loaded.\n\nPlease check that data filepaths in the summary file are correct\nand ensure that the data files are not empty.")
			data_empty_warning.setAlignment(QtCore.Qt.AlignCenter)
			self.plotter_params_dropdown_layout.addWidget(data_empty_warning)
			log_message("No data could be loaded into the plotting tab.")
			return

		# Check if the loaded experiments were completed successfully
		if not self.experiments_complete_bool:
			experiments_complete_warning = QtWidgets.QLabel("WARNING: According to the summary file the experiments chosen\nfor plotting did not all complete successfully.")
			experiments_complete_warning.setAlignment(QtCore.Qt.AlignCenter)
			self.plotter_params_dropdown_layout.addWidget(experiments_complete_warning)
			log_message("WARNING: The chosen experiments did not successfully complete")
			QtWidgets.QMessageBox.warning(
				mainwidget,
				"Incomplete experiments",
				"WARNING: According to the summary file the experiments chosen\nfor plotting did not all complete successfully."
			)

		# Check if all chosen experiments were loaded
		if self.all_experiments_loaded_bool:
			log_message("All chosen experiments for plotting were loaded successfully")
		else:
			if self.unloaded_filepaths:
				unloaded_filepaths_str = ""
				unloaded_filenames_str = ""
				for unloaded_filepath in self.unloaded_filepaths:
					unloaded_filepaths_str += f"{unloaded_filepath}\n\n"
					unloaded_filenames_str += f"{os.path.basename(unloaded_filepath)}\n"
				experiments_loaded_warning = QtWidgets.QLabel("WARNING: Not all chosen experiments were located\nand loaded successfully.")
				experiments_loaded_warning.setAlignment(QtCore.Qt.AlignCenter)
				self.plotter_params_dropdown_layout.addWidget(experiments_loaded_warning)
				self.plotter_params_dropdown_layout.addWidget(QtWidgets.QLabel(f"The following data files could be not be located:\n{unloaded_filenames_str}"))
				log_message("WARNING: Not all chosen experiments were located and loaded successfully")
				QtWidgets.QMessageBox.warning(
					mainwidget,
					"Unable to load all data files",
					"WARNING: The following data files listed in the summary file could not be located:\n\n"
					f"{unloaded_filepaths_str}"
				)

		if self.experiment_type == "CV":

			plot_params_label = QtWidgets.QLabel("<b>CV plotting parameters</b>")
			plot_params_label.setAlignment(QtCore.Qt.AlignCenter)
			self.plotter_params_dropdown_layout.addWidget(plot_params_label)

			self.plotter_params_dropdown_layout.addWidget(create_line())

			self.plotter_params_dropdown_layout.addWidget(QtWidgets.QLabel("<u>Experiment and cycle select:</u>"))

			# Create a grid layout for consistent alignment
			grid_layout = QtWidgets.QGridLayout()
			grid_layout.setHorizontalSpacing(20)
			grid_layout.setVerticalSpacing(5)

			# Add header row to the grid
			header_select = QtWidgets.QLabel("Select")
			header_experiment_info = QtWidgets.QLabel("Experiment info")
			header_cycle_selection = QtWidgets.QLabel("Cycle selection")

			# Center-align the headers
			header_select.setAlignment(QtCore.Qt.AlignCenter)
			header_experiment_info.setAlignment(QtCore.Qt.AlignCenter)
			header_cycle_selection.setAlignment(QtCore.Qt.AlignCenter)

			# Add headers to the grid layout
			grid_layout.addWidget(header_select, 0, 0)
			grid_layout.addWidget(header_experiment_info, 0, 1)
			grid_layout.addWidget(header_cycle_selection, 0, 2)

			# Populate the grid with data
			row = 1
			for filepath, file_info in self.data.items():
				# Checkbox for the file
				file_checkbox = QtWidgets.QCheckBox()
				file_checkbox.setChecked(False)
				grid_layout.addWidget(file_checkbox, row, 0, alignment=QtCore.Qt.AlignCenter)

				# Label for experiment info
				file_label = QtWidgets.QLabel(f"{file_info['lbound']} / {file_info['ubound']} V; {file_info['scanrate_mV']} mV/s")
				grid_layout.addWidget(file_label, row, 1)

				# Custom dropdown for cycle numbers
				cycle_dropdown = CheckableComboBox(self)
				cycle_dropdown.button.setText("Select cycles")
				cycle_dropdown.addItem("All cycles")
				for cycle_number in sorted(file_info['data'].keys()):
					cycle_dropdown.addItem(str(cycle_number))

				# Make the dropdown 50% wider
				default_width = cycle_dropdown.sizeHint().width()
				cycle_dropdown.setFixedWidth(int(default_width * 1.5))

				grid_layout.addWidget(cycle_dropdown, row, 2)

				# Store the checkbox and cycle dropdown references by file path
				self.file_checkboxes[filepath] = file_checkbox
				self.cycle_dropdowns[filepath] = cycle_dropdown

				row += 1

			# Add the grid layout to the main dropdown layout
			self.plotter_params_dropdown_layout.addLayout(grid_layout)

			self.plotter_params_dropdown_layout.addWidget(create_line())
			self.plotter_params_dropdown_layout.addWidget(QtWidgets.QLabel("<u>Other options:</u>"))

			options_hlayout = QtWidgets.QHBoxLayout()
			options_units_label = QtWidgets.QLabel("Units:")
			options_xunits_label = QtWidgets.QLabel("Potential")
			self.options_xunits_dropdown = QtWidgets.QComboBox()
			self.options_xunits_dropdown.addItems(["V", "mV", "µV"])
			options_yunits_label = QtWidgets.QLabel("; Current")
			self.options_yunits_dropdown = QtWidgets.QComboBox()
			self.options_yunits_dropdown.addItems(["A", "mA", "µA"])
			self.options_legend_checkbox = QtWidgets.QCheckBox("Legend?")
			self.options_legend_checkbox.setChecked(True)
			options_hlayout.addWidget(options_units_label)
			options_hlayout.addWidget(options_xunits_label)
			options_hlayout.addWidget(self.options_xunits_dropdown)
			options_hlayout.addWidget(options_yunits_label)
			options_hlayout.addWidget(self.options_yunits_dropdown)
			options_hlayout.addWidget(QtWidgets.QLabel("     "))
			options_hlayout.addWidget(self.options_legend_checkbox)

			self.plotter_params_dropdown_layout.addLayout(options_hlayout)

		elif self.experiment_type == "LSV":

			plot_params_label = QtWidgets.QLabel("<b>LSV plotting parameters</b>")
			plot_params_label.setAlignment(QtCore.Qt.AlignCenter)
			self.plotter_params_dropdown_layout.addWidget(plot_params_label)

			self.plotter_params_dropdown_layout.addWidget(create_line())

			self.plotter_params_dropdown_layout.addWidget(QtWidgets.QLabel("<u>Experiment and segment select:</u>"))

			# Create a grid layout for consistent alignment
			grid_layout = QtWidgets.QGridLayout()
			grid_layout.setHorizontalSpacing(20)
			grid_layout.setVerticalSpacing(5)

			# Add header row to the grid
			header_select = QtWidgets.QLabel("Select")
			header_experiment_info = QtWidgets.QLabel("Experiment info")
			header_init = QtWidgets.QLabel("Show initialisation?")

			# Center-align the headers
			header_select.setAlignment(QtCore.Qt.AlignCenter)
			header_experiment_info.setAlignment(QtCore.Qt.AlignCenter)
			header_init.setAlignment(QtCore.Qt.AlignCenter)

			# Add headers to the grid layout
			grid_layout.addWidget(header_select, 0, 0)
			grid_layout.addWidget(header_experiment_info, 0, 1)
			grid_layout.addWidget(header_init, 0, 2)

			# Populate the grid with data
			row = 1
			for filepath, file_info in self.data.items():
				# Checkbox for the file
				file_checkbox = QtWidgets.QCheckBox()
				file_checkbox.setChecked(False)
				grid_layout.addWidget(file_checkbox, row, 0, alignment=QtCore.Qt.AlignCenter)

				# Label for experiment info
				file_label = QtWidgets.QLabel(
					f"{file_info['lbound']} / {file_info['ubound']} V; {file_info['scanrate_mV']} mV/s")
				grid_layout.addWidget(file_label, row, 1)

				# Checkbox for initialisation
				init_checkbox = QtWidgets.QCheckBox()
				init_checkbox.setChecked(False)
				grid_layout.addWidget(init_checkbox, row, 2, alignment=QtCore.Qt.AlignCenter)

				# Store the checkbox and initialisation checkbox references by file path
				self.file_checkboxes[filepath] = file_checkbox
				self.init_checkboxes[filepath] = init_checkbox

				row += 1

			# Add the grid layout to the main dropdown layout
			self.plotter_params_dropdown_layout.addLayout(grid_layout)

			self.plotter_params_dropdown_layout.addWidget(create_line())
			self.plotter_params_dropdown_layout.addWidget(QtWidgets.QLabel("<u>Other options:</u>"))

			options_hlayout = QtWidgets.QHBoxLayout()
			options_units_label = QtWidgets.QLabel("Units:")
			options_xunits_label = QtWidgets.QLabel("Potential")
			self.options_xunits_dropdown = QtWidgets.QComboBox()
			self.options_xunits_dropdown.addItems(["V", "mV", "µV"])
			options_yunits_label = QtWidgets.QLabel("; Current")
			self.options_yunits_dropdown = QtWidgets.QComboBox()
			self.options_yunits_dropdown.addItems(["A", "mA", "µA"])
			self.options_legend_checkbox = QtWidgets.QCheckBox("Legend?")
			self.options_legend_checkbox.setChecked(True)
			options_hlayout.addWidget(options_units_label)
			options_hlayout.addWidget(options_xunits_label)
			options_hlayout.addWidget(self.options_xunits_dropdown)
			options_hlayout.addWidget(options_yunits_label)
			options_hlayout.addWidget(self.options_yunits_dropdown)
			options_hlayout.addWidget(QtWidgets.QLabel("     "))
			options_hlayout.addWidget(self.options_legend_checkbox)

			self.plotter_params_dropdown_layout.addLayout(options_hlayout)

		elif self.experiment_type == "GCD":

			plot_params_label = QtWidgets.QLabel("<b>GCD plotting parameters</b>")
			plot_params_label.setAlignment(QtCore.Qt.AlignCenter)
			self.plotter_params_dropdown_layout.addWidget(plot_params_label)

			self.plotter_params_dropdown_layout.addWidget(create_line())

			self.plotter_params_dropdown_layout.addWidget(QtWidgets.QLabel("<u>Experiment and cycle select:</u>"))

			# Create a grid layout for consistent alignment
			grid_layout = QtWidgets.QGridLayout()
			grid_layout.setHorizontalSpacing(20)
			grid_layout.setVerticalSpacing(5)

			# Add header row to the grid
			header_select = QtWidgets.QLabel("Select")
			header_experiment_info = QtWidgets.QLabel("Experiment info")
			header_cycle_selection = QtWidgets.QLabel("Cycle selection")

			# Center-align the headers
			header_select.setAlignment(QtCore.Qt.AlignCenter)
			header_experiment_info.setAlignment(QtCore.Qt.AlignCenter)
			header_cycle_selection.setAlignment(QtCore.Qt.AlignCenter)

			# Add headers to the grid layout
			grid_layout.addWidget(header_select, 0, 0)
			grid_layout.addWidget(header_experiment_info, 0, 1)
			grid_layout.addWidget(header_cycle_selection, 0, 2)

			# Populate the grid with data
			row = 1
			for filepath, file_info in self.data.items():
				# Checkbox for the file
				file_checkbox = QtWidgets.QCheckBox()
				file_checkbox.setChecked(False)
				grid_layout.addWidget(file_checkbox, row, 0, alignment=QtCore.Qt.AlignCenter)

				# Label for experiment info
				file_label = QtWidgets.QLabel(f"{file_info['lbound']} / {file_info['ubound']} V; {file_info['charge_current']} / {file_info['discharge_current']} µA")
				grid_layout.addWidget(file_label, row, 1)

				# Custom dropdown for cycle numbers
				cycle_dropdown = CheckableComboBox(self)
				cycle_dropdown.button.setText("Select cycles")
				cycle_dropdown.addItem("All cycles")
				for cycle_number in sorted(file_info['data'].keys()):
					cycle_dropdown.addItem(str(cycle_number))

				# Make the dropdown 50% wider
				default_width = cycle_dropdown.sizeHint().width()
				cycle_dropdown.setFixedWidth(int(default_width * 1.5))

				grid_layout.addWidget(cycle_dropdown, row, 2)

				# Store the checkbox and cycle dropdown references by file path
				self.file_checkboxes[filepath] = file_checkbox
				self.cycle_dropdowns[filepath] = cycle_dropdown

				row += 1

			# Add the grid layout to the main dropdown layout
			self.plotter_params_dropdown_layout.addLayout(grid_layout)

			self.plotter_params_dropdown_layout.addWidget(create_line())
			self.plotter_params_dropdown_layout.addWidget(QtWidgets.QLabel("<u>Other options:</u>"))

			options_hlayout = QtWidgets.QHBoxLayout()
			options_units_label = QtWidgets.QLabel("Units:")
			options_yunits_label = QtWidgets.QLabel("Potential")
			self.options_yunits_dropdown = QtWidgets.QComboBox()
			self.options_yunits_dropdown.addItems(["V", "mV", "µV"])
			self.options_legend_checkbox = QtWidgets.QCheckBox("Legend?")
			self.options_legend_checkbox.setChecked(True)
			options_hlayout.addWidget(options_units_label)
			options_hlayout.addWidget(options_yunits_label)
			options_hlayout.addWidget(self.options_yunits_dropdown)
			options_hlayout.addWidget(QtWidgets.QLabel("     "))
			options_hlayout.addWidget(self.options_legend_checkbox)

			self.plotter_params_dropdown_layout.addLayout(options_hlayout)

		elif self.experiment_type == "CA":

			plot_params_label = QtWidgets.QLabel("<b>Chronoamperometry plotting parameters</b>")
			plot_params_label.setAlignment(QtCore.Qt.AlignCenter)
			self.plotter_params_dropdown_layout.addWidget(plot_params_label)

			self.plotter_params_dropdown_layout.addWidget(create_line())

			self.plotter_params_dropdown_layout.addWidget(QtWidgets.QLabel("<u>Segment select:</u>"))

			# Create a grid layout for consistent alignment
			grid_layout = QtWidgets.QGridLayout()
			grid_layout.setHorizontalSpacing(20)
			grid_layout.setVerticalSpacing(5)

			# Add header row to the grid
			header_select = QtWidgets.QLabel("Select")
			header_segment_info = QtWidgets.QLabel("Segment pair & hold potentials")

			# Add headers to the grid layout
			grid_layout.addWidget(header_select, 0, 0, alignment=QtCore.Qt.AlignCenter)
			grid_layout.addWidget(header_segment_info, 0, 1, alignment=QtCore.Qt.AlignCenter)

			# Populate the grid with data
			row = 1
			segments = list(self.data.keys())
			for i in range(len(segments)-1):

				# Checkbox for the segment pair
				pair_checkbox = QtWidgets.QCheckBox()
				pair_checkbox.setChecked(False)
				grid_layout.addWidget(pair_checkbox, row, 0, alignment=QtCore.Qt.AlignCenter)

				# Construct segment pair info
				segment1 = segments[i]
				segment2 = segments[i + 1]

				hold_potential1 = self.data[segment1]['hold_potential']
				hold_potential2 = self.data[segment2]['hold_potential']
				segment_pair_label = QtWidgets.QLabel(
					f"Segment {segment1} - {segment2}: {hold_potential1} / {hold_potential2} V")
				grid_layout.addWidget(segment_pair_label, row, 1)

				# Store the checkbox and initialisation checkbox references by segment pair
				self.file_checkboxes[(segment1, segment2)] = pair_checkbox

				row += 1

			# Add the grid layout to the main dropdown layout
			self.plotter_params_dropdown_layout.addLayout(grid_layout)

			self.plotter_params_dropdown_layout.addWidget(create_line())

			options1_hlayout = QtWidgets.QHBoxLayout()
			options2_hlayout = QtWidgets.QHBoxLayout()

			self.options_potential_checkbox = QtWidgets.QCheckBox("Plot potential?")
			self.options_potential_checkbox.setChecked(True)
			options_adjust_xvals_label = QtWidgets.QLabel("Adjust start time (s):")
			self.options_adjust_xvals_entry = QtWidgets.QLineEdit()
			options1_hlayout.addWidget(QtWidgets.QLabel("<u>Other options:</u>"))
			options1_hlayout.addWidget(self.options_potential_checkbox)
			options1_hlayout.addWidget(options_adjust_xvals_label)
			options1_hlayout.addWidget(self.options_adjust_xvals_entry)

			options_units_label = QtWidgets.QLabel("Units:")
			options_y1units_label = QtWidgets.QLabel("Current")
			self.options_y1units_dropdown = QtWidgets.QComboBox()
			self.options_y1units_dropdown.addItems(["A", "mA", "µA"])
			options_y2units_label = QtWidgets.QLabel("; Potential")
			self.options_y2units_dropdown = QtWidgets.QComboBox()
			self.options_y2units_dropdown.addItems(["V", "mV", "µV"])

			self.options_legend_checkbox = QtWidgets.QCheckBox("Legend?")
			self.options_legend_checkbox.setChecked(True)
			options2_hlayout.addWidget(options_units_label)
			options2_hlayout.addWidget(options_y1units_label)
			options2_hlayout.addWidget(self.options_y1units_dropdown)
			options2_hlayout.addWidget(options_y2units_label)
			options2_hlayout.addWidget(self.options_y2units_dropdown)
			options2_hlayout.addWidget(QtWidgets.QLabel("     "))
			options2_hlayout.addWidget(self.options_legend_checkbox)

			self.plotter_params_dropdown_layout.addLayout(options1_hlayout)
			self.plotter_params_dropdown_layout.addLayout(options2_hlayout)

		elif self.experiment_type == "CP":

			plot_params_label = QtWidgets.QLabel("<b>Chronopotentiometry plotting parameters</b>")
			plot_params_label.setAlignment(QtCore.Qt.AlignCenter)
			self.plotter_params_dropdown_layout.addWidget(plot_params_label)

			self.plotter_params_dropdown_layout.addWidget(create_line())

			self.plotter_params_dropdown_layout.addWidget(QtWidgets.QLabel("<u>Segment select:</u>"))

			# Create a grid layout for consistent alignment
			grid_layout = QtWidgets.QGridLayout()
			grid_layout.setHorizontalSpacing(20)
			grid_layout.setVerticalSpacing(5)

			# Add header row to the grid
			header_select = QtWidgets.QLabel("Select")
			header_segment_info = QtWidgets.QLabel("Segment pair & hold currents")

			# Add headers to the grid layout
			grid_layout.addWidget(header_select, 0, 0, alignment=QtCore.Qt.AlignCenter)
			grid_layout.addWidget(header_segment_info, 0, 1, alignment=QtCore.Qt.AlignCenter)

			# Populate the grid with data
			row = 1
			segments = list(self.data.keys())
			for i in range(len(segments)-1):

				# Checkbox for the segment pair
				pair_checkbox = QtWidgets.QCheckBox()
				pair_checkbox.setChecked(False)
				grid_layout.addWidget(pair_checkbox, row, 0, alignment=QtCore.Qt.AlignCenter)

				# Construct segment pair info
				segment1 = segments[i]
				segment2 = segments[i + 1]

				hold_current1 = self.data[segment1]['hold_current']
				hold_current2 = self.data[segment2]['hold_current']
				segment_pair_label = QtWidgets.QLabel(
					f"Segment {segment1} - {segment2}: {hold_current1} / {hold_current2} µA")
				grid_layout.addWidget(segment_pair_label, row, 1)

				# Store the checkbox and initialisation checkbox references by segment pair
				self.file_checkboxes[(segment1, segment2)] = pair_checkbox

				row += 1

			# Add the grid layout to the main dropdown layout
			self.plotter_params_dropdown_layout.addLayout(grid_layout)

			self.plotter_params_dropdown_layout.addWidget(create_line())

			options1_hlayout = QtWidgets.QHBoxLayout()
			options2_hlayout = QtWidgets.QHBoxLayout()

			self.options_current_checkbox = QtWidgets.QCheckBox("Plot current?")
			self.options_current_checkbox.setChecked(True)
			options_adjust_xvals_label = QtWidgets.QLabel("Adjust start time (s):")
			self.options_adjust_xvals_entry = QtWidgets.QLineEdit()
			options1_hlayout.addWidget(QtWidgets.QLabel("<u>Other options:</u>"))
			options1_hlayout.addWidget(self.options_current_checkbox)
			options1_hlayout.addWidget(options_adjust_xvals_label)
			options1_hlayout.addWidget(self.options_adjust_xvals_entry)

			options_units_label = QtWidgets.QLabel("Units:")
			options_y1units_label = QtWidgets.QLabel("Potential")
			self.options_y1units_dropdown = QtWidgets.QComboBox()
			self.options_y1units_dropdown.addItems(["V", "mV", "µV"])
			options_y2units_label = QtWidgets.QLabel("; Current")
			self.options_y2units_dropdown = QtWidgets.QComboBox()
			self.options_y2units_dropdown.addItems(["A", "mA", "µA"])

			self.options_legend_checkbox = QtWidgets.QCheckBox("Legend?")
			self.options_legend_checkbox.setChecked(True)
			options2_hlayout.addWidget(options_units_label)
			options2_hlayout.addWidget(options_y1units_label)
			options2_hlayout.addWidget(self.options_y1units_dropdown)
			options2_hlayout.addWidget(options_y2units_label)
			options2_hlayout.addWidget(self.options_y2units_dropdown)
			options2_hlayout.addWidget(QtWidgets.QLabel("     "))
			options2_hlayout.addWidget(self.options_legend_checkbox)

			self.plotter_params_dropdown_layout.addLayout(options1_hlayout)
			self.plotter_params_dropdown_layout.addLayout(options2_hlayout)

		elif self.experiment_type == "SD":

			plot_params_label = QtWidgets.QLabel("<b>Self-discharge plotting parameters</b>")
			plot_params_label.setAlignment(QtCore.Qt.AlignCenter)
			self.plotter_params_dropdown_layout.addWidget(plot_params_label)

			self.plotter_params_dropdown_layout.addWidget(create_line())

			self.plotter_params_dropdown_layout.addWidget(QtWidgets.QLabel("<u>Segment select:</u>"))

			# Create a grid layout for consistent alignment
			grid_layout = QtWidgets.QGridLayout()
			grid_layout.setHorizontalSpacing(20)
			grid_layout.setVerticalSpacing(5)

			# Add header row to the grid
			header_select = QtWidgets.QLabel("Select")
			header_segment_info = QtWidgets.QLabel("Segment charge potential")

			# Add headers to the grid layout
			grid_layout.addWidget(header_select, 0, 0, alignment=QtCore.Qt.AlignCenter)
			grid_layout.addWidget(header_segment_info, 0, 1, alignment=QtCore.Qt.AlignCenter)

			# Populate the grid with data
			row = 1
			segments = list(self.data.keys())
			for i in range(len(segments)):
				# Checkbox for the segment pair
				segment_checkbox = QtWidgets.QCheckBox()
				segment_checkbox.setChecked(False)
				grid_layout.addWidget(segment_checkbox, row, 0, alignment=QtCore.Qt.AlignCenter)

				# Construct segment info
				segment = segments[i]

				hold_potential = self.data[segment]['hold_potential']
				segment_label = QtWidgets.QLabel(
					f"Segment {segment}: {hold_potential} V")
				grid_layout.addWidget(segment_label, row, 1)

				# Store the checkbox and initialisation checkbox references by file path
				self.file_checkboxes[segment] = segment_checkbox

				row += 1

			# Add the grid layout to the main dropdown layout
			self.plotter_params_dropdown_layout.addLayout(grid_layout)

			self.plotter_params_dropdown_layout.addWidget(create_line())

			options1_hlayout = QtWidgets.QHBoxLayout()
			options2_hlayout = QtWidgets.QHBoxLayout()

			self.options_charging_checkbox = QtWidgets.QCheckBox("Plot charging?")
			self.options_charging_checkbox.setChecked(False)
			self.options_current_checkbox = QtWidgets.QCheckBox("Plot current?")
			self.options_current_checkbox.setChecked(False)
			options1_hlayout.addWidget(QtWidgets.QLabel("<u>Other options:</u>"))
			options1_hlayout.addWidget(self.options_charging_checkbox)
			options1_hlayout.addWidget(self.options_current_checkbox)

			options_units_label = QtWidgets.QLabel("Units:")
			options_y1units_label = QtWidgets.QLabel("Voltage")
			self.options_y1units_dropdown = QtWidgets.QComboBox()
			self.options_y1units_dropdown.addItems(["V", "mV", "µV"])
			options_y2units_label = QtWidgets.QLabel("; Current")
			self.options_y2units_dropdown = QtWidgets.QComboBox()
			self.options_y2units_dropdown.addItems(["A", "mA", "µA"])
			self.options_legend_checkbox = QtWidgets.QCheckBox("Legend?")
			self.options_legend_checkbox.setChecked(True)
			options2_hlayout.addWidget(options_units_label)
			options2_hlayout.addWidget(options_y1units_label)
			options2_hlayout.addWidget(self.options_y1units_dropdown)
			options2_hlayout.addWidget(options_y2units_label)
			options2_hlayout.addWidget(self.options_y2units_dropdown)
			options2_hlayout.addWidget(QtWidgets.QLabel("     "))
			options2_hlayout.addWidget(self.options_legend_checkbox)

			self.plotter_params_dropdown_layout.addLayout(options1_hlayout)
			self.plotter_params_dropdown_layout.addLayout(options2_hlayout)

		elif self.experiment_type == "Rate":

			plot_params_label = QtWidgets.QLabel("<b>Rate-testing plotting parameters</b>")
			plot_params_label.setAlignment(QtCore.Qt.AlignCenter)
			self.plotter_params_dropdown_layout.addWidget(plot_params_label)

			self.plotter_params_dropdown_layout.addWidget(create_line())

			self.plotter_params_dropdown_layout.addWidget(QtWidgets.QLabel("<u>Experiment select:</u>"))

			# Create a grid layout for consistent alignment
			grid_layout = QtWidgets.QGridLayout()
			grid_layout.setHorizontalSpacing(20)
			grid_layout.setVerticalSpacing(5)

			# Add header row to the grid
			header_select = QtWidgets.QLabel("Select")
			header_experiment_info = QtWidgets.QLabel("Experiment info")

			# Center-align the headers
			header_select.setAlignment(QtCore.Qt.AlignCenter)
			header_experiment_info.setAlignment(QtCore.Qt.AlignCenter)

			# Add headers to the grid layout
			grid_layout.addWidget(header_select, 0, 0)
			grid_layout.addWidget(header_experiment_info, 0, 1)

			# Populate the grid with data
			row = 1
			for filepath, file_info in self.data.items():
				# Checkbox for the file
				file_checkbox = QtWidgets.QCheckBox()
				file_checkbox.setChecked(False)
				grid_layout.addWidget(file_checkbox, row, 0, alignment=QtCore.Qt.AlignCenter)

				# Label for experiment info
				file_label = QtWidgets.QLabel(f"{file_info['lbound']} / {file_info['ubound']} V; {', '.join(str(rate) for rate in file_info['c_rates'])} C")
				grid_layout.addWidget(file_label, row, 1)

				# Store the checkbox references by file path
				self.file_checkboxes[filepath] = file_checkbox

				row += 1

			# Add the grid layout to the main dropdown layout
			self.plotter_params_dropdown_layout.addLayout(grid_layout)

			final_cycle_warning_label = QtWidgets.QLabel("<b>Note:</b> Only the charge/discharge capacity<br>from the final cycle will be plotted")
			final_cycle_warning_label.setAlignment(QtCore.Qt.AlignCenter)
			self.plotter_params_dropdown_layout.addWidget(final_cycle_warning_label)

			self.plotter_params_dropdown_layout.addWidget(create_line())
			self.plotter_params_dropdown_layout.addWidget(QtWidgets.QLabel("<u>Other options:</u>"))

			options_hlayout = QtWidgets.QHBoxLayout()
			options_units_label = QtWidgets.QLabel("Units:")
			options_yunits_label = QtWidgets.QLabel("Capacity")
			self.options_yunits_dropdown = QtWidgets.QComboBox()
			self.options_yunits_dropdown.addItems(["Ah", "mAh", "µAh"])
			self.options_legend_checkbox = QtWidgets.QCheckBox("Legend?")
			self.options_legend_checkbox.setChecked(True)
			self.options_logx_checkbox = QtWidgets.QCheckBox("Log scale x-axis?")
			self.options_logx_checkbox.setChecked(False)
			options_hlayout.addWidget(options_units_label)
			options_hlayout.addWidget(options_yunits_label)
			options_hlayout.addWidget(self.options_yunits_dropdown)
			options_hlayout.addWidget(QtWidgets.QLabel("     "))
			options_hlayout.addWidget(self.options_logx_checkbox)
			options_hlayout.addWidget(self.options_legend_checkbox)

			self.plotter_params_dropdown_layout.addLayout(options_hlayout)

		self.plotter_params_dropdown_layout.addWidget(create_line())
		self.plot_button = QtWidgets.QPushButton("PLOT")
		self.plot_button.clicked.connect(self.handlePlotButtonClick)
		self.plotter_params_dropdown_layout.addWidget(self.plot_button)

	def handlePlotButtonClick(self):
		self.extractData()
		self.plotData()

	def removeDropdown(self):
		self.dropdown_frame.setVisible(False)
		self.load_button.setText("Load data")
		self.clearData()

	def extractData(self):
		"""Extract data according to plotting parameters."""
		self.plot_data.clear()  # Clear any previous data

		if self.experiment_type == "CV":
			for filepath, file_checkbox in self.file_checkboxes.items():
				if file_checkbox.isChecked():  # Only process selected files

					dropdown = self.cycle_dropdowns[filepath]
					selected_cycles = dropdown.getSelectedItems()

					if selected_cycles != []:

						self.plot_data[filepath] = {}

						# Handle "All cycles" selection
						if "All cycles" in selected_cycles:
							selected_cycles = list(self.data[filepath]['data'].keys())

						# Store the experiment info
						self.plot_data[filepath]['lbound'] = self.data[filepath]['lbound']
						self.plot_data[filepath]['ubound'] = self.data[filepath]['ubound']
						self.plot_data[filepath]['scanrate_mV'] = self.data[filepath]['scanrate_mV']

						# Store the selected data
						self.plot_data[filepath]['data'] = {
							cycle: self.data[filepath]['data'][int(cycle)]
							for cycle in selected_cycles
						}

		elif self.experiment_type == "LSV":
			for filepath, file_checkbox in self.file_checkboxes.items():
				if file_checkbox.isChecked():  # Only process selected files

					self.plot_data[filepath] = {}

					# Determine whether initialisation data should be plotted
					if self.init_checkboxes[filepath].isChecked():
						plot_init = True
					else:
						plot_init = False

					self.plot_data[filepath]['lbound'] = self.data[filepath]['lbound']
					self.plot_data[filepath]['ubound'] = self.data[filepath]['ubound']
					self.plot_data[filepath]['scanrate_mV'] = self.data[filepath]['scanrate_mV']

					# Store the selected data
					data = []
					if plot_init:  # If initialisation segments to be plotted
						# Append "Initialising" and "Holding" segments into the dataset
						if "Initialising" in self.data[filepath]['data']:
							data.append(self.data[filepath]['data']['Initialising'])
						if "Holding" in self.data[filepath]['data']:
							data.append(self.data[filepath]['data']['Holding'])

					if "Sweeping" in self.data[filepath]['data']:
						data.append(self.data[filepath]['data']['Sweeping'])

					self.plot_data[filepath]['data'] = data

		elif self.experiment_type == "GCD":
			for filepath, file_checkbox in self.file_checkboxes.items():
				if file_checkbox.isChecked():  # Only process selected files

					dropdown = self.cycle_dropdowns[filepath]
					selected_cycles = dropdown.getSelectedItems()

					if selected_cycles != []:

						self.plot_data[filepath] = {}

						# Handle "All cycles" selection
						if "All cycles" in selected_cycles:
							selected_cycles = list(self.data[filepath]['data'].keys())

						# Store the experiment info
						self.plot_data[filepath]['lbound'] = self.data[filepath]['lbound']
						self.plot_data[filepath]['ubound'] = self.data[filepath]['ubound']
						self.plot_data[filepath]['charge_current'] = self.data[filepath]['charge_current']
						self.plot_data[filepath]['discharge_current'] = self.data[filepath]['discharge_current']

						# Store the selected data
						self.plot_data[filepath]['data'] = {
							cycle: {
								halfcycle: self.data[filepath]['data'][int(cycle)][int(halfcycle)]
								for halfcycle in self.data[filepath]['data'][int(cycle)]
							}
							for cycle in selected_cycles
						}

		elif self.experiment_type == "CA":
			for (segment1, segment2), pair_checkbox in self.file_checkboxes.items():
				if pair_checkbox.isChecked():  # Only process selected segment pairs

					self.plot_data[(segment1, segment2)] = {}

					self.plot_data[(segment1, segment2)]['hold_potential'] = (self.data[segment1]['hold_potential'], self.data[segment2]['hold_potential'])

					# Store the selected data
					data = []
					if "Ramping" in self.data[segment1]['data']:
						data.append(self.data[segment1]['data']['Ramping'])
					if "Holding" in self.data[segment1]['data']:
						data.append(self.data[segment1]['data']['Holding'])
					if "Ramping" in self.data[segment2]['data']:
						data.append(self.data[segment2]['data']['Ramping'])
					if "Holding" in self.data[segment2]['data']:
						data.append(self.data[segment2]['data']['Holding'])

					self.plot_data[(segment1, segment2)]['data'] = data

		elif self.experiment_type == "CP":
			for (segment1, segment2), pair_checkbox in self.file_checkboxes.items():
				if pair_checkbox.isChecked():  # Only process selected segment pairs

					self.plot_data[(segment1, segment2)] = {}

					self.plot_data[(segment1, segment2)]['hold_current'] = (self.data[segment1]['hold_current'], self.data[segment2]['hold_current'])

					# Store the selected data
					data = []
					if "Ramping" in self.data[segment1]['data']:
						data.append(self.data[segment1]['data']['Ramping'])
					if "Holding" in self.data[segment1]['data']:
						data.append(self.data[segment1]['data']['Holding'])
					if "Ramping" in self.data[segment2]['data']:
						data.append(self.data[segment2]['data']['Ramping'])
					if "Holding" in self.data[segment2]['data']:
						data.append(self.data[segment2]['data']['Holding'])

					self.plot_data[(segment1, segment2)]['data'] = data

		elif self.experiment_type == "SD":
			for segment, segment_checkbox in self.file_checkboxes.items():
				if segment_checkbox.isChecked():  # Only process selected segments

					self.plot_data[segment] = {}

					self.plot_data[segment]['hold_potential'] = self.data[segment]['hold_potential']

					# Store the selected data
					data = {}
					if self.options_charging_checkbox.isChecked() and "Charging" in self.data[segment]['data']:
							data['Charging'] = [self.data[segment]['data']['Charging']]
					if "Self-discharging" in self.data[segment]['data']:
						data['Self-discharging'] = [self.data[segment]['data']['Self-discharging']]

					self.plot_data[segment]['data'] = data

		elif self.experiment_type == "Rate":
			for filepath, file_checkbox in self.file_checkboxes.items():
				if file_checkbox.isChecked():  # Only process selected files

					self.plot_data[filepath] = {}

					# Store the experiment data
					self.plot_data[filepath]['lbound'] = self.data[filepath]['lbound']
					self.plot_data[filepath]['ubound'] = self.data[filepath]['ubound']
					self.plot_data[filepath]['c_rates'] = self.data[filepath]['c_rates']
					self.plot_data[filepath]['charge_capacities'] = []
					self.plot_data[filepath]['discharge_capacities'] = []

					# Extract final cycle capacities for each C-rate
					for c_rate in self.data[filepath]['c_rates']:
						c_rate_data = self.data[filepath]['data'].get(c_rate, {})
						if c_rate_data:
							final_cycle = max(c_rate_data.keys())
							self.plot_data[filepath]['charge_capacities'].append(c_rate_data[final_cycle]['charge_capacity'])
							self.plot_data[filepath]['discharge_capacities'].append(c_rate_data[final_cycle]['discharge_capacity'])


	def plotData(self):
		"""Plot the extracted data according to plotting parameters."""
		if not self.plot_data:
			QtWidgets.QMessageBox.warning(
				self,
				"No data selected",
				"No data selected for plotting."
			)
			return

		# Create the pop-up window
		popup = QtWidgets.QDialog(self)
		popup.setWindowTitle("Plot data")
		popup.setModal(True)
		popup.setMinimumSize(800, 600)

		# Layout for the pop-up
		layout = QtWidgets.QVBoxLayout(popup)

		# Initialise the plot
		fig = Figure(figsize=(6, 4))
		ax = fig.add_subplot(111)

		if self.experiment_type == "CV":

			# Store axis units and legend boolean
			xunits = self.options_xunits_dropdown.currentText()
			yunits = self.options_yunits_dropdown.currentText()
			legend_bool = self.options_legend_checkbox.isChecked()

			# Store unit multipliers
			xunit_multiplier = {"V": 1, "mV": 1e3, "µV": 1e6}[xunits]
			yunit_multiplier = {"A": 1, "mA": 1e3, "µA": 1e6}[yunits]

			# Axis labels
			ax.set_xlabel(f"Potential ({xunits})")
			ax.set_ylabel(f"Current ({yunits})")

			# Iterate over the plot_data and plot each file's data
			i = 0
			for filepath, file_data in self.plot_data.items():
				lbound = file_data['lbound']
				ubound = file_data['ubound']
				scanrate_mV = file_data['scanrate_mV']

				# Plot data for each cycle
				for cycle_number, cycle_data in file_data['data'].items():
					color = CB_color_cycle[i % len(CB_color_cycle)]
					potentials = [entry['potential'] * xunit_multiplier for entry in cycle_data]
					currents = [entry['current'] * yunit_multiplier for entry in cycle_data]
					ax.plot(potentials, currents, label=f"{lbound}/{ubound} V; {scanrate_mV} mV/s; Cycle no. {cycle_number}", color=color)

					i += 1

		elif self.experiment_type == "LSV":

			# Store axis units and legend boolean
			xunits = self.options_xunits_dropdown.currentText()
			yunits = self.options_yunits_dropdown.currentText()
			legend_bool = self.options_legend_checkbox.isChecked()

			# Store unit multipliers
			xunit_multiplier = {"V": 1, "mV": 1e3, "µV": 1e6}[xunits]
			yunit_multiplier = {"A": 1, "mA": 1e3, "µA": 1e6}[yunits]

			# Axis labels
			ax.set_xlabel(f"Potential ({xunits})")
			ax.set_ylabel(f"Current ({yunits})")

			# Iterate over the plot_data and plot each file's data
			i = 0
			for filepath, file_data in self.plot_data.items():
				color = CB_color_cycle[i % len(CB_color_cycle)]
				lbound = file_data['lbound']
				ubound = file_data['ubound']
				scanrate_mV = file_data['scanrate_mV']

				# Plot data for each cycle
				potentials = [entry['potential'] * xunit_multiplier for sublist in file_data['data'] for entry in sublist]
				currents = [entry['current'] * yunit_multiplier for sublist in file_data['data'] for entry in sublist]
				ax.plot(potentials, currents,
						label=f"{lbound}/{ubound} V; {scanrate_mV} mV/s", color=color)

				i += 1

		elif self.experiment_type == "GCD":

			# Store axis units and legend boolean
			yunits = self.options_yunits_dropdown.currentText()
			legend_bool = self.options_legend_checkbox.isChecked()

			# Store unit multipliers
			yunit_multiplier = {"V": 1, "mV": 1e3, "µV": 1e6}[yunits]

			# Axis labels
			ax.set_xlabel(f"Time (s)")
			ax.set_ylabel(f"Potential ({yunits})")

			# Iterate over the plot_data and plot each file's data
			i = 0
			for filepath, file_data in self.plot_data.items():
				lbound = file_data['lbound']
				ubound = file_data['ubound']
				charge_current = file_data['charge_current']
				discharge_current = file_data['discharge_current']

				# Plot data for each cycle
				for cycle_number, cycle_data in file_data['data'].items():
					color = CB_color_cycle[i % len(CB_color_cycle)]
					# Flatten halfcycle data into one list
					times = []
					potentials = []
					# Iterate over each halfcycle's data and collect it
					for halfcycle_data in cycle_data.values():
						for entry in halfcycle_data:
							times.append(entry['elapsed_time'])
							potentials.append(entry['potential'] * yunit_multiplier)

					times = numpy.array(times)
					times = times - times[0]

					# Plot data for the entire cycle
					ax.plot(times, potentials,
							label=f"{lbound}/{ubound} V; {charge_current}/{discharge_current} µA; Cycle no. {cycle_number}", color=color)

					i += 1

		elif self.experiment_type == "CA":

			# Store axis units and legend boolean
			y1units = self.options_y1units_dropdown.currentText()
			legend_bool = self.options_legend_checkbox.isChecked()

			# Store unit multipliers
			y1unit_multiplier = {"A": 1, "mA": 1e3, "µA": 1e6}[y1units]

			# Axis labels
			ax.set_xlabel(f"Time (s)")
			ax.set_ylabel(f"Current ({y1units})")

			# Do same for potential if checkbox is checked
			if self.options_potential_checkbox.isChecked():
				y2units = self.options_y2units_dropdown.currentText()
				y2unit_multiplier = {"V": 1, "mV": 1e3, "µV": 1e6}[y2units]

				# Create a secondary axis for the potential
				ax2 = ax.twinx()
				ax2.set_ylabel(f"Potential ({y2units})")

			# Iterate over the plot_data and plot each segment pair
			i = 0
			for (segment1, segment2), segment_pair_data in self.plot_data.items():
				hold_potential1, hold_potential2 = segment_pair_data['hold_potential']
				color = CB_color_cycle[i % len(CB_color_cycle)]

				# Plot data for each segment pair
				times = numpy.array([entry['elapsed_time'] for sublist in segment_pair_data['data'] for entry in sublist])
				times = times - times[0]
				time_adjust = self.options_adjust_xvals_entry.text().strip()

				try:
					if time_adjust:
						time_adjust = float(time_adjust)
						adjusted_indices = times >= time_adjust
					else:
						adjusted_indices = numpy.full(times.shape, True)
				except ValueError:
					QtWidgets.QMessageBox.critical(mainwidget, "Invalid time adjust","Time adjust must be a valid number.")
					return False

				times = times[adjusted_indices]
				times = times - times[0]
				currents = numpy.array([entry['current'] * y1unit_multiplier for sublist in segment_pair_data['data'] for entry in sublist])
				currents = currents[adjusted_indices]

				ax.plot(times, currents, label=f"Current at hold potentials {hold_potential1} - {hold_potential2} V", color=color)

				if self.options_potential_checkbox.isChecked():
					potentials = numpy.array([entry['potential'] * y2unit_multiplier for sublist in segment_pair_data['data'] for entry in sublist])
					potentials = potentials[adjusted_indices]
					ax2.plot(times, potentials, alpha=0.5, linestyle='dashed', color=color)

				i += 1

		elif self.experiment_type == "CP":

			# Store axis units and legend boolean
			y1units = self.options_y1units_dropdown.currentText()
			legend_bool = self.options_legend_checkbox.isChecked()

			# Store unit multipliers
			y1unit_multiplier = {"V": 1, "mV": 1e3, "µV": 1e6}[y1units]

			# Axis labels
			ax.set_xlabel(f"Time (s)")
			ax.set_ylabel(f"Potential ({y1units})")

			# Do same for current if checkbox is checked
			if self.options_current_checkbox.isChecked():
				y2units = self.options_y2units_dropdown.currentText()
				y2unit_multiplier = {"A": 1, "mA": 1e3, "µA": 1e6}[y2units]

				# Create a secondary axis for the current
				ax2 = ax.twinx()
				ax2.set_ylabel(f"Potential ({y2units})")

			# Iterate over the plot_data and plot each segment pair
			i = 0
			for (segment1, segment2), segment_pair_data in self.plot_data.items():
				hold_current1, hold_current2 = segment_pair_data['hold_current']
				color = CB_color_cycle[i % len(CB_color_cycle)]

				# Plot data for each segment pair
				times = numpy.array([entry['elapsed_time'] for sublist in segment_pair_data['data'] for entry in sublist])
				times = times - times[0]
				time_adjust = self.options_adjust_xvals_entry.text().strip()

				try:
					if time_adjust:
						time_adjust = float(time_adjust)
						adjusted_indices = times >= time_adjust
					else:
						adjusted_indices = numpy.full(times.shape, True)
				except ValueError:
					QtWidgets.QMessageBox.critical(mainwidget, "Invalid time adjust","Time adjust must be a valid number.")
					return False

				times = times[adjusted_indices]
				times = times - times[0]
				potentials = numpy.array([entry['potential'] * y1unit_multiplier for sublist in segment_pair_data['data'] for entry in sublist])
				potentials = potentials[adjusted_indices]

				ax.plot(times, potentials, label=f"Potential at hold currents {hold_current1} - {hold_current2} µA", color=color)

				if self.options_current_checkbox.isChecked():
					currents = numpy.array([entry['current'] * y2unit_multiplier for sublist in segment_pair_data['data'] for entry in sublist])
					currents = currents[adjusted_indices]
					ax2.plot(times, currents, alpha=0.5, linestyle='dashed', color=color)

				i += 1

		elif self.experiment_type == "SD":

			# Store axis units and legend boolean
			y1units = self.options_y1units_dropdown.currentText()
			legend_bool = self.options_legend_checkbox.isChecked()

			# Store unit multipliers
			y1unit_multiplier = {"V": 1, "mV": 1e3, "µV": 1e6}[y1units]

			# Axis labels
			ax.set_xlabel(f"Time (s)")
			ax.set_ylabel(f"Potential ({y1units})")

			# Do same for current if checkbox is checked
			if self.options_current_checkbox.isChecked():
				y2units = self.options_y2units_dropdown.currentText()
				y2unit_multiplier = {"A": 1, "mA": 1e3, "µA": 1e6}[y2units]

				# Create a secondary axis for the potential
				ax2 = ax.twinx()
				ax2.set_ylabel(f"Current ({y2units})")

			# Iterate over the plot_data and plot each segment pair
			i = 0
			for segment, segment_data in self.plot_data.items():
				hold_potential = segment_data['hold_potential']
				color = CB_color_cycle[i % len(CB_color_cycle)]

				# Plot data for each segment
				if self.options_charging_checkbox.isChecked() and "Charging" in segment_data['data']:
						charge_times = numpy.array([entry['elapsed_time'] for sublist in segment_data['data']['Charging'] for entry in sublist])
						timeshift = charge_times[0]
						charge_times = charge_times - timeshift
						charge_potentials = [entry['potential'] * y1unit_multiplier for sublist in segment_data['data']['Charging'] for entry in sublist]
						ax.plot(charge_times, charge_potentials, linestyle='dashed', alpha=0.5, color=color)
						if self.options_current_checkbox.isChecked():
							charge_currents = [entry['current'] * y2unit_multiplier for sublist in segment_data['data']['Charging'] for entry in sublist]
							ax2.plot(charge_times, charge_currents, linestyle='dotted', color=color)

				if "Self-discharging" in segment_data['data']:
					discharge_times = numpy.array([entry['elapsed_time'] for sublist in segment_data['data']['Self-discharging'] for entry in sublist])
					if self.options_charging_checkbox.isChecked() and "Charging" in segment_data['data']:
						discharge_times = discharge_times - timeshift
					else:
						discharge_times = discharge_times - discharge_times[0]

					discharge_potentials = [entry['potential'] * y1unit_multiplier for sublist in
										 segment_data['data']['Self-discharging'] for entry in sublist]
					ax.plot(discharge_times, discharge_potentials, label=f"Charge potential: {hold_potential} V", color=color)
					if self.options_current_checkbox.isChecked():
						discharge_currents = [entry['current'] * y2unit_multiplier for sublist in
										   segment_data['data']['Self-discharging'] for entry in sublist]
						ax2.plot(discharge_times, discharge_currents, linestyle='dotted', color=color)

				i += 1

		elif self.experiment_type == "Rate":

			# Store axis units and legend boolean
			yunits = self.options_yunits_dropdown.currentText()
			legend_bool = self.options_legend_checkbox.isChecked()

			# Store unit multipliers
			yunit_multiplier = {"Ah": 1, "mAh": 1e3, "µAh": 1e6}[yunits]

			# Axis labels
			ax.set_ylabel(f"Inserted/extracted charge ({yunits})")

			all_plotted_c_rates = set()

			log_scale_bool = self.options_logx_checkbox.isChecked()

			# Log scale for x-axis if checkbox is checked
			if log_scale_bool:
				ax.set_xscale('log')
				ax.set_xlabel(f"log\u2081\u2080 of C-rate")
			else:
				ax.set_xlabel("C-rate")
				ax.xaxis.set_major_locator(MaxNLocator(integer=True))

			# Iterate over the plot_data and plot each file's data
			i = 0
			for filepath, file_data in self.plot_data.items():
				lbound = file_data['lbound']
				ubound = file_data['ubound']
				c_rates = file_data['c_rates']
				charge_capacities = [charge_capacity * yunit_multiplier for charge_capacity in file_data['charge_capacities']]
				discharge_capacities = [discharge_capacity * yunit_multiplier for discharge_capacity in file_data['discharge_capacities']]

				color = CB_color_cycle[i % len(CB_color_cycle)]

				# Track used C-rates for x-ticks
				for rate, charge_capacity, discharge_capacity in zip(c_rates, charge_capacities, discharge_capacities):
					if charge_capacity is not None or discharge_capacity is not None:
						all_plotted_c_rates.add(rate)

				# Plot charge with filled upwards triangle
				ax.scatter(c_rates, charge_capacities, label=f"Charge: {lbound}/{ubound} V", color=color, marker='^', s=36)

				# Plot discharge with unfilled downwards triangle
				ax.scatter(c_rates, discharge_capacities, label=f"Discharge: {lbound}/{ubound} V", color=color, marker='v', s=36, facecolors='none', edgecolors=color)

				i += 1

			# Set x-ticks
			if not log_scale_bool:
				xticks = sorted(all_plotted_c_rates)
				ax.set_xticks(xticks)
				ax.set_xticklabels([f"{x:g}" for x in xticks])

		if legend_bool:
			ax.legend()

		# Embed the matplotlib figure into the pop-up
		canvas = FigureCanvas(fig)
		layout.addWidget(canvas)

		# Toolbar for zoom/pan functionality
		save_dir = os.path.dirname(self.summary_filepath) if self.summary_filepath else "~"
		matplotlib.rcParams['savefig.directory'] = save_dir
		toolbar = NavigationToolbar(canvas, popup)
		layout.addWidget(toolbar)

		# Show the pop-up
		popup.exec_()

	def toggleDropdown(self):
		if self.dropdown_frame.isHidden():
			self.dropdown_frame.setVisible(True)
			self.load_button.setText("Remove loaded data")
			self.adjustSize()  # Adjust the size of the widget dynamically
			self.parent().adjustSize()  # Adjust the size of the parent layout

		else:
			self.dropdown_frame.setVisible(False)
			self.load_button.setText("Load data")
			self.adjustSize()  # Adjust the size of the widget dynamically
			self.parent().adjustSize()  # Adjust the size of the parent layout
			self.clearData()

	def clearData(self):
		self.summary_filepath = None
		self.experiment_type = None
		self.data = {}
		self.plot_data = {}
		self.experiments_complete_bool = None
		self.all_experiments_loaded_bool = None
		self.unloaded_filepaths = []

		self.file_checkboxes = {}
		self.init_checkboxes = {}
		self.cycle_dropdowns = {}


	def clearWidgets(self):
		"""Clear all widgets and layouts from the dropdown layout."""
		def clearLayout(layout):
			"""Recursively clear all widgets and sub-layouts from a layout."""
			while layout.count():
				item = layout.takeAt(0)
				if item.widget():
					item.widget().deleteLater()
				elif item.layout():
					clearLayout(item.layout())
					item.layout().deleteLater()

		# Clear the main dropdown layout
		clearLayout(self.plotter_params_dropdown_layout)


plotter_params_dropdown = Plotter_DropdownArea()
plotter_params_layout.addWidget(plotter_params_dropdown)

plotter_file_entry.textChanged.connect(lambda: plotter_params_dropdown.removeDropdown())

plotter_vbox.addWidget(plotter_params_box)

plotter_vbox.setSpacing(5)
plotter_vbox.setContentsMargins(3, 3, 3, 3)

# Make scrollable area
plotter_widget = QtWidgets.QWidget()
plotter_widget.setLayout(plotter_vbox)
plotter_widget.setContentsMargins(0, 0, 0, 0)

plotter_scroll_area = QtWidgets.QScrollArea()
plotter_scroll_area.setWidgetResizable(True)
plotter_scroll_area.setWidget(plotter_widget)
plotter_scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
plotter_scroll_area.setContentsMargins(0, 0, 0, 0)

plotter_layout = QtWidgets.QVBoxLayout()
plotter_layout.setContentsMargins(0, 0, 0, 0)
plotter_layout.addWidget(plotter_scroll_area)
tabs[8].setLayout(plotter_layout)

hbox = QtWidgets.QHBoxLayout()
hbox.addLayout(display_plot_frame)
hbox.addWidget(tab_frame)

vbox = QtWidgets.QVBoxLayout()
statustext = QtWidgets.QPlainTextEdit()
statustext.setFixedHeight(90)
apply_tab_frame_width()
vbox.addLayout(hbox)
vbox.addWidget(statustext)

mainwidget = QtWidgets.QWidget()
win.setCentralWidget(mainwidget)
vbox.setContentsMargins(0, 0, 0, 0)
mainwidget.setLayout(vbox)


# ????? Updated tab frame code

tab_frame_min_width_pixels = tab_frame.sizeHint().width()
print("min width pixels:", tab_frame_min_width_pixels)
font_metrics = QtGui.QFontMetrics(statustext.font())
char_width = font_metrics.horizontalAdvance(' ')
tab_frame_min_width_chars = int(round((tab_frame_min_width_pixels / char_width) * 0.8))
print("min width spaces:", tab_frame_min_width_chars)
global_software_settings['tab_frame_width'] = tab_frame_min_width_chars
apply_tab_frame_width()

# ?????


def periodic_update():  # A state machine is used to determine which functions need to be called, depending on the current state of the program
	if state == States.Idle_Init:
		idle_init()
	elif state == States.Idle:
		read_potential_current()
		update_live_graph()

	elif state == States.Stationary_Graph:
		read_potential_current()


	# Experiment updates

	elif state == States.Measuring_CV:
		cv_update(cv_current_exp_index)

	elif state == States.Measuring_LSV:
		lsv_update(lsv_current_exp_index)

	elif state == States.Measuring_GCD:
		gcd_update(gcd_current_exp_index)

	elif state == States.Measuring_CA:
		ca_update(ca_current_segment_index)

	elif state == States.Measuring_CP:
		cp_update(cp_current_segment_index)

	elif state == States.Measuring_SD:
		sd_update(sd_current_segment_index)

	elif state == States.Measuring_Rate:
		rate_update(rate_current_exp_index, rate_current_c_rate_index)

	elif state == States.Measuring_Rate_One_C_Calc:
		rate_one_c_calc_update(rate_current_exp_index)


	# OCP equilibration

	elif state == States.Measuring_CV_OCP_eq:
		OCP_equilibration_loop(cv_parameters, cv_data, cv_current_exp_index)

	elif state == States.Measuring_LSV_OCP_eq:
		OCP_equilibration_loop(lsv_parameters, lsv_data, lsv_current_exp_index)

	elif state == States.Measuring_GCD_OCP_eq:
		OCP_equilibration_loop(gcd_parameters, gcd_data, gcd_current_exp_index)

	elif state == States.Measuring_CA_OCP_eq:
		OCP_equilibration_loop(ca_parameters, ca_data, ca_current_segment_index)

	elif state == States.Measuring_CP_OCP_eq:
		OCP_equilibration_loop(cp_parameters, cp_data, cp_current_segment_index)

	elif state == States.Measuring_SD_OCP_eq:
		OCP_equilibration_loop(sd_parameters, sd_data, sd_current_segment_index)

	elif state == States.Measuring_Rate_OCP_eq:
		OCP_equilibration_loop(rate_parameters, rate_data, rate_current_exp_index)


"""CV TIMERS"""
cv_delay_timer = QtCore.QTimer()
cv_delay_timer.setSingleShot(True)
cv_delay_timer.timeout.connect(lambda: cv_start(cv_current_exp_index))

cv_experiment_progress_update_interval = 1000  # 1 second in milliseconds
cv_experiment_progress_timer = QtCore.QTimer()
cv_experiment_progress_timer.timeout.connect(cv_experiment_progress_controller)

"""LSV TIMERS"""
lsv_delay_timer = QtCore.QTimer()
lsv_delay_timer.setSingleShot(True)
lsv_delay_timer.timeout.connect(lambda: lsv_start(lsv_current_exp_index))

lsv_experiment_progress_update_interval = 1000  # 1 second in milliseconds
lsv_experiment_progress_timer = QtCore.QTimer()
lsv_experiment_progress_timer.timeout.connect(lsv_experiment_progress_controller)

"""GCD TIMER"""
gcd_delay_timer = QtCore.QTimer()
gcd_delay_timer.setSingleShot(True)
gcd_delay_timer.timeout.connect(lambda: gcd_start(gcd_current_exp_index))

"""CA TIMER"""
ca_delay_timer = QtCore.QTimer()
ca_delay_timer.setSingleShot(True)
ca_delay_timer.timeout.connect(lambda: ca_start(ca_current_segment_index))

"""CP TIMER"""
cp_delay_timer = QtCore.QTimer()
cp_delay_timer.setSingleShot(True)
cp_delay_timer.timeout.connect(lambda: cp_start(cp_current_segment_index))

"SD TIMER"
sd_delay_timer = QtCore.QTimer()
sd_delay_timer.setSingleShot(True)
sd_delay_timer.timeout.connect(lambda: sd_start(sd_current_segment_index))

"""RATE-TESTING TIMER"""
rate_delay_timer = QtCore.QTimer()
rate_delay_timer.setSingleShot(True)
rate_delay_timer.timeout.connect(lambda: rate_start(rate_current_exp_index))

"""CORE PROGRAM TIMER"""
timer = QtCore.QTimer()
timer.timeout.connect(periodic_update)
timer.start(int(qt_timer_period))  # Calls periodic_update() every adcread_interval (as defined in the beginning of this program)

log_message("Program started. Press the \"Connect\" button in the hardware tab to connect to the USB interface.")
log_message("Default software parameters loaded. Press the \"Global parameters\" button in the hardware tab to modify them.")

def main():
	win.show()  # Show the main window
	sys.exit(app.exec_())  # Keep the program running by periodically calling the periodic_update() until the GUI window is closed