# Potentiostat control software

This GUI-based software is designed to work with a low-cost USB Potentiostat/Galvanostat presented by [Dobbelaere et al. (2017)](https://doi.org/10.1016/j.ohx.2017.08.001), providing users with an expanded suite of electrochemical experiments and features.


## Features

- Supports cyclic voltammetry, linear sweep voltammetry, Galvanostatic charge/discharge, chronoamperometry, chronopotentiometry, self-discharge, and rate-testing experiments.
- Experiment queuing via comma-separated variable lists as input parameters.
- Support for complex workflows (e.g., automated open-circuit potential equilibration using `"OCP"` as an input parameter, and automated 1C calculation using GCD analysis for rate-testing experiments).
- User-friendly interface with input validation and tooltips.
- Human-readable summary file with metadata for all queued experiments.
- Designed to run on a range of light-weight hardware configurations via memory management and GUI widget resizing.


## Installation

It is recommended to use a virtual environment to install this program, e.g. conda.

You can install directly from GitHub using pip:

```bash
pip install git+https://github.com/shatners-bassoon/JOSS_test.git
```

Alternatively, you can clone the repository and install using:
```bash
git clone https://github.com/shatners-bassoon/JOSS_test
cd JOSS_test
pip install .
```

For users wishing to use the test script found under `tests/`, instead run:
```bash
git clone https://github.com/shatners-bassoon/JOSS_test
cd JOSS_test
pip install ".[tests]"
```

### Windows and MacOS users using a Conda environment: Install USB backend

`PyUSB` requires the `libusb` backend to communicate with USB devices when using Python installed via Conda on Windows or MacOS. Install it with:

```bash
conda install -c conda-forge libusb
```

### Optional for Linux users: Set up USB permissions (recommended)

To avoid running the program with `sudo`, create a udev rule to give your user access to the USB device automatically.

For example (replace the IDs with your device's values):
```bash
sudo bash -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"a0a0\", ATTR{idProduct}==\"0002\", MODE=\"0666\"" > /etc/udev/rules.d/99-usb-potentiostat.rules'
sudo udevadm control --reload
sudo udevadm trigger
```
After connecting the USB Potentiostat/Galvanostat, you can check your device's Vendor and Product IDs by running:
```bash
lsusb
```


## Usage

This program requires USB access to communicate with the potentiostat.

### Normal use

Launch the program with:
```bash
potentiostat-controller
```
or equivalently:
```bash
python -m potentiostat_controller
```

### Linux users without udev rule (requires `sudo`)

If your system does not have USB permissions configured, run with elevated privileges:
```bash
sudo -E $(which potentiostat-controller)
```
or equivalently:
```bash
sudo -E $(which python) -m potentiostat_controller
```
> `sudo -E` is required to preserve your Python environment when running with elevated privileges.


## Demonstration script

A demonstration script (`tests/run_test.py`) is provided to help users test the GUI and parameter validation functionality. The script:
1. Prompts the user to select an experiment type.
2. Launches the GUI.
3. Automatically populates the chosen experiment's input fields with parameters from `tests/test_parameters.toml`.
4. Simulates a `CHECK` button click using PySide6's `QtTest` framework to verify that GUI responses and parameter validation work as intended.

After the automated steps, users can follow the on-screen instructions displayed in a GUI information dialog to:
- Connect the USB Potentiostat/Galvanostat device.
- Choose a save filepath.
- Start the selected experiment.

> **Note:** After selecting a filepath, the `CHECK` button in the experiment tab must be pressed again to ensure the filepath is valid.

Example data used by the test script was generated using a dummy cell. A schematic of the dummy cell and instructions for connecting it to the device are available at `examples/dummy_cell_schematic.png`. Example data files, along with screenshots of the GUI after each experiment, can be found in `examples/example_data`.

### Option 1: Run after installing the package

From the root of the repository, run:
```bash
python -m tests.run_test
```
Or, for Linux users running with elevated privileges:
```bash
sudo -E $(which python) -m tests.run_test
```

### Option 2: Run from the cloned repository (no installation required)

For MacOS users or Linux users who have successfully set up udev rules, run the following from the root of the repository:
```bash
PYTHONPATH=src python -m tests.run_test
```
Or, for Linux users running with elevated privileges:
```bash
sudo -E PYTHONPATH=src $(which python) -m tests.run_test
```
For Windows users, run:
```bash
set PYTHONPATH=src
python -m tests.run_test
```


## Documentation

For a detailed description of the software's structure and usage, please refer to the [User and Developer Guide](User_and_Developer_Guide.pdf). This document is intended to assist users and developers who wish to understand, modify, or extend the code.

## License

This project is licensed under the [GNU General Public License v3.0](LICENSE).
