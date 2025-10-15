"""
Semi-automated GUI test for the potentiostat controller.

Reads test parameters from a TOML file, populates GUI fields automatically, switches to the "CV" tab,
and then lets the user interact manually (connect the device, choose save path, etc.)

Usage:
    python -m tests.test_parameters_populate_fields
"""

import toml
from PyQt5.QtCore import Qt
from PyQt5.QtCore import QTimer
from PyQt5.QtTest import QTest
from PyQt5 import QtWidgets
from potentiostat_controller import potentiostat_controller_v1 as ctrl

TAB_INDEX = {
    "CV": 1,
    "LSV": 2,
    "GCD": 3,
    "Chronoamperometry": 4,
    "Chronopotentiometry": 5,
    "Self-discharge": 6,
    "C-Rate": 7,
}

FIELD_MAP = {
    "CV": {
        "lower_bounds": lambda v: ctrl.cv_params_lbound_entry.setText(str(v)),
        "upper_bounds": lambda v: ctrl.cv_params_ubound_entry.setText(str(v)),
        "start_potentials": lambda v: ctrl.cv_params_startpot_entry.setText(str(v)),
        "stop_potentials": lambda v: ctrl.cv_params_stoppot_entry.setText(str(v)),
        "scan_rates": lambda v: ctrl.cv_params_scanrate_entry.setText(str(v)),
        "reverse_currents_negative": lambda v: ctrl.cv_params_reverse_current_negative_entry.setText(str(v)),
        "reverse_currents_negative_checkbox": lambda v: ctrl.cv_params_reverse_current_negative_checkbox.setChecked(bool(v)),
        "reverse_currents_positive": lambda v: ctrl.cv_params_reverse_current_positive_entry.setText(str(v)),
        "reverse_currents_positive_checkbox": lambda v: ctrl.cv_params_reverse_current_positive_checkbox.setChecked(bool(v)),
        "number_of_cycles": lambda v: ctrl.cv_params_num_cycles_entry.setText(str(v)),
        "samples_to_average": lambda v: ctrl.cv_params_num_samples_entry.setText(str(v)),
        "scan_rate_delay": lambda v: ctrl.cv_params_scanrate_delay_entry.setText(str(v)),
        "potential_window_delay": lambda v: ctrl.cv_params_pot_window_delay_entry.setText(str(v)),
        "OCP_equilibration_checkbox": lambda v: ctrl.cv_params_pot_window_delay_OCP_checkbox.setChecked(bool(v)),
    },
    "LSV": {
        "start_potentials": lambda v: ctrl.lsv_params_startpot_entry.setText(str(v)),
        "stop_potentials": lambda v: ctrl.lsv_params_stoppot_entry.setText(str(v)),
        "scan_rates": lambda v: ctrl.lsv_params_scanrate_entry.setText(str(v)),
        "current_limits_negative": lambda v: ctrl.lsv_params_current_limit_negative_entry.setText(str(v)),
        "current_limits_negative_checkbox": lambda v: ctrl.lsv_params_current_limit_negative_checkbox.setChecked(bool(v)),
        "current_limits_positive": lambda v: ctrl.lsv_params_current_limit_positive_entry.setText(str(v)),
        "current_limits_positive_checkbox": lambda v: ctrl.lsv_params_current_limit_positive_checkbox.setChecked(bool(v)),
        "initialisation_scan_rates": lambda v: ctrl.lsv_params_init_scanrate_entry.setText(str(v)),
        "initialisation_scan_rates_checkbox": lambda v: ctrl.lsv_params_init_scanrate_checkbox.setChecked(bool(v)),
        "initialisation_hold_time": lambda v: ctrl.lsv_params_init_holdtime_entry.setText(str(v)),
        "samples_to_average": lambda v: ctrl.lsv_params_num_samples_entry.setText(str(v)),
        "scan_rate_delay": lambda v: ctrl.lsv_params_scanrate_delay_entry.setText(str(v)),
        "potential_window_delay": lambda v: ctrl.lsv_params_pot_window_delay_entry.setText(str(v)),
        "OCP_equilibration_checkbox": lambda v: ctrl.lsv_params_pot_window_delay_OCP_checkbox.setChecked(bool(v)),
    },
    "GCD": {
        "lower_bounds": lambda v: ctrl.gcd_params_lbound_entry.setText(str(v)),
        "upper_bounds": lambda v: ctrl.gcd_params_ubound_entry.setText(str(v)),
        "charge_currents": lambda v: ctrl.gcd_params_chargecurrent_entry.setText(str(v)),
        "discharge_currents": lambda v: ctrl.gcd_params_dischargecurrent_entry.setText(str(v)),
        "number_of_halfcycles": lambda v: ctrl.gcd_params_num_halfcycles_entry.setText(str(v)),
        "samples_to_average": lambda v: ctrl.gcd_params_num_samples_entry.setText(str(v)),
        "current_delay": lambda v: ctrl.gcd_params_current_delay_entry.setText(str(v)),
        "potential_window_delay": lambda v: ctrl.gcd_params_pot_window_delay_entry.setText(str(v)),
        "OCP_equilibration_checkbox": lambda v: ctrl.gcd_params_pot_window_delay_OCP_checkbox.setChecked(bool(v)),
    },
    "Chronoamperometry": {
        "potentials_sequence_A": lambda v: ctrl.ca_params_A_potential_entry.setText(str(v)),
        "hold_times_sequence_A": lambda v: ctrl.ca_params_A_hold_time_entry.setText(str(v)),
        "ramp_rates_sequence_A": lambda v: ctrl.ca_params_A_ramp_rate_entry.setText(str(v)),
        "ramp_rates_sequence_A_checkbox": lambda v: ctrl.ca_params_A_ramp_rate_checkbox.setChecked(bool(v)),
        "equilibration_sequence_A_checkbox": lambda v: ctrl.ca_params_A_equilibration_checkbox.setChecked(bool(v)),
        "equilibration_sequence_A_tolerance": lambda v: ctrl.ca_params_A_equilibration_tolerance_entry.setText(str(v)),
        "equilibration_sequence_A_timescale": lambda v: ctrl.ca_params_A_equilibration_timescale_entry.setText(str(v)),

        "potentials_sequence_B": lambda v: ctrl.ca_alternating_parameters_dropdown.ca_params_B_potential_entry.setText(str(v)),
        "hold_times_sequence_B": lambda v: ctrl.ca_alternating_parameters_dropdown.ca_params_B_hold_time_entry.setText(str(v)),
        "ramp_rates_sequence_B": lambda v: ctrl.ca_alternating_parameters_dropdown.ca_params_B_ramp_rate_entry.setText(str(v)),
        "ramp_rates_sequence_B_checkbox": lambda v: ctrl.ca_alternating_parameters_dropdown.ca_params_B_ramp_rate_checkbox.setChecked(bool(v)),
        "equilibration_sequence_B_checkbox": lambda v: ctrl.ca_alternating_parameters_dropdown.ca_params_B_equilibration_checkbox.setChecked(bool(v)),
        "equilibration_sequence_B_tolerance": lambda v: ctrl.ca_alternating_parameters_dropdown.ca_params_B_equilibration_tolerance_entry.setText(str(v)),
        "equilibration_sequence_B_timescale": lambda v: ctrl.ca_alternating_parameters_dropdown.ca_params_B_equilibration_timescale_entry.setText(str(v)),

        "current_limits_checkbox": lambda v: ctrl.ca_params_curr_limits_checkbox.setChecked(bool(v)),
        "lower_current_limit": lambda v: ctrl.ca_params_curr_limits_lower_entry.setText(str(v)),
        "upper_current_limit": lambda v: ctrl.ca_params_curr_limits_upper_entry.setText(str(v)),
        "samples_to_average": lambda v: ctrl.ca_params_num_samples_entry.setText(str(v)),
        "delay": lambda v: ctrl.ca_params_delay_entry.setText(str(v)),
        "OCP_equilibration_checkbox": lambda v: ctrl.ca_params_delay_OCP_checkbox.setChecked(bool(v)),
    },
    "Chronopotentiometry": {
        "currents_sequence_A": lambda v: ctrl.cp_params_A_current_entry.setText(str(v)),
        "hold_times_sequence_A": lambda v: ctrl.cp_params_A_hold_time_entry.setText(str(v)),
        "ramp_rates_sequence_A": lambda v: ctrl.cp_params_A_ramp_rate_entry.setText(str(v)),
        "ramp_rates_sequence_A_checkbox": lambda v: ctrl.cp_params_A_ramp_rate_checkbox.setChecked(bool(v)),
        "equilibration_sequence_A_checkbox": lambda v: ctrl.cp_params_A_equilibration_checkbox.setChecked(bool(v)),
        "equilibration_sequence_A_tolerance": lambda v: ctrl.cp_params_A_equilibration_tolerance_entry.setText(str(v)),
        "equilibration_sequence_A_timescale": lambda v: ctrl.cp_params_A_equilibration_timescale_entry.setText(str(v)),

        "currents_sequence_B": lambda v: ctrl.cp_alternating_parameters_dropdown.cp_params_B_current_entry.setText(str(v)),
        "hold_times_sequence_B": lambda v: ctrl.cp_alternating_parameters_dropdown.cp_params_B_hold_time_entry.setText(str(v)),
        "ramp_rates_sequence_B": lambda v: ctrl.cp_alternating_parameters_dropdown.cp_params_B_ramp_rate_entry.setText(str(v)),
        "ramp_rates_sequence_B_checkbox": lambda v: ctrl.cp_alternating_parameters_dropdown.cp_params_B_ramp_rate_checkbox.setChecked(bool(v)),
        "equilibration_sequence_B_checkbox": lambda v: ctrl.cp_alternating_parameters_dropdown.cp_params_B_equilibration_checkbox.setChecked(bool(v)),
        "equilibration_sequence_B_tolerance": lambda v: ctrl.cp_alternating_parameters_dropdown.cp_params_B_equilibration_tolerance_entry.setText(str(v)),
        "equilibration_sequence_B_timescale": lambda v: ctrl.cp_alternating_parameters_dropdown.cp_params_B_equilibration_timescale_entry.setText(str(v)),

        "potential_limits_checkbox": lambda v: ctrl.cp_params_pot_limits_checkbox.setChecked(bool(v)),
        "lower_potential_limit": lambda v: ctrl.cp_params_pot_limits_lower_entry.setText(str(v)),
        "upper_potential_limit": lambda v: ctrl.cp_params_pot_limits_upper_entry.setText(str(v)),
        "samples_to_average": lambda v: ctrl.cp_params_num_samples_entry.setText(str(v)),
        "delay": lambda v: ctrl.cp_params_delay_entry.setText(str(v)),
        "OCP_equilibration_checkbox": lambda v: ctrl.cp_params_delay_OCP_checkbox.setChecked(bool(v)),
    },
    "Self-discharge": {
        "charge_potentials": lambda v: ctrl.sd_params_charge_potential_entry.setText(str(v)),
        "hold_times": lambda v: ctrl.sd_params_hold_time_entry.setText(str(v)),
        "ramp_rates": lambda v: ctrl.sd_params_ramp_rate_entry.setText(str(v)),
        "ramp_rates_checkbox": lambda v: ctrl.sd_params_ramp_rate_checkbox.setChecked(bool(v)),
        "acquisition_times": lambda v: ctrl.sd_params_acquisition_time_entry.setText(str(v)),
        "acquisition_cutoffs": lambda v: ctrl.sd_params_acquisition_cutoff_entry.setText(str(v)),
        "acquisition_cutoffs_checkbox": lambda v: ctrl.sd_params_acquisition_cutoff_checkbox.setChecked(bool(v)),
        "acquisition_equilibration_checkbox": lambda v: ctrl.sd_params_acquisition_equilibration_checkbox.setChecked(bool(v)),
        "acquisition_equilibration_tolerance": lambda v: ctrl.sd_params_acquisition_equilibration_tolerance_entry.setText(str(v)),
        "acquisition_equilibration_timescale": lambda v: ctrl.sd_params_acquisition_equilibration_timescale_entry.setText(str(v)),
        "samples_to_average": lambda v: ctrl.sd_params_num_samples_entry.setText(str(v)),
        "delay": lambda v: ctrl.sd_params_delay_entry.setText(str(v)),
        "OCP_equilibration_checkbox": lambda v: ctrl.sd_params_delay_OCP_checkbox.setChecked(bool(v)),
    },
    "C-Rate": {
        "lower_bounds": lambda v: ctrl.rate_params_lbound_entry.setText(str(v)),
        "upper_bounds": lambda v: ctrl.rate_params_ubound_entry.setText(str(v)),
        "one_c": lambda v: ctrl.rate_params_one_c_entry.setText(str(v)),
        "one_c_calc_checkbox": lambda v: ctrl.rate_params_one_c_calc_checkbox.setChecked(bool(v)),
        "one_c_calc_current": lambda v: ctrl.rate_one_c_calc_dropdown.rate_params_one_c_calc_current_entry.setText(str(v)),
        "one_c_calc_number_of_cycles": lambda v: ctrl.rate_one_c_calc_dropdown.rate_params_one_c_calc_num_cycles_entry.setText(str(v)),
        "one_c_calc_samples_to_average": lambda v: ctrl.rate_one_c_calc_dropdown.rate_params_one_c_calc_num_samples_entry.setText(str(v)),
        "one_c_calc_post_calc_delay": lambda v: ctrl.rate_one_c_calc_dropdown.rate_params_one_c_calc_post_delay_entry.setText(str(v)),
        "one_c_calc_post_calc_OCP_checkbox": lambda v: ctrl.rate_one_c_calc_dropdown.rate_params_one_c_calc_post_delay_OCP_checkbox.setChecked(bool(v)),
        "c_rates": lambda v: ctrl.rate_params_c_rate_entry.setText(str(v)),
        "number_of_cycles": lambda v: ctrl.rate_params_c_rate_num_cycles_entry.setText(str(v)),
        "c_rate_delay": lambda v: ctrl.rate_params_c_rate_delay_entry.setText(str(v)),
        "potential_window_delay": lambda v: ctrl.rate_params_delay_entry.setText(str(v)),
        "OCP_equilibration_checkbox": lambda v: ctrl.rate_params_delay_OCP_checkbox.setChecked(bool(v)),
    }
}

DROPDOWN_MAP = {
    "Chronoamperometry": {
        "attr": "ca_alternating_parameters_dropdown",
        "condition_key": "use_alternating_parameters_bool",
        "checkbox_attr": None,  # Toggled directly
        "dropdown_name": "alternating parameters dropdown",
    },
    "Chronopotentiometry": {
        "attr": "cp_alternating_parameters_dropdown",
        "condition_key": "use_alternating_parameters_bool",
        "checkbox_attr": None,  # Toggled directly
        "dropdown_name": "alternating parameters dropdown",
    },
    "C-Rate": {
        "attr": "rate_one_c_calc_dropdown",
        "condition_key": "one_c_calc_checkbox",
        "checkbox_attr": "rate_params_one_c_calc_checkbox",
        "dropdown_name": "1C calculation parameters dropdown",
    }
}

CHECKBUTTON_MAP = {
    "CV": ctrl.cv_variables_checkbutton,
    "LSV": ctrl.lsv_variables_checkbutton,
    "GCD": ctrl.gcd_variables_checkbutton,
    "Chronoamperometry": ctrl.ca_variables_checkbutton,
    "Chronopotentiometry": ctrl.cp_variables_checkbutton,
    "Self-discharge": ctrl.sd_variables_checkbutton,
    "C-Rate": ctrl.rate_variables_checkbutton,
}

def prompt_experiment():
    """Prompt the user to select which experiment to test."""
    experiment_map = {
        "1": "CV",
        "2": "LSV",
        "3": "GCD",
        "4": "Chronoamperometry",
        "5": "Chronopotentiometry",
        "6": "Self-discharge",
        "7": "C-Rate",
    }
    choice = ""
    while choice not in experiment_map:
        print("\nSelect experiment to test:")
        print("--------------------------")
        print("    1: Cyclic voltammetry")
        print("    2: Linear-sweep voltammetry")
        print("    3: Galvanostatic charge-discharge")
        print("    4: Chronoamperometry")
        print("    5: Chronopotentiometry")
        print("    6: Self-discharge")
        print("    7: C-Rate")
        choice = input("\nEnter choice: ").strip()
    return experiment_map[choice]

def populate_fields_from_toml(config_path="tests/example_parameters.toml", experiment="cv"):
    """Populate GUI widgets using values from the TOML config file."""

    try:
        params = toml.load(config_path)
        exp_params = params.get(experiment, {})

        # Handle dropdown toggles
        dropdown_info = DROPDOWN_MAP.get(experiment)
        if dropdown_info:
            attr = dropdown_info["attr"]
            cond_key = dropdown_info["condition_key"]
            checkbox_attr = dropdown_info["checkbox_attr"]
            dropdown_name = dropdown_info["dropdown_name"]

            condition = exp_params.get(cond_key, False)
            dropdown_widget = getattr(ctrl, attr, None)

            if dropdown_widget:
                # Sync checkbox if one exists
                if checkbox_attr:
                    checkbox = getattr(ctrl, checkbox_attr, None)
                    if checkbox:
                        checkbox.setChecked(bool(condition))

                # Toggle dropdowns if necessary
                if condition and dropdown_widget.dropdown_frame.isHidden():
                    dropdown_widget.toggleDropdown()
                    print(f"\nExpanded {experiment} {dropdown_name} ({cond_key} = True).")
                elif not condition and not dropdown_widget.dropdown_frame.isHidden():
                    dropdown_widget.toggleDropdown()
                    print(f"\nCollapsed {experiment} {dropdown_name} ({cond_key} = False).")

        for key, setter in FIELD_MAP[experiment].items():
            if key in exp_params:
                setter(exp_params[key])
        print(f"\nPopulated {experiment} tab with parameters:")
        for k, v in exp_params.items():
            print(f"    {k} = {v}")

    except FileNotFoundError:
        print(f"Config file not found: {config_path}")
    except Exception as e:
        print(f"Error populating {experiment} tab: {e}")


def switch_to_tab(experiment="cv"):
    """Switch the GUI to the tab for the selected experiment."""
    idx = TAB_INDEX.get(experiment, 0)
    ctrl.tab_frame.setCurrentIndex(idx)
    print(f"\nSwitched to {experiment} tab.")

def click_checkbutton(experiment="cv"):
    """Simulate a click of the 'CHECK' button within the GUI experiment tab."""
    checkbutton = CHECKBUTTON_MAP.get(experiment)
    QTest.mouseClick(checkbutton, Qt.LeftButton)
    print(f"\nSimulated mouse click on 'CHECK' button for {experiment} experiments.")

def run_test():
    """Launch GUI and schedule automated population of fields from TOML."""

    experiment = prompt_experiment()
    config_path = "tests/test_parameters.toml"

    # Flag to track success
    success = {"ok": True}

    def safe_switch_tab():
        try:
            switch_to_tab(experiment)
        except Exception as e:
            print(f"\nError switching tab: {e}")
            success["ok"] = False

    def safe_populate_fields():
        try:
            populate_fields_from_toml(config_path, experiment)
        except Exception as e:
            print(f"\nError populating fields: {e}")
            success["ok"] = False

    def safe_click_check():
        try:
            click_checkbutton(experiment)
        except Exception as e:
            print(f"\nError clicking check button: {e}")
            success["ok"] = False

    def show_instructions_if_success():
        """Show instructions to the user on how to run the example experiments using an information dialog."""
        if success["ok"]:
            print("\nAll automated steps completed successfully. Please see the GUI window for next steps.")
            QtWidgets.QMessageBox.information(
                ctrl.mainwidget,
                "Next steps",
                "The GUI has been successfully populated with parameters for the chosen experiment.\n\n"
                "Next steps:\n"
                "1. Navigate to the 'Hardware' tab and connect the USB potentiostat.\n"
                "2. Return to the experiment tab.\n"
                "3. Enter an appropriate filepath in the 'Output filepath' box.\n"
                "4. Press the 'CHECK' button again to ensure that the filepath is valid.\n"
                "5. Press the 'Start' button to run the experiments."
            )
        else:
            print("\nPrevious steps did not complete successfully. The user is not shown next steps instructions.")

    # Delay population slightly so GUI is initialised first
    QTimer.singleShot(1000, safe_switch_tab)
    QTimer.singleShot(1500, safe_populate_fields)
    QTimer.singleShot(2000, safe_click_check)
    QTimer.singleShot(3000, show_instructions_if_success)

    # Launch the main program
    ctrl.main()


if __name__ == "__main__":
    run_test()
