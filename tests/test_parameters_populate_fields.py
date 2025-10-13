"""
Semi-automated GUI test for the potentiostat controller.

Reads test parameters from a TOML file, populates GUI fields automatically, switches to the "CV" tab,
and then lets the user interact manually (connect the device, choose save path, etc.)

Usage:
    python -m tests.test_parameters_populate_fields
"""

import toml
from PyQt5.QtCore import QTimer
from potentiostat_controller import potentiostat_controller_v1 as ctrl

def populate_fields_from_toml(config_path="tests/example_parameters.toml"):
    """Populate GUI widgets using values from the TOML config file."""

    try:
        params = toml.load(config_path)
        cv_params = params.get("cv", {})

        # Set CV parameter values
        if "lower_bound" in cv_params:
            ctrl.cv_params_lbound_entry.setText(str(cv_params["lower_bound"]))
        if "upper_bound" in cv_params:
            ctrl.cv_params_ubound_entry.setText(str(cv_params["upper_bound"]))
        if "OCP_equilibration" in cv_params:
            ctrl.cv_params_pot_window_delay_OCP_checkbox.setChecked(bool(cv["OCP_equilibration"]))

        print(f"Populated GUI fields from {config_path}:")
        print(f"    lower_bound = {cv_params.get('lower_bound', 'N/A')}")
        print(f"    upper_bound = {cv_params.get('upper_bound', 'N/A')}")
        print(f"    OCP_equilibration = {cv.get('OCP_equilibration', 'N/A')}")

    except FileNotFoundError:
        print(f"Config file not found: {config_path}")
    except Exception as e:
        print(f"Error reading {config_path}: {e}")

def switch_to_tab():
    """Switch the GUI to show the CV tab."""
    try:
        ctrl.tab_frame.setCurrentIndex(1)
        print("Switched to 'CV' tab.")
    except Exception as e:
        print(f"Could not switch to 'CV' tab: {e}")

def run_test():
    """Launch GUI and schedule automated population of fields from TOML."""
    # Delay population slightly so GUI is initialised first
    QTimer.singleShot(1000, switch_to_tab)
    QTimer.singleShot(1000, populate_fields_from_toml)

    # Launch the main program
    ctrl.main()

if __name__ == "__main__":
    run_test()