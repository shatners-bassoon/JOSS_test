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

For users wishing to use the test scripts found under `tests/`, instead run:
```bash
git clone https://github.com/shatners-bassoon/JOSS_test
cd JOSS_test
pip install ".[tests]"
```

### Windows and MacOS users using a Conda environment: Install USB backend

`PyUSB` requires the `libusb` backend to communicate with USB devices when using Puthon installed via Conda on Windows or MacOS. Install it with:

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
> You can check your device's Vendor and Product IDs by running:
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


## Running example tests

A demonstration script (`tests/run_test.py`) is provided. It prompts users to select an experiment type to test, launches the GUI, automatically populates the chosen experiment input fields with parameters (using those stored in `tests/test_parameters.toml`), and simulates a "CHECK" button click using PySide6's `QtTest` framework. This verifies that GUI responses and parameter parsing work as intended. The user may then connect a potentiostat, choose a save path, and start the experiments.
> Note: After choosing a filepath, the `CHECK` button within the GUI experiment tab must be pressed again to ensure that a valid filepath was chosen.

Example data from the test scripts has been generated using a dummy cell. A schematic for the dummy cell is located under `examples/dummy_cell_schematic.png`, and example data is stored in `examples/example_data/`.

### Option 1: Run after installing the package

If you have installed the package (editable or normal installation), run the following from the root of the repository:
```bash
python -m tests.run_test
```
Or, for users running with elevated privileges:
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
