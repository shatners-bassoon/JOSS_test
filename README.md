# Potentiostat control software

This GUI-based software is designed to work with a low-cost USB Potentiostat/Galvanostat presented by [Dobbelaere et al. (2017)](https://doi.org/10.1016/j.ohx.2017.08.001), providing users with an expanded suite of experiments and features.

## Features

- Supports cyclic voltammetry, linear sweep voltammetry, Galvanostatic charge/discharge, chronoamperometry, chronopotentiometry, self-discharge, and rate-testing experiments.
- Experiment queuing via comma-separated variable lists as input parameters.
- Support for complex workflows (e.g., automated open-circuit potential equilibration using `"OCP"` as an input parameter, and automated 1C calculation using GCD analysis for rate-testing experiments).
- User-friendly interface with input validation and tooltips.
- Human-readable summary file with metadata for all queued experiments.
- Designed to run on a range of light-weight hardware configurations via memory management and GUI widget resizing.

## Installation

```bash
git clone https://github.com/ajhbell/potentiostatcontroller_repo.git
cd potentiostatcontroller_repo
pip install -r requirements.txt
```

## Usage

To launch the program (requires USB access), run:

```bash
sudo python potentiostat_controller_v1.0.py
```

> The program must be run with `sudo` to interface with the USB-connected potentiostat hardware.

## Documentation

For a detailed description of the software's structure and usage, please refer to the [User and Developer Guide](User_and_Developer_Guide.pdf). This document is intended to assist users and developers who wish to understand, modify, or extend the code.

## License

This project is licensed under the [GNU General Public License v3.0](LICENSE).
