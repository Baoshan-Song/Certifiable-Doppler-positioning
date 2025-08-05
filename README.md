# Certifiable-Doppler-positioning

This repository provides a certifiable LEO satellite Doppler positioning framework based on **SDP (semidefinite programming) relaxation**. The goal is to enable **robust and globally optimal Doppler-based positioning**, even in the presence of nonconvexities.

## Features

- Certifiable LEO Doppler positioning via SDP relaxation
- Synthetic and real-world examples included
- Easily extensible to new Doppler measurement models

## Requirements

This project requires [CVX](https://cvxr.com/cvx/doc/install.html), a package for specifying and solving convex programs in MATLAB. Please follow the official instructions to install CVX before running any SDP-based examples.

## File Structure

- `matlab/simulation/simulate_doppler.m`: Script for generating synthetic Doppler measurement data using known satellite orbits and trajectories.
- `matlab/example_simulation.m`: Example using simulated data to perform SDP-based Doppler positioning.
- `matlab/example_iridium.m`: Example using real Doppler data from the Iridium satellite constellation.

## Citation

If you find this repository useful, please consider citing or referencing it in your work. Questions or suggestions are welcome via issue or pull request.

## License
[Specify your license here, e.g., MIT, if any]
The Certifiable-Doppler-positioning software package is distributed under GPL v3 license. Users are freedom to modify and distribute the software as they see fit, provided that they adhere to the terms and conditions set forth in the license. This includes the ability to incorporate or use Certifiable-Doppler-positioning with other software, whether for non-commercial or commercial purposes. However, any modifications or derivative works must also be distributed under the GPL v3 license, ensuring that the software remains free and accessible to all users.
