# Pendulum on a Cart Project

Welcome to the Pendulum on a Cart project repository! This project involves designing, modeling, and controlling a pendulum attached to a cart. The system is a classic example of a dynamic system that can be used to explore control theory and robotics concepts.

## Repository Structure

The repository is organized into the following folders:

- **Design Files**: Contains the Fusion360 model and other related files for the physical construction of the pendulum on a cart.
- **Matlab**: Contains scripts for modeling the system, validating the model, and designing the controller.
- **CodeComposerStudio**: Contains code for the TivaC TM4C123GH6PM microcontroller used to implement the control algorithms.
- **References**: Contains useful references that provide the theoretical background for the project.

## Contents

### Design Files

This directory contains all the necessary files for the physical design and construction of the pendulum on a cart. 

- **Fusion360 Model**: The 3D model of the pendulum on a cart created using Autodesk Fusion360.
- **Additional Design Files**: Other files related to the physical construction of the device, such as drawings, part lists, and assembly instructions.

### Matlab

This directory contains MATLAB scripts and resources for various stages of the project:

- **A_Model_Pend_Estimation.m**: Estimates the pendulum model based on experimental data gathered from the device via UART.
- **B_Model_Pend_Validation.m**: Compares the pendulum model with a set of experimental observations.
- **C_Model_Full_Validation.m**: Validates the entire model (pendulum plus cart) by comparing it with experimental data.
- **D_LQR.m**: Designs an LQR (Linear-Quadratic Regulator) controller for the system.

The directory also includes the following subfolders:

- **Datasources**: Contains the experimental data obtained from the device via UART.
- **Functions**: Contains miscellaneous MATLAB functions used in the scripts.
- **Results**: Stores the results from the scripts.


### CodeComposerStudio

This directory contains the code for the TivaC TM4C123GH6PM microcontroller used in the project.

Many functions from [this repository](https://github.com/duarterr/CodeComposerStudio-Libs) are used.


### References

This directory contains references that provide theoretical background and additional information about the pendulum on a cart system.

## Contributions

Contributions to the project are welcome! If you have any improvements or new features to add, please follow these steps:

1. Fork the repository.
2. Create a new branch (`git checkout -b feature-new-feature`).
3. Commit your changes (`git commit -am 'Add new feature'`).
4. Push to the branch (`git push origin feature-new-feature`).
5. Create a new Pull Request.

## License

Anyone is free to use and modify the contentes of this repository, but please provide credit to the author, Renan Duarte.
