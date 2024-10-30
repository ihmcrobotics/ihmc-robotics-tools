# IHMC Math
============

Useful general math tools. This includes complex numbers, linear algebra tools, and systems using linear dynamics
systems.

[![Automated Tests](https://github.com/ihmcrobotics/ihmc-math/actions/workflows/gradle-test.yml/badge.svg?branch=develop)](https://github.com/ihmcrobotics/ihmc-math/actions/workflows/gradle-test.yml)

### Download

In your build.gradle:

`compile group: "us.ihmc", name: "ihmc-math", version: `
[ ![ihmc-math](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-math/badge.svg?style=plastic)](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-math)

`compile group: "us.ihmc", name: "ihmc-math-linear-algebra", version: `
[ ![ihmc-math-linear-algebra](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-math-linear-algebra/badge.svg?style=plastic)](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-math-linear-algebra)

`compile group: "us.ihmc", name: "ihmc-math-linear-dynamic-systems", version: `
[ ![ihmc-math-linear-dynamic-systems](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-math-linear-dynamic-systems/badge.svg?style=plastic)](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-math-linear-dynamic-systems)

### What's Included

##### Main Dependencies

- IHMC's matrix library, which provides tools for manipulating matrices
- IHMC's Robotics Tools library, which provides sides and other robot structure information
- EJML Simple, which provides some matrix tools for simple multiplication
- JAMA, which provides similar tools to EJML Simple

##### Linear Algebra Dependencies

- Main

##### Linear Dynamic Sysems Dependencies

- Main
- Linear Algebra
- J free chart for viewing data

##### Main Distribution

- Definition of complex numbers
- Tools for inferencing complex numbers from functions
- Tools for assembling complex numbers into a matrix
- Tools for decomposing matrices into eigen values.
- A garbage free fast fourier transform calculator.

##### Linear Algebra Distribution

- Tools for solving Continuous Algebraic Riccati Equations, which implement `CARESolver`
- Tools for solving Continuous Differential Riccati Equations, which implement `CDRESolver`
- Tools for computing matrix exponentials `MatrixExponentialCalculator`.
- Tools for calculating nullspaces (`NullspaceCalculator`) and damped nullspaces (implementing `DampedNullspaceCalculator`)
- Tools for computing pseudo-inverse `ConfigurableSolvePseudoInverseSVD`
- Tools for computing damped least squares (`DampedLeastSquaresSolver`)

##### Linear Dynamic Systems Distribution

- Definition of a Multi-Input, Multi-Output dynamic system, `LinearDynamicSystem`
- Implementation of SISO and MIMO transfer functions (`TransferFunction` and `TransferFunctionMatrix`)
- Tools for using the matrix exponential.

### Contributing

This build requires Gradle 5.0+.

### License

[Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0)
