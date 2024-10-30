# IHMC Robotics Tools
============

Useful tools afor generating and describing robot motions. This includes trajectories, state machines, and control output data structures. It also includes tools for computing actuator data from joint data and vice versa given coupled mechanisms.

[![Automated Tests](https://github.com/ihmcrobotics/ihmc-robotics-tools/actions/workflows/gradle-test.yml/badge.svg?branch=develop)](https://github.com/ihmcrobotics/ihmc-robotics-tool/actions/workflows/gradle-test.yml)

### Download

In your build.gradle:

`compile group: "us.ihmc", name: "ihmc-robotics-tools", version: `
[ ![ihmc-robotics-tools](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-robotics-tools/badge.svg?style=plastic)](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-robotics-tools)

`compile group: "us.ihmc", name: "ihmc-robotics-tools-joint-kinematics", version: `
[ ![ihmc-robotics-tools-joint-kinematics](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-robotics-tools-joint-kinematics/badge.svg?style=plastic)](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-robotics-tools-joint-kinematics)

`compile group: "us.ihmc", name: "ihmc-robotics-tools-visualizers", version: `
[ ![ihmc-robotics-tools-visualizers](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-robotics-tools-visualizers/badge.svg?style=plastic)](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-robotics-tools-visualizers)

### What's Included

##### Main Dependencies

- IHMC's Commons library.
- IHMC's geometry library, Euclid
- IHMC's YoVariable library
- IHMC's graphics description library

##### Joint Kinematics Dependencies

- Main

##### Visualizers Dependencies

- Main
- JGraph for visualization of the state machine
  
##### Main Distribution

- Definition of rigid bodies that can be used for contact by the controller, `ContactableBody`, `ContactablePlaneBody`, and their basic implementation, `ListOfPointsContactablePlaneBody`
- Definitions for center of mass state and center of pressure holders, `CenterOfMassProvider` and `CenterOfPressureDataHolder`
- Tools for recycling Euclid and Matrix lists, `DenseMatrixArrayList`, `FrameTuple2DArrayList`, and `FrameTuple3DArrayList`
- Interfaces for passing data out of controllers that can be used for hardware and software, `JointDesiredOutput` and `JointDesiredOutputList`.
- Vectors of parameters
- Names for joints on robots, including `LegJointName` and `ArmJointName`
- State machines and their corresponding constructors, `StateMachineFactory`
- Tools for constructing trajectories, `Polynomial`, `YoPolynomial`, `Polynomial3D`, and `YoPolynomial3D`, as well as framed versions of the 3D polynomials.

##### Joint Kinematics Distribution

- Data structures for joint pair kinematics, which define coupled joints.
- Tools for computing joint data from actuator data and vice versa, given the robot mechanism.

##### Visualizers Kinematics Distribution

- Visualizer fo rthe construction of a state machine.

### Contributing

This build requires Gradle 5.0+.

### License

[Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0)
