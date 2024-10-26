IHMC Robotics Tools
============

Useful tools and utilities that are meant for defining robot data structures and assembling basic robot motions.

[![Automated Tests](https://github.com/ihmcrobotics/ihmc-robotics-tools/actions/workflows/gradle-test.yml/badge.svg?branch=develop)](https://github.com/ihmcrobotics/ihmc-robotics-tools/actions/workflows/gradle-test.yml)

### Download

In your build.gradle:

`compile group: "us.ihmc", name: "ihmc-robotics-tools", version: `
[ ![ihmc-robotics-tools](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-robotics-tools/badge.svg?style=plastic)](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-robotics-tools)

`compile group: "us.ihmc", name: "ihmc-robotics-tools-visualizers", version: `
[ ![ihmc-robotics-tools-visualizers](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-robotics-tools-visualizers/badge.svg?style=plastic)](https://maven-badges.herokuapp.com/maven-central/us.ihmc/ihmc-robotics-tools-visualizers)

### What's Included

##### Main Dependencies

- IHMC's Commons Library
- IHMC's Euclid Frame Library
- IHMC's Yo Variables Library
- IHMC's Yo Variables Filter Library
- IHMC's Graphics Description Library

##### Visualizers Dependencies

- Main
- JGraph for visualization


##### Robotics Dependencies

- Apache Commons Lang 3
- Apache Commons I/O
- IHMC's Log Tools logging library providing setting log level from CLI
- IHMC's Euclid Geometry Library
- Trove4j for garbage free lists of primitives

##### Main Distribution

- RobotSide, RobotQuadrant, and RobotSextant, which extend RobotSegment definitions for conveniently defining sides for robots.
- SegmentDependentLists, which provide conveniences for containing sided values.
- Common enums for defining robot structure, such as ArmJointName and LegJointName.
- Recycling lists for DenseMatrix from EJML and FrameTuple2D and 3D objects.
- Generic holders of data, such as contactable bodies and center of mass state.
- Holders of output data that is useful to pass between controller instances.
- Tools for deadband
- State Machine and associated state machine construction tools
- Execution timers
- General trajectory interfaces.
- Polynomials for 1D and 3D, and ability to create Framed 3D

##### Robotics Distribution

- Visualization panel for state machines.

### Contributing

This build requires Gradle 5.0+.

### License

[Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0)
