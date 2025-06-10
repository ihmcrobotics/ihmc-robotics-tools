package us.ihmc.robotics.kinematics.jointPair.interfaces;

import us.ihmc.robotics.kinematics.jointPair.DifferentialKinematicsSpecification;
import us.ihmc.robotics.kinematics.rotaryDifferential.RotaryActuatorDifferentialKinematicsSpecifications;

/**
 * The forward kinematics are defined as the kinematics that convert joint configurations into actuators positions. The "Forward" and "Inverse" are defined
 * relative to the controller, which "forward" converts desired joint objectives into actuator objectives and "inverse" computes measured joint data from actuator
 * data.
 */
public interface JointPairForwardKinematics
{
   /**
    * Computes the actuator positions from the specified joint angles.
    * @param rollAngle roll angle in radians.
    * @param pitchAngle pitch angle in radians.
    */
   void computeActuatorPositions(double rollAngle, double pitchAngle);

   /**
    * Returns the left actuator position, in radians if rotary and meters if linear.
    */
   double getLeftActuatorPosition();

   /**
    * Returns the right actuator position, in radians if rotary and meters if linear.
    */
   double getRightActuatorPosition();

   /**
    * @return roll joint angle in radians.
    */
   double getRollAngle();

   /**
    * @return pitch joint angle in radians.
    */
   double getPitchAngle();

   /**
    * @return lower joint limit in radians for the roll joint, assuming the pitch joint is ignored (box constraints).
    */
   double getRollJointLowerLimit();

   /**
    * @return upper joint limit in radians for the roll joint, assuming the pitch joint is ignored (box constraints).
    */
   double getRollJointUpperLimit();

   /**
    * @return lower joint limit in radians for the pitch joint, assuming the roll joint is ignored (box constraints).
    */
   double getPitchJointLowerLimit();

   /**
    * @return upper joint limit in radians for the pitch joint, assuming the roll joint is ignored (box constraints).
    */
   double getPitchJointUpperLimit();
}