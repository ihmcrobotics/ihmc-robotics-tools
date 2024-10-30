package us.ihmc.robotics.kinematics.rotaryDifferential;

import us.ihmc.robotics.kinematics.jointPair.AdaptiveStepJacobianBasedInverseKinematics;

/**
 * This class uses the Jacobian at the internal joint position guess and the error in the position at that value to compute the modification to the
 * joint position guess to match the actuator position.
 */
public class RotaryActuatorDifferentialInverseKinematics extends AdaptiveStepJacobianBasedInverseKinematics
{
   public RotaryActuatorDifferentialInverseKinematics(RotaryActuatorDifferentialKinematicsSpecifications kinematicsSpecification)

   {
      super(new RotaryActuatorDifferentialJacobianCalculator(new RotaryActuatorDifferentialForwardKinematics(kinematicsSpecification, false, false)));
   }

   public RotaryActuatorDifferentialInverseKinematics(RotaryActuatorDifferentialJacobianCalculator spineDifferentialJacobianCalculator)
   {
      super(spineDifferentialJacobianCalculator);
   }
}