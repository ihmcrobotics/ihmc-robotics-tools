package us.ihmc.robotics.kinematics.rotaryDifferential;

import us.ihmc.robotics.kinematics.jointPair.JointPairMechanism;

public class RotaryActuatorDifferentialMechanism extends JointPairMechanism
{
   public RotaryActuatorDifferentialMechanism(RotaryActuatorDifferentialKinematicsSpecifications kinematicsSpecification)
   {
      super(new RotaryActuatorDifferentialJacobianCalculator(new RotaryActuatorDifferentialForwardKinematics(kinematicsSpecification)),
            new RotaryActuatorDifferentialInverseKinematics(kinematicsSpecification));
   }

   public RotaryActuatorDifferentialMechanism(RotaryActuatorDifferentialForwardKinematics forwardKinematics,
                                              RotaryActuatorDifferentialInverseKinematics inverseKinematics)
   {
      super(new RotaryActuatorDifferentialJacobianCalculator(forwardKinematics), inverseKinematics);
   }
}