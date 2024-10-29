package us.ihmc.robotics.kinematics.jointPair.interfaces;

import us.ihmc.robotics.kinematics.jointPair.IKConvergenceCondition;

public interface JointPairInverseKinematics
{
   IKConvergenceCondition computeJointAngles(double rightActuatorPosition, double leftActuatorPosition);

   double getRollJointAngle();

   double getPitchJointAngle();

   int getNumberOfIterations();

   long getComputationTime();

   default double getJacobianDeterminant()
   {
      return Double.NaN;
   }

   IKConvergenceCondition getConvergenceCondition();

   double getResidualSquaredError();

   boolean successfullyWarmStarted();

   void setMinimumTotalIterations(int minIterations);
}
