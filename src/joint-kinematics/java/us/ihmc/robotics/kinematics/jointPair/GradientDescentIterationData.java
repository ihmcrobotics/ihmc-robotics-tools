package us.ihmc.robotics.kinematics.jointPair;

import us.ihmc.euclid.tools.EuclidCoreTools;

class GradientDescentIterationData
{
   private double rightActuatorPosition;
   private double leftActuatorPosition;
   private double rollAngle;
   private double pitchAngle;
   private double candidateRollAngleStepSize;
   private double candidatePitchAngleStepSize;
   private boolean currentPositionsAreSingular;

   public void set(GradientDescentIterationData other)
   {
      this.rightActuatorPosition = other.rightActuatorPosition;
      this.leftActuatorPosition = other.leftActuatorPosition;
      this.pitchAngle = other.pitchAngle;
      this.rollAngle = other.rollAngle;
      this.candidatePitchAngleStepSize = other.candidatePitchAngleStepSize;
      this.candidateRollAngleStepSize = other.candidateRollAngleStepSize;
      this.currentPositionsAreSingular = other.currentPositionsAreSingular;
   }

   public void setIfCurrentPositionsAreSingular(boolean currentPositionsAreSingular)
   {
      this.currentPositionsAreSingular = currentPositionsAreSingular;
   }

   public void setJointAngles(double rollAngle, double pitchAngle)
   {
      this.rollAngle = rollAngle;
      this.pitchAngle = pitchAngle;
   }

   public void setCandidateJointAngleStepSizes(double candidateRollAngleStepSize, double candidatePitchAngleStepSize)
   {
      this.candidateRollAngleStepSize = candidateRollAngleStepSize;
      this.candidatePitchAngleStepSize = candidatePitchAngleStepSize;
   }

   public void setActuatorPositions(double rightActuatorPosition, double leftActuatorPosition)
   {
      this.rightActuatorPosition = rightActuatorPosition;
      this.leftActuatorPosition = leftActuatorPosition;
   }

   public double getRollAngle()
   {
      return rollAngle;
   }

   public double getPitchAngle()
   {
      return pitchAngle;
   }

   public double getCandidateRollAngleStepSize()
   {
      return candidateRollAngleStepSize;
   }

   public double getCandidatePitchAngleStepSize()
   {
      return candidatePitchAngleStepSize;
   }

   public double getRightActuatorPosition()
   {
      return rightActuatorPosition;
   }

   public double getLeftActuatorPosition()
   {
      return leftActuatorPosition;
   }

   public boolean areCurrentPositionsSingular()
   {
      return currentPositionsAreSingular;
   }

   public void reset()
   {
      rightActuatorPosition = Double.NaN;
      leftActuatorPosition = Double.NaN;
      rollAngle = Double.NaN;
      pitchAngle = Double.NaN;
      candidateRollAngleStepSize = Double.NaN;
      candidatePitchAngleStepSize = Double.NaN;
      currentPositionsAreSingular = false;
   }

   /**
    * Update the contained roll and pitch angles, taking the computed step size.
    *
    * @param learningRate                      scalar multiplier to apply to the step size.
    * @param maxRatioForSteppingInOneDirection ratio between gradients such that if one is significantly larger than the other, we only step in that direction.
    */
   public void takeStepFromCurrentState(double learningRate, double maxRatioForSteppingInOneDirection)
   {
      // if the gradient with respect to pitch is considerably larger than the gradient with respect to roll, we don't want to vary from the current roll state
      if (Math.abs(candidatePitchAngleStepSize / candidateRollAngleStepSize) < maxRatioForSteppingInOneDirection)
         rollAngle = takeStepSize(learningRate, rollAngle, candidateRollAngleStepSize);

      // if the gradient with respect to roll is considerably larger than the gradient with respect to pitch, we don't want to vary from the current pitch state
      if (Math.abs(candidateRollAngleStepSize / candidatePitchAngleStepSize) < maxRatioForSteppingInOneDirection)
         pitchAngle = takeStepSize(learningRate, pitchAngle, candidatePitchAngleStepSize);
   }

   static double takeStepSize(double learningRate, double currentState, double candidateStepSize)
   {
      return EuclidCoreTools.trimAngleMinusPiToPi(currentState + learningRate * candidateStepSize);
   }

   /**
    * Checks whether the joint angles contained in {@param stepStateToCheck} that result in its corresponding actuator configurations have converged to the
    * provided desired actuator configuration, within some tolerance threshold.
    *
    * @param desiredRightActuatorPosition position for the right actuator to converge to
    * @param desiredLeftActuatorPosition position for the left actuator to converge to
    * @return whether the system has converged.
    */
   public boolean haveActuatorPositionsConverged(double desiredRightActuatorPosition,
                                                 double desiredLeftActuatorPosition,
                                                 double actuatorAngleConvergenceEpsilon)
   {
      return checkIfConverged(desiredRightActuatorPosition - getRightActuatorPosition(),
                              desiredLeftActuatorPosition - getLeftActuatorPosition(),
                              actuatorAngleConvergenceEpsilon);
   }

   /**
    * Checks whether the step size in {@param stepStateToCheck} is below some magnitude threshold, indicating convergence.
    *
    * @return whether the step size is converged.
    */
   public boolean haveStepSizesConverged(double stepSizeConvergenceEpsilon)
   {
      return checkIfConverged(getCandidatePitchAngleStepSize(), getCandidateRollAngleStepSize(), stepSizeConvergenceEpsilon);
   }

   /**
    * Checks that the two fields are below some magnitude threshold, indicating convergence.
    *
    * @param fieldA    first field in question
    * @param fieldB    second field in question
    * @param threshold maximum absolute value for eitehr field
    * @return whether the fields indicate converge.
    */
   public static boolean checkIfConverged(double fieldA, double fieldB, double threshold)
   {
      return Math.abs(fieldA) < threshold && Math.abs(fieldB) < threshold;
   }
}
