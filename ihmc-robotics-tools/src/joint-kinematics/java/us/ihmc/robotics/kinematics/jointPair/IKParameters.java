package us.ihmc.robotics.kinematics.jointPair;

public class IKParameters
{
   static final boolean LIMIT_EXECUTION_TIME = false;

   // When the gradient descent takes a step to a new possible solution, it does so by stepping towards that solution, applying a candidate step size.
   // This candidate step size comes from the Jacobian, and is scaled by the learning rate to account for system nonlinearities. In this implementation, the
   // learning rate is adaptive, so that a smaller learning rate is applied for small steps, and a smaller learning rate is applied to big steps, so that the
   // angle doesn't overshoot quickly, and increases  the rate of change as it gets closer. Between these two thresholds,
   // the applied learning rate is linearly interpolated.
   static final double learningRateForLargeSteps = 0.2; // The learning rate to apply for large steps.
   static final double learningRateForSmallSteps = 0.8; // The learning rate to apply for small steps.
   static final double smallStepSizeThreshold = 0.0; // The maximum candidate step size to apply the small learning rate
   static final double largeStepSizeThreshold = 0.5; // The minimum candidate step size at which to start applying the larger learning rate

   // As the gradient descent is adaptive, it checks for overshoot. If the new candidate solution overshoots the optimal solution (which is determined by the
   // gradient changing signs), the learning rate is reduced, and the candidate solution is recomputed.
   static final int maximumNumberOfOvershootCorrections = 5; // maximum number of iterations to perform at a single configuration
   // If the candidate correction overshoots, but the new gradient value is low enough (defined by being less than this value), the new solution is accepted
   static final double minimumGradientForOvershootCorrection = 1e-2;
   // This is the rate at which the learning rate is reduced every time the candidate solution overshoots. learningRate *= reduction
   static final double learningRateScaleForOvershootCorrection = 0.5;

   // These are the values that determine if the solver has converged to a solution. If the actuator angle convergence, that means the correct roll/pitch
   // combination was found for the desired angles. This is an angle difference check. If the actuator angles have not converged by the step size has converged,
   // that means the system has reached a local minima, and continuing to search will result in no changes.
   static final double actuatorAngleConvergenceEpsilon = 1e-7;
   static final double stepSizeConvergenceEpsilon = 1e-8;

   static final double jacobianSingularityThreshold = 1e-5;
   static final double explodingJacobianThreshold = 1e10;

   // These are the termination conditions that forcefully terminate the solver iterations. If it reaches a maximum number of iterations, whether this is
   // stepping to a new solution or correcting an overshoot, it should stop. If it reaches a maximum time, it should also stop.
   static final int maxTotalIterations = 50;
   static final int minTotalIterations = 0;
   static final long maxTimeNS = 100000;

   public boolean getLimitExecutionTime()
   {
      return LIMIT_EXECUTION_TIME;
   }

   public double getLearningRateForLargeSteps()
   {
      return learningRateForLargeSteps;
   }

   public double getLearningRateForSmallSteps()
   {
      return learningRateForSmallSteps;
   }

   public double getSmallStepSizeThreshold()
   {
      return smallStepSizeThreshold;
   }

   public double getLargeStepSizeThreshold()
   {
      return largeStepSizeThreshold;
   }

   public int getMaximumNumberOfOvershootCorrections()
   {
      return maximumNumberOfOvershootCorrections;
   }

   public double getMinimumGradientForOvershootCorrection()
   {
      return minimumGradientForOvershootCorrection;
   }

   public double getLearningRateScaleForOvershootCorrection()
   {
      return learningRateScaleForOvershootCorrection;
   }

   public double getActuatorAngleConvergenceEpsilon()
   {
      return actuatorAngleConvergenceEpsilon;
   }

   public double getStepSizeConvergenceEpsilon()
   {
      return stepSizeConvergenceEpsilon;
   }

   public double getJacobianSingularityThreshold()
   {
      return jacobianSingularityThreshold;
   }

   public double getExplodingJacobianThreshold()
   {
      return explodingJacobianThreshold;
   }

   public int getMaxTotalIterations()
   {
      return maxTotalIterations;
   }

   public int getMinTotalIterations()
   {
      return minTotalIterations;
   }

   public long getMaxTimeNS()
   {
      return maxTimeNS;
   }
}
