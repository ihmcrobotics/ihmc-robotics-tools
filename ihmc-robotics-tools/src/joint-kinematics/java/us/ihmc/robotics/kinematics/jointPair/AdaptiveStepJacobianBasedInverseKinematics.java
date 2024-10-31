package us.ihmc.robotics.kinematics.jointPair;

import us.ihmc.robotics.kinematics.jointPair.interfaces.JointPairForwardKinematics;
import us.ihmc.robotics.kinematics.jointPair.interfaces.JointPairInverseKinematics;
import us.ihmc.robotics.kinematics.jointPair.interfaces.JointPairJacobian;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.misc.UnrolledDeterminantFromMinor_DDRM;
import org.ejml.dense.row.mult.MatrixMatrixMult_DDRM;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;

import java.util.Arrays;

/**
 * This class uses the Jacobian at the internal joint Angle guess and the error in the Angle at that value to compute the modification to the
 * joint Angle guess to match the actuator Angle. It will iterate the joint angle estimate until some convergence threshold is reached.
 */
public class AdaptiveStepJacobianBasedInverseKinematics implements JointPairInverseKinematics
{
   private static final boolean LIMIT_EXECUTION_TIME = false;

   private static final double maxRatioForSteppingInOneDirection = Double.POSITIVE_INFINITY;

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
   private int maximumNumberOfOvershootCorrections = 5; // maximum number of iterations to perform at a single configuration
   // If the candidate correction overshoots, but the new gradient value is low enough (defined by being less than this value), the new solution is accepted
   private static final double minimumGradientForOvershootCorrection = 1e-2;
   // This is the rate at which the learning rate is reduced every time the candidate solution overshoots. learningRate *= reduction
   private static final double learningRateScaleForOvershootCorrection = 0.5;

   // These are the values that determine if the solver has converged to a solution. If the actuator angle convergence, that means the correct roll/pitch
   // combination was found for the desired angles. This is an angle difference check. If the actuator angles have not converged by the step size has converged,
   // that means the system has reached a local minima, and continuing to search will result in no changes.
   private static final double actuatorAngleConvergenceEpsilon = 1e-8;
   private static final double stepSizeConvergenceEpsilon = 1e-8;

   private static final double jacobianSingularityThreshold = 1e-5;
   private static final double explodingJacobianThreshold = 1e10;

   // These are the termination conditions that forcefully terminate the solver iterations. If it reaches a maximum number of iterations, whether this is
   // stepping to a new solution or correcting an overshoot, it should stop. If it reaches a maximum time, it should also stop.
   private int maxTotalIterations = 50;
   private int minTotalIterations = 0;
   private static final long maxTimeNS = 100000;

   // These are the things that define the kinematic structure for the problem solution. It defines the mechanism in question.
   private final JointPairForwardKinematics forwardKinematics;
   private final JointPairJacobian jacobianCalculator;
   private final int pitchIndex;
   private final int rollIndex;

   // State variables used to return on the solver statistics.
   private int iterations;
   private long timeElapsed;

   // State variables specifiying where the solver should start solving from, as well as whether the angles should be reset to 0.0 on the next compute cycle.
   private double rollAngle = Double.NaN;
   private double pitchAngle = Double.NaN;
   private boolean reinitializeOnNextCompute = true;

   // Temporary state variables used to return the solver solutions.
   private final GradientDescentIterationData currentJointAngleState = new GradientDescentIterationData();
   private final GradientDescentIterationData candidateJointAngleState = new GradientDescentIterationData();

   // Temporary variables used for computing the step size.
   private final DMatrixRMaj actuatorError = new DMatrixRMaj(2, 1);
   private final DMatrixRMaj jointCorrection = new DMatrixRMaj(2, 1);

   private IKConvergenceCondition convergenceCondition = IKConvergenceCondition.FAILED;
   private double residualSquaredError = Double.NaN;
   private boolean successfullyWarmStarted = true;

   public AdaptiveStepJacobianBasedInverseKinematics(JointPairJacobian jacobianCalculator)
   {
      this.jacobianCalculator = jacobianCalculator;
      this.forwardKinematics = jacobianCalculator.getForwardKinematics();
      pitchIndex = jacobianCalculator.getPitchIndex();
      rollIndex = jacobianCalculator.getRollIndex();
   }

   public void setMaximumTotalIterations(int maximumTotalIterations)
   {
      this.maxTotalIterations = maximumTotalIterations;
   }

   public void setMinimumTotalIterations(int minTotalIterations)
   {
      this.minTotalIterations = minTotalIterations;
   }

   public void setMaximumNumberOfOvershootCorrections(int maximumNumberOfOvershootCorrections)
   {
      this.maximumNumberOfOvershootCorrections = maximumNumberOfOvershootCorrections;
   }

   public void triggerReinitialize()
   {
      this.reinitializeOnNextCompute = true;
   }

   /**
    * Sets up the solver to perform gradient descent to find the joint angles based on the desired actuator positions. The actuators can be either linear or
    * rotary. If linear, the position is assumed to be a length in meters. If rotary, the position is assumed to be an angle in radians.
    *
    * @param desiredLeftActuatorPosition  the desired left actuator position to find the solution for
    * @param desiredRightActuatorPosition the desired right actuator position to find the solution for
    * @return whether true if the initialization failed because the problem cannot be solved for the desired actuator positions.
    */
   private boolean initialize(double desiredLeftActuatorPosition, double desiredRightActuatorPosition)
   {
      successfullyWarmStarted = true;
      convergenceCondition = IKConvergenceCondition.FAILED;
      residualSquaredError = Double.NaN;
      iterations = 0;
      // set everything to null
      currentJointAngleState.reset();
      // set up the forward kinematics for the current angle guess. If this is being "warm started", it will iterate from the previous roll and pitch solution
      currentJointAngleState.setJointAngles(rollAngle, pitchAngle);
      // get the initial step size for the current position.
      computeJointStepSizeAtPose(desiredLeftActuatorPosition, desiredRightActuatorPosition, currentJointAngleState);

      // If either of the step sizes is NaN or Infinite, that means that the problem is ill conditione dand the Jacobian is in a singularity. This problem
      // cannot be solved given those setpoints.
      return !Double.isFinite(currentJointAngleState.getCandidatePitchAngleStepSize())
             || !Double.isFinite(currentJointAngleState.getCandidateRollAngleStepSize()) || currentJointAngleState.areCurrentPositionsSingular();
   }

   @Override
   public IKConvergenceCondition computeJointAngles(double desiredRightActuatorPosition, double desiredLeftActuatorPosition)
   {
      if (Double.isNaN(desiredRightActuatorPosition) || Double.isNaN(desiredLeftActuatorPosition))
         throw new IllegalArgumentException(
               "Invalid actuator setpoints, right = " + desiredRightActuatorPosition + ", left = " + desiredLeftActuatorPosition);
      if (reinitializeOnNextCompute || Double.isNaN(rollAngle) || Double.isNaN(pitchAngle))
      {
         // If the reinitialize flag is true, then either the solver determined on the previous compute call that it was stuck in a local minima, or it
         // never reached convergence, either timing out or running through all the allowable iterations. If either joint angle is NaN, then the angles need to
         // be reinitialized, regardless.
         rollAngle = 0.0;
         pitchAngle = 0.0;
         successfullyWarmStarted = false;
      }
      if (initialize(desiredRightActuatorPosition, desiredLeftActuatorPosition))
      {
         // If the initialize call returns true, that means that the step size at the specified roll and pitch angles is invalid. If that is the case, we
         // should set them to zero, and try to reinitialize.
         rollAngle = 0.0;
         pitchAngle = 0.0;
         successfullyWarmStarted = false;
         // We set the roll and pitch angle to zero, but still can't reinitialize the system. That means that the system is in a state from which it cannot
         // recover. There are no known conditions that cause this.
         if (initialize(desiredRightActuatorPosition, desiredLeftActuatorPosition))
            throw new RuntimeException("We have an internal bug, and won't be able to recover. System failed at roll : " + rollAngle + ", pitch : " + pitchAngle
                                       + ", and desired right position : " + desiredRightActuatorPosition + ", desired left"
                                       + " position : " + desiredLeftActuatorPosition);
      }
      reinitializeOnNextCompute = true;
      long startTime = System.nanoTime();
      timeElapsed = 0;

      while (iterations < maxTotalIterations && (!LIMIT_EXECUTION_TIME || timeElapsed < maxTimeNS))
      {
         boolean hasIteratedEnough = iterations >= minTotalIterations;
         if (hasIteratedEnough && currentJointAngleState.haveActuatorPositionsConverged(desiredRightActuatorPosition,
                                                                                        desiredLeftActuatorPosition,
                                                                                        actuatorAngleConvergenceEpsilon))
         {
            // If this is true, that means fthat the right combination of pitch and roll angles to achieve the desired actuator configurations have been found.
            // The solver is complete.
            reinitializeOnNextCompute = false;
            timeElapsed = System.nanoTime() - startTime;
            convergenceCondition = IKConvergenceCondition.OBJECTIVE_CONVERGED;
            residualSquaredError =
                  MathTools.square(currentJointAngleState.getRightActuatorPosition() - desiredRightActuatorPosition) + MathTools.square(
                        currentJointAngleState.getLeftActuatorPosition() - desiredLeftActuatorPosition);
            break;
         }
         if (hasIteratedEnough && currentJointAngleState.haveStepSizesConverged(stepSizeConvergenceEpsilon))
         {
            // If this is true, that means the step size is below some threshold, but the objective was not reached. That means that the solver is stuck in a
            // local minima. Because it's in a local minima, next time around, we should reinitialize the joint positions
            timeElapsed = System.nanoTime() - startTime;
            convergenceCondition = IKConvergenceCondition.GRADIENT_CONVERGED;
            residualSquaredError =
                  MathTools.square(currentJointAngleState.getRightActuatorPosition() - desiredRightActuatorPosition) + MathTools.square(
                        currentJointAngleState.getLeftActuatorPosition() - desiredLeftActuatorPosition);
            break;
         }

         // The
         iterations += stepToNextValidPose(desiredRightActuatorPosition, desiredLeftActuatorPosition, currentJointAngleState, iterations);

         // Record the elapsed time, such that if we have a maximum time we can ensure it doesn't get exceeded.
         timeElapsed = System.nanoTime() - startTime;
      }
      rollAngle = currentJointAngleState.getRollAngle();
      pitchAngle = currentJointAngleState.getPitchAngle();

      return convergenceCondition;
   }

   /**
    * This method advances the state contained inside {@param jointAngleStateToPack} to the next desired state, performing a full iteration of the gradient
    * descent. It is seeking to achieve the actuator configurations {@param desiredInsideActuatorConfiguration} and {@param desiredOutsideActuatorConfiguration}
    * by modifying the joint pitch and roll angles. The actuator configurations for a given pitch/roll pair can be computed using forward kinematics. This
    * method terminates if the desired actuator configurations are achieved, or if the maximum number of iterations allowed are reached. This step also
    * performs an overshoot correction step, if the configurations are determined to have overshot. It does this by increasing the learning rate discount
    * factor, which decreases the learning rate to use.
    *
    * @param desiredInsideActuatorConfiguration  desired inside actuator configuration that the solver is trying to achieve.
    * @param desiredOutsideActuatorConfiguration desired outside actuator configuration that the solver is trying to achieve.
    * @param jointAngleStateToPack               container used to hold the joint angles and their corresponding actuator configurations. Modified.
    * @param iterationsAlreadyPerformed          number of iterations that have already been performed by the solver. used to enforce a computation limit.
    * @return number of iterations the solver performed.
    */
   private int stepToNextValidPose(double desiredInsideActuatorConfiguration,
                                   double desiredOutsideActuatorConfiguration,
                                   GradientDescentIterationData jointAngleStateToPack,
                                   int iterationsAlreadyPerformed)
   {
      int iterationsPerformed = 0;
      double learningRateDiscount = 1.0;
      // We want to make sure that the iterations performed when stepping doesn't exceed the total allowable iterations
      int allowableIterations = Math.min(maxTotalIterations - iterationsAlreadyPerformed, maximumNumberOfOvershootCorrections);

      do
      {
         iterationsPerformed++;
         takeJointStep(jointAngleStateToPack, candidateJointAngleState, learningRateDiscount);

         // Compute what the step size. If this returns true, it determined that convergence was reached, so break the loop.
         if (computeJointStepSizeAtPose(desiredInsideActuatorConfiguration, desiredOutsideActuatorConfiguration, candidateJointAngleState))
            break;
         // Don't bother checking for overshoot if we're in a singular configuration.
         if (!candidateJointAngleState.areCurrentPositionsSingular())
         {
            // check whether the step overshoots the optimal solution. This is done by comparing the directino of step sizes, and their total magnitude. If it has
            // not overshot, the step is valid, and the reduction should terminate.
            if (!didJointCorrectionOvershoot(jointAngleStateToPack, candidateJointAngleState))
               break;
         }
         learningRateDiscount *= learningRateScaleForOvershootCorrection;
      }
      while (iterationsPerformed < allowableIterations);

      // Set the joint angle state from the new joint angle
      jointAngleStateToPack.set(candidateJointAngleState);

      return iterationsPerformed;
   }

   /**
    * Applies the candidate joint step to the starting joint angle step, minus the learning rate, to reach some new configuration. Computes the learning rate
    * to apply to the step based on the proposed step size, and applies an additional discount factor to that learning rate.
    *
    * @param startingJointAngleStep    starting configuration to keep. Not modified.
    * @param jointAngleStepGuessToPack resulting configuration after applying the step. Modified.
    * @param learningRateDiscount      discount factor to apply to the learning rate.
    */
   private static void takeJointStep(GradientDescentIterationData startingJointAngleStep,
                                     GradientDescentIterationData jointAngleStepGuessToPack,
                                     double learningRateDiscount)
   {
      double learningRate = getLearningRate(Math.min(Math.abs(startingJointAngleStep.getCandidateRollAngleStepSize()),
                                                     Math.abs(startingJointAngleStep.getCandidatePitchAngleStepSize())));
      learningRate *= learningRateDiscount;
      jointAngleStepGuessToPack.set(startingJointAngleStep);
      jointAngleStepGuessToPack.takeStepFromCurrentState(learningRate, maxRatioForSteppingInOneDirection);
   }

   /**
    * This computes a candidate change in joint angle given the current joint angle and the desired actuator positions. The goal is to compute the joint
    * angles that acheive the desired actuator positions. These are likely not at the current joint positions contained {@param jointAngleStepToPack},
    * so a proposed position change must be computed. This method computes that change by transforming the actuator error to the joint error via the
    * computed Jacobian of the system. However, if the actuator error is below some threshold, the optimal solution is achieved, and this method returns that
    * convergence has been reached.
    *
    * @param desiredRightActuatorPosition  desired position (angle or length) of the right actuator at the optimal solution
    * @param desiredLeftActuatorPosition desired position (angle or length) of the left actuator at the optimal solution
    * @param jointAngleStepToPack                current joint configuration of the system against which to compute the change.
    * @return Whether the current pose solution has converged
    */
   private boolean computeJointStepSizeAtPose(double desiredRightActuatorPosition,
                                              double desiredLeftActuatorPosition,
                                              GradientDescentIterationData jointAngleStepToPack)
   {
      // Compute the actuator angles that correspond to the current joint angle guess.
      forwardKinematics.computeActuatorPositions(jointAngleStepToPack.getRollAngle(), jointAngleStepToPack.getPitchAngle());
      jointAngleStepToPack.setActuatorPositions(forwardKinematics.getRightActuatorPosition(), forwardKinematics.getLeftActuatorPosition());

      // Compute the resulting error between the actuator state at the current joint state and the desired actuator state.
      double residualRightPositionError = desiredRightActuatorPosition - jointAngleStepToPack.getRightActuatorPosition();
      double residualLeftPositionError = desiredLeftActuatorPosition - jointAngleStepToPack.getLeftActuatorPosition();

      if (GradientDescentIterationData.checkIfConverged(residualRightPositionError, residualLeftPositionError, actuatorAngleConvergenceEpsilon))
      {
         // If the system converged, the step size should be zero.
         jointAngleStepToPack.setCandidateJointAngleStepSizes(0.0, 0.0);
         return true;
      }
      else
      {
         computeJointStepSize(residualRightPositionError, residualLeftPositionError, jointAngleStepToPack);
         return false;
      }
   }

   /**
    * This gets a scalar modifier for the search step size that is linearly ramped. When the error is small (below {#errorForSmallCorrection}), the scalar
    * is returned as {#minCorrectionScalar}. When the error is big (above {#errorForBigCorrection}), the scalar is returned as {#maxCorrectionScalar}. In
    * between, the correction scalar is linearly increased.
    */
   static double getLearningRate(double errorMagnitude)
   {
      double absError = Math.abs(errorMagnitude);
      double alpha = (absError - smallStepSizeThreshold) / (largeStepSizeThreshold - smallStepSizeThreshold);
      alpha = MathTools.clamp(alpha, 0.0, 1.0);
      return InterpolationTools.linearInterpolate(learningRateForSmallSteps, learningRateForLargeSteps, alpha);
   }

   /**
    * This computes the change in the joint positions necessary to drive the configuration error to zero.
    * It is derived from the observation that qDot<sub>joint</sub> = J<sup>-1</sup> qDot<sub>actuator</sub>
    * We can then say that, in discrete space, q<sub>error,joint</sub> = J<sup>-1</sup> q<sub>error,actuator</sub>
    *
    * @return whether the current configuration is singular
    */
   private boolean computeJointStepSize(double rightActuatorPositionError,
                                        double leftActuatorPositionError,
                                        GradientDescentIterationData angleStepToPack)
   {
      // compute the jacobian. These rely on the forward kinematics having been previously computed in the #computeStepSizeAtPose() method.
      jacobianCalculator.computeJacobian();

      double determinant = UnrolledDeterminantFromMinor_DDRM.det2(jacobianCalculator.getJacobianMatrix());
      if (determinant < jacobianSingularityThreshold || determinant > explodingJacobianThreshold
          || containsNaN(jacobianCalculator.getJacobianMatrix()))
      {
         angleStepToPack.setIfCurrentPositionsAreSingular(true);
         return false;
      }

      // pack the error into a vector for the matrix multiply.
      actuatorError.set(0, 0, rightActuatorPositionError);
      actuatorError.set(1, 0, leftActuatorPositionError);

      // compute the joint correction
      MatrixMatrixMult_DDRM.mult_small(jacobianCalculator.getJacobianMatrixInverse(), actuatorError, jointCorrection);

      // pull the results into the data structure from the vector
      angleStepToPack.setCandidateJointAngleStepSizes(jointCorrection.get(rollIndex, 0), jointCorrection.get(pitchIndex, 0));
      angleStepToPack.setIfCurrentPositionsAreSingular(false);
      return true;
   }

   private static boolean containsNaN(DMatrixRMaj matrix)
   {
      for (int i = 0; i < matrix.getNumElements(); i++)
      {
         if (Double.isNaN(matrix.data[i]))
            return true;
      }

      return false;
   }

   /**
    * This checks whether the candidate new joint configuration overshot the optimal solution from the previous joint configuration. It does this by checking whether
    * the step at the proposed new configuration changes direction, as well as asserting that the step at the proposed new configuration is above a certain
    * threshold. If it is below that threshold, it means the proposed new configuration is very close to the optimal solution, so should be taken.
    *
    * @param previousAngleStep         previous configuration before applying the candidate step
    * @param currentAngleAfterStepping new configuration after applying the candidate step
    * @return whether the candidate step causes the new solution to overshoot.
    */
   private static boolean didJointCorrectionOvershoot(GradientDescentIterationData previousAngleStep, GradientDescentIterationData currentAngleAfterStepping)
   {
      // If the previous state didn't have a step size, we're just getting started. This means we don't care about overshooting.
      if (Double.isNaN(previousAngleStep.getCandidateRollAngleStepSize()) || Double.isNaN(previousAngleStep.getCandidatePitchAngleStepSize()))
         return false;

      double rollSign = Math.signum(currentAngleAfterStepping.getCandidateRollAngleStepSize());
      double absDeltaRoll = rollSign * currentAngleAfterStepping.getCandidateRollAngleStepSize();

      double maximumAllowableGradientIfSignChange = Math.min(minimumGradientForOvershootCorrection, stepSizeConvergenceEpsilon);

      // If the roll sign changed, and the roll step size is above the allowable gradient, we overshot the optimal solution.
      if (rollSign != Math.signum(previousAngleStep.getCandidateRollAngleStepSize()) && absDeltaRoll > maximumAllowableGradientIfSignChange)
         return true;

      double pitchSign = Math.signum(currentAngleAfterStepping.getCandidatePitchAngleStepSize());
      double absDeltaPitch = pitchSign * currentAngleAfterStepping.getCandidatePitchAngleStepSize();

      // If the roll sign changed, and the roll step size is above the allowable gradient, we overshot the optimal solution.
      if (pitchSign != Math.signum(previousAngleStep.getCandidatePitchAngleStepSize()) && absDeltaPitch > maximumAllowableGradientIfSignChange)
         return true;
      return false;
   }

   /**
    * Returns the current roll joint Angle found by {@link #computeJointAngles(double, double)}.
    */
   public double getRollJointAngle()
   {
      return rollAngle;
   }

   /**
    * Returns the current pitch joint Angle found by {@link #computeJointAngles(double, double)}.
    */
   public double getPitchJointAngle()
   {
      return pitchAngle;
   }

   public int getNumberOfIterations()
   {
      return iterations;
   }

   /**
    * Returns the time taken to convert actuator data into joint data, in Nanoseconds.
    */
   public long getComputationTime()
   {
      return timeElapsed;
   }

   @Override
   public double getJacobianDeterminant()
   {
      return UnrolledDeterminantFromMinor_DDRM.det2(jacobianCalculator.getJacobianMatrix());
   }

   @Override
   public IKConvergenceCondition getConvergenceCondition()
   {
      return convergenceCondition;
   }

   @Override
   public double getResidualSquaredError()
   {
      return residualSquaredError;
   }

   @Override
   public boolean successfullyWarmStarted()
   {
      return successfullyWarmStarted;
   }
}