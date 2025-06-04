package us.ihmc.robotics.kinematics.jointPair;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.RandomNumbers;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.robotics.kinematics.jointPair.AdaptiveStepJacobianBasedInverseKinematics.*;

public class AdaptiveStepJacobianBasedInversedKinematicsTest
{
   private static final double EPSILON = 1e-12;

   @Test
   public void testGetLearningRate()
   {
      double rateForLargeSteps = IKParameters.learningRateForLargeSteps;
      double rateForSmallSteps = IKParameters.learningRateForSmallSteps;
      double smallStepThreshold = IKParameters.smallStepSizeThreshold;
      double largeStepThreshold = IKParameters.largeStepSizeThreshold;

      // Test at bounds of the linear interpolation
      assertEquals(rateForSmallSteps, getLearningRate(smallStepThreshold,smallStepThreshold, largeStepThreshold, rateForSmallSteps, rateForLargeSteps), EPSILON);
      assertEquals(rateForSmallSteps, getLearningRate(-smallStepThreshold, smallStepThreshold, largeStepThreshold, rateForSmallSteps, rateForLargeSteps), EPSILON);
      assertEquals(rateForLargeSteps, getLearningRate(largeStepThreshold, smallStepThreshold, largeStepThreshold, rateForSmallSteps, rateForLargeSteps), EPSILON);
      assertEquals(rateForLargeSteps, getLearningRate(-largeStepThreshold, smallStepThreshold, largeStepThreshold, rateForSmallSteps, rateForLargeSteps), EPSILON);

      // outside of bounds
      assertEquals(rateForSmallSteps, getLearningRate(0.1 * smallStepThreshold, smallStepThreshold, largeStepThreshold, rateForSmallSteps, rateForLargeSteps), EPSILON);
      assertEquals(rateForSmallSteps, getLearningRate(-0.1 * smallStepThreshold, smallStepThreshold, largeStepThreshold, rateForSmallSteps, rateForLargeSteps), EPSILON);
      assertEquals(rateForLargeSteps, getLearningRate(2.0 * largeStepThreshold, smallStepThreshold, largeStepThreshold, rateForSmallSteps, rateForLargeSteps), EPSILON);
      assertEquals(rateForLargeSteps, getLearningRate(-2.0 * largeStepThreshold, smallStepThreshold, largeStepThreshold, rateForSmallSteps, rateForLargeSteps), EPSILON);

      Random random = new Random(1738L);
      for (int iter = 0; iter < 1000; iter++)
      {
         double alpha = RandomNumbers.nextDouble(random, -1.0, 1.0);
         double stepSize = InterpolationTools.linearInterpolate(smallStepThreshold, largeStepThreshold, alpha);
         double expectedLearningRate = InterpolationTools.linearInterpolate(rateForSmallSteps, rateForLargeSteps, Math.abs(alpha));

         assertEquals(expectedLearningRate, getLearningRate(stepSize, smallStepThreshold, largeStepThreshold, rateForSmallSteps, rateForLargeSteps), EPSILON);
      }
   }
}
