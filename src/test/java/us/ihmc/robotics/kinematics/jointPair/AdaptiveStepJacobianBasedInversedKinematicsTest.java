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
      // Test at bounds of the linear interpolation
      assertEquals(learningRateForSmallSteps, getLearningRate(smallStepSizeThreshold), EPSILON);
      assertEquals(learningRateForSmallSteps, getLearningRate(-smallStepSizeThreshold), EPSILON);
      assertEquals(learningRateForLargeSteps, getLearningRate(largeStepSizeThreshold), EPSILON);
      assertEquals(learningRateForLargeSteps, getLearningRate(-largeStepSizeThreshold), EPSILON);

      // outside of bounds
      assertEquals(learningRateForSmallSteps, getLearningRate(0.1 * smallStepSizeThreshold), EPSILON);
      assertEquals(learningRateForSmallSteps, getLearningRate(-0.1 * smallStepSizeThreshold), EPSILON);
      assertEquals(learningRateForLargeSteps, getLearningRate(2.0 * largeStepSizeThreshold), EPSILON);
      assertEquals(learningRateForLargeSteps, getLearningRate(-2.0 * largeStepSizeThreshold), EPSILON);

      Random random = new Random(1738L);
      for (int iter = 0; iter < 1000; iter++)
      {
         double alpha = RandomNumbers.nextDouble(random, -1.0, 1.0);
         double stepSize = InterpolationTools.linearInterpolate(smallStepSizeThreshold, largeStepSizeThreshold, alpha);
         double expectedLearningRate = InterpolationTools.linearInterpolate(learningRateForSmallSteps, learningRateForLargeSteps, Math.abs(alpha));

         assertEquals(expectedLearningRate, getLearningRate(stepSize), EPSILON);
      }
   }
}
