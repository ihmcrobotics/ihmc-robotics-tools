package us.ihmc.robotics.kinematics.jointPair;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreTools;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class GradientDescentIterationDataTest
{
   private static final int iters = 1000;
   private static final double EPSILON = 1e-12;

   @Test
   public void testSettingAndGetting()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         // Get a random data container, which shold initially not be equal to everything, but then set the angles and assert that these ended up being equal to
         // everything.
         GradientDescentIterationData gradientDescentIterationData = getRandomState(random);

         double rollAngle = RandomNumbers.nextDouble(random, Math.PI);
         double pitchAngle = RandomNumbers.nextDouble(random, Math.PI);
         double innerActuatorConfiguration = RandomNumbers.nextDouble(random, Math.PI);
         double outerActuatorConfiguration = RandomNumbers.nextDouble(random, Math.PI);
         double candidateRollStepSize = RandomNumbers.nextDouble(random, 10.0);
         double candidatePitchStepSize = RandomNumbers.nextDouble(random, 10.0);

         assertNotEquals(rollAngle, gradientDescentIterationData.getRollAngle(), EPSILON);
         assertNotEquals(pitchAngle, gradientDescentIterationData.getPitchAngle(), EPSILON);
         assertNotEquals(innerActuatorConfiguration, gradientDescentIterationData.getRightActuatorPosition(), EPSILON);
         assertNotEquals(outerActuatorConfiguration, gradientDescentIterationData.getLeftActuatorPosition(), EPSILON);
         assertNotEquals(candidateRollStepSize, gradientDescentIterationData.getCandidateRollAngleStepSize(), EPSILON);
         assertNotEquals(candidatePitchStepSize, gradientDescentIterationData.getCandidatePitchAngleStepSize(), EPSILON);

         gradientDescentIterationData.setJointAngles(rollAngle, pitchAngle);
         gradientDescentIterationData.setActuatorPositions(innerActuatorConfiguration, outerActuatorConfiguration);
         gradientDescentIterationData.setCandidateJointAngleStepSizes(candidateRollStepSize, candidatePitchStepSize);

         assertEquals(rollAngle, gradientDescentIterationData.getRollAngle(), EPSILON);
         assertEquals(pitchAngle, gradientDescentIterationData.getPitchAngle(), EPSILON);
         assertEquals(innerActuatorConfiguration, gradientDescentIterationData.getRightActuatorPosition(), EPSILON);
         assertEquals(outerActuatorConfiguration, gradientDescentIterationData.getLeftActuatorPosition(), EPSILON);
         assertEquals(candidateRollStepSize, gradientDescentIterationData.getCandidateRollAngleStepSize(), EPSILON);
         assertEquals(candidatePitchStepSize, gradientDescentIterationData.getCandidatePitchAngleStepSize(), EPSILON);

         // Set up the check for the copy setter. In this case, the random state should initially not be equal to the current one, but after setting everything,
         // it should become equal.
         GradientDescentIterationData randomState = getRandomState(random);

         assertNotEquals(rollAngle, randomState.getRollAngle(), EPSILON);
         assertNotEquals(pitchAngle, randomState.getPitchAngle(), EPSILON);
         assertNotEquals(innerActuatorConfiguration, randomState.getRightActuatorPosition(), EPSILON);
         assertNotEquals(outerActuatorConfiguration, randomState.getLeftActuatorPosition(), EPSILON);
         assertNotEquals(candidateRollStepSize, randomState.getCandidateRollAngleStepSize(), EPSILON);
         assertNotEquals(candidatePitchStepSize, randomState.getCandidatePitchAngleStepSize(), EPSILON);

         randomState.set(gradientDescentIterationData);

         assertEquals(rollAngle, randomState.getRollAngle(), EPSILON);
         assertEquals(pitchAngle, randomState.getPitchAngle(), EPSILON);
         assertEquals(innerActuatorConfiguration, randomState.getRightActuatorPosition(), EPSILON);
         assertEquals(outerActuatorConfiguration, randomState.getLeftActuatorPosition(), EPSILON);
         assertEquals(candidateRollStepSize, randomState.getCandidateRollAngleStepSize(), EPSILON);
         assertEquals(candidatePitchStepSize, randomState.getCandidatePitchAngleStepSize(), EPSILON);

         // Reset the joint angle step state, which should set everything to NaN.
         gradientDescentIterationData.reset();

         assertEquals(Double.NaN, gradientDescentIterationData.getRollAngle());
         assertEquals(Double.NaN, gradientDescentIterationData.getPitchAngle());
         assertEquals(Double.NaN, gradientDescentIterationData.getRightActuatorPosition());
         assertEquals(Double.NaN, gradientDescentIterationData.getLeftActuatorPosition());
         assertEquals(Double.NaN, gradientDescentIterationData.getCandidateRollAngleStepSize());
         assertEquals(Double.NaN, gradientDescentIterationData.getCandidatePitchAngleStepSize());
      }
   }

   @Test
   public void testHasStepSizeConverged()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double convergenceEpsilon = RandomNumbers.nextDouble(random, 0.0, 10.0);
         GradientDescentIterationData gradientDescentIterationData = new GradientDescentIterationData();

         double rollStepSizeWellBelowConvergence = getRandomSign(random) * convergenceEpsilon * 0.1;
         double pitchStepSizeWellBelowConvergence = getRandomSign(random) * convergenceEpsilon * 0.1;
         double rollStepSizeWellAboveConvergence = getRandomSign(random) * convergenceEpsilon * 2.0;
         double pitchStepSizeWellAboveConvergence = getRandomSign(random) * convergenceEpsilon * 2.0;
         double rollStepSizeJustBelowConvergence = getRandomSign(random) * convergenceEpsilon * (1.0 - 1e-8);
         double pitchStepSizeJustBelowConvergence = getRandomSign(random) * convergenceEpsilon * (1.0 - 1e-8);
         double rollStepSizeJustAboveConvergence = getRandomSign(random) * convergenceEpsilon * (1.0 + 1e-8);
         double pitchStepSizeJustAboveConvergence = getRandomSign(random) * convergenceEpsilon * (1.0 + 1e-8);

         // test well below the convergence threshold
         gradientDescentIterationData.setCandidateJointAngleStepSizes(rollStepSizeWellBelowConvergence, pitchStepSizeWellBelowConvergence);
         assertTrue(gradientDescentIterationData.haveStepSizesConverged(convergenceEpsilon));

         // test well above the convergence threshold
         gradientDescentIterationData.setCandidateJointAngleStepSizes(rollStepSizeWellAboveConvergence, pitchStepSizeWellAboveConvergence);
         assertFalse(gradientDescentIterationData.haveStepSizesConverged(convergenceEpsilon));

         // test combos of well below and above threshold
         gradientDescentIterationData.setCandidateJointAngleStepSizes(rollStepSizeWellBelowConvergence, pitchStepSizeWellAboveConvergence);
         assertFalse(gradientDescentIterationData.haveStepSizesConverged(convergenceEpsilon));

         gradientDescentIterationData.setCandidateJointAngleStepSizes(pitchStepSizeWellAboveConvergence, pitchStepSizeWellBelowConvergence);
         assertFalse(gradientDescentIterationData.haveStepSizesConverged(convergenceEpsilon));

         // test just below the convergence threshold
         gradientDescentIterationData.setCandidateJointAngleStepSizes(rollStepSizeJustBelowConvergence, pitchStepSizeJustBelowConvergence);
         assertTrue(gradientDescentIterationData.haveStepSizesConverged(convergenceEpsilon));

         // test just above the convergence threshold
         gradientDescentIterationData.setCandidateJointAngleStepSizes(rollStepSizeJustAboveConvergence, pitchStepSizeJustAboveConvergence);
         assertFalse(gradientDescentIterationData.haveStepSizesConverged(convergenceEpsilon));

         // test combos of just below and above threshold
         gradientDescentIterationData.setCandidateJointAngleStepSizes(rollStepSizeJustBelowConvergence, pitchStepSizeJustAboveConvergence);
         assertFalse(gradientDescentIterationData.haveStepSizesConverged(convergenceEpsilon));

         gradientDescentIterationData.setCandidateJointAngleStepSizes(rollStepSizeJustAboveConvergence, pitchStepSizeJustBelowConvergence);
         assertFalse(gradientDescentIterationData.haveStepSizesConverged(convergenceEpsilon));

         // test right at convergence threshold
         gradientDescentIterationData.setCandidateJointAngleStepSizes(convergenceEpsilon, convergenceEpsilon);
         assertFalse(gradientDescentIterationData.haveStepSizesConverged(convergenceEpsilon));

         // test at zero step size
         gradientDescentIterationData.setCandidateJointAngleStepSizes(0.0, 0.0);
         assertTrue(gradientDescentIterationData.haveStepSizesConverged(convergenceEpsilon));
      }
   }

   @Test
   public void testHasActuatorConfigurationConverged()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < 75; iter++)
      {
         double convergenceEpsilon = RandomNumbers.nextDouble(random, 0.0, 10.0);
         GradientDescentIterationData gradientDescentIterationData = new GradientDescentIterationData();

         double desiredInnerActuatorConfiguration = RandomNumbers.nextDouble(random, Math.PI);
         double desiredOuterActuatorConfiguration = RandomNumbers.nextDouble(random, Math.PI);

         double innerActuatorConfigurationWellBelowConvergence =
               desiredInnerActuatorConfiguration - Math.signum(desiredInnerActuatorConfiguration) * convergenceEpsilon * 0.1;
         double outerActuatorConfigurationWellBelowConvergence =
               desiredOuterActuatorConfiguration - Math.signum(desiredOuterActuatorConfiguration) * convergenceEpsilon * 0.1;
         double innerActuatorConfigurationWellAboveConvergence =
               desiredInnerActuatorConfiguration + Math.signum(desiredInnerActuatorConfiguration) * convergenceEpsilon * 2.0;
         double outerActuatorConfigurationWellAboveConvergence =
               desiredOuterActuatorConfiguration + Math.signum(desiredOuterActuatorConfiguration) * convergenceEpsilon * 2.0;
         double innerActuatorConfigurationJustBelowConvergence =
               desiredInnerActuatorConfiguration - Math.signum(desiredInnerActuatorConfiguration) * convergenceEpsilon * (1.0 - 1e-8);
         double outerActuatorConfigurationJustBelowConvergence =
               desiredOuterActuatorConfiguration - Math.signum(desiredOuterActuatorConfiguration) * convergenceEpsilon * (1.0 - 1e-8);
         double innerActuatorConfigurationJustAboveConvergence =
               desiredInnerActuatorConfiguration + Math.signum(desiredInnerActuatorConfiguration) * convergenceEpsilon * (1.0 + 1e-8);
         double outerActuatorConfigurationJustAboveConvergence =
               desiredOuterActuatorConfiguration + Math.signum(desiredOuterActuatorConfiguration) * convergenceEpsilon * (1.0 + 1e-8);

         // test well below the convergence threshold
         gradientDescentIterationData.setActuatorPositions(innerActuatorConfigurationWellBelowConvergence, outerActuatorConfigurationWellBelowConvergence);
         assertTrue(gradientDescentIterationData.haveActuatorPositionsConverged(desiredInnerActuatorConfiguration,
                                                                                desiredOuterActuatorConfiguration,
                                                                                convergenceEpsilon));

         // test well above the convergence threshold
         gradientDescentIterationData.setActuatorPositions(innerActuatorConfigurationWellAboveConvergence, outerActuatorConfigurationWellAboveConvergence);
         assertFalse(gradientDescentIterationData.haveActuatorPositionsConverged(desiredInnerActuatorConfiguration,
                                                                                 desiredOuterActuatorConfiguration,
                                                                                 convergenceEpsilon));

         // test combos of well below and above threshold
         gradientDescentIterationData.setActuatorPositions(innerActuatorConfigurationWellBelowConvergence, outerActuatorConfigurationWellAboveConvergence);
         assertFalse(gradientDescentIterationData.haveActuatorPositionsConverged(desiredInnerActuatorConfiguration,
                                                                                 desiredOuterActuatorConfiguration,
                                                                                 convergenceEpsilon));

         gradientDescentIterationData.setActuatorPositions(outerActuatorConfigurationWellAboveConvergence, outerActuatorConfigurationWellBelowConvergence);
         assertFalse(gradientDescentIterationData.haveActuatorPositionsConverged(desiredInnerActuatorConfiguration,
                                                                                 desiredOuterActuatorConfiguration,
                                                                                 convergenceEpsilon));

         // test just below the convergence threshold
         gradientDescentIterationData.setActuatorPositions(innerActuatorConfigurationJustBelowConvergence, outerActuatorConfigurationJustBelowConvergence);
         assertTrue(gradientDescentIterationData.haveActuatorPositionsConverged(desiredInnerActuatorConfiguration,
                                                                                desiredOuterActuatorConfiguration,
                                                                                convergenceEpsilon));

         // test just above the convergence threshold
         gradientDescentIterationData.setActuatorPositions(innerActuatorConfigurationJustAboveConvergence, outerActuatorConfigurationJustAboveConvergence);
         assertFalse(gradientDescentIterationData.haveActuatorPositionsConverged(desiredInnerActuatorConfiguration,
                                                                                 desiredOuterActuatorConfiguration,
                                                                                 convergenceEpsilon));

         // test combos of just below and above threshold
         gradientDescentIterationData.setActuatorPositions(innerActuatorConfigurationJustBelowConvergence, outerActuatorConfigurationJustAboveConvergence);
         assertFalse(gradientDescentIterationData.haveActuatorPositionsConverged(desiredInnerActuatorConfiguration,
                                                                                 desiredOuterActuatorConfiguration,
                                                                                 convergenceEpsilon));

         gradientDescentIterationData.setActuatorPositions(innerActuatorConfigurationJustAboveConvergence, outerActuatorConfigurationJustBelowConvergence);
         assertFalse(gradientDescentIterationData.haveActuatorPositionsConverged(desiredInnerActuatorConfiguration,
                                                                                 desiredOuterActuatorConfiguration,
                                                                                 convergenceEpsilon));

         // test right at convergence threshold
         gradientDescentIterationData.setActuatorPositions(convergenceEpsilon + 1e-15 + desiredInnerActuatorConfiguration,
                                                                convergenceEpsilon + 1e-15 + desiredOuterActuatorConfiguration);
         assertFalse(gradientDescentIterationData.haveActuatorPositionsConverged(desiredInnerActuatorConfiguration,
                                                                                 desiredOuterActuatorConfiguration,
                                                                                 convergenceEpsilon));
         gradientDescentIterationData.setActuatorPositions(desiredInnerActuatorConfiguration - convergenceEpsilon - 1e-15,
                                                                desiredOuterActuatorConfiguration - convergenceEpsilon - 1e-15);
         assertFalse(gradientDescentIterationData.haveActuatorPositionsConverged(desiredInnerActuatorConfiguration,
                                                                                 desiredOuterActuatorConfiguration,
                                                                                 convergenceEpsilon));

         // test at zero step size
         gradientDescentIterationData.setActuatorPositions(desiredInnerActuatorConfiguration, desiredOuterActuatorConfiguration);
         assertTrue(gradientDescentIterationData.haveActuatorPositionsConverged(desiredInnerActuatorConfiguration,
                                                                                desiredOuterActuatorConfiguration,
                                                                                convergenceEpsilon));
      }
   }

   @Test
   public void testStepSize()
   {
      Random random = new Random(1738L);

      double learningRate = 0.5;
      double currentAngle = 0.1;
      double stepSize = 0.2;
      double expectedAngle = 0.2;
      double newAngle = GradientDescentIterationData.takeStepSize(learningRate, currentAngle, stepSize);
      assertEquals(expectedAngle, newAngle, EPSILON);

      // overshoot and test the wrap around
      currentAngle = Math.PI - 0.1;
      stepSize = 0.4;
      expectedAngle = -Math.PI + 0.1;
      newAngle = GradientDescentIterationData.takeStepSize(learningRate, currentAngle, stepSize);
      assertEquals(expectedAngle, newAngle, EPSILON);

      for (int iter = 0; iter < iters; iter++)
      {
         learningRate = RandomNumbers.nextDouble(random, 0.0, 5.0);
         currentAngle = RandomNumbers.nextDouble(random, Math.PI);
         stepSize = RandomNumbers.nextDouble(random, 10.0);

         newAngle = GradientDescentIterationData.takeStepSize(learningRate, currentAngle, stepSize);
         expectedAngle = currentAngle + learningRate * stepSize;
         expectedAngle = EuclidCoreTools.trimAngleMinusPiToPi(expectedAngle);

         assertEquals(expectedAngle, newAngle, EPSILON);
      }
   }

   private static double getRandomSign(Random random)
   {
      if (random.nextBoolean())
         return 1.0;
      else
         return -1.0;
   }

   private static GradientDescentIterationData getRandomState(Random random)
   {
      GradientDescentIterationData gradientDescentIterationData = new GradientDescentIterationData();
      gradientDescentIterationData.setJointAngles(RandomNumbers.nextDouble(random, Math.PI), RandomNumbers.nextDouble(random, Math.PI));
      gradientDescentIterationData.setActuatorPositions(RandomNumbers.nextDouble(random, Math.PI), RandomNumbers.nextDouble(random, Math.PI));
      gradientDescentIterationData.setCandidateJointAngleStepSizes(RandomNumbers.nextDouble(random, 10.0), RandomNumbers.nextDouble(random, 10.0));

      return gradientDescentIterationData;
   }
}
