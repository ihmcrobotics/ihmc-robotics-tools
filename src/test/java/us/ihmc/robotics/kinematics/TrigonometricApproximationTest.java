package us.ihmc.robotics.kinematics;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.log.LogTools;

import static org.junit.jupiter.api.Assertions.*;

public class TrigonometricApproximationTest
{
   private static double taylorStepEvaluationOfArcCos(double x, int n)
   {
      return factorial(2 * n) / (MathTools.pow(2.0, 2 * n) * MathTools.square(factorial(n))) * MathTools.pow(x, 2 * n + 1) / (2 * n + 1);
   }

   private static double taylorStepEvaluationOfSin(double x, int n)
   {
      return Math.pow(-1.0, n) * Math.pow(x, 2.0 * n + 1) / factorial(2 * n + 1);
   }

   private static double taylorStepEvaluationOfCos(double x, int n)
   {
      return Math.pow(-1.0, n) * Math.pow(x, 2.0 * n) / factorial(2 * n);
   }

   private static double taylorStepEvaluationOfArcTan(double x, int n)
   {
      return Math.pow(-1.0, n) * Math.pow(x, 2.0 * n + 1) / (2 * n + 1);
   }

   private static int factorial(int value)
   {
      int ret = 1;
      for (int c = 1; c <= value; c++)
         ret = ret * c;

      return ret;
   }

   @Test
   public void compareACosTimes()
   {
      Mean regularMean = new Mean();
      Mean atanMean = new Mean();
      Mean taylorMean = new Mean();
      Mean taylorATanMean = new Mean();
      for (double angle = -1.0; angle < 1.0; angle += 1e-6)
      {
         double value = Math.cos(angle);
         long regularStart = System.nanoTime();
         double regularAngle = Math.acos(value);
         regularMean.increment(System.nanoTime() - regularStart);

         long atanStart = System.nanoTime();
         double atanAngle = TrigonometricApproximation.aTan2ApproximationOfArcCos(value);
         atanMean.increment(System.nanoTime() - atanStart);

         long taylorStart = System.nanoTime();
         double taylorAngle = TrigonometricApproximation.taylorApproximationOfArcCos(value, 3);
         taylorMean.increment(System.nanoTime() - taylorStart);

         long taylorATanStart = System.nanoTime();
         double taylorATanAngle = TrigonometricApproximation.taylorArcTan2ApproximationOfArcCos(value, 3);
         taylorATanMean.increment(System.nanoTime() - taylorATanStart);

         //         assertEquals(angle, regularAngle, 1e-5);
         //         assertEquals(angle, atanAngle, 1e-5);
         //         assertEquals(angle, taylorAngle, 1e-5);
      }

      LogTools.info("Regular time : " + regularMean.getResult());
      LogTools.info("ATan Approximation time : " + atanMean.getResult());
      LogTools.info("Taylor Approximation time : " + taylorMean.getResult());
      LogTools.info("Taylor ATan Approximation time : " + taylorATanMean.getResult());
   }

   @Test
   public void compareCosTimes()
   {
      Mean regularMean = new Mean();
      Mean taylorMean = new Mean();
      for (double angle = -Math.PI; angle <= Math.PI; angle += 1e-6)
      {
         double value = Math.cos(angle);
         long regularStart = System.nanoTime();
         double regularAngle = Math.cos(value);
         regularMean.increment(System.nanoTime() - regularStart);

         long taylorStart = System.nanoTime();
         double taylorAngle = TrigonometricApproximation.taylorApproximationOfCos(value, 6);
         taylorMean.increment(System.nanoTime() - taylorStart);

         //         assertEquals(angle, regularAngle, 1e-5);
         //         assertEquals(angle, atanAngle, 1e-5);
         //         assertEquals(angle, taylorAngle, 1e-5);
      }

      double regularTime = regularMean.getResult();
      double taylorTime = taylorMean.getResult();
      //      LogTools.info("Regular time : " + regularTime);
      //      LogTools.info("Taylor Approximation time : " + taylorTime);
      LogTools.info("Taylor divided by Regular Time : " + taylorTime / regularTime);
   }

   @Test
   public void compareATanTimes()
   {
      Mean regularMean = new Mean();
      Mean taylorMean = new Mean();
      for (double angle = -Math.PI; angle <= Math.PI; angle += 1e-6)
      {
         double value = Math.tan(angle);
         long regularStart = System.nanoTime();
         double regularAngle = Math.atan(value);
         regularMean.increment(System.nanoTime() - regularStart);

         long taylorStart = System.nanoTime();
         double taylorAngle = TrigonometricApproximation.taylorApproximationOfArcTan(value, 3);
         taylorMean.increment(System.nanoTime() - taylorStart);

         //                  assertEquals(angle, regularAngle, 1e-5);
         //                  assertEquals(angle, atanAngle, 1e-5);
         //                  assertEquals(angle, taylorAngle, 1e-5);
      }

      double regularTime = regularMean.getResult();
      double taylorTime = taylorMean.getResult();
      //      LogTools.info("Regular time : " + regularTime);
      //      LogTools.info("Taylor Approximation time : " + taylorTime);
      LogTools.info("Taylor divided by Regular Time : " + taylorTime / regularTime);
   }

   @Test
   public void testTaylorApproximationOfArcCos()
   {
      double x = 0.5;
      for (int stepDepth = 1; stepDepth < 5; stepDepth++)
      {
         double value = Math.PI / 2.0;
         for (int i = 0; i <= stepDepth; i++)
            value -= taylorStepEvaluationOfArcCos(x, i);

         assertEquals(value, TrigonometricApproximation.taylorApproximationOfArcCos(x, stepDepth), 1e-5, "Depth " + stepDepth + " failed.");
      }

      double value = Math.acos(x);
      double taylorValue = TrigonometricApproximation.taylorApproximationOfArcCos(x, 5);
      assertEquals(value, taylorValue, 1e-5);
   }

   @Test
   public void testTaylorApproximationOfCos()
   {
      for (int stepDepth = 1; stepDepth < 10; stepDepth++)
      {
         double x = 0.5;
         double value = 1.0;
         for (int i = 1; i <= stepDepth; i++)
            value += taylorStepEvaluationOfCos(x, i);

         assertEquals(value, TrigonometricApproximation.taylorApproximationOfCos(x, stepDepth), 1e-7, "Depth " + stepDepth + " failed.");
      }
   }

   @Test
   public void testTaylorApproximationOfSin()
   {
      for (int stepDepth = 1; stepDepth < 10; stepDepth++)
      {
         double x = 0.5;
         double value = x;
         for (int i = 1; i <= stepDepth; i++)
            value += taylorStepEvaluationOfSin(x, i);

         assertEquals(value, TrigonometricApproximation.taylorApproximationOfSin(x, stepDepth), 1e-7, "Depth " + stepDepth + " failed.");
      }
   }

   @Test
   public void testTaylorApproximationOfArcTan()
   {
      for (int stepDepth = 1; stepDepth < 10; stepDepth++)
      {
         double x = 0.5;
         double value = 0.0;
         for (int i = 0; i <= stepDepth; i++)
            value += taylorStepEvaluationOfArcTan(x, i);

         assertEquals(value, TrigonometricApproximation.taylorApproximationOfArcTan(x, stepDepth), 1e-7, "Depth " + stepDepth + " failed.");
      }
   }

   @Test
   public void testRotationMatrixSetting()
   {
      RotationMatrix rollMatrix = new RotationMatrix();
      RotationMatrix rollMatrixAlt = new RotationMatrix();
      RotationMatrix pitchMatrix = new RotationMatrix();
      RotationMatrix pitchMatrixAlt = new RotationMatrix();

      int depth = 6;

      for (double angle = -Math.PI; angle <= Math.PI; angle += 0.001)
      {
         rollMatrix.setToRollOrientation(angle);
         pitchMatrix.setToPitchOrientation(angle);

         TrigonometricApproximation.computeRollMatrix(angle, rollMatrixAlt, depth);
         TrigonometricApproximation.computePitchMatrix(angle, pitchMatrixAlt, depth);

         EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(rollMatrix, rollMatrixAlt, 5e-3);
         EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(pitchMatrix, pitchMatrixAlt, 5e-3);
      }
   }

   @Test
   public void testApproximateArgumentForAddedArcCos()
   {
      for (double cosTheta1 = 0.0; cosTheta1 <= 1.0; cosTheta1 += 1e-3)
      {
         for (double cosTheta2 = 0.0; cosTheta2 <= 1.0; cosTheta2 += 1e-3)
         {
            double expected = Math.acos(cosTheta1) + Math.acos(cosTheta2);
            assertEquals(expected, Math.acos(TrigonometricApproximation.approximateArgumentForAddedArcCosAngles(cosTheta1, cosTheta2)), 1e-5);
         }
      }
   }

   @Test
   public void testApproximateArgumentForSubtractedArcCos()
   {
      for (double cosTheta1 = 0.0; cosTheta1 <= 1.0; cosTheta1 += 1e-3)
      {
         for (double cosTheta2 = 0.0; cosTheta2 <= 1.0; cosTheta2 += 1e-3)
         {
            double expected = Math.acos(cosTheta1) - Math.acos(cosTheta2);
            if (cosTheta1 <= cosTheta2)
               assertEquals(expected, Math.acos(TrigonometricApproximation.approximateArgumentForSubtractedArcCosAngles(cosTheta1, cosTheta2)), 1e-4);
            else
               assertEquals(Double.NaN, Math.acos(TrigonometricApproximation.approximateArgumentForSubtractedArcCosAngles(cosTheta1, cosTheta2)));
         }
      }
   }
}
