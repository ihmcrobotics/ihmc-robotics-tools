package us.ihmc.robotics.kinematics;

import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.tools.EuclidCoreTools;

public class TrigonometricApproximation
{
   public static double approximateArgumentForAddedArcCosAngles(double cosTheta1, double cosTheta2)
   {
      return cosTheta1 * cosTheta2 - Math.sqrt((1.0 - cosTheta1 * cosTheta1) * (1.0 - cosTheta2 * cosTheta2));
   }

   public static double approximateArgumentForSubtractedArcCosAngles(double cosTheta1, double cosTheta2)
   {
      // Warning: If solution is expected to be negative, it'll be out of the range of arccos()
      if (cosTheta1 > cosTheta2)
         return Double.NaN;
      return cosTheta1 * cosTheta2 + Math.sqrt((1.0 - cosTheta1 * cosTheta1) * (1.0 - cosTheta2 * cosTheta2));
   }

   public static double aTan2ApproximationOfArcCos(double cosTheta)
   {
      return 2.0 * Math.atan2(Math.sqrt(1 - cosTheta * cosTheta), 1 + cosTheta);
   }

   public static double taylorArcTan2ApproximationOfArcCos(double cosTheta, int seriesDepth)
   {
      return 2.0 * taylorApproximationOfArcTan(Math.sqrt(1 - cosTheta * cosTheta) / (1 + cosTheta), seriesDepth);
   }

   public static double taylorApproximationOfArcCos(double x, int seriesDepth)
   {
      if (seriesDepth < 1)
         throw new IllegalArgumentException("Depth must be at least 1");
      double value = Math.PI / 2.0 - x;
      int topFactorial = 1;
      int bottomFactorial = 1;
      int bottomConstExponential = 1;
      double valueExponential = x;
      double xSquared = x * x;
      int rhsDenominator = 1;
      for (int n = 1; n <= seriesDepth; n++)
      {
         topFactorial *= (2 * n) * (2 * n - 1);
         bottomFactorial *= n;
         bottomConstExponential *= 4;
         rhsDenominator += 2;
         valueExponential *= xSquared;
         double factorial = topFactorial / ((double) (bottomFactorial * bottomFactorial));
         double fullDenominator = bottomConstExponential * rhsDenominator;
         value -= factorial * valueExponential / fullDenominator;
      }
      return value;
   }

   /**
    * https://proofwiki.org/wiki/Power_Series_Expansion_for_Real_Arctangent_Function
    */
   public static double taylorApproximationOfArcTan(double x, int seriesDepth)
   {
      if (seriesDepth < 1)
         throw new IllegalArgumentException("Depth must be at least 1");
      double xSquared = x * x;
      double numerator = x;
      double ret = x;
      int denominator = 1;
      for (int i = 1; i <= seriesDepth; i++)
      {
         numerator *= -xSquared;
         denominator += 2;
         ret += numerator / denominator;
      }
      return ret;
   }

   public static double taylorApproximationOfCos(double x, int seriesDepth)
   {
      double xSquared = x * x;
      int denominator = 1;
      double numerator = 1.0;
      double ret = 1.0;
      int doubleI = 0;
      for (int i = 1; i <= seriesDepth; i++)
      {
         numerator *= -xSquared;
         doubleI += 2;
         denominator *= doubleI * (doubleI - 1);
         ret += numerator / denominator;
      }
      return ret;
   }

   public static double taylorApproximationOfSin(double x, int seriesDepth)
   {
      double xSquared = x * x;
      int denominator = 1;
      double numerator = x;
      double ret = x;
      int doubleI = 1;
      for (int i = 1; i <= seriesDepth; i++)
      {
         numerator *= -xSquared;
         doubleI += 2;
         denominator *= doubleI * (doubleI - 1);
         ret += numerator / denominator;
      }
      return ret;
   }

   public static void computePitchMatrix(double pitch, RotationMatrixBasics matrixToPack, int depth)
   {
      if (EuclidCoreTools.isAngleZero(pitch, RotationMatrixConversion.EPS))
      {
         matrixToPack.setToZero();
      }
      else
      {
         double sinPitch = TrigonometricApproximation.taylorApproximationOfSin(pitch, depth);
         double cosPitch = TrigonometricApproximation.taylorApproximationOfCos(pitch, depth);
         matrixToPack.setUnsafe(cosPitch, 0.0, sinPitch, 0.0, 1.0, 0.0, -sinPitch, 0.0, cosPitch);
      }
   }

   public static void computeRollMatrix(double roll, RotationMatrixBasics matrixToPack, int depth)
   {
      if (EuclidCoreTools.isAngleZero(roll, RotationMatrixConversion.EPS))
      {
         matrixToPack.setToZero();
      }
      else
      {
         double sinRoll = TrigonometricApproximation.taylorApproximationOfSin(roll, depth);
         double cosRoll = TrigonometricApproximation.taylorApproximationOfCos(roll, depth);
         matrixToPack.setUnsafe(1.0, 0.0, 0.0, 0.0, cosRoll, -sinRoll, 0.0, sinRoll, cosRoll);
      }
   }
}