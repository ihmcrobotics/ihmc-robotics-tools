package us.ihmc.robotics.trajectories;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.trajectories.interfaces.FramePolynomial3DBasics;
import us.ihmc.robotics.trajectories.interfaces.Polynomial3DBasics;
import us.ihmc.robotics.trajectories.yoVariables.YoFramePolynomial3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoFramePolynomial3DTest extends FramePolynomial3DBasicsTest
{
   String namePrefix = "YoPolynomialTest";

   @Override
   public Polynomial3DBasics getPolynomial(int maxNumberOfCoefficients)
   {
      return getPolynomial(maxNumberOfCoefficients, ReferenceFrame.getWorldFrame());
   }


   @Override
   public FramePolynomial3DBasics getPolynomial(int maxNumberOfCoefficients, ReferenceFrame referenceFrame)
   {
      YoRegistry registry = new YoRegistry(namePrefix);
      return new YoFramePolynomial3D(namePrefix + "Linear", maxNumberOfCoefficients, referenceFrame, registry);
   }
}