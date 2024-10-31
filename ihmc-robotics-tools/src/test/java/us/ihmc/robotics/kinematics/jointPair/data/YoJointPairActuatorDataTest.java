package us.ihmc.robotics.kinematics.jointPair.data;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class YoJointPairActuatorDataTest
{
   private static final int iters = 1000;
   private static final double epsilon = 1e-10;

   @Test
   public void testGettingAndSettingData()
   {
      YoJointPairActuatorData actuatorData = new YoJointPairActuatorData("test", true, new YoRegistry("test"));
      Random random = new Random(1738L);

      for (int i = 0; i < iters; i++)
      {
         double insidePosition = RandomNumbers.nextDouble(random, 1000);
         double outsidePosition = RandomNumbers.nextDouble(random, 1000);
         double insideVelocity = RandomNumbers.nextDouble(random, 1000);
         double outsideVelocity = RandomNumbers.nextDouble(random, 1000);
         double insideForce = RandomNumbers.nextDouble(random, 1000);
         double outsideForce = RandomNumbers.nextDouble(random, 10000);

         actuatorData.setPosition(insidePosition, outsidePosition);
         actuatorData.setVelocity(insideVelocity, outsideVelocity);
         actuatorData.setForce(insideForce, outsideForce);

         assertEquals(insidePosition, actuatorData.getRightPosition(), epsilon);
         assertEquals(outsidePosition, actuatorData.getLeftPosition(), epsilon);
         assertEquals(insideVelocity, actuatorData.getRightVelocity(), epsilon);
         assertEquals(outsideVelocity, actuatorData.getLeftVelocity(), epsilon);
         assertEquals(insideForce, actuatorData.getRightForce(), epsilon);
         assertEquals(outsideForce, actuatorData.getLeftForce(), epsilon);

         actuatorData.setRightPosition(insidePosition);
         actuatorData.setLeftPosition(outsidePosition);
         actuatorData.setRightVelocity(insideVelocity);
         actuatorData.setLeftVelocity(outsideVelocity);
         actuatorData.setRightForce(insideForce);
         actuatorData.setLeftForce(outsideForce);

         assertEquals(insidePosition, actuatorData.getRightPosition(), epsilon);
         assertEquals(outsidePosition, actuatorData.getLeftPosition(), epsilon);
         assertEquals(insideVelocity, actuatorData.getRightVelocity(), epsilon);
         assertEquals(outsideVelocity, actuatorData.getLeftVelocity(), epsilon);
         assertEquals(insideForce, actuatorData.getRightForce(), epsilon);
         assertEquals(outsideForce, actuatorData.getLeftForce(), epsilon);
      }
   }
}
