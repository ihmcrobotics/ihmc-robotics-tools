package us.ihmc.robotics.kinematics.jointPair.data;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.robotics.outputData.JointDesiredLoadMode;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class YoJointDataTest
{
   private static final int iters = 1000;
   private static final double epsilon = 1e-10;

   @Test
   public void testGettingAndSettingData()
   {
      YoJointData jointData = new YoJointData("test", true, new YoRegistry("test"));
      Random random = new Random(1738l);

      for (int i = 0; i < iters; i++)
      {
         double position = RandomNumbers.nextDouble(random, 1000.0);
         double velocity = RandomNumbers.nextDouble(random, 1000.0);
         double acceleration = RandomNumbers.nextDouble(random, 1000.0);
         double torque = RandomNumbers.nextDouble(random, 1000.0);
         double stiffness = RandomNumbers.nextDouble(random, 1000.0);
         double damping = RandomNumbers.nextDouble(random, 1000.0);
         JointDesiredLoadMode loadMode = RandomNumbers.nextEnum(random, JointDesiredLoadMode.class);

         jointData.setPosition(position);
         jointData.setVelocity(velocity);
         jointData.setAcceleration(acceleration);
         jointData.setForce(torque);
         jointData.setStiffness(stiffness);
         jointData.setDamping(damping);
         jointData.setLoadMode(loadMode);

         assertEquals(position, jointData.getPosition(), epsilon);
         assertEquals(velocity, jointData.getVelocity(), epsilon);
         assertEquals(acceleration, jointData.getAcceleration(), epsilon);
         assertEquals(torque, jointData.getForce(), epsilon);
         assertEquals(stiffness, jointData.getStiffness(), epsilon);
         assertEquals(damping, jointData.getDamping(), epsilon);
         assertEquals(loadMode, jointData.getLoadMode());
      }
   }

   @Test
   public void testSetter()
   {
      YoJointData jointData = new YoJointData("test", true, new YoRegistry("test"));
      Random random = new Random(1738l);

      for (int i = 0; i < iters; i++)
      {
         JointData randomJointData = JointDataTest.getRandomJointData(random);
         jointData.set(randomJointData);

         assertEquals(randomJointData.getPosition(), jointData.getPosition(), epsilon);
         assertEquals(randomJointData.getVelocity(), jointData.getVelocity(), epsilon);
         assertEquals(randomJointData.getAcceleration(), jointData.getAcceleration(), epsilon);
         assertEquals(randomJointData.getForce(), jointData.getForce(), epsilon);
         assertEquals(randomJointData.getStiffness(), jointData.getStiffness(), epsilon);
         assertEquals(randomJointData.getDamping(), jointData.getDamping(), epsilon);
         assertEquals(randomJointData.getLoadMode(), jointData.getLoadMode());
      }
   }
}
