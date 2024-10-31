package us.ihmc.robotics.kinematics.jointPair.data;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.robotics.outputData.JointDesiredLoadMode;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class YoJointPairJointDataTest
{
   private static final int iters = 1000;
   private static final double epsilon = 1e-10;

   @Test
   public void testGettingAndSettingData()
   {
      YoJointPairJointData jointData = new YoJointPairJointData("test", true, new YoRegistry("test"));
      Random random = new Random(1738L);

      for (int i = 0; i < iters; i++)
      {
         double rollPosition = RandomNumbers.nextDouble(random, 1000);
         double pitchPosition = RandomNumbers.nextDouble(random, 1000);
         double rollVelocity = RandomNumbers.nextDouble(random, 1000);
         double pitchVelocity = RandomNumbers.nextDouble(random, 1000);
         double rollAcceleration = RandomNumbers.nextDouble(random, 1000);
         double pitchAcceleration = RandomNumbers.nextDouble(random, 1000);
         double rollTorque = RandomNumbers.nextDouble(random, 1000);
         double pitchTorque = RandomNumbers.nextDouble(random, 10000);
         double rollStiffness = RandomNumbers.nextDouble(random, 10000);
         double pitchStiffness = RandomNumbers.nextDouble(random, 10000);
         double rollDamping = RandomNumbers.nextDouble(random, 10000);
         double pitchDamping = RandomNumbers.nextDouble(random, 10000);
         JointDesiredLoadMode rollLoadMode = RandomNumbers.nextEnum(random, JointDesiredLoadMode.class);
         JointDesiredLoadMode pitchLoadMode = RandomNumbers.nextEnum(random, JointDesiredLoadMode.class);

         jointData.setPosition(rollPosition, pitchPosition);
         jointData.setVelocity(rollVelocity, pitchVelocity);
         jointData.setTorque(rollTorque, pitchTorque);
         jointData.setAcceleration(rollAcceleration, pitchAcceleration);
         jointData.setStiffness(rollStiffness, pitchStiffness);
         jointData.setDamping(rollDamping, pitchDamping);
         jointData.setLoadMode(rollLoadMode, pitchLoadMode);

         assertEquals(rollPosition, jointData.getRollPosition(), epsilon);
         assertEquals(pitchPosition, jointData.getPitchPosition(), epsilon);
         assertEquals(rollVelocity, jointData.getRollVelocity(), epsilon);
         assertEquals(pitchVelocity, jointData.getPitchVelocity(), epsilon);
         assertEquals(rollAcceleration, jointData.getRollAcceleration(), epsilon);
         assertEquals(pitchAcceleration, jointData.getPitchAcceleration(), epsilon);
         assertEquals(rollTorque, jointData.getRollTorque(), epsilon);
         assertEquals(pitchTorque, jointData.getPitchTorque(), epsilon);
         assertEquals(rollStiffness, jointData.getRollStiffness(), epsilon);
         assertEquals(pitchStiffness, jointData.getPitchStiffness(), epsilon);
         assertEquals(rollDamping, jointData.getRollDamping(), epsilon);
         assertEquals(pitchDamping, jointData.getPitchDamping(), epsilon);
         assertEquals(rollLoadMode, jointData.getRollLoadMode());
         assertEquals(pitchLoadMode, jointData.getPitchLoadMode());

         rollPosition = RandomNumbers.nextDouble(random, 1000);
         pitchPosition = RandomNumbers.nextDouble(random, 1000);
         rollVelocity = RandomNumbers.nextDouble(random, 1000);
         pitchVelocity = RandomNumbers.nextDouble(random, 1000);
         rollAcceleration = RandomNumbers.nextDouble(random, 1000);
         pitchAcceleration = RandomNumbers.nextDouble(random, 1000);
         rollTorque = RandomNumbers.nextDouble(random, 1000);
         pitchTorque = RandomNumbers.nextDouble(random, 10000);
         rollStiffness = RandomNumbers.nextDouble(random, 10000);
         pitchStiffness = RandomNumbers.nextDouble(random, 10000);
         rollDamping = RandomNumbers.nextDouble(random, 10000);
         pitchDamping = RandomNumbers.nextDouble(random, 10000);
         rollLoadMode = RandomNumbers.nextEnum(random, JointDesiredLoadMode.class);
         pitchLoadMode = RandomNumbers.nextEnum(random, JointDesiredLoadMode.class);

         jointData.setRollPosition(rollPosition);
         jointData.setPitchPosition(pitchPosition);
         jointData.setRollVelocity(rollVelocity);
         jointData.setPitchVelocity(pitchVelocity);
         jointData.setRollTorque(rollTorque);
         jointData.setPitchTorque(pitchTorque);
         jointData.setRollAcceleration(rollAcceleration);
         jointData.setPitchAcceleration(pitchAcceleration);
         jointData.setRollStiffness(rollStiffness);
         jointData.setPitchStiffness(pitchStiffness);
         jointData.setRollDamping(rollDamping);
         jointData.setPitchDamping(pitchDamping);
         jointData.setRollLoadMode(rollLoadMode);
         jointData.setPitchLoadMode(pitchLoadMode);

         assertEquals(rollPosition, jointData.getRollPosition(), epsilon);
         assertEquals(pitchPosition, jointData.getPitchPosition(), epsilon);
         assertEquals(rollVelocity, jointData.getRollVelocity(), epsilon);
         assertEquals(pitchVelocity, jointData.getPitchVelocity(), epsilon);
         assertEquals(rollAcceleration, jointData.getRollAcceleration(), epsilon);
         assertEquals(pitchAcceleration, jointData.getPitchAcceleration(), epsilon);
         assertEquals(rollTorque, jointData.getRollTorque(), epsilon);
         assertEquals(pitchTorque, jointData.getPitchTorque(), epsilon);
         assertEquals(rollStiffness, jointData.getRollStiffness(), epsilon);
         assertEquals(pitchStiffness, jointData.getPitchStiffness(), epsilon);
         assertEquals(rollDamping, jointData.getRollDamping(), epsilon);
         assertEquals(pitchDamping, jointData.getPitchDamping(), epsilon);
         assertEquals(rollLoadMode, jointData.getRollLoadMode());
         assertEquals(pitchLoadMode, jointData.getPitchLoadMode());
      }
   }

   @Test
   public void testSetters()
   {
      YoJointPairJointData jointData = new YoJointPairJointData("test", true, new YoRegistry("test"));
      Random random = new Random(1738L);

      for (int i = 0; i < iters; i++)
      {
         JointPairJointData randomJointData = JointPairJointDataTest.getRandomJointPairJointData(random);

         jointData.set(randomJointData);

         assertEquals(randomJointData.getRollPosition(), jointData.getRollPosition(), epsilon);
         assertEquals(randomJointData.getRollVelocity(), jointData.getRollVelocity(), epsilon);
         assertEquals(randomJointData.getRollAcceleration(), jointData.getRollAcceleration(), epsilon);
         assertEquals(randomJointData.getRollTorque(), jointData.getRollTorque(), epsilon);
         assertEquals(randomJointData.getRollStiffness(), jointData.getRollStiffness(), epsilon);
         assertEquals(randomJointData.getRollDamping(), jointData.getRollDamping(), epsilon);
         assertEquals(randomJointData.getRollLoadMode(), jointData.getRollLoadMode());

         assertEquals(randomJointData.getPitchPosition(), jointData.getPitchPosition(), epsilon);
         assertEquals(randomJointData.getPitchVelocity(), jointData.getPitchVelocity(), epsilon);
         assertEquals(randomJointData.getPitchAcceleration(), jointData.getPitchAcceleration(), epsilon);
         assertEquals(randomJointData.getPitchTorque(), jointData.getPitchTorque(), epsilon);
         assertEquals(randomJointData.getPitchStiffness(), jointData.getPitchStiffness(), epsilon);
         assertEquals(randomJointData.getPitchDamping(), jointData.getPitchDamping(), epsilon);
         assertEquals(randomJointData.getPitchLoadMode(), jointData.getPitchLoadMode());
      }
   }
}
