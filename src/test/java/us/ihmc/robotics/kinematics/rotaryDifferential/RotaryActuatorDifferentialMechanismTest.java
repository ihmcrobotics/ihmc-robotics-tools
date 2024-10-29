package us.ihmc.robotics.kinematics.rotaryDifferential;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.kinematics.jointPair.data.JointPairActuatorData;
import us.ihmc.robotics.kinematics.jointPair.data.JointPairJointData;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public abstract class RotaryActuatorDifferentialMechanismTest
{
   public abstract RotaryActuatorDifferentialKinematicsSpecifications getKinematicsSpecification();

   @Test
   public void testEasyGeometry()
   {
      RotaryActuatorDifferentialKinematicsSpecifications kinematicsSpecifications = getKinematicsSpecification();

      RotaryActuatorDifferentialForwardKinematics forwardKinematics = new RotaryActuatorDifferentialForwardKinematics(kinematicsSpecifications, false, false);
      RotaryActuatorDifferentialInverseKinematics inverseKinematics = new RotaryActuatorDifferentialInverseKinematics(new RotaryActuatorDifferentialJacobianCalculator(
            forwardKinematics));

      RotaryActuatorDifferentialMechanism mechanism = new RotaryActuatorDifferentialMechanism(forwardKinematics, inverseKinematics);
      RotaryActuatorDifferentialForwardKinematics easyForwardKinematics = RotaryActuatorDifferentialTestHelper.createEasyGeometryForwardKinematics(true);
      // check zero position
      easyForwardKinematics.computeActuatorPositions(0.0, 0.0);
      assertEquals(0.0, easyForwardKinematics.getLeftMotorAngle(), 1e-5);
      assertEquals(0.0, easyForwardKinematics.getRightMotorAngle(), 1e-5);
      JointPairJointData forwardJointData = new JointPairJointData();
      JointPairJointData inverseJointData = new JointPairJointData();
      JointPairActuatorData actuatorData = new JointPairActuatorData();


      boolean rollIsFirstJoint = kinematicsSpecifications.isTheFirstJointRoll();
      double pitchLower = rollIsFirstJoint ? kinematicsSpecifications.getSecondJointLowerLimit() : kinematicsSpecifications.getFirstJointLowerLimit();
      double pitchUpper = rollIsFirstJoint ? kinematicsSpecifications.getSecondJointUpperLimit() : kinematicsSpecifications.getFirstJointUpperLimit();
      double rollLower = rollIsFirstJoint ? kinematicsSpecifications.getFirstJointLowerLimit() : kinematicsSpecifications.getSecondJointLowerLimit();
      double rollUpper = rollIsFirstJoint ? kinematicsSpecifications.getFirstJointUpperLimit() : kinematicsSpecifications.getSecondJointUpperLimit();

      // check pitch
      for (double pitchAngle = pitchLower; pitchAngle <= pitchUpper; pitchAngle += Math.toRadians(1.0))
      {
         easyForwardKinematics.computeActuatorPositions(0.0, pitchAngle);
         forwardJointData.setPitchPosition(pitchAngle);
         mechanism.computeActuatorDataFromJoint(forwardJointData, actuatorData);
         mechanism.computeJointDataFromActuator(actuatorData, inverseJointData);
         String error = "angle expected " + Math.toDegrees(-pitchAngle);
         // Run through a bunch of pitch angles, and assert that the angles are what is expected. For this mechanism arrangement, the four bar is square,
         // So the magnitudes should be the same. A positive actuator rotation results in a negative joint pitch rotation.
         assertEquals(0.0, EuclidCoreTools.angleDifferenceMinusPiToPi(-pitchAngle, easyForwardKinematics.getRightMotorAngle()), 1e-5, error);
         assertEquals(0.0, EuclidCoreTools.angleDifferenceMinusPiToPi(-pitchAngle, easyForwardKinematics.getLeftMotorAngle()), 1e-5, error);
         assertEquals(forwardJointData.getPitchJointData().getPosition(), inverseJointData.getPitchJointData().getPosition(), 1e-5, error);
         assertEquals(forwardJointData.getRollJointData().getPosition(), inverseJointData.getRollJointData().getPosition(), 1e-5, error);
      }
      // check roll
      for (double rollAngle = rollLower;
           rollAngle <= rollUpper; rollAngle += Math.toRadians(1.0))
      {
         easyForwardKinematics.computeActuatorPositions(rollAngle, 0.0);
         forwardJointData.setPitchPosition(0.0);
         inverseJointData.setRollPosition(rollAngle);
         mechanism.computeActuatorDataFromJoint(forwardJointData, actuatorData);
         mechanism.computeJointDataFromActuator(actuatorData, inverseJointData);
         String error = "angle expected " + Math.toDegrees(rollAngle);
         assertEquals(forwardJointData.getPitchJointData().getPosition(), inverseJointData.getPitchJointData().getPosition(), 1e-5, error);
         assertEquals(forwardJointData.getRollJointData().getPosition(), inverseJointData.getRollJointData().getPosition(), 1e-5, error);
      }
      // check a combination of the two
      for (double pitchAngle = 0.85 * pitchLower; pitchAngle <= 0.85 * pitchUpper; pitchAngle += Math.toRadians(1.0))
      {
         for (double rollAngle = 0.8 * rollLower; rollAngle <= 0.8 * rollUpper; rollAngle += Math.toRadians(1.0))
         {
            forwardJointData.setRollPosition(rollAngle);
            forwardJointData.setPitchPosition(pitchAngle);
            mechanism.computeActuatorDataFromJoint(forwardJointData, actuatorData);
            mechanism.computeJointDataFromActuator(actuatorData, inverseJointData);
            String error = "angle expected " + Math.toDegrees(pitchAngle);
            assertEquals(forwardJointData.getPitchJointData().getPosition(), inverseJointData.getPitchJointData().getPosition(), 1e-5, error);
            assertEquals(forwardJointData.getRollJointData().getPosition(), inverseJointData.getRollJointData().getPosition(), 1e-5, error);
         }
      }

      Random random = new Random(1738L);
      for (int i = 0; i < 10000; i++)
      {
         double rollAngle = RandomNumbers.nextDouble(random,
                                                     0.8 * rollLower,
                                                     0.8 * rollUpper);
         double pitchAngle = RandomNumbers.nextDouble(random,
                                                      0.85 * pitchLower,
                                                      0.85 * pitchUpper);
         forwardJointData.setRollPosition(rollAngle);
         forwardJointData.setPitchPosition(pitchAngle);
         mechanism.computeActuatorDataFromJoint(forwardJointData, actuatorData);
         mechanism.computeJointDataFromActuator(actuatorData, inverseJointData);
         String error = "angle expected " + Math.toDegrees(pitchAngle);

         assertEquals(pitchAngle, inverseJointData.getPitchJointData().getPosition(), 1e-5, error);
         assertEquals(rollAngle, inverseJointData.getRollJointData().getPosition(), 1e-5, error);
      }
   }
}