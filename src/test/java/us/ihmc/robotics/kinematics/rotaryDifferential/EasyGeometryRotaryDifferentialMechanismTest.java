package us.ihmc.robotics.kinematics.rotaryDifferential;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.kinematics.jointPair.data.JointPairActuatorData;
import us.ihmc.robotics.kinematics.jointPair.data.JointPairJointData;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class EasyGeometryRotaryDifferentialMechanismTest extends RotaryActuatorDifferentialMechanismTest
{
   @Override
   public RotaryActuatorDifferentialKinematicsSpecifications getKinematicsSpecification()
   {
      return RotaryActuatorDifferentialTestHelper.createEasyGeometrySpecifications(true);
   }

   @Test
   public void testEasyGeometry()
   {
      Vector3DReadOnly rightFirstJointToPelvisRodEnd = RotaryActuatorDifferentialTestHelper.getRightFirstJointToBaseRodEnd();
      Vector3DReadOnly rightSecondJointToActuator = RotaryActuatorDifferentialTestHelper.getRightSecondJointToActuator();
      Vector3DReadOnly rightActuatorToRodEnd = RotaryActuatorDifferentialTestHelper.getRightActuatorToRodEnd();
      Vector3DReadOnly leftFirstJointToPelvisRodEnd = RotaryActuatorDifferentialTestHelper.getLeftFirstJointToBaseRodEnd();
      Vector3DReadOnly leftSecondJointToActuator = RotaryActuatorDifferentialTestHelper.getLeftSecondJointToActuator();
      Vector3DReadOnly leftActuatorToRodEnd = RotaryActuatorDifferentialTestHelper.getLeftActuatorToRodEnd();
      double rightRodEndLength = RotaryActuatorDifferentialTestHelper.getEasyGeometryRodEndLength();
      double leftRodEndLength = RotaryActuatorDifferentialTestHelper.getEasyGeometryRodEndLength();

      RotaryActuatorDifferentialForwardKinematics forwardKinematics = new RotaryActuatorDifferentialForwardKinematics(leftFirstJointToPelvisRodEnd,
                                                                                                                      leftSecondJointToActuator,
                                                                                                                      leftActuatorToRodEnd,
                                                                                                                      leftRodEndLength,
                                                                                                                      rightFirstJointToPelvisRodEnd,
                                                                                                                      rightSecondJointToActuator,
                                                                                                                      rightActuatorToRodEnd,
                                                                                                                      rightRodEndLength,
                                                                                                                      new Vector3D(),
                                                                                                                      true,
                                                                                                                      false,
                                                                                                                      false);
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
      // check pitch
      for (double pitchAngle = -Math.toRadians(45.0); pitchAngle <= Math.toRadians(45.0); pitchAngle += Math.toRadians(1.0))
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
      for (double rollAngle = -Math.toRadians(25.0); rollAngle <= Math.toRadians(25.0); rollAngle += Math.toRadians(1.0))
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
      for (double pitchAngle = -Math.toRadians(40.0); pitchAngle <= Math.toRadians(40.0); pitchAngle += Math.toRadians(1.0))
      {
         for (double rollAngle = -Math.toRadians(20.0); rollAngle <= Math.toRadians(20.0); rollAngle += Math.toRadians(1.0))
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
         double rollAngle = RandomNumbers.nextDouble(random, Math.toRadians(20.0));
         double pitchAngle = RandomNumbers.nextDouble(random, Math.toRadians(40.0));
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
