package us.ihmc.robotics.kinematics.rotaryDifferential;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;

import static org.junit.jupiter.api.Assertions.*;

public class EasyGeometryRotaryForwardKinematicsTest extends RotaryActuatorDifferentialForwardKinematicsTest
{
   @Override
   public RotaryActuatorDifferentialKinematicsSpecifications getKinematicsSpecification()
   {
      return RotaryActuatorDifferentialTestHelper.createEasyGeometrySpecifications(true);
   }

   @Test
   public void testForwardKinematicsMatchExpectedValues()
   {
      // set up to iterate over roll and pitch being the first joint
      boolean[] rollIsFirstJoints = {true, false};

      for (boolean rollIsFirstJoint : rollIsFirstJoints)
      {
         RotaryActuatorDifferentialForwardKinematics forwardKinematics = RotaryActuatorDifferentialTestHelper.createEasyGeometryForwardKinematics(rollIsFirstJoint);

         // check zero position
         forwardKinematics.computeActuatorPositions(0.0, 0.0);
         assertEquals(0.0, forwardKinematics.getLeftMotorAngle(), 1e-5);
         assertEquals(0.0, forwardKinematics.getRightMotorAngle(), 1e-5);

         // check pitch
         for (double angle = -Math.toRadians(45.0); angle <= Math.toRadians(45.0); angle += Math.toRadians(5.0))
         {
            // Run through a bunch of pitch angles, and assert that the angles are what is expected. For this mechanism arrangement, the four bar is square,
            // So the magnitudes should be the same. A positive actuator rotation results in a negative joint pitch rotation.
            forwardKinematics.computeActuatorPositions(0.0, angle);
            String error = "actuator angle expected " + Math.toDegrees(-angle);
            assertEquals(0.0, EuclidCoreTools.angleDifferenceMinusPiToPi(-angle, forwardKinematics.getRightMotorAngle()), 1e-5, error);
            assertEquals(0.0, EuclidCoreTools.angleDifferenceMinusPiToPi(-angle, forwardKinematics.getLeftMotorAngle()), 1e-5, error);

            // The length of the rod end should be physically consistent, so check that.
            FramePoint3D leftActuatorRodEndAttachment = new FramePoint3D(forwardKinematics.getLeftActuatorRodEndAttachment());
            FramePoint3D leftPelvisRodEndAttachment = new FramePoint3D(forwardKinematics.getLeftBaseRodEndAttachment());
            leftActuatorRodEndAttachment.changeFrame(ReferenceFrame.getWorldFrame());
            leftPelvisRodEndAttachment.changeFrame(ReferenceFrame.getWorldFrame());

            assertEquals(RotaryActuatorDifferentialTestHelper.getEasyGeometryRodEndLength(), leftActuatorRodEndAttachment.distance(leftPelvisRodEndAttachment), 1e-5);

            FramePoint3D rightActuatorRodEndAttachment = new FramePoint3D(forwardKinematics.getRightActuatorRodEndAttachment());
            FramePoint3D rightPelvisRodEndAttachment = new FramePoint3D(forwardKinematics.getRightBaseRodEndAttachment());
            rightActuatorRodEndAttachment.changeFrame(ReferenceFrame.getWorldFrame());
            rightPelvisRodEndAttachment.changeFrame(ReferenceFrame.getWorldFrame());

            assertEquals(RotaryActuatorDifferentialTestHelper.getEasyGeometryRodEndLength(), rightActuatorRodEndAttachment.distance(rightPelvisRodEndAttachment), 1e-5);
         }

         // check roll
         for (double angle = -Math.toRadians(25.0); angle <= Math.toRadians(25.0); angle += Math.toRadians(5.0))
         {
            forwardKinematics.computeActuatorPositions(angle, 0.0);
            String errorMessage = "Failed on angle " + angle;
            if (angle < -1e-4)
            {
               // if the joint angle is rolling a negative value, the left actuator must be going up (which is negative) and the right actuator must be going
               // down (which is positive).
               assertTrue(forwardKinematics.getLeftMotorAngle() < 0.0, errorMessage);
               assertTrue(forwardKinematics.getRightMotorAngle() > 0.0, errorMessage);
            }
            else if (angle > 1e-4)
            {
               // if the joint angle is rolling a positive value, the left actuator must be going down (which is positive) and the right actuator must be going
               // up (which is negative).
               assertTrue(forwardKinematics.getLeftMotorAngle() > 0.0, errorMessage);
               assertTrue(forwardKinematics.getRightMotorAngle() < 0.0, errorMessage);
            }
            else
            {
               // if the joint angle is moving, the actuators should be zero.
               assertEquals(forwardKinematics.getLeftMotorAngle(), 0.0, 1e-5, errorMessage);
               assertEquals(forwardKinematics.getRightMotorAngle(), 0.0, 1e-5, errorMessage);
            }

            // check to make sure the rod end position is correct
            FramePoint3D leftActuatorRodEndAttachment = new FramePoint3D(forwardKinematics.getLeftActuatorRodEndAttachment());
            FramePoint3D leftTorsoRodEndAttachment = new FramePoint3D(forwardKinematics.getLeftBaseRodEndAttachment());
            FramePoint3D rightActuatorRodEndAttachment = new FramePoint3D(forwardKinematics.getRightActuatorRodEndAttachment());
            FramePoint3D rightTorsoRodEndAttachment = new FramePoint3D(forwardKinematics.getRightBaseRodEndAttachment());
            leftActuatorRodEndAttachment.changeFrame(ReferenceFrame.getWorldFrame());
            leftTorsoRodEndAttachment.changeFrame(ReferenceFrame.getWorldFrame());
            rightActuatorRodEndAttachment.changeFrame(ReferenceFrame.getWorldFrame());
            rightTorsoRodEndAttachment.changeFrame(ReferenceFrame.getWorldFrame());
            assertEquals(RotaryActuatorDifferentialTestHelper.getEasyGeometryRodEndLength(),
                         leftActuatorRodEndAttachment.distance(leftTorsoRodEndAttachment),
                         1e-5,
                         errorMessage);
            assertEquals(RotaryActuatorDifferentialTestHelper.getEasyGeometryRodEndLength(),
                         rightActuatorRodEndAttachment.distance(rightTorsoRodEndAttachment),
                         1e-5,
                         errorMessage);
         }
      }
   }
}
