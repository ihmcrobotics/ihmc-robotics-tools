package us.ihmc.robotics.kinematics.rotaryDifferential;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;

import static org.junit.jupiter.api.Assertions.*;

public abstract class RotaryActuatorDifferentialForwardKinematicsTest
{
   private static final double EPSILON = 1e-12;

   public abstract RotaryActuatorDifferentialKinematicsSpecifications getKinematicsSpecification();

   @Test
   public void testKinematicsMatchExpectedAtZero()
   {
      RotaryActuatorDifferentialKinematicsSpecifications kinematicsSpecifications = getKinematicsSpecification();

      RotaryActuatorDifferentialForwardKinematics forwardKinematics = new RotaryActuatorDifferentialForwardKinematics(getKinematicsSpecification(), false, false);
      forwardKinematics.computeActuatorPositions(0.0, 0.0);

      assertEquals(0.0, forwardKinematics.getRightMotorAngle(), EPSILON);
      assertEquals(0.0, forwardKinematics.getLeftMotorAngle(), EPSILON);
      assertEquals(0.0, forwardKinematics.getRightActuatorPosition(), EPSILON);
      assertEquals(0.0, forwardKinematics.getRightMotorAngle(), EPSILON);

      FramePoint3D leftBaseRodEnd = new FramePoint3D(forwardKinematics.getLeftBaseRodEndAttachment());
      FramePoint3D rightBaseRodEnd = new FramePoint3D(forwardKinematics.getRightBaseRodEndAttachment());
      leftBaseRodEnd.changeFrame(ReferenceFrame.getWorldFrame());
      rightBaseRodEnd.changeFrame(ReferenceFrame.getWorldFrame());

      // The first joint is treated as the origin. So we know then that the vector to that joint should be the absolute position.
      FramePoint3D leftBaseRodEndExpected = new FramePoint3D(ReferenceFrame.getWorldFrame(),
                                                               kinematicsSpecifications.getVectorToLeftBaseRodEndAttachmentFromFirstJoint());
      FramePoint3D rightBaseRodEndExpected = new FramePoint3D(ReferenceFrame.getWorldFrame(),
                                                                kinematicsSpecifications.getVectorToRightBaseRodEndAttachmentFromFirstJoint());

      EuclidFrameTestTools.assertEquals(leftBaseRodEndExpected, leftBaseRodEnd, EPSILON);
      EuclidFrameTestTools.assertEquals(rightBaseRodEndExpected, rightBaseRodEnd, EPSILON);

      // Test the origin positions of the actuators
      FramePoint3D leftActuatorPositionExpected = new FramePoint3D(ReferenceFrame.getWorldFrame(),
                                                                   kinematicsSpecifications.getVectorToSecondJointFromFirstJoint());
      FramePoint3D rightActuatorPositionExpected = new FramePoint3D(ReferenceFrame.getWorldFrame(),
                                                                    kinematicsSpecifications.getVectorToSecondJointFromFirstJoint());
      leftActuatorPositionExpected.add(kinematicsSpecifications.getVectorToLeftActuatorAttachmentFromSecondJoint());
      rightActuatorPositionExpected.add(kinematicsSpecifications.getVectorToRightActuatorAttachmentFromSecondJoint());

      FramePoint3D leftActuatorPosition = new FramePoint3D(forwardKinematics.getLeftActuatorFrame());
      FramePoint3D rightActuatorPosition = new FramePoint3D(forwardKinematics.getRightActuatorFrame());
      leftActuatorPosition.changeFrame(ReferenceFrame.getWorldFrame());
      rightActuatorPosition.changeFrame(ReferenceFrame.getWorldFrame());

      EuclidFrameTestTools.assertEquals(leftActuatorPositionExpected, leftActuatorPosition, EPSILON);
      EuclidFrameTestTools.assertEquals(rightActuatorPositionExpected, rightActuatorPosition, EPSILON);

      // Test the origin positions of the rod ends on the left and right actuator.
      FramePoint3D leftActuatorRodEndPositionExpected = new FramePoint3D(leftActuatorPositionExpected);
      FramePoint3D rightActuatorRodEndPositionExpected = new FramePoint3D(rightActuatorPositionExpected);
      leftActuatorRodEndPositionExpected.add(kinematicsSpecifications.getVectorToLeftActuatorRodEndAttachmentFromActuator());
      rightActuatorRodEndPositionExpected.add(kinematicsSpecifications.getVectorToRightActuatorRodEndAttachmentFromActuator());

      FramePoint3D leftActuatorRodEndPosition = new FramePoint3D(forwardKinematics.getLeftActuatorRodEndAttachment());
      FramePoint3D rightActuatorRodEndPosition = new FramePoint3D(forwardKinematics.getRightActuatorRodEndAttachment());
      leftActuatorRodEndPosition.changeFrame(ReferenceFrame.getWorldFrame());
      rightActuatorRodEndPosition.changeFrame(ReferenceFrame.getWorldFrame());

      EuclidFrameTestTools.assertEquals(leftActuatorRodEndPositionExpected, leftActuatorRodEndPosition, EPSILON);
      EuclidFrameTestTools.assertEquals(rightActuatorRodEndPositionExpected, rightActuatorRodEndPosition, EPSILON);

      // Test that the distance between the two rod ends is correct
      assertEquals(kinematicsSpecifications.getLeftTieRodLength(), leftActuatorRodEndPosition.distance(leftBaseRodEnd), EPSILON);
      assertEquals(kinematicsSpecifications.getRightTieRodLength(), rightActuatorRodEndPosition.distance(rightBaseRodEnd), EPSILON);
   }
}
