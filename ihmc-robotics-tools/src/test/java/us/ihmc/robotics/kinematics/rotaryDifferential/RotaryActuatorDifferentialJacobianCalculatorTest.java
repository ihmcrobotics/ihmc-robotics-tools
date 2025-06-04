package us.ihmc.robotics.kinematics.rotaryDifferential;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.kinematics.jointPair.SamplingFiniteDifferenceJointPairJacobianCalculator;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public abstract class RotaryActuatorDifferentialJacobianCalculatorTest
{
   private static final int iters = 1000;

   protected abstract RotaryActuatorDifferentialKinematicsSpecifications getKinematicsSpecification(boolean rollIsFirstJoint);

   @Test
   public void testComputeForceAlongRodEnd()
   {
      Random random = new Random(1738L);

      // First test with easy geometry
      for (boolean rollIsFirstJoint : new boolean[] {true, false})
      {
         RotaryActuatorDifferentialKinematicsSpecifications kinematicsSpecifications = getKinematicsSpecification(rollIsFirstJoint);
         RotaryActuatorDifferentialForwardKinematics forwardKinematics = new RotaryActuatorDifferentialForwardKinematics(kinematicsSpecifications);
         RotaryActuatorDifferentialJacobianCalculator jacobianCalculator = new RotaryActuatorDifferentialJacobianCalculator(forwardKinematics);

         double pitchLower = rollIsFirstJoint ? kinematicsSpecifications.getSecondJointLowerLimit() : kinematicsSpecifications.getFirstJointLowerLimit();
         double pitchUpper = rollIsFirstJoint ? kinematicsSpecifications.getSecondJointUpperLimit() : kinematicsSpecifications.getFirstJointUpperLimit();
         double rollLower = rollIsFirstJoint ? kinematicsSpecifications.getFirstJointLowerLimit() : kinematicsSpecifications.getSecondJointLowerLimit();
         double rollUpper = rollIsFirstJoint ? kinematicsSpecifications.getFirstJointUpperLimit() : kinematicsSpecifications.getSecondJointUpperLimit();
         for (double pitchAngle = 0.75 * pitchLower; pitchAngle <= 0.75 * pitchUpper; pitchAngle += Math.toRadians(2.5))
         {
            for (double rollAngle = 0.6 * rollLower; rollAngle <= 0.6 * rollUpper; rollAngle += Math.toRadians(2.5))
            {
               forwardKinematics.computeActuatorPositions(rollAngle, pitchAngle);

               //               secondJointPosition.setFromReferenceFrame(forwardKinematics.getFrameAfterSecondJoint());
               //               firstJointPosition.setFromReferenceFrame(forwardKinematics.getFrameAfterFirstJoint());
               FramePoint3D rightActuatorPosition = new FramePoint3D(forwardKinematics.getRightActuatorFrame());
               FramePoint3D leftActuatorPosition = new FramePoint3D(forwardKinematics.getLeftActuatorFrame());
               FramePoint3D rightActuatorRodEndAttachmentPosition = new FramePoint3D(forwardKinematics.getRightActuatorRodEndAttachment());
               FramePoint3D leftActuatorRodEndAttachmentPosition = new FramePoint3D(forwardKinematics.getLeftActuatorRodEndAttachment());
               FramePoint3D rightBaseRodEndAttachmentPosition = new FramePoint3D(forwardKinematics.getRightBaseRodEndAttachment());
               FramePoint3D leftBaseRodEndAttachmentPosition = new FramePoint3D(forwardKinematics.getLeftBaseRodEndAttachment());

               leftActuatorRodEndAttachmentPosition.changeFrame(ReferenceFrame.getWorldFrame());
               rightActuatorRodEndAttachmentPosition.changeFrame(ReferenceFrame.getWorldFrame());
               leftActuatorPosition.changeFrame(ReferenceFrame.getWorldFrame());
               rightActuatorPosition.changeFrame(ReferenceFrame.getWorldFrame());
               leftBaseRodEndAttachmentPosition.changeFrame(ReferenceFrame.getWorldFrame());
               rightBaseRodEndAttachmentPosition.changeFrame(ReferenceFrame.getWorldFrame());

               // Compute some of the geometry vectors, which are used to compute force along the rod ends
               FrameVector3D leftVectorFromActuatorToRodEnd = new FrameVector3D();
               FrameVector3D rightVectorFromActuatorToRodEnd = new FrameVector3D();
               FrameVector3D rightRodEndVector = new FrameVector3D();
               FrameVector3D leftRodEndVector = new FrameVector3D();
               rightVectorFromActuatorToRodEnd.sub(rightActuatorRodEndAttachmentPosition, rightActuatorPosition);
               leftVectorFromActuatorToRodEnd.sub(leftActuatorRodEndAttachmentPosition, leftActuatorPosition);

               rightRodEndVector.sub(rightBaseRodEndAttachmentPosition, rightActuatorRodEndAttachmentPosition);
               leftRodEndVector.sub(leftBaseRodEndAttachmentPosition, leftActuatorRodEndAttachmentPosition);

               // get the force along the rod ends resulting from unit torques. This may have some problems
               double rightActuatorTorqueExpected = RandomNumbers.nextDouble(random, 100.0);
               double leftActuatorTorqueExpected = RandomNumbers.nextDouble(random, 100.0);
               FrameVector3D forceAlongRightRodEnd = new FrameVector3D();
               FrameVector3D forceAlongLeftRodEnd = new FrameVector3D();
               jacobianCalculator.getForceAlongRodEnd(leftActuatorTorqueExpected,
                                                      leftVectorFromActuatorToRodEnd,
                                                      leftRodEndVector,
                                                      forwardKinematics.getLeftMotorRotationAxis(),
                                                      forceAlongLeftRodEnd);
               jacobianCalculator.getForceAlongRodEnd(rightActuatorTorqueExpected,
                                                      rightVectorFromActuatorToRodEnd,
                                                      rightRodEndVector,
                                                      forwardKinematics.getRightMotorRotationAxis(),
                                                      forceAlongRightRodEnd);

               // force along the rod ends should be colinear with the rod ends
               double allowableError = 1e-10 * forceAlongRightRodEnd.norm() * rightRodEndVector.norm();
               assertEquals(forceAlongRightRodEnd.norm() * rightRodEndVector.norm(), Math.abs(forceAlongRightRodEnd.dot(rightRodEndVector)), allowableError);
               allowableError = 1e-10 * forceAlongLeftRodEnd.norm() * leftRodEndVector.norm();
               assertEquals(forceAlongLeftRodEnd.norm() * leftRodEndVector.norm(), Math.abs(forceAlongLeftRodEnd.dot(leftRodEndVector)), allowableError);

               // if the force is computed correctly, the torque back calculated should be correct
               FrameVector3D rightActuatorTorque = new FrameVector3D();
               FrameVector3D leftActuatorTorque = new FrameVector3D();
               rightActuatorTorque.cross(rightVectorFromActuatorToRodEnd, forceAlongRightRodEnd);
               leftActuatorTorque.cross(leftVectorFromActuatorToRodEnd, forceAlongLeftRodEnd);
               double rightMotorTorque = rightActuatorTorque.dot(forwardKinematics.getRightMotorRotationAxis());
               double leftMotorTorque = leftActuatorTorque.dot(forwardKinematics.getLeftMotorRotationAxis());

               String failureMessage = "Failed at roll = " + Math.toDegrees(rollAngle) + ", pitch = " + Math.toDegrees(pitchAngle);
               assertEquals(rightActuatorTorqueExpected, rightMotorTorque, 1e-6, failureMessage);
               assertEquals(leftActuatorTorqueExpected, leftMotorTorque, 1e-6, failureMessage);
            }
         }
      }
   }

   @Test
   public void testAgainstFiniteDiferenceCalculator()
   {
      double epsilon = 1e-3;
      boolean rollIsFirst = true;
      LogTools.info("Testing roll is first " + rollIsFirst);

      RotaryActuatorDifferentialKinematicsSpecifications kinematicsSpecifications = getKinematicsSpecification(rollIsFirst);
      RotaryActuatorDifferentialForwardKinematics forwardKinematics = new RotaryActuatorDifferentialForwardKinematics(kinematicsSpecifications);
      RotaryActuatorDifferentialForwardKinematics forwardKinematicsForFiniteDifference = new RotaryActuatorDifferentialForwardKinematics(kinematicsSpecifications);

      RotaryActuatorDifferentialJacobianCalculator jacobianCalculator = new RotaryActuatorDifferentialJacobianCalculator(forwardKinematics);
      SamplingFiniteDifferenceJointPairJacobianCalculator groundTruthJacobianCalculator = new SamplingFiniteDifferenceJointPairJacobianCalculator(
            forwardKinematicsForFiniteDifference,
            rollIsFirst);

      // first test it at zero
      forwardKinematics.computeActuatorPositions(0.0, 0.0);
      jacobianCalculator.computeJacobian();

      forwardKinematicsForFiniteDifference.computeActuatorPositions(0.0, 0.0);
      groundTruthJacobianCalculator.computeJacobian();

      EjmlUnitTests.assertEquals(groundTruthJacobianCalculator.getJacobianMatrix(), jacobianCalculator.getJacobianMatrix(), epsilon);
      EjmlUnitTests.assertEquals(groundTruthJacobianCalculator.getJacobianMatrixInverse(), jacobianCalculator.getJacobianMatrixInverse(), epsilon);
      EjmlUnitTests.assertEquals(groundTruthJacobianCalculator.getJacobianTransposeMatrix(), jacobianCalculator.getJacobianTransposeMatrix(), epsilon);
      EjmlUnitTests.assertEquals(groundTruthJacobianCalculator.getJacobianTransposeMatrixInverse(),
                                 jacobianCalculator.getJacobianTransposeMatrixInverse(),
                                 epsilon);

      double pitchLower = rollIsFirst ? kinematicsSpecifications.getSecondJointLowerLimit() : kinematicsSpecifications.getFirstJointLowerLimit();
      double pitchUpper = rollIsFirst ? kinematicsSpecifications.getSecondJointUpperLimit() : kinematicsSpecifications.getFirstJointUpperLimit();
      double rollLower = rollIsFirst ? kinematicsSpecifications.getFirstJointLowerLimit() : kinematicsSpecifications.getSecondJointLowerLimit();
      double rollUpper = rollIsFirst ? kinematicsSpecifications.getFirstJointUpperLimit() : kinematicsSpecifications.getSecondJointUpperLimit();

      // first test just pitch
      for (double pitchAngle = pitchLower; pitchAngle <= pitchUpper; pitchAngle += Math.toRadians(1.0))
      {
         forwardKinematics.computeActuatorPositions(0.0, pitchAngle);
         jacobianCalculator.computeJacobian();

         forwardKinematicsForFiniteDifference.computeActuatorPositions(0.0, pitchAngle);
         groundTruthJacobianCalculator.computeJacobian();

         EjmlUnitTests.assertEquals(groundTruthJacobianCalculator.getJacobianMatrix(), jacobianCalculator.getJacobianMatrix(), epsilon);
         EjmlUnitTests.assertEquals(groundTruthJacobianCalculator.getJacobianMatrixInverse(), jacobianCalculator.getJacobianMatrixInverse(), epsilon);
         EjmlUnitTests.assertEquals(groundTruthJacobianCalculator.getJacobianTransposeMatrix(), jacobianCalculator.getJacobianTransposeMatrix(), epsilon);
         EjmlUnitTests.assertEquals(groundTruthJacobianCalculator.getJacobianTransposeMatrixInverse(),
                                    jacobianCalculator.getJacobianTransposeMatrixInverse(),
                                    epsilon);
      }

      // Now test just roll
      for (double rollAngle = rollLower; rollAngle <= rollUpper; rollAngle += Math.toRadians(1.0))
      {
         forwardKinematics.computeActuatorPositions(rollAngle, 0.0);
         jacobianCalculator.computeJacobian();

         forwardKinematicsForFiniteDifference.computeActuatorPositions(rollAngle, 0.0);
         groundTruthJacobianCalculator.computeJacobian();

         EjmlUnitTests.assertEquals(groundTruthJacobianCalculator.getJacobianMatrix(), jacobianCalculator.getJacobianMatrix(), epsilon);
         EjmlUnitTests.assertEquals(groundTruthJacobianCalculator.getJacobianMatrixInverse(), jacobianCalculator.getJacobianMatrixInverse(), epsilon);
         EjmlUnitTests.assertEquals(groundTruthJacobianCalculator.getJacobianTransposeMatrix(), jacobianCalculator.getJacobianTransposeMatrix(), epsilon);
         EjmlUnitTests.assertEquals(groundTruthJacobianCalculator.getJacobianTransposeMatrixInverse(),
                                    jacobianCalculator.getJacobianTransposeMatrixInverse(),
                                    epsilon);
      }

      // Now test pitch and roll
      for (double pitchScale = -1.0; pitchScale <= 1.0; pitchScale += 0.1)
      {
         double rollScale = 1.0 - Math.abs(pitchScale);
         double rollLowerLocal = rollScale * rollLower;
         double rollUpperLocal = rollScale * rollUpper;

         double pitchAlpha = (pitchScale + 1.0) / 2.0;
         double pitchAngle = InterpolationTools.linearInterpolate(pitchLower, pitchUpper, pitchAlpha);

         for (double rollAngle = rollLowerLocal; rollAngle <= rollUpperLocal; rollAngle += Math.toRadians(1.0))
         {
            forwardKinematics.computeActuatorPositions(rollAngle, pitchAngle);
            jacobianCalculator.computeJacobian();

            forwardKinematicsForFiniteDifference.computeActuatorPositions(rollAngle, pitchAngle);
            groundTruthJacobianCalculator.computeJacobian();

            EjmlUnitTests.assertEquals(groundTruthJacobianCalculator.getJacobianMatrix(), jacobianCalculator.getJacobianMatrix(), epsilon);
            EjmlUnitTests.assertEquals(groundTruthJacobianCalculator.getJacobianMatrixInverse(), jacobianCalculator.getJacobianMatrixInverse(), epsilon);
            EjmlUnitTests.assertEquals(groundTruthJacobianCalculator.getJacobianTransposeMatrix(), jacobianCalculator.getJacobianTransposeMatrix(), epsilon);
            EjmlUnitTests.assertEquals(groundTruthJacobianCalculator.getJacobianTransposeMatrixInverse(),
                                       jacobianCalculator.getJacobianTransposeMatrixInverse(),
                                       epsilon);
         }
      }
   }

   @Test
   public void testVelocityViaFiniteDifference()
   {
      double epsilon = 1e-3;
      boolean rollIsFirst = true;
      LogTools.info("Testing roll is first " + rollIsFirst);

      RotaryActuatorDifferentialKinematicsSpecifications kinematicsSpecifications = getKinematicsSpecification(rollIsFirst);
      RotaryActuatorDifferentialForwardKinematics forwardKinematics = new RotaryActuatorDifferentialForwardKinematics(kinematicsSpecifications);
      RotaryActuatorDifferentialForwardKinematics forwardKinematicsForFiniteDifference = new RotaryActuatorDifferentialForwardKinematics(kinematicsSpecifications);

      RotaryActuatorDifferentialJacobianCalculator jacobianCalculator = new RotaryActuatorDifferentialJacobianCalculator(forwardKinematics);

      Random random = new Random(1738L);

      double pitchLower = rollIsFirst ? kinematicsSpecifications.getSecondJointLowerLimit() : kinematicsSpecifications.getFirstJointLowerLimit();
      double pitchUpper = rollIsFirst ? kinematicsSpecifications.getSecondJointUpperLimit() : kinematicsSpecifications.getFirstJointUpperLimit();
      double rollLower = rollIsFirst ? kinematicsSpecifications.getFirstJointLowerLimit() : kinematicsSpecifications.getSecondJointLowerLimit();
      double rollUpper = rollIsFirst ? kinematicsSpecifications.getFirstJointUpperLimit() : kinematicsSpecifications.getSecondJointUpperLimit();

      double pitchScale = RandomNumbers.nextDouble(random, 0.8);
      double rollScale = Math.min(1.0 - Math.abs(pitchScale), 0.8);
      double pitchLowerLocal = pitchScale * pitchLower;
      double pitchUpperLocal = pitchScale * pitchUpper;
      double rollLowerLocal = rollScale * rollLower;
      double rollUpperLocal = rollScale * rollUpper;

      double rollAngle = RandomNumbers.nextDouble(random, rollLowerLocal, rollUpperLocal);
      double pitchAngle = RandomNumbers.nextDouble(random, pitchLowerLocal, pitchUpperLocal);


      // initialize the forward kinematics test it at zero
      forwardKinematics.computeActuatorPositions(rollAngle, pitchAngle);
      jacobianCalculator.computeJacobian();

      double leftMotorAngle = forwardKinematics.getLeftMotorAngle();
      double rightMotorAngle = forwardKinematics.getRightMotorAngle();

      double dt = 1e-4;
      for (int i = 0; i < 100000; i++)
      {
         double rollVelocity = RandomNumbers.nextDouble(random, 3.0);
         double pitchVelocity = RandomNumbers.nextDouble(random, 3.0);

         double rollDelta = rollVelocity * dt;
         double pitchDelta = pitchVelocity * dt;

         double newRollAngle = rollAngle + rollDelta;
         double newPitchAngle = pitchAngle + pitchDelta;

         forwardKinematics.computeActuatorPositions(newRollAngle, newPitchAngle);
         jacobianCalculator.computeJacobian();

         double newLeftMotorAngle = forwardKinematics.getLeftMotorAngle();
         double newRightMotorAngle = forwardKinematics.getRightMotorAngle();

         double leftMotorVelocity = EuclidCoreTools.angleDifferenceMinusPiToPi(newLeftMotorAngle, leftMotorAngle) / dt;
         double rightMotorVelocity = EuclidCoreTools.angleDifferenceMinusPiToPi(newRightMotorAngle, rightMotorAngle) / dt;

         DMatrixRMaj motorVector = new DMatrixRMaj(2, 1);
         DMatrixRMaj jointVector = new DMatrixRMaj(2, 1);

         motorVector.set(jacobianCalculator.getLeftIndex(), 0, leftMotorVelocity);
         motorVector.set(jacobianCalculator.getRightIndex(), 0, rightMotorVelocity);

         CommonOps_DDRM.mult(jacobianCalculator.getJacobianMatrixInverse(), motorVector, jointVector);

         String failureMessage = "Failed at roll = " + Math.toDegrees(rollAngle) + ", pitch = " + Math.toDegrees(pitchAngle) + " on iter " + i;
         assertEquals(jointVector.get(jacobianCalculator.getRollIndex(), 0), rollVelocity, 5e-3, failureMessage);
         assertEquals(jointVector.get(jacobianCalculator.getPitchIndex(), 0), pitchVelocity, 5e-3, failureMessage);

         leftMotorAngle = newLeftMotorAngle;
         rightMotorAngle = newRightMotorAngle;

         rollAngle = newRollAngle;
         pitchAngle = newPitchAngle;
      }


   }
}
