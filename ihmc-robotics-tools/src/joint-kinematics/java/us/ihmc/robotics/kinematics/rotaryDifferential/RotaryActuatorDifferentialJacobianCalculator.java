package us.ihmc.robotics.kinematics.rotaryDifferential;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.misc.TransposeAlgs_DDRM;
import org.ejml.dense.row.misc.UnrolledInverseFromMinor_DDRM;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.kinematics.jointPair.interfaces.JointPairForwardKinematics;
import us.ihmc.robotics.kinematics.jointPair.interfaces.JointPairJacobian;

/**
 * This class computes the Jacobian that maps joint angular velocities to actuator angular velocities. That is to say,
 * <p>
 * qdot<sub>actuator</sub> = J qdot<sub>joint</sub>
 * </p>
 * <p>
 * The following is then also true
 * </p>
 * <p>
 * tau<sub>joint</sub> = J<sup>T</sup> tau<sub>actuator</sub>
 * </p>
 */
public class RotaryActuatorDifferentialJacobianCalculator implements JointPairJacobian
{
   private static final int rightIndex = 0;
   private static final int leftIndex = 1;
   private final int rollIndex;
   private final int pitchIndex;

   private final FramePoint3D secondJointPosition = new FramePoint3D();
   private final FramePoint3D firstJointPosition = new FramePoint3D();
   private final FramePoint3D rightActuatorPosition = new FramePoint3D();
   private final FramePoint3D leftActuatorPosition = new FramePoint3D();

   private final FrameVector3D forceAlongRightRodEnd = new FrameVector3D();
   private final FrameVector3D forceAlongLeftRodEnd = new FrameVector3D();
   private final FrameVector3D rightActuatorForceAboutRoll = new FrameVector3D();
   private final FrameVector3D leftActuatorForceAboutRoll = new FrameVector3D();
   private final FrameVector3D rightActuatorForceAboutPitch = new FrameVector3D();
   private final FrameVector3D leftActuatorForceAboutPitch = new FrameVector3D();
   private final FrameVector3D rightMomentArm = new FrameVector3D();
   private final FrameVector3D leftMomentArm = new FrameVector3D();
   private final FramePoint3D rightPelvisRodEndAttachmentPosition = new FramePoint3D();
   private final FramePoint3D leftPelvisRodEndAttachmentPosition = new FramePoint3D();
   private final FramePoint3D rightActuatorRodEndAttachmentPosition = new FramePoint3D();
   private final FramePoint3D leftActuatorRodEndAttachmentPosition = new FramePoint3D();
   private final FrameVector3D rightVectorFromActuatorToRodEnd = new FrameVector3D();
   private final FrameVector3D leftVectorFromActuatorToRodEnd = new FrameVector3D();
   private final FrameVector3D rightRodEndVector = new FrameVector3D();
   private final FrameVector3D leftRodEndVector = new FrameVector3D();

   // These are the inputs
   private final RotaryActuatorDifferentialForwardKinematics forwardKinematics;
   private final boolean rollIsFirstJoint;

   // These are the results matrices
   private boolean inverseUpToDate = false;
   private boolean transposeUpToDate = false;
   private boolean inverseTransposeUpToDate = false;

   private final DMatrixRMaj jacobian = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj jacobianInverse = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj jacobianTranspose = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj jacobianTransposeInverse = new DMatrixRMaj(2, 2);

   public RotaryActuatorDifferentialJacobianCalculator(RotaryActuatorDifferentialForwardKinematics forwardKinematics)
   {
      this.forwardKinematics = forwardKinematics;
      this.rollIsFirstJoint = forwardKinematics.getRollIsFirstJoint();

      if (rollIsFirstJoint)
      {
         rollIndex = 0;
         pitchIndex = 1;
      }
      else
      {
         pitchIndex = 0;
         rollIndex = 1;
      }
   }

   public int getRollIndex()
   {
      return rollIndex;
   }

   public int getPitchIndex()
   {
      return pitchIndex;
   }

   public int getLeftIndex()
   {
      return leftIndex;
   }

   public int getRightIndex()
   {
      return rightIndex;
   }

   public JointPairForwardKinematics getForwardKinematics()
   {
      return forwardKinematics;
   }

   /**
    * Warning: This changes the state of the forward kinematics model slightly
    */
   public void computeJacobian()
   {
      // reset the data containers for the dependent transformations, since the Jacobian itself is going to update.
      inverseUpToDate = false;
      transposeUpToDate = false;
      inverseTransposeUpToDate = false;

      // update the kinematic position of the different points
      secondJointPosition.setFromReferenceFrame(forwardKinematics.getFrameAfterSecondJoint());
      firstJointPosition.setFromReferenceFrame(forwardKinematics.getFrameAfterFirstJoint());
      rightActuatorPosition.setFromReferenceFrame(forwardKinematics.getRightActuatorFrame());
      leftActuatorPosition.setFromReferenceFrame(forwardKinematics.getLeftActuatorFrame());
      rightActuatorRodEndAttachmentPosition.setMatchingFrame(forwardKinematics.getRightActuatorRodEndAttachment());
      leftActuatorRodEndAttachmentPosition.setMatchingFrame(forwardKinematics.getLeftActuatorRodEndAttachment());
      rightPelvisRodEndAttachmentPosition.setMatchingFrame(forwardKinematics.getRightBaseRodEndAttachment());
      leftPelvisRodEndAttachmentPosition.setMatchingFrame(forwardKinematics.getLeftBaseRodEndAttachment());

      // Compute some of the geometry vectors, which are used to compute force along the rod ends
      rightVectorFromActuatorToRodEnd.sub(rightActuatorRodEndAttachmentPosition, rightActuatorPosition);
      leftVectorFromActuatorToRodEnd.sub(leftActuatorRodEndAttachmentPosition, leftActuatorPosition);

      rightRodEndVector.sub(rightPelvisRodEndAttachmentPosition, rightActuatorRodEndAttachmentPosition);
      leftRodEndVector.sub(leftPelvisRodEndAttachmentPosition, leftActuatorRodEndAttachmentPosition);

      // get the force along the rod ends resulting from unit torques. This may have some problems
      double rightMotorTorque = 1.0;
      double leftMotorTorque = 1.0;
      getForceAlongRodEnd(leftMotorTorque, leftVectorFromActuatorToRodEnd, leftRodEndVector, forwardKinematics.getLeftMotorRotationAxis(), forceAlongLeftRodEnd);
      getForceAlongRodEnd(rightMotorTorque, rightVectorFromActuatorToRodEnd, rightRodEndVector, forwardKinematics.getRightMotorRotationAxis(), forceAlongRightRodEnd);

      FramePoint3DReadOnly rollJointPosition = rollIsFirstJoint ? firstJointPosition : secondJointPosition;
      FramePoint3DReadOnly pitchJointPosition = rollIsFirstJoint ? secondJointPosition : firstJointPosition;

      // compute the forces about the roll joints by knowing that we have the force, and computing the lever arm
      rightMomentArm.sub(rightPelvisRodEndAttachmentPosition, rollJointPosition);
      leftMomentArm.sub(leftPelvisRodEndAttachmentPosition, rollJointPosition);
      rightActuatorForceAboutRoll.cross(rightMomentArm, forceAlongRightRodEnd);
      leftActuatorForceAboutRoll.cross(leftMomentArm, forceAlongLeftRodEnd);

      // compute the forces about the pitch joints
      rightMomentArm.sub(rightPelvisRodEndAttachmentPosition, pitchJointPosition);
      leftMomentArm.sub(leftPelvisRodEndAttachmentPosition, pitchJointPosition);
      rightActuatorForceAboutPitch.cross(rightMomentArm, forceAlongRightRodEnd);
      leftActuatorForceAboutPitch.cross(leftMomentArm, forceAlongLeftRodEnd);

      // pack this data into the Jacobian transpose. The only force that goes into the joint is that along the axis, so take the dot product. The rest is
      // aborbed by the joint bearings. It's negative, however, because the force exerted by the joint is the reaction force to the force exerted by the
      // actuator
      jacobian.set(rightIndex, rollIndex, -rightActuatorForceAboutRoll.dot(forwardKinematics.getRollJointAxis()));
      jacobian.set(rightIndex, pitchIndex, -rightActuatorForceAboutPitch.dot(forwardKinematics.getPitchJointAxis()));
      jacobian.set(leftIndex, rollIndex, -leftActuatorForceAboutRoll.dot(forwardKinematics.getRollJointAxis()));
      jacobian.set(leftIndex, pitchIndex, -leftActuatorForceAboutPitch.dot(forwardKinematics.getPitchJointAxis()));

//      TransposeAlgs_DDRM.standard(jacobianTranspose, jacobian);
   }

   private final FrameVector3D torqueVector = new FrameVector3D();

   public void getForceAlongRodEnd(double actuatorTorque,
                                   FrameTuple3DReadOnly rodEndAttachmentVectorFromActuator,
                                   FrameTuple3DReadOnly rodEndVector,
                                   FrameVector3DReadOnly jointAxis,
                                   FrameVector3DBasics forceVectorToPack)
   {
      // if we apply a unit force along the rod end, we can compute the torque about the actuator. This gives us the ratio of force along the rod end
      // to the actuator. We can then scale that by the amount of torque that is actually being applied.
      torqueVector.cross(rodEndAttachmentVectorFromActuator, rodEndVector);
      double normalizedTorque = torqueVector.dot(jointAxis);
      forceVectorToPack.setIncludingFrame(rodEndVector);
      forceVectorToPack.scale(actuatorTorque / normalizedTorque);
   }

   public DMatrixRMaj getJacobianMatrix()
   {
      return jacobian;
   }

   public DMatrixRMaj getJacobianTransposeMatrix()
   {
      if (!transposeUpToDate)
      {
         TransposeAlgs_DDRM.standard(getJacobianMatrix(), jacobianTranspose);
         transposeUpToDate = true;
      }
      return jacobianTranspose;
   }

   public DMatrixRMaj getJacobianMatrixInverse()
   {
      if (!inverseUpToDate)
      {
         UnrolledInverseFromMinor_DDRM.inv2(getJacobianMatrix(), jacobianInverse, 1.0);
         inverseUpToDate = true;
      }
      return jacobianInverse;
   }

   public DMatrixRMaj getJacobianTransposeMatrixInverse()
   {
      if (!inverseTransposeUpToDate)
      {
         TransposeAlgs_DDRM.standard(getJacobianMatrixInverse(), jacobianTransposeInverse);
         inverseTransposeUpToDate = true;
      }
      return jacobianTransposeInverse;
   }
}