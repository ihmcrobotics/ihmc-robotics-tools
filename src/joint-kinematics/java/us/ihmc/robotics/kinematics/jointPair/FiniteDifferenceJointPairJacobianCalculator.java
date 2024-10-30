package us.ihmc.robotics.kinematics.jointPair;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.misc.TransposeAlgs_DDRM;
import org.ejml.dense.row.misc.UnrolledInverseFromMinor_DDRM;
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
 * It does so via finite difference. This is extremely computationally intensive, so this class should be used only for testing purposes or if there is no
 * closed form Jacobian available.
 */
public class FiniteDifferenceJointPairJacobianCalculator implements JointPairJacobian
{
   private static final int rightIndex = 0;
   private static final int leftIndex = 1;
   private final int rollIndex;
   private final int pitchIndex;

   private final DMatrixRMaj jacobian = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj jacobianInverse = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj jacobianTranspose = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj jacobianTransposeInverse = new DMatrixRMaj(2, 2);

   private final JointPairForwardKinematics forwardKinematics;

   private boolean inverseUpToDate = false;
   private boolean transposeUpToDate = false;
   private boolean inverseTransposeUpToDate = false;

   public FiniteDifferenceJointPairJacobianCalculator(JointPairForwardKinematics forwardKinematics, boolean rollIsFirstJoint)
   {
      this.forwardKinematics = forwardKinematics;

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

   @Override
   public int getRollIndex()
   {
      return rollIndex;
   }

   @Override
   public int getPitchIndex()
   {
      return pitchIndex;
   }

   @Override
   public int getRightIndex()
   {
      return rightIndex;
   }

   @Override
   public int getLeftIndex()
   {
      return leftIndex;
   }

   @Override
   public JointPairForwardKinematics getForwardKinematics()
   {
      return forwardKinematics;
   }

   /**
    * Warning: This changes the state of the forward kinematics model slightly
    */
   public void computeJacobian()
   {
      inverseUpToDate = false;
      transposeUpToDate = false;
      inverseTransposeUpToDate = false;

      double roll = forwardKinematics.getRollAngle();
      double pitch = forwardKinematics.getPitchAngle();

      double rightPosition = forwardKinematics.getRightActuatorPosition();
      double leftPosition = forwardKinematics.getLeftActuatorPosition();

      double jointEpsilonForFiniteDifference = 2e-3;

      forwardKinematics.computeActuatorPositions(roll - jointEpsilonForFiniteDifference, pitch);

      double rightSlightlyRollLess = forwardKinematics.getRightActuatorPosition();
      double leftSlightlyRollLess = forwardKinematics.getLeftActuatorPosition();

      forwardKinematics.computeActuatorPositions(roll, pitch - jointEpsilonForFiniteDifference);

      double rightSlightlyPitchLess = forwardKinematics.getRightActuatorPosition();
      double leftSlightlyPitchLess = forwardKinematics.getLeftActuatorPosition();

      double jRightPitch = (rightPosition - rightSlightlyPitchLess) / jointEpsilonForFiniteDifference;
      double jLeftPitch = (leftPosition - leftSlightlyPitchLess) / jointEpsilonForFiniteDifference;

      double jRightRoll = (rightPosition - rightSlightlyRollLess) / jointEpsilonForFiniteDifference;
      double jLeftRoll = (leftPosition - leftSlightlyRollLess) / jointEpsilonForFiniteDifference;

      jacobian.set(rightIndex, rollIndex, jRightRoll);
      jacobian.set(leftIndex, rollIndex, jLeftRoll);
      jacobian.set(rightIndex, pitchIndex, jRightPitch);
      jacobian.set(leftIndex, pitchIndex, jLeftPitch);

      // reset to the original position
      forwardKinematics.computeActuatorPositions(roll, pitch);
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
