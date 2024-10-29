package us.ihmc.robotics.kinematics.jointPair;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.misc.TransposeAlgs_DDRM;
import org.ejml.dense.row.misc.UnrolledInverseFromMinor_DDRM;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.robotics.kinematics.jointPair.interfaces.JointPairForwardKinematics;
import us.ihmc.robotics.kinematics.jointPair.interfaces.JointPairJacobian;

import java.util.Random;

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
 * It does so via finite difference of many samples. This is not computationally efficient
 */
public class SamplingFiniteDifferenceJointPairJacobianCalculator implements JointPairJacobian
{
   private final Random random = new Random(1738L);
   private static final double samplingEpsilon = 2e-3;
   private static final int samples = 200;

   private static final int rightIndex = 0;
   private static final int leftIndex = 1;
   private final int rollIndex;
   private final int pitchIndex;

   private double originalRoll;
   private double originalPitch;
   private double originalInsideAngle;
   private double originalOutsideAngle;
   private final DMatrixRMaj jacobianTemp = new DMatrixRMaj(2, 2);

   private final DMatrixRMaj jacobian = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj jacobianInverse = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj jacobianTranspose = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj jacobianTransposeInverse = new DMatrixRMaj(2, 2);

   private final JointPairForwardKinematics forwardKinematics;

   private boolean inverseUpToDate = false;
   private boolean transposeUpToDate = false;
   private boolean inverseTransposeUpToDate = false;

   public SamplingFiniteDifferenceJointPairJacobianCalculator(JointPairForwardKinematics forwardKinematics, boolean rollIsFirstJoint)
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
   public int getLeftIndex()
   {
      return leftIndex;
   }

   @Override
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
      inverseUpToDate = false;
      transposeUpToDate = false;
      inverseTransposeUpToDate = false;

      originalRoll = forwardKinematics.getRollAngle();
      originalPitch = forwardKinematics.getPitchAngle();

      originalInsideAngle = forwardKinematics.getRightActuatorPosition();
      originalOutsideAngle = forwardKinematics.getLeftActuatorPosition();

      jacobian.zero();

      for (int sample = 0; sample < samples; sample++)
      {
         double epsilon = RandomNumbers.nextDouble(random, samplingEpsilon);
         computeJacobian(epsilon);

         CommonOps_DDRM.addEquals(jacobian, jacobianTemp);
      }

      CommonOps_DDRM.scale(1.0 / samples, jacobian);

      // reset to the original position
      forwardKinematics.computeActuatorPositions(originalRoll, originalPitch);
   }

   private void computeJacobian(double epsilon)
   {
      forwardKinematics.computeActuatorPositions(originalRoll + epsilon, originalPitch);

      double insideSlightlyRollLess = forwardKinematics.getRightActuatorPosition();
      double outsideSlightlyRollLess = forwardKinematics.getLeftActuatorPosition();

      forwardKinematics.computeActuatorPositions(originalRoll, originalPitch + epsilon);

      double insideSlightlyPitchLess = forwardKinematics.getRightActuatorPosition();
      double outsideSlightlyPitchLess = forwardKinematics.getLeftActuatorPosition();

      double jInPitch = (insideSlightlyPitchLess - originalInsideAngle) / epsilon;
      double jOutPitch = (outsideSlightlyPitchLess - originalOutsideAngle) / epsilon;

      double jInRoll = (insideSlightlyRollLess - originalInsideAngle) / epsilon;
      double jOutRoll = (outsideSlightlyRollLess - originalOutsideAngle) / epsilon;

      jacobianTemp.set(rightIndex, rollIndex, jInRoll);
      jacobianTemp.set(leftIndex, rollIndex, jOutRoll);
      jacobianTemp.set(rightIndex, pitchIndex, jInPitch);
      jacobianTemp.set(leftIndex, pitchIndex, jOutPitch);
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
