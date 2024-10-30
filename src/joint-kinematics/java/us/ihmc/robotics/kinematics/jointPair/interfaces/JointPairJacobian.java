package us.ihmc.robotics.kinematics.jointPair.interfaces;

import org.ejml.data.DMatrixRMaj;

public interface JointPairJacobian
{
   void computeJacobian();

   int getPitchIndex();

   int getRollIndex();

   int getRightIndex();

   int getLeftIndex();

   JointPairForwardKinematics getForwardKinematics();

   DMatrixRMaj getJacobianMatrix();

   DMatrixRMaj getJacobianTransposeMatrix();

   DMatrixRMaj getJacobianMatrixInverse();

   DMatrixRMaj getJacobianTransposeMatrixInverse();
}