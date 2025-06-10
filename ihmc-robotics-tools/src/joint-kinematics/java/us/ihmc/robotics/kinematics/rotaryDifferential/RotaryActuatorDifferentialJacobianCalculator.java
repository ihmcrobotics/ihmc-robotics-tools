package us.ihmc.robotics.kinematics.rotaryDifferential;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.misc.TransposeAlgs_DDRM;
import org.ejml.dense.row.misc.UnrolledInverseFromMinor_DDRM;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
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

   // Kinematic positions
   private final FramePoint3D p = new FramePoint3D();
   private final FramePoint3D o = new FramePoint3D();
   private final FramePoint3D a2 = new FramePoint3D();
   private final FramePoint3D a1 = new FramePoint3D();
   private final FramePoint3D c2 = new FramePoint3D();
   private final FramePoint3D c1 = new FramePoint3D();
   private final FramePoint3D b2 = new FramePoint3D();
   private final FramePoint3D b1 = new FramePoint3D();

   // Vectors mapping positions
   private final Vector3D vPO = new Vector3D();
   private final Vector3D vOC1 = new Vector3D();
   private final Vector3D vOC2 = new Vector3D();
   private final Vector3D vB1C1 = new Vector3D();
   private final Vector3D vB2C2 = new Vector3D();
   private final Vector3D vA1B1 = new Vector3D();
   private final Vector3D vA2B2 = new Vector3D();

   // Cross product terms
   private final Vector3D vOC1_cross_vB1C1 = new Vector3D();
   private final Vector3D vOC2_cross_vB2C2 = new Vector3D();
   private final Vector3D vPO_cross_vB1C1 = new Vector3D();
   private final Vector3D vPO_cross_vB2C2 = new Vector3D();
   private final Vector3D vA1B1_cross_vB1C1 = new Vector3D();
   private final Vector3D vA2B2_cross_vB2C2 = new Vector3D();

   private final DMatrixRMaj Jleft = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj JrightInverse = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj jacobianTemp = new DMatrixRMaj(2, 2);

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


      // In this class, we will refer to the first joint as O, the second joint as P, the actuator location as A#, the rod end base attachment as C#,
      // and the rod end actuator attachment as B#. The # refers to the left or right side, where 1 is the left side and 2 is the right side.
      // The rotation axis of the first joint is rO, the second joint is rP, and the actuator is rA#.

      // update the kinematic position of the different points
      p.setFromReferenceFrame(forwardKinematics.getFrameAfterSecondJoint());
      o.setFromReferenceFrame(forwardKinematics.getFrameAfterFirstJoint());
      a1.setFromReferenceFrame(forwardKinematics.getLeftActuatorFrame());
      a2.setFromReferenceFrame(forwardKinematics.getRightActuatorFrame());
      b1.setMatchingFrame(forwardKinematics.getLeftActuatorRodEndAttachment());
      b2.setMatchingFrame(forwardKinematics.getRightActuatorRodEndAttachment());
      c1.setMatchingFrame(forwardKinematics.getLeftBaseRodEndAttachment());
      c2.setMatchingFrame(forwardKinematics.getRightBaseRodEndAttachment());
      Vector3DReadOnly rA1 = forwardKinematics.getLeftMotorRotationAxis();
      Vector3DReadOnly rA2 = forwardKinematics.getRightMotorRotationAxis();
      Vector3DReadOnly rO = forwardKinematics.getRollIsFirstJoint() ? forwardKinematics.getRollJointAxis() : forwardKinematics.getPitchJointAxis();
      Vector3DReadOnly rP = forwardKinematics.getRollIsFirstJoint() ? forwardKinematics.getPitchJointAxis() : forwardKinematics.getRollJointAxis();

      // Compute some of the geometry vectors, which are used to compute the Jacobian.
      vPO.sub(o, p);
      vOC1.sub(c1, o);
      vOC2.sub(c2, o);
      vB1C1.sub(c1, b1);
      vB2C2.sub(c2, b2);
      vA1B1.sub(b1, a1);
      vA2B2.sub(b2, a2);


      //////////////////// We can now set up the Jacobian matrices
      // Jleft * jointRate = Jright * actuatorRate

      // Take the cross products
      vOC1_cross_vB1C1.cross(vOC1, vB1C1);
      vOC2_cross_vB2C2.cross(vOC2, vB2C2);
      vPO_cross_vB1C1.cross(vPO, vB1C1);
      vPO_cross_vB2C2.cross(vPO, vB2C2);
      vA1B1_cross_vB1C1.cross(vA1B1, vB1C1);
      vA2B2_cross_vB2C2.cross(vA2B2, vB2C2);

      Jleft.set(0, 0, rO.dot(vOC1_cross_vB1C1));
      Jleft.set(0, 1, rP.dot(vOC1_cross_vB1C1) + rP.dot(vPO_cross_vB1C1));
      Jleft.set(1, 0, rO.dot(vOC2_cross_vB2C2));
      Jleft.set(1, 1, rP.dot(vOC2_cross_vB2C2) + rP.dot(vPO_cross_vB2C2));

      JrightInverse.zero();
      JrightInverse.set(0, 0, 1.0 / rA1.dot(vA1B1_cross_vB1C1));
      JrightInverse.set(1, 1, 1.0 / rA2.dot(vA2B2_cross_vB2C2));

      CommonOps_DDRM.mult(-1.0, JrightInverse, Jleft, jacobianTemp);

      jacobian.set(rightIndex, rollIndex, jacobianTemp.get(1, forwardKinematics.getRollIsFirstJoint() ? 0 : 1));
      jacobian.set(leftIndex, rollIndex, jacobianTemp.get(0, forwardKinematics.getRollIsFirstJoint() ? 0 : 1));
      jacobian.set(rightIndex, pitchIndex, jacobianTemp.get(1, forwardKinematics.getRollIsFirstJoint() ? 1 : 0));
      jacobian.set(leftIndex, pitchIndex, jacobianTemp.get(0, forwardKinematics.getRollIsFirstJoint() ? 1 : 0));
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