package us.ihmc.robotics.kinematics.jointPair;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.kinematics.jointPair.data.interfaces.JointPairActuatorDataBasics;
import us.ihmc.robotics.kinematics.jointPair.data.interfaces.JointPairActuatorDataReadOnly;
import us.ihmc.robotics.kinematics.jointPair.data.interfaces.JointPairJointDataBasics;
import us.ihmc.robotics.kinematics.jointPair.data.interfaces.JointPairJointDataReadOnly;
import us.ihmc.robotics.kinematics.jointPair.interfaces.JointPairForwardKinematics;
import us.ihmc.robotics.kinematics.jointPair.interfaces.JointPairInverseKinematics;
import us.ihmc.robotics.kinematics.jointPair.interfaces.JointPairJacobian;

/**
 * This class computes the actuator positions, velocities, and forces given the joint angles, velocities, and torques, and vice-versa.
 * <p>
 * This can be done as follows:
 * <p>
 * We can define the mechanism Jacobian such that
 * </p>
 * <p>
 * &tau; = J<sup>T</sup> f
 * </p>
 * <p>
 * l\u0307 = J q\u0307
 * </p>
 * <p> where J is the 2&times;2 Jacobian, f is the 2D vector of actuator forces, &tau; is the 2D vector of joint torques, q\u0307 is the 2D vector of joint
 * velocities, and l\u0307 is the 2D vector of actuator velocities.</p>
 *
 * <p>
 * To calculate this Jacobian, we can first define the axis of rotation of the i<sup>th</sup> joint as e<sub>i</sub>, the vector from the joint to the
 * attachment point of the actuator on the shin as p<sub>i</sub>, and the vector from the joint to the attachment point of the actuator on the foot as
 * r<sub>i</sub>. This means that the lever arm is r<sub>i</sub>.
 * </p>
 * <p>
 * From here, we can see that the normalized direction of the force n<sub>i</sub> can be calculated by
 * </p>
 * <p>
 * n<sub>i</sub> = (r<sub>i</sub> - p<sub>i</sub>) / || r<sub>i</sub> - p<sub>i</sub> ||  = (r<sub>i</sub> - p<sub>i</sub>) / l<sub>i</sub>
 * </p>
 * We then know, if there are n actuators at this joint, the torque about the i<sup>th</sup> joint is
 * <p>
 * &tau;<sub>i</sub> = &sum;<sub>j=0</sub><sup>n</sup> f<sub>j</sub> (r<sub>j</sub> &times; n<sub>j</sub>) &sdot; e<sub>j</sub>
 * </p>
 * We can compute r<Sub>i</Sub> and e<sub>i</sub> using the rotation matrix R(q),
 * <p>
 * e<sub>i</sub> = R(q) e<sub>0,i</sub>
 * </p>
 * <p>
 * r<sub>i</sub> = R(q) r<sub>0,i</sub>
 * </p>
 * Then, using the assembled Jacobian, we can see that
 * <p>
 * f = J<sup>-T</sup> &tau;
 * </p>
 */
public class JointPairMechanism
{
   private static final boolean FAIL_ON_NAN = false;

   private final JointPairForwardKinematics forwardKinematics;
   private final JointPairInverseKinematics inverseKinematics;
   private final JointPairJacobian jacobianCalculator;
   private final int pitchIndex;
   private final int rollIndex;
   private final int rightIndex;
   private final int leftIndex;

   public JointPairMechanism(JointPairJacobian jacobianCalculator, JointPairInverseKinematics inverseKinematics)
   {
      this.jacobianCalculator = jacobianCalculator;
      this.inverseKinematics = inverseKinematics;
      this.forwardKinematics = jacobianCalculator.getForwardKinematics();
      pitchIndex = jacobianCalculator.getPitchIndex();
      rollIndex = jacobianCalculator.getRollIndex();
      rightIndex = jacobianCalculator.getRightIndex();
      leftIndex = jacobianCalculator.getLeftIndex();
   }

   private final DMatrixRMaj jointVelocityVector = new DMatrixRMaj(2, 1);
   private final DMatrixRMaj jointTorqueVector = new DMatrixRMaj(2, 1);
   private final DMatrixRMaj jointStiffnessMatrix = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj jointDampingMatrix = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj actuatorVelocityVector = new DMatrixRMaj(2, 1);
   private final DMatrixRMaj actuatorForceVector = new DMatrixRMaj(2, 1);
   private final DMatrixRMaj actuatorStiffnessMatrix = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj actuatorDampingMatrix = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj tempMatrix = new DMatrixRMaj(2, 2);

   // NOTE: This does not copy actuator gains back up to the joints.
   public void computeJointDataFromActuator(JointPairActuatorDataReadOnly actuatorData, JointPairJointDataBasics jointDataToPack)
   {
      inverseKinematics.computeJointAngles(actuatorData.getRightPosition(), actuatorData.getLeftPosition());
      double rollAngle = inverseKinematics.getRollJointAngle();
      double pitchAngle = inverseKinematics.getPitchJointAngle();

      forwardKinematics.computeActuatorPositions(rollAngle, pitchAngle);

      jacobianCalculator.computeJacobian();
      actuatorVelocityVector.set(rightIndex, 0, actuatorData.getRightVelocity());
      actuatorVelocityVector.set(leftIndex, 0, actuatorData.getLeftVelocity());
      actuatorForceVector.set(rightIndex, 0, actuatorData.getRightForce());
      actuatorForceVector.set(leftIndex, 0, actuatorData.getLeftForce());
      CommonOps_DDRM.mult(jacobianCalculator.getJacobianMatrixInverse(), actuatorVelocityVector, jointVelocityVector);
      CommonOps_DDRM.mult(jacobianCalculator.getJacobianTransposeMatrix(), actuatorForceVector, jointTorqueVector);

      jointDataToPack.setPitchPosition(pitchAngle);
      jointDataToPack.setPitchVelocity(jointVelocityVector.get(pitchIndex, 0));
      jointDataToPack.setPitchTorque(jointTorqueVector.get(pitchIndex, 0));
      jointDataToPack.setRollPosition(rollAngle);
      jointDataToPack.setRollVelocity(jointVelocityVector.get(rollIndex, 0));
      jointDataToPack.setRollTorque(jointTorqueVector.get(rollIndex, 0));
   }

   public void computeActuatorDataFromJoint(JointPairJointDataReadOnly jointData, JointPairActuatorDataBasics actuatorDataToPack)
   {
      computeActuatorDataFromJoint(jointData, actuatorDataToPack, false);
   }

   public void computeActuatorDataFromJoint(JointPairJointDataReadOnly jointData,
                                            JointPairActuatorDataBasics actuatorDataToPack,
                                            boolean useJacobianFromComputeJointDataFromActuator)
   {
      double rollAngle = jointData.getRollPosition();
      double pitchAngle = jointData.getPitchPosition();
      double pitchVelocity = jointData.getPitchVelocity();
      double rollVelocity = jointData.getRollVelocity();
      double pitchTorque = jointData.getPitchTorque();
      double rollTorque = jointData.getRollTorque();
      checkNaN(rollAngle);
      checkNaN(pitchAngle);
      checkNaN(pitchVelocity);
      checkNaN(rollVelocity);
      checkNaN(pitchTorque);
      checkNaN(rollTorque);

      rollAngle = MathTools.clamp(forwardKinematics.getRollJointLowerLimit(), forwardKinematics.getRollJointUpperLimit());
      pitchAngle = MathTools.clamp(forwardKinematics.getPitchJointLowerLimit(), forwardKinematics.getPitchJointUpperLimit());

      forwardKinematics.computeActuatorPositions(rollAngle, pitchAngle);
      actuatorDataToPack.setRightPosition(forwardKinematics.getRightActuatorPosition());
      actuatorDataToPack.setLeftPosition(forwardKinematics.getLeftActuatorPosition());
      // Note: When doing torque control, the Jacobian should use the actual joint angles, in order to get the actuator forces correct. 
      // Therefore, do not compute the Jacobian again here when useJacobianFromComputeJointDataFromActuator is true.
      if (!useJacobianFromComputeJointDataFromActuator)
         jacobianCalculator.computeJacobian();

      jointVelocityVector.set(pitchIndex, 0, pitchVelocity);
      jointVelocityVector.set(rollIndex, 0, rollVelocity);
      jointTorqueVector.set(pitchIndex, 0, pitchTorque);
      jointTorqueVector.set(rollIndex, 0, rollTorque);

      CommonOps_DDRM.mult(jacobianCalculator.getJacobianMatrix(), jointVelocityVector, actuatorVelocityVector);
      CommonOps_DDRM.mult(jacobianCalculator.getJacobianTransposeMatrixInverse(), jointTorqueVector, actuatorForceVector);

      actuatorDataToPack.setRightVelocity(actuatorVelocityVector.get(rightIndex, 0));
      actuatorDataToPack.setRightForce(actuatorForceVector.get(rightIndex, 0));
      actuatorDataToPack.setLeftVelocity(actuatorVelocityVector.get(leftIndex, 0));
      actuatorDataToPack.setLeftForce(actuatorForceVector.get(leftIndex, 0));

      if (jointData.hasPitchStiffness() && jointData.hasRollStiffness())
      {
         jointStiffnessMatrix.set(pitchIndex, pitchIndex, jointData.getPitchStiffness());
         jointStiffnessMatrix.set(rollIndex, rollIndex, jointData.getRollStiffness());
         transformGainMatrix(jointStiffnessMatrix, jacobianCalculator, actuatorStiffnessMatrix);
         // NOTE: this ignores the off-diagonal terms
         actuatorDataToPack.setRightStiffness(actuatorStiffnessMatrix.get(rightIndex, rightIndex));
         actuatorDataToPack.setLeftStiffness(actuatorStiffnessMatrix.get(leftIndex, leftIndex));
      }
      else
      {
         actuatorDataToPack.setStiffness(Double.NaN, Double.NaN);
      }
      if (jointData.hasPitchDamping() && jointData.hasRollDamping())
      {
         jointDampingMatrix.set(pitchIndex, pitchIndex, jointData.getPitchDamping());
         jointDampingMatrix.set(rollIndex, rollIndex, jointData.getRollDamping());
         transformGainMatrix(jointDampingMatrix, jacobianCalculator, actuatorDampingMatrix);
         // NOTE: this ignores the off-diagonal terms
         actuatorDataToPack.setLeftDamping(actuatorDampingMatrix.get(rightIndex, rightIndex));
         actuatorDataToPack.setRightDamping(actuatorDampingMatrix.get(leftIndex, leftIndex));
      }
      else
      {
         actuatorDataToPack.setDamping(Double.NaN, Double.NaN);
      }
   }

   private void transformGainMatrix(DMatrixRMaj gainMatrix, JointPairJacobian jacobianCalculator, DMatrixRMaj transformedGainMatrixToPack)
   {
      CommonOps_DDRM.mult(gainMatrix, jacobianCalculator.getJacobianMatrixInverse(), tempMatrix);
      CommonOps_DDRM.mult(jacobianCalculator.getJacobianTransposeMatrixInverse(), tempMatrix, transformedGainMatrixToPack);
   }

   private void checkNaN(double number)
   {
      if (FAIL_ON_NAN && Double.isNaN(number))
         throw new RuntimeException("NaN!");
   }

   public long getJointDataFromActuatorComputationTime()
   {
      return inverseKinematics.getComputationTime();
   }

   public int getJointDataFromActuatorIterations()
   {
      return inverseKinematics.getNumberOfIterations();
   }

   public double getJacobianDeterminant()
   {
      return inverseKinematics.getJacobianDeterminant();
   }

   public IKConvergenceCondition getIKConvergenceCondition()
   {
      return inverseKinematics.getConvergenceCondition();
   }

   public double getIKResidualSquaredError()
   {
      return inverseKinematics.getResidualSquaredError();
   }

   public double getIKPitchStepSize()
   {
      return inverseKinematics.getPitchStepSize();
   }

   public double getIKRollStepSize()
   {
      return inverseKinematics.getRollStepSize();
   }

   public boolean getIKSuccessfullyWarmStarted()
   {
      return inverseKinematics.successfullyWarmStarted();
   }

   public void setIKMinimumTotalIterations(int minIterations)
   {
      inverseKinematics.setMinimumTotalIterations(minIterations);
   }
}