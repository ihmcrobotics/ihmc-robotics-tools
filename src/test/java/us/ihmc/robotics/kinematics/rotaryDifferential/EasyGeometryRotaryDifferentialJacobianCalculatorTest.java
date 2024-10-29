package us.ihmc.robotics.kinematics.rotaryDifferential;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.robotics.kinematics.jointPair.interfaces.JointPairJacobian;

import static org.junit.jupiter.api.Assertions.*;

public class EasyGeometryRotaryDifferentialJacobianCalculatorTest extends RotaryActuatorDifferentialJacobianCalculatorTest
{
   @Override
   public RotaryActuatorDifferentialKinematicsSpecifications getKinematicsSpecification(boolean rollIsFirstJoint)
   {
      return RotaryActuatorDifferentialTestHelper.createEasyGeometrySpecifications(rollIsFirstJoint);
   }

   @Test
   public void testEasyGeometry()
   {
      for (boolean rollIsFirst : new boolean[] {true, false})
      {
         // because this is a square mechanism, there are no mechanical advantage anywhere in the range of motion. This means that the Jacobian should consist
         // of ones only

         RotaryActuatorDifferentialForwardKinematics forwardKinematics = RotaryActuatorDifferentialTestHelper.createEasyGeometryForwardKinematics(rollIsFirst);
         RotaryActuatorDifferentialJacobianCalculator jacobianCalculator = new RotaryActuatorDifferentialJacobianCalculator(forwardKinematics);

         // check when pitched
         //         for (double pitchAngle = -Math.toRadians(45.0);  pitchAngle <= Math.toRadians(45.0); pitchAngle += Math.toRadians(5.0))
         //         {
         forwardKinematics.computeActuatorPositions(0.0, 0.0);
         jacobianCalculator.computeJacobian();
         DMatrixRMaj jacobian = jacobianCalculator.getJacobianMatrix();
         DMatrixRMaj jacobianExpected = new DMatrixRMaj(2, 2);
         DMatrixRMaj jacobianInverseExpected = new DMatrixRMaj(2, 2);

         // We know that the Jacobian maps from joint velocity to actuator velocity, v = J qdot
         // a positive rotation about the joint  is a negative motion in both actuators
         jacobianExpected.set(jacobianCalculator.getLeftIndex(), jacobianCalculator.getPitchIndex(), -1.0);
         jacobianExpected.set(jacobianCalculator.getRightIndex(), jacobianCalculator.getPitchIndex(), -1.0);
         // a positive rotation from the right actuator is negative about roll, whereas left is positive
         jacobianExpected.set(jacobianCalculator.getRightIndex(), jacobianCalculator.getRollIndex(), -1.0);
         jacobianExpected.set(jacobianCalculator.getLeftIndex(), jacobianCalculator.getRollIndex(), 1.0);

         // We know the inverse is true, going from actuator to joint, qdot = J^-1 v
         // a positive rotation down from either actuator is a negative motion in pitch
         jacobianInverseExpected.set(jacobianCalculator.getLeftIndex(), jacobianCalculator.getPitchIndex(), -1.0);
         jacobianInverseExpected.set(jacobianCalculator.getRightIndex(), jacobianCalculator.getPitchIndex(), -1.0);
         // a positive rotation from the right actuator is negative about roll, whereas left is positive
         jacobianInverseExpected.set(jacobianCalculator.getRightIndex(), jacobianCalculator.getRollIndex(), -1.0);
         jacobianInverseExpected.set(jacobianCalculator.getLeftIndex(), jacobianCalculator.getRollIndex(), 1.0);

         // We also know that the Jacobian transponse maps from actuator torque to joint torque, tau_joint = J^T tau_actuator
         // a positive torque down from either actuator is a negative motion in pitch
         jacobianExpected.set(jacobianCalculator.getLeftIndex(), jacobianCalculator.getPitchIndex(), -1.0);
         jacobianExpected.set(jacobianCalculator.getRightIndex(), jacobianCalculator.getPitchIndex(), -1.0);
         // a positive rotation from the right actuator is negative about roll, whereas left is positive
         jacobianExpected.set(jacobianCalculator.getRightIndex(), jacobianCalculator.getRollIndex(), -1.0);
         jacobianExpected.set(jacobianCalculator.getLeftIndex(), jacobianCalculator.getRollIndex(), 1.0);

         EjmlUnitTests.assertEquals(jacobianExpected, jacobian, 1e-5);
         //         }
      }
   }

   @Test
   public void testJacobianUsingFiniteDifferencesAtZero()
   {
      double jointEpsilonForFiniteDifference = 0.001;
      double epsilon = 1e-3;

      for (boolean rollIsFirst : new boolean[] {true, false})
      {
         RotaryActuatorDifferentialForwardKinematics forwardKinematics = RotaryActuatorDifferentialTestHelper.createEasyGeometryForwardKinematics(rollIsFirst);

         JointPairJacobian jacobianCalculator = new RotaryActuatorDifferentialJacobianCalculator(forwardKinematics);

         double roll = 0.0;
         double pitch = 0.0;

         forwardKinematics.computeActuatorPositions(roll, pitch);
         jacobianCalculator.computeJacobian();

         double insideAngle = forwardKinematics.getRightActuatorPosition();
         double outsideAngle = forwardKinematics.getLeftActuatorPosition();

         forwardKinematics.computeActuatorPositions(roll - jointEpsilonForFiniteDifference, pitch);

         double insideSlightlyRollLess = forwardKinematics.getRightActuatorPosition();
         double outsideSlightlyRollLess = forwardKinematics.getLeftActuatorPosition();

         forwardKinematics.computeActuatorPositions(roll + jointEpsilonForFiniteDifference, pitch);

         double insideSlightlyRollMore = forwardKinematics.getRightActuatorPosition();
         double outsideSlightlyRollMore = forwardKinematics.getLeftActuatorPosition();

         double jOutRollTotal = (outsideSlightlyRollMore - outsideSlightlyRollLess) / (2.0 * jointEpsilonForFiniteDifference);
         double jOutRollLess = (outsideAngle - outsideSlightlyRollLess) / (jointEpsilonForFiniteDifference);
         double jOutRollMore = (outsideSlightlyRollMore - outsideAngle) / (jointEpsilonForFiniteDifference);

         double jOutRoll = (2.0 * jOutRollTotal + jOutRollLess + jOutRollMore) / 4.0;

         double jInRollTotal = (insideSlightlyRollMore - insideSlightlyRollLess) / (2.0 * jointEpsilonForFiniteDifference);
         double jInRollLess = (insideAngle - insideSlightlyRollLess) / (jointEpsilonForFiniteDifference);
         double jInRollMore = (insideSlightlyRollMore - insideAngle) / (jointEpsilonForFiniteDifference);

         double jInRoll = (2.0 * jInRollTotal + jInRollLess + jInRollMore) / 4.0;

         forwardKinematics.computeActuatorPositions(roll, pitch - jointEpsilonForFiniteDifference);

         double insideSlightlyPitchLess = forwardKinematics.getRightActuatorPosition();
         double outsideSlightlyPitchLess = forwardKinematics.getLeftActuatorPosition();

         forwardKinematics.computeActuatorPositions(roll, pitch + jointEpsilonForFiniteDifference);

         double insideSlightlyPitchMore = forwardKinematics.getRightActuatorPosition();
         double outsideSlightlyPitchMore = forwardKinematics.getLeftActuatorPosition();

         double jInPitchTotal = (insideSlightlyPitchMore - insideSlightlyPitchLess) / (2.0 * jointEpsilonForFiniteDifference);
         double jInPitchLess = (insideAngle - insideSlightlyPitchLess) / (jointEpsilonForFiniteDifference);
         double jInPitchMore = (insideSlightlyPitchMore - insideAngle) / (jointEpsilonForFiniteDifference);

         double jInPitch = (2.0 * jInPitchTotal + jInPitchLess + jInPitchMore) / 4.0;

         double jOutPitchTotal = (outsideSlightlyPitchMore - outsideSlightlyPitchLess) / (2.0 * jointEpsilonForFiniteDifference);
         double jOutPitchLess = (outsideAngle - outsideSlightlyPitchLess) / (jointEpsilonForFiniteDifference);
         double jOutPitchMore = (outsideSlightlyPitchMore - outsideAngle) / (jointEpsilonForFiniteDifference);

         double jOutPitch = (2.0 * jOutPitchTotal + jOutPitchLess + jOutPitchMore) / 4.0;

         DMatrixRMaj jacobian = jacobianCalculator.getJacobianMatrix();

         int insideIndex = 0;
         int outsideIndex = 1;
         int rollIndex = jacobianCalculator.getRollIndex();
         int pitchIndex = jacobianCalculator.getPitchIndex();

         DMatrixRMaj jacobianExpected = new DMatrixRMaj(2, 2);
         jacobianExpected.set(outsideIndex, rollIndex, jOutRoll);
         jacobianExpected.set(outsideIndex, pitchIndex, jOutPitch);
         jacobianExpected.set(insideIndex, rollIndex, jInRoll);
         jacobianExpected.set(insideIndex, pitchIndex, jInPitch);

         EjmlUnitTests.assertEquals(jacobianExpected, jacobian, epsilon);

         assertEquals(jOutRoll, jacobian.get(outsideIndex, rollIndex), epsilon);
         assertEquals(jInRoll, jacobian.get(insideIndex, rollIndex), epsilon);

         assertEquals(jInPitch, jacobian.get(insideIndex, pitchIndex), epsilon);
         assertEquals(jOutPitch, jacobian.get(outsideIndex, pitchIndex), epsilon);
      }
   }


}