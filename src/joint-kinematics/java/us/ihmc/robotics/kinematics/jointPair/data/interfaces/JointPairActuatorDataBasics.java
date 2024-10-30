package us.ihmc.robotics.kinematics.jointPair.data.interfaces;

public interface JointPairActuatorDataBasics extends JointPairActuatorDataReadOnly
{
   @Override
   JointDataBasics getRightActuator();

   @Override
   JointDataBasics getLeftActuator();

   default void set(JointPairActuatorDataReadOnly actuatorData)
   {
      getRightActuator().set(actuatorData.getRightActuator());
      getLeftActuator().set(actuatorData.getLeftActuator());
   }

   default void setPosition(double rightPosition, double leftPosition)
   {
      setRightPosition(rightPosition);
      setLeftPosition(leftPosition);
   }

   default void setVelocity(double rightVelocity, double leftVelocity)
   {
      setRightVelocity(rightVelocity);
      setLeftVelocity(leftVelocity);
   }

   default void setForce(double rightForce, double leftForce)
   {
      setRightForce(rightForce);
      setLeftForce(leftForce);
   }

   default void setStiffness(double rightStiffness, double leftStiffness)
   {
      setRightStiffness(rightStiffness);
      setLeftStiffness(leftStiffness);
   }

   default void setDamping(double leftDamping, double rightDamping)
   {
      setLeftDamping(leftDamping);
      setRightDamping(rightDamping);
   }

   default void setRightPosition(double rightPosition)
   {
      checkNaN(rightPosition);
      getRightActuator().setPosition(rightPosition);
   }

   default void setLeftPosition(double leftPosition)
   {
      checkNaN(leftPosition);
      getLeftActuator().setPosition(leftPosition);
   }

   default void setRightVelocity(double rightVelocity)
   {
      checkNaN(rightVelocity);
      getRightActuator().setVelocity(rightVelocity);
   }

   default void setLeftVelocity(double leftVelocity)
   {
      checkNaN(leftVelocity);
      getLeftActuator().setVelocity(leftVelocity);
   }

   default void setRightForce(double rightForce)
   {
      checkNaN(rightForce);
      getRightActuator().setForce(rightForce);
   }

   default void setLeftForce(double leftForce)
   {
      checkNaN(leftForce);
      getLeftActuator().setForce(leftForce);
   }

   default void setRightStiffness(double rightStiffness)
   {
      getRightActuator().setStiffness(rightStiffness);
   }

   default void setRightDamping(double rightDamping)
   {
      getRightActuator().setDamping(rightDamping);
   }

   default void setLeftDamping(double leftDamping)
   {
      getLeftActuator().setDamping(leftDamping);
   }

   default void setLeftStiffness(double leftStiffness)
   {
      getLeftActuator().setStiffness(leftStiffness);
   }

   public static void checkNaN(double number)
   {
      //      if (Double.isNaN(number))
      //         throw new RuntimeException("NaN!");
   }
}