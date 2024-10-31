package us.ihmc.robotics.kinematics.jointPair.data.interfaces;

public interface JointPairActuatorDataReadOnly
{
   JointDataReadOnly getRightActuator();

   JointDataReadOnly getLeftActuator();

   default double getRightPosition()
   {
      return getRightActuator().getPosition();
   }

   default double getLeftPosition()
   {
      return getLeftActuator().getPosition();
   }

   default double getRightVelocity()
   {
      return getRightActuator().getVelocity();
   }

   default double getLeftVelocity()
   {
      return getLeftActuator().getVelocity();
   }

   default double getRightForce()
   {
      return getRightActuator().getForce();
   }

   default double getLeftForce()
   {
      return getLeftActuator().getForce();
   }

   default double getRightStiffness()
   {
      return getRightActuator().getStiffness();
   }

   default double getLeftStiffness()
   {
      return getLeftActuator().getStiffness();
   }

   default double getRightDamping()
   {
      return getRightActuator().getDamping();
   }

   default double getLeftDamping()
   {
      return getLeftActuator().getDamping();
   }

   default boolean hasRightStiffness()
   {
      return getRightActuator().hasStiffness();
   }

   default boolean hasLeftStiffness()
   {
      return getLeftActuator().hasStiffness();
   }

   default boolean hasRightDamping()
   {
      return getRightActuator().hasDamping();
   }

   default boolean hasLeftDamping()
   {
      return getLeftActuator().hasDamping();
   }
}