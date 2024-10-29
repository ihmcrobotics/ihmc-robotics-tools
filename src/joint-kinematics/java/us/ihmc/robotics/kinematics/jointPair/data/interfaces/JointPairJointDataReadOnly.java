package us.ihmc.robotics.kinematics.jointPair.data.interfaces;

import us.ihmc.robotics.outputData.JointDesiredLoadMode;

/**
 * Data structure for providing access to the data from a roll/pitch joint pair.
 */
public interface JointPairJointDataReadOnly
{
   JointDataReadOnly getRollJointData();

   JointDataReadOnly getPitchJointData();

   default double getRollPosition()
   {
      return getRollJointData().getPosition();
   }

   default double getPitchPosition()
   {
      return getPitchJointData().getPosition();
   }

   default double getRollVelocity()
   {
      return getRollJointData().getVelocity();
   }

   default double getPitchVelocity()
   {
      return getPitchJointData().getVelocity();
   }

   default double getRollAcceleration()
   {
      return getRollJointData().getAcceleration();
   }

   default double getPitchAcceleration()
   {
      return getPitchJointData().getAcceleration();
   }

   default double getRollTorque()
   {
      return getRollJointData().getForce();
   }

   default double getPitchTorque()
   {
      return getPitchJointData().getForce();
   }

   default double getPitchStiffness()
   {
      return getPitchJointData().getStiffness();
   }

   default double getRollStiffness()
   {
      return getRollJointData().getStiffness();
   }

   default double getPitchDamping()
   {
      return getPitchJointData().getDamping();
   }

   default double getRollDamping()
   {
      return getRollJointData().getDamping();
   }

   default JointDesiredLoadMode getPitchLoadMode()
   {
      return getPitchJointData().getLoadMode();
   }

   default JointDesiredLoadMode getRollLoadMode()
   {
      return getRollJointData().getLoadMode();
   }

   default boolean hasPitchStiffness()
   {
      return getPitchJointData().hasStiffness();
   }

   default boolean hasRollStiffness()
   {
      return getRollJointData().hasStiffness();
   }

   default boolean hasPitchDamping()
   {
      return getPitchJointData().hasDamping();
   }

   default boolean hasRollDamping()
   {
      return getRollJointData().hasDamping();
   }

   default boolean hasPitchLoadMode()
   {
      return getPitchJointData().hasLoadMode();
   }

   default boolean hasRollLoadMode()
   {
      return getRollJointData().hasLoadMode();
   }
}