package us.ihmc.robotics.kinematics.jointPair.data.interfaces;

import us.ihmc.robotics.outputData.JointDesiredLoadMode;

public interface JointDataReadOnly
{
   double getPosition();

   double getVelocity();

   double getAcceleration();

   double getForce();

   double getStiffness();

   double getDamping();

   JointDesiredLoadMode getLoadMode();

   default boolean hasStiffness()
   {
      return !Double.isNaN(getStiffness());
   }

   default boolean hasDamping()
   {
      return !Double.isNaN(getDamping());
   }

   default boolean hasLoadMode()
   {
      return getLoadMode() != null;
   }
}