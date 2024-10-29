package us.ihmc.robotics.kinematics.jointPair.data.interfaces;

import us.ihmc.robotics.outputData.JointDesiredLoadMode;

public interface JointDataBasics extends JointDataReadOnly
{
   default void set(JointDataReadOnly jointData)
   {
      setPosition(jointData.getPosition());
      setVelocity(jointData.getVelocity());
      setAcceleration(jointData.getAcceleration());
      setForce(jointData.getForce());
      setStiffness(jointData.getStiffness());
      setDamping(jointData.getDamping());
      setLoadMode(jointData.getLoadMode());
   }

   void setPosition(double position);

   void setVelocity(double velocity);

   void setAcceleration(double acceleration);

   void setForce(double torque);

   void setStiffness(double stiffness);

   void setDamping(double damping);

   void setLoadMode(JointDesiredLoadMode loadMode);
}