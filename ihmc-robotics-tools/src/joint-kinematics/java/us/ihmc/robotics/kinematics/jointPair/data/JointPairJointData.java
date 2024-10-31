package us.ihmc.robotics.kinematics.jointPair.data;

import us.ihmc.robotics.kinematics.jointPair.data.interfaces.JointDataBasics;
import us.ihmc.robotics.kinematics.jointPair.data.interfaces.JointPairJointDataBasics;

/**
 * Data structure for providing the ankle joint data.
 */
public class JointPairJointData implements JointPairJointDataBasics
{
   private final JointData rollJoint = new JointData();
   private final JointData pitchJoint = new JointData();

   @Override
   public JointDataBasics getRollJointData()
   {
      return rollJoint;
   }

   @Override
   public JointDataBasics getPitchJointData()
   {
      return pitchJoint;
   }

   @Override
   public String toString()
   {
      return "JointPairJointData [rollJoint=" + rollJoint + ", pitchJoint=" + pitchJoint + "]";
   }
}