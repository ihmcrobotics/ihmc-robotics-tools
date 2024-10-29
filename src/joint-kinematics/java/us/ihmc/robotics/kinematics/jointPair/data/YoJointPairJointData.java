package us.ihmc.robotics.kinematics.jointPair.data;

import us.ihmc.robotics.kinematics.jointPair.data.interfaces.JointDataBasics;
import us.ihmc.robotics.kinematics.jointPair.data.interfaces.JointPairJointDataBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Yo variablized data structure for providing the ankle joint data.
 */
public class YoJointPairJointData implements JointPairJointDataBasics
{
   private final YoJointData rollJoint, pitchJoint;

   public YoJointPairJointData(String prefix, boolean createGainVariables, YoRegistry registry)
   {
      rollJoint = new YoJointData(prefix + "_Roll", createGainVariables, registry);
      pitchJoint = new YoJointData(prefix + "_Pitch", createGainVariables, registry);
   }

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
}