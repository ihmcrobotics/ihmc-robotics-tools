package us.ihmc.robotics.kinematics.jointPair.data;

import us.ihmc.robotics.kinematics.jointPair.data.interfaces.JointDataBasics;
import us.ihmc.robotics.kinematics.jointPair.data.interfaces.JointPairActuatorDataBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoJointPairActuatorData implements JointPairActuatorDataBasics
{
   private final YoJointData rightActuator, leftActuator;

   public YoJointPairActuatorData(String prefix, boolean createGainVariables, YoRegistry registry)
   {
      rightActuator = new YoJointData(prefix + "_Right", createGainVariables, registry);
      leftActuator = new YoJointData(prefix + "_Left", createGainVariables, registry);
   }

   @Override
   public JointDataBasics getRightActuator()
   {
      return rightActuator;
   }

   @Override
   public JointDataBasics getLeftActuator()
   {
      return leftActuator;
   }
}