package us.ihmc.robotics.kinematics.jointPair.data;

import us.ihmc.robotics.kinematics.jointPair.data.interfaces.JointPairActuatorDataBasics;

public class JointPairActuatorData implements JointPairActuatorDataBasics
{
   private final JointData rightActuator = new JointData();
   private final JointData leftActuator = new JointData();

   @Override
   public JointData getRightActuator()
   {
      return rightActuator;
   }

   @Override
   public JointData getLeftActuator()
   {
      return leftActuator;
   }

   @Override
   public String toString()
   {
      return "JointPairActuatorData [rightActuator=" + rightActuator + ", leftActuator=" + leftActuator + "]";
   }
}