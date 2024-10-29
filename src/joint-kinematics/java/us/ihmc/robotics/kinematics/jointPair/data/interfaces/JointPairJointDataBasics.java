package us.ihmc.robotics.kinematics.jointPair.data.interfaces;

import us.ihmc.robotics.outputData.JointDesiredLoadMode;

/**
 * Data structure for providing settable ankle joint data.
 */
public interface JointPairJointDataBasics extends JointPairJointDataReadOnly
{
   @Override
   JointDataBasics getRollJointData();

   @Override
   JointDataBasics getPitchJointData();

   default void set(JointPairJointDataReadOnly jointData)
   {
      getRollJointData().set(jointData.getRollJointData());
      getPitchJointData().set(jointData.getPitchJointData());
   }

   default void setPosition(double rollPosition, double pitchPosition)
   {
      setRollPosition(rollPosition);
      setPitchPosition(pitchPosition);
   }

   default void setVelocity(double rollVelocity, double pitchVelocity)
   {
      setRollVelocity(rollVelocity);
      setPitchVelocity(pitchVelocity);
   }

   default void setAcceleration(double rollAcceleration, double pitchAcceleration)
   {
      setRollAcceleration(rollAcceleration);
      setPitchAcceleration(pitchAcceleration);
   }

   default void setTorque(double rollTorque, double pitchTorque)
   {
      setRollTorque(rollTorque);
      setPitchTorque(pitchTorque);
   }

   default void setStiffness(double rollStiffness, double pitchStiffness)
   {
      setRollStiffness(rollStiffness);
      setPitchStiffness(pitchStiffness);
   }

   default void setDamping(double rollDamping, double pitchDamping)
   {
      setRollDamping(rollDamping);
      setPitchDamping(pitchDamping);
   }

   default void setLoadMode(JointDesiredLoadMode rollLoadMode, JointDesiredLoadMode pitchLoadMode)
   {
      setRollLoadMode(rollLoadMode);
      setPitchLoadMode(pitchLoadMode);
   }

   default void setRollPosition(double rollPosition)
   {
      checkNaN(rollPosition);
      getRollJointData().setPosition(rollPosition);
   }

   default void setPitchPosition(double pitchPosition)
   {
      checkNaN(pitchPosition);
      getPitchJointData().setPosition(pitchPosition);
   }

   default void setRollVelocity(double rollVelocity)
   {
      checkNaN(rollVelocity);
      getRollJointData().setVelocity(rollVelocity);
   }

   default void setPitchVelocity(double pitchVelocity)
   {
      checkNaN(pitchVelocity);
      getPitchJointData().setVelocity(pitchVelocity);
   }

   default void setRollAcceleration(double rollAcceleration)
   {
      checkNaN(rollAcceleration);
      getRollJointData().setAcceleration(rollAcceleration);
   }

   default void setPitchAcceleration(double pitchAcceleration)
   {
      checkNaN(pitchAcceleration);
      getPitchJointData().setAcceleration(pitchAcceleration);
   }

   default void setRollTorque(double rollTorque)
   {
      checkNaN(rollTorque);
      getRollJointData().setForce(rollTorque);
   }

   default void setPitchTorque(double pitchTorque)
   {
      checkNaN(pitchTorque);
      getPitchJointData().setForce(pitchTorque);
   }

   default void setRollStiffness(double rollStiffness)
   {
      getRollJointData().setStiffness(rollStiffness);
   }

   default void setPitchStiffness(double pitchStiffness)
   {
      getPitchJointData().setStiffness(pitchStiffness);
   }

   default void setRollDamping(double rollDamping)
   {
      getRollJointData().setDamping(rollDamping);
   }

   default void setPitchDamping(double pitchDamping)
   {
      getPitchJointData().setDamping(pitchDamping);
   }

   default void setRollLoadMode(JointDesiredLoadMode rollLoadMode)
   {
      getRollJointData().setLoadMode(rollLoadMode);
   }

   default void setPitchLoadMode(JointDesiredLoadMode pitchLoadMode)
   {
      getPitchJointData().setLoadMode(pitchLoadMode);
   }

   public static void checkNaN(double number)
   {
      //      if (Double.isNaN(number))
      //         throw new RuntimeException("NaN!");
   }
}