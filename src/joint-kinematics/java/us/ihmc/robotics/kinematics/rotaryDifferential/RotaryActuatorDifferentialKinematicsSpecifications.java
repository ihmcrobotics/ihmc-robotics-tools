package us.ihmc.robotics.kinematics.rotaryDifferential;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * These are the kinematics that describe a differential joint where there are two rotary actuators that are rigidly attached to the child link. For this
 * definition, all things are relative to the base, which does not contain the actuators, and is considered stationary.
 * <p>
 * For example, if this describes a spine differential where the actuators are located in the torso, the base link is the pelvis, while the child link is the
 * torso.
 * </p>
 * <p>
 * Alternatively, if this describes an ankle differential, where the actuators are located in the shin, the base link is the foot, while the child link is the
 * shin.
 * </p>
 * <p>
 * The first joint is the first joint that is rigidly attached to the base. The second joint follows it, and is rigidly attached to the child.
 * </p>
 */
public interface RotaryActuatorDifferentialKinematicsSpecifications
{
   /**
    * This is the vector in 3D from the first joint to the left rod end attachment in the base link.
    */
   Vector3DReadOnly getVectorToLeftBaseRodEndAttachmentFromFirstJoint();

   /**
    * This is the vector in 3D from the second joint to the center of the left actuator face in the child link.
    */
   Vector3DReadOnly getVectorToLeftActuatorAttachmentFromSecondJoint();

   /**
    * This is the vector in 3D from the center of the left actuator face to the rod end that is attached to the actuator.
    */
   Vector3DReadOnly getVectorToLeftActuatorRodEndAttachmentFromActuator();

   /**
    * This is the vector in 3D from the first joint to the right rod end attachment in the base link.
    */
   Vector3DReadOnly getVectorToRightBaseRodEndAttachmentFromFirstJoint();

   /**
    * This is the vector in 3D from the second joint to the center of the right actuator face in the child link.
    */
   Vector3DReadOnly getVectorToRightActuatorAttachmentFromSecondJoint();

   /**
    * This is the vector in 3D from the center of the left actuator face to the rod end that is attached to the actuator.
    */
   Vector3DReadOnly getVectorToRightActuatorRodEndAttachmentFromActuator();

   Vector3DReadOnly getVectorToSecondJointFromFirstJoint();

   boolean isTheFirstJointRoll();

   /**
    * This is the total length of the left tie rod, which goes from the center of one rod end to the center of the other rod end.
    */
   double getLeftTieRodLength();

   /**
    * This is the total length of the right tie rod, which goes from the center of one rod end to the center of the other rod end.
    */
   double getRightTieRodLength();

   /**
    * @return lower joint limit in radians for the first joint, assuming the second joint is ignored (box constraints).
    */
   double getFirstJointLowerLimit();

   /**
    * @return upper joint limit in radians for the first joint, assuming the second joint is ignored (box constraints).
    */
   double getFirstJointUpperLimit();

   /**
    * @return lower joint limit in radians for the second joint, assuming the first joint is ignored (box constraints).
    */
   double getSecondJointLowerLimit();

   /**
    * @return upper joint limit in radians for the second joint, assuming the first joint is ignored (box constraints).
    */
   double getSecondJointUpperLimit();
}
