package us.ihmc.robotics.kinematics.jointPair;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface DifferentialKinematicsSpecification
{
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

   boolean isTheFirstJointRoll();

   Vector3DReadOnly getVectorToSecondJointFromFirstJoint();

}
