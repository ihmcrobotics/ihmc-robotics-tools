package us.ihmc.robotics.kinematics.rotaryDifferential;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class RotaryActuatorDifferentialTestHelper
{
   private static final double easyGeometryMechanismHeight = 0.1;
   private static final double easyGeometryLeverArmLength = 0.1;
   private static final double easyGeometryMechanismWidth = 0.2;

   public static RotaryActuatorDifferentialForwardKinematics createEasyGeometryForwardKinematics(boolean rollIsFirstJoint)
   {
      Vector3DReadOnly leftFirstJointToBaseRodEnd = getLeftFirstJointToBaseRodEnd();
      Vector3DReadOnly leftSecondJointToActuator = getLeftSecondJointToActuator();
      Vector3DReadOnly leftActuatorToRodEnd = getLeftActuatorToRodEnd();

      Vector3DReadOnly rightFirstJointToBaseRodEnd = getRightFirstJointToBaseRodEnd();
      Vector3DReadOnly rightSecondJointToActuator = getRightSecondJointToActuator();
      Vector3DReadOnly rightActuatorToRodEnd = getRightActuatorToRodEnd();

      // set the rod end length to be the same height as the actuator center so it's square.
      double leftRodEndLength = easyGeometryMechanismHeight;
      double rightRodEndLength = easyGeometryMechanismHeight;
      // set zero offset between the joints, so that the axes intersect
      Vector3D vectorFromFirstToSecondJoint = new Vector3D();

      RotaryActuatorDifferentialForwardKinematics forwardKinematics = new RotaryActuatorDifferentialForwardKinematics(leftFirstJointToBaseRodEnd,
                                                                                                                      leftSecondJointToActuator,
                                                                                                                      leftActuatorToRodEnd,
                                                                                                                      leftRodEndLength,
                                                                                                                      rightFirstJointToBaseRodEnd,
                                                                                                                      rightSecondJointToActuator,
                                                                                                                      rightActuatorToRodEnd,
                                                                                                                      rightRodEndLength,
                                                                                                                      vectorFromFirstToSecondJoint,
                                                                                                                      rollIsFirstJoint,
                                                                                                                      false,
                                                                                                                      false);
      return forwardKinematics;
   }

   public static RotaryActuatorDifferentialKinematicsSpecifications createEasyGeometrySpecifications(boolean rollIsFirstJoint)
   {
      return new RotaryActuatorDifferentialKinematicsSpecifications()
      {
         @Override
         public Vector3DReadOnly getVectorToLeftBaseRodEndAttachmentFromFirstJoint()
         {
            return getLeftFirstJointToBaseRodEnd();
         }

         @Override
         public Vector3DReadOnly getVectorToLeftActuatorAttachmentFromSecondJoint()
         {
            return getLeftSecondJointToActuator();
         }

         @Override
         public Vector3DReadOnly getVectorToLeftActuatorRodEndAttachmentFromActuator()
         {
            return getLeftActuatorToRodEnd();
         }

         @Override
         public Vector3DReadOnly getVectorToRightBaseRodEndAttachmentFromFirstJoint()
         {
            return getRightFirstJointToBaseRodEnd();
         }

         @Override
         public Vector3DReadOnly getVectorToRightActuatorAttachmentFromSecondJoint()
         {
            return getRightSecondJointToActuator();
         }

         @Override
         public Vector3DReadOnly getVectorToRightActuatorRodEndAttachmentFromActuator()
         {
            return getRightActuatorToRodEnd();
         }

         @Override
         public Vector3DReadOnly getVectorToSecondJointFromFirstJoint()
         {
            return new Vector3D();
         }

         @Override
         public boolean isTheFirstJointRoll()
         {
            return rollIsFirstJoint;
         }

         @Override
         public double getLeftTieRodLength()
         {
            return getEasyGeometryRodEndLength();
         }

         @Override
         public double getRightTieRodLength()
         {
            return getEasyGeometryRodEndLength();
         }

         @Override
         public double getFirstJointLowerLimit()
         {
            if (rollIsFirstJoint)
               return Math.toRadians(-20.0);
            return Math.toRadians(-40.0);
         }

         @Override
         public double getSecondJointLowerLimit()
         {
            if (rollIsFirstJoint)
               return Math.toRadians(-40.0);
            return Math.toRadians(-20.0);
         }

         @Override
         public double getFirstJointUpperLimit()
         {
            if (rollIsFirstJoint)
               return Math.toRadians(20.0);
            return Math.toRadians(40.0);
         }

         @Override
         public double getSecondJointUpperLimit()
         {
            if (rollIsFirstJoint)
               return Math.toRadians(40.0);
            return Math.toRadians(20.0);
         }
      };
   }

   public static Vector3DReadOnly getLeftFirstJointToBaseRodEnd()
   {
      return new Vector3D(easyGeometryLeverArmLength, easyGeometryMechanismWidth / 2.0, 0.0);
   }

   public static Vector3DReadOnly getRightFirstJointToBaseRodEnd()
   {
      return new Vector3D(easyGeometryLeverArmLength, -easyGeometryMechanismWidth / 2.0, 0.0);
   }

   public static Vector3DReadOnly getLeftSecondJointToActuator()
   {
      return new Vector3D(0.0, easyGeometryMechanismWidth / 2.0, easyGeometryMechanismHeight);
   }

   public static Vector3DReadOnly getRightSecondJointToActuator()
   {
      return new Vector3D(0.0, -easyGeometryMechanismWidth / 2.0, easyGeometryMechanismHeight);
   }

   public static Vector3DReadOnly getLeftActuatorToRodEnd()
   {
      return new Vector3D(easyGeometryLeverArmLength, 0.0, 0.0);
   }

   public static Vector3DReadOnly getRightActuatorToRodEnd()
   {
      return new Vector3D(easyGeometryLeverArmLength, 0.0, 0.0);
   }

   public static double getEasyGeometryRodEndLength()
   {
      return easyGeometryMechanismHeight;
   }
}
