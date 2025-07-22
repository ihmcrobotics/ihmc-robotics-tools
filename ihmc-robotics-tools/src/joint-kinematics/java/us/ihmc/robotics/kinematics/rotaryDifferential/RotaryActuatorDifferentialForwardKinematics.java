package us.ihmc.robotics.kinematics.rotaryDifferential;

import us.ihmc.robotics.kinematics.TrigonometricApproximation;
import us.ihmc.robotics.kinematics.jointPair.DifferentialKinematicsSpecification;
import us.ihmc.robotics.kinematics.jointPair.interfaces.JointPairForwardKinematics;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * This class computes the vector of the actuator, as well as the corresponding actuator length.
 */
public class RotaryActuatorDifferentialForwardKinematics implements JointPairForwardKinematics
{
   static int trigonometricApproximationForwardDepth = 6;
   static int trigonometricApproximationInverseDepth = 3;

   private static final Vector3DReadOnly pitchAxis = new Vector3D(0.0, 1.0, 0.0); // e1 in the above equations
   private static final Vector3DReadOnly rollAxis = new Vector3D(1.0, 0.0, 0.0);  // e2 in the above equations

   private final double rollUpperLimit;
   private final double rollLowerLimit;
   private final double pitchUpperLimit;
   private final double pitchLowerLimit;

   private final FrameVector3D pitchJointAxis = new FrameVector3D();
   private final FrameVector3D rollJointAxis = new FrameVector3D();
   private final RotationMatrix rollRotation = new RotationMatrix();
   private final RotationMatrix pitchRotation = new RotationMatrix();

   private final FrameVector3D leftMotorRotationAxis = new FrameVector3D();
   private final FrameVector3D rightMotorRotationAxis = new FrameVector3D();

   final ReferenceFrame baseFrame = ReferenceFrame.getWorldFrame();
   final ReferenceFrame frameAfterFirstJoint;
   final ReferenceFrame frameAfterSecondJoint;
   final ReferenceFrame leftActuatorFrame;
   final ReferenceFrame rightActuatorFrame;
   final RotaryActuatorDifferentialMotorMechanism leftActuatorMechanism;
   final RotaryActuatorDifferentialMotorMechanism rightActuatorMechanism;
   private final FramePoint3D pitchInRollFrame = new FramePoint3D();
   private final Vector3DReadOnly vectorToSecondJointFromFirstJoint;
   private double rollPosition;
   private double pitchPosition;
   private final boolean useTrigonometricForwardApproximations;
   private final boolean rollIsFirstJoint;

   public RotaryActuatorDifferentialForwardKinematics(RotaryActuatorDifferentialKinematicsSpecifications kinematicsSpecifications)
   {
      this(kinematicsSpecifications, false, false);
   }

   public RotaryActuatorDifferentialForwardKinematics(RotaryActuatorDifferentialKinematicsSpecifications kinematicsSpecifications,
                                                      boolean useTrigonometricForwardApproximations,
                                                      boolean useTrigonometricInverseApproximations)
   {
      this(kinematicsSpecifications.getVectorToLeftBaseRodEndAttachmentFromFirstJoint(),
           kinematicsSpecifications.getVectorToLeftActuatorAttachmentFromSecondJoint(),
           kinematicsSpecifications.getVectorToLeftActuatorRodEndAttachmentFromActuator(),
           kinematicsSpecifications.getLeftTieRodLength(),
           kinematicsSpecifications.getVectorToRightBaseRodEndAttachmentFromFirstJoint(),
           kinematicsSpecifications.getVectorToRightActuatorAttachmentFromSecondJoint(),
           kinematicsSpecifications.getVectorToRightActuatorRodEndAttachmentFromActuator(),
           kinematicsSpecifications.getRightTieRodLength(),
           kinematicsSpecifications.getVectorToSecondJointFromFirstJoint(),
           kinematicsSpecifications.isTheFirstJointRoll(),
           kinematicsSpecifications.isTheFirstJointRoll() ? kinematicsSpecifications.getFirstJointLowerLimit() :
                 kinematicsSpecifications.getSecondJointLowerLimit(),
           kinematicsSpecifications.isTheFirstJointRoll() ? kinematicsSpecifications.getFirstJointUpperLimit() :
                 kinematicsSpecifications.getSecondJointUpperLimit(),
           kinematicsSpecifications.isTheFirstJointRoll() ? kinematicsSpecifications.getSecondJointLowerLimit() :
                 kinematicsSpecifications.getFirstJointLowerLimit(),
           kinematicsSpecifications.isTheFirstJointRoll() ? kinematicsSpecifications.getSecondJointUpperLimit() :
                 kinematicsSpecifications.getFirstJointUpperLimit(),
           useTrigonometricForwardApproximations,
           useTrigonometricInverseApproximations);
   }

   public RotaryActuatorDifferentialForwardKinematics(Vector3DReadOnly vectorToLeftBaseRodEndAttachmentFromFirstJoint,
                                                      Vector3DReadOnly vectorToLeftActuatorAttachmentFromSecondJoint,
                                                      Vector3DReadOnly vectorToLeftActuatorRodEndAttachmentFromActuator,
                                                      double leftTieRodLength,
                                                      Vector3DReadOnly vectorToRightBaseRodEndAttachmentFromFirstJoint,
                                                      Vector3DReadOnly vectorToRightActuatorAttachmentFromSecondJoint,
                                                      Vector3DReadOnly vectorToRightActuatorRodEndAttachmentFromActuator,
                                                      double rightTieRodLength,
                                                      Vector3DReadOnly vectorToSecondJointFromFirstJoint,
                                                      boolean rollIsFirstJoint,
                                                      double rollLowerLimit,
                                                      double rollUpperLimit,
                                                      double pitchLowerLimit,
                                                      double pitchUpperLimit,
                                                      boolean useTrigonometricForwardApproximations,
                                                      boolean useTrigonometricInverseApproximations)
   {
      // This is the translation vector from the roll joint (which is the first joint) to the pitch joint (which is the second joint). It is expressed in the
      // frame after the roll joint, such that it is a constant offset vector.
      this.vectorToSecondJointFromFirstJoint = vectorToSecondJointFromFirstJoint;
      this.useTrigonometricForwardApproximations = useTrigonometricForwardApproximations;
      this.rollIsFirstJoint = rollIsFirstJoint;
      this.rollLowerLimit = rollLowerLimit;
      this.rollUpperLimit = rollUpperLimit;
      this.pitchLowerLimit = pitchLowerLimit;
      this.pitchUpperLimit = pitchUpperLimit;
      // this is the frame after the first joint.
      frameAfterFirstJoint = new ReferenceFrame("frameAfterFirstJoint", baseFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            if (rollIsFirstJoint)
               transformToParent.getRotation().set(rollRotation);
            else
               transformToParent.getRotation().set(pitchRotation);
         }
      };
      // this is the frame after the second joint, and is the child of the first joint. The transform also accounts for the static translation from
      // vectorToSecondJointFromFirstJoint
      frameAfterSecondJoint = new ReferenceFrame("frameAfterSecondJoint", frameAfterFirstJoint)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            if (rollIsFirstJoint)
               transformToParent.set(pitchRotation, vectorToSecondJointFromFirstJoint);
            else
               transformToParent.set(rollRotation, vectorToSecondJointFromFirstJoint);
         }
      };
      // This is the frame centered at the origin of the face of the left actuator, which is coincident with the axis of rotation. It is a child of the second
      // joint, as the actuator is located in the torso.
      leftActuatorFrame = new ReferenceFrame("leftActuatorFrame", frameAfterSecondJoint)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.getTranslation().set(vectorToLeftActuatorAttachmentFromSecondJoint);
         }
      };
      // This is the frame centered at the origin of the face of the right actuator, which is coincident with the axis of rotation. It is a child of the second
      // joint, as the actuator is located in the torso.
      rightActuatorFrame = new ReferenceFrame("rightActuatorFrame", frameAfterSecondJoint)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.getTranslation().set(vectorToRightActuatorAttachmentFromSecondJoint);
         }
      };
      frameAfterFirstJoint.update();
      frameAfterSecondJoint.update();
      leftActuatorFrame.update();
      rightActuatorFrame.update();
      leftActuatorMechanism = new RotaryActuatorDifferentialMotorMechanism(frameAfterSecondJoint,
                                                                           leftActuatorFrame,
                                                                           vectorToLeftBaseRodEndAttachmentFromFirstJoint,
                                                                           vectorToLeftActuatorRodEndAttachmentFromActuator,
                                                                           leftTieRodLength,
                                                                           useTrigonometricInverseApproximations);
      rightActuatorMechanism = new RotaryActuatorDifferentialMotorMechanism(frameAfterSecondJoint,
                                                                            rightActuatorFrame,
                                                                            vectorToRightBaseRodEndAttachmentFromFirstJoint,
                                                                            vectorToRightActuatorRodEndAttachmentFromActuator,
                                                                            rightTieRodLength,
                                                                            useTrigonometricInverseApproximations);
   }

   double getLeftActuatorRotation()
   {
      return leftActuatorMechanism.getActuatorRotation();
   }

   double getRightActuatorRotation()
   {
      return rightActuatorMechanism.getActuatorRotation();
   }

   public boolean getRollIsFirstJoint()
   {
      return rollIsFirstJoint;
   }

   @Override
   public double getRollAngle()
   {
      return rollPosition;
   }

   @Override
   public double getPitchAngle()
   {
      return pitchPosition;
   }

   /**
    * This method computes the rotation of the actuators based on the pitch and roll joint positions. The
    *
    * @param rollAngle  desired roll position, in radians
    * @param pitchAngle desired pitch position, in radians
    */
   public void computeActuatorPositions(double rollAngle, double pitchAngle)
   {
      if (Double.isNaN(rollAngle) || Double.isNaN(pitchAngle))
         throw new IllegalArgumentException("Invalid joint setpoints, roll = " + rollAngle + ", pitch = " + pitchAngle);

      this.rollPosition = rollAngle;
      this.pitchPosition = pitchAngle;
      if (useTrigonometricForwardApproximations)
      {
         TrigonometricApproximation.computeRollMatrix(rollAngle, rollRotation, trigonometricApproximationForwardDepth);
         TrigonometricApproximation.computePitchMatrix(pitchAngle, pitchRotation, trigonometricApproximationForwardDepth);
      }
      else
      {
         rollRotation.setToRollOrientation(rollAngle);
         pitchRotation.setToPitchOrientation(pitchAngle);
      }
      // update all the frames of the kinematics, using the rolls
      frameAfterFirstJoint.update();
      frameAfterSecondJoint.update();
      leftActuatorFrame.update();
      rightActuatorFrame.update();

      // compute the rotation axes in the world frame
      rollJointAxis.setIncludingFrame(frameAfterFirstJoint, rollAxis);
      rollJointAxis.changeFrame(baseFrame);
      pitchJointAxis.setIncludingFrame(frameAfterSecondJoint, pitchAxis);
      pitchJointAxis.changeFrame(baseFrame);
      leftMotorRotationAxis.setIncludingFrame(leftActuatorFrame, pitchAxis);
      leftMotorRotationAxis.changeFrame(baseFrame);
      rightMotorRotationAxis.setIncludingFrame(rightActuatorFrame, pitchAxis);
      rightMotorRotationAxis.changeFrame(baseFrame);

      leftActuatorMechanism.computeActuatorConfigurationForCurrentJointState();
      rightActuatorMechanism.computeActuatorConfigurationForCurrentJointState();

      pitchInRollFrame.setIncludingFrame(frameAfterFirstJoint, vectorToSecondJointFromFirstJoint);
   }

   @Override
   public double getRightActuatorPosition()
   {
      return getRightMotorAngle();
   }

   @Override
   public double getLeftActuatorPosition()
   {
      return getLeftMotorAngle();
   }

   public double getLeftMotorAngle()
   {
      return leftActuatorMechanism.getActuatorRotation() * leftActuatorMechanism.getRotationAngleMapToJoint();
   }

   public double getRightMotorAngle()
   {
      return rightActuatorMechanism.getActuatorRotation() * rightActuatorMechanism.getRotationAngleMapToJoint();
   }

   public FrameVector3DReadOnly getRollJointAxis()
   {
      return rollJointAxis;
   }

   public FrameVector3DReadOnly getPitchJointAxis()
   {
      return pitchJointAxis;
   }

   public FrameVector3DReadOnly getLeftMotorRotationAxis()
   {
      return leftMotorRotationAxis;
   }

   public FrameVector3DReadOnly getRightMotorRotationAxis()
   {
      return rightMotorRotationAxis;
   }

   public FramePoint3DReadOnly getLeftBaseRodEndAttachment()
   {
      return leftActuatorMechanism.getBaseRodEndAttachment();
   }

   public FramePoint3DReadOnly getLeftActuatorRodEndAttachment()
   {
      return leftActuatorMechanism.getActuatorRodEndAttachment();
   }

   public FramePoint3DReadOnly getRightBaseRodEndAttachment()
   {
      return rightActuatorMechanism.getBaseRodEndAttachment();
   }

   public FramePoint3DReadOnly getRightActuatorRodEndAttachment()
   {
      return rightActuatorMechanism.getActuatorRodEndAttachment();
   }

   public ReferenceFrame getLeftActuatorFrame()
   {
      return leftActuatorFrame;
   }

   public ReferenceFrame getRightActuatorFrame()
   {
      return rightActuatorFrame;
   }

   public ReferenceFrame getFrameAfterFirstJoint()
   {
      return frameAfterFirstJoint;
   }

   public ReferenceFrame getFrameAfterSecondJoint()
   {
      return frameAfterSecondJoint;
   }

   public FramePoint3DReadOnly getHipPitchInRollFrame()
   {
      return pitchInRollFrame;
   }

   public RotaryActuatorDifferentialMotorMechanism getLeftSpineMotorMechanism()
   {
      return leftActuatorMechanism;
   }

   public RotaryActuatorDifferentialMotorMechanism getRightSpineMotorMechanism()
   {
      return rightActuatorMechanism;
   }

   @Override
   public double getRollJointLowerLimit()
   {
      return rollLowerLimit;
   }

   @Override
   public double getRollJointUpperLimit()
   {
      return rollUpperLimit;
   }

   @Override
   public double getPitchJointUpperLimit()
   {
      return pitchUpperLimit;
   }

   @Override
   public double getPitchJointLowerLimit()
   {
      return pitchLowerLimit;
   }
}