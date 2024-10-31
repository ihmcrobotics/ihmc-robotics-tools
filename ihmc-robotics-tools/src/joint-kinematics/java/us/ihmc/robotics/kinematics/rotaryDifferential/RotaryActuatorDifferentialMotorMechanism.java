package us.ihmc.robotics.kinematics.rotaryDifferential;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.fourBar.FourBarTools;
import us.ihmc.robotics.kinematics.TrigonometricApproximation;

/**
 * <p>
 * This class represents the closed loop passive mechanism made up by the dual four bars in a differential. It is used to compute the actuator angles based on
 * the  pitch and
 * roll.
 * This works by doing the geometry calculations in a datum plane defined that is coplanar with the surface of the actuator.
 * </p>
 * <p>
 * The four bar mechanism inside the data plane consists of two static lengths that don't change with the joint angles, and two lengths that do. One side of
 * the four bar is the vector from the actuator center of rotation (point A) to the rod end that's attached to the actuator (the child) (point A).
 * This vector does not change in magnitude in the datum plane. The second side of the four bar is the vector from the second joint (point D) to the
 * actuator center of rotation (point A). This vector also does not change in magnitude. The third vector is the rod end, which goes from the attachment on the
 * face of the actuator (point B) to the base (point C). As there is more roll, the length of this vector in the datum plane changes. The fourth vector
 * is the vector from the second joint (point D) to the rod end attachment in the base (point C).  This vector also changes in magnitude as the system
 * rolls.
 * </p>
 *
 * <p>
 * actuator +A------B+ child rod end
 * |a    b|
 * |      |
 * |      |
 * |d    c|
 * second joint +D------C+ base rod end
 * </p>
 *
 * <p>
 * The resulting actuator angle is the angle a in the above diagram, which is the angle between side AB and side AD, or the angle between the actuator rod end
 * and the second joint.
 * </p>
 */
public class RotaryActuatorDifferentialMotorMechanism
{
   // These are the static parameters that define the kinematics.
   private final Vector3DReadOnly firstJointToBaseRodEndVector;
   private final Vector3DReadOnly actuatorToActuatorRodEndVector;
   private final double tieRodLength;

   // These are the frames that update with changes in the joint positions
   private final ReferenceFrame frameAfterSecondJoint;
   private final ReferenceFrame actuatorFrame;

   // These are the frames that update with the changes in the actuator positions
   private final RotationMatrix actuatorRotationMatrix = new RotationMatrix();
   private final ReferenceFrame frameAfterActuatorRotation;

   // These are the points of the rod end attachments. They are used to help calculate the parameters of the four bar.
   private final FramePoint3D secondJoint = new FramePoint3D();
   private final FramePoint3D baseRodEndAttachment = new FramePoint3D();
   private final FramePoint3D actuatorRodEndAttachment = new FramePoint3D();

   private final double actuatorToRodEndLengthInDatum;
   private final double secondJointToActuatorInDatum;
   private final double rotationAngleMapToJoint;
   private double actuatorAngle;
   private final double actuatorAngleAtZero;
   private final boolean useTrigonometricApproximations;

   public RotaryActuatorDifferentialMotorMechanism(ReferenceFrame frameAfterSecondJoint,
                                                   ReferenceFrame actuatorFrame,
                                                   Vector3DReadOnly firstJointToBaseRodEndVector,
                                                   Vector3DReadOnly actuatorToActuatorRodEndVector,
                                                   double tieRodLength)
   {
      this(frameAfterSecondJoint, actuatorFrame, firstJointToBaseRodEndVector, actuatorToActuatorRodEndVector, tieRodLength, false);
   }

   public RotaryActuatorDifferentialMotorMechanism(ReferenceFrame frameAfterSecondJoint,
                                                   ReferenceFrame actuatorFrame,
                                                   Vector3DReadOnly firstJointToBaseRodEndVector,
                                                   Vector3DReadOnly actuatorToActuatorRodEndVector,
                                                   double tieRodLength,
                                                   boolean useTrigonometricApproximations)
   {
      // This is the frame after the second joint.
      this.frameAfterSecondJoint = frameAfterSecondJoint;
      // This is the frame that is coincident with the axis of rotation on the actuator. It is a constant displacement in the frame after the pitch joint.
      this.actuatorFrame = actuatorFrame;
      // This is the vector that goes from the first joint to the base rod end attachment.
      this.firstJointToBaseRodEndVector = firstJointToBaseRodEndVector;
      // This is the vector from the actuator frame to the rod end that is attached to the actuator. This vector is constant in the frame after the actuator,
      // but changes value relative to the frame after the pitch joint as the actuator rotates.
      this.actuatorToActuatorRodEndVector = actuatorToActuatorRodEndVector;
      // This is the length of the rod end in meters from the actuator attachment to the base attachment
      this.tieRodLength = tieRodLength;

      this.useTrigonometricApproximations = useTrigonometricApproximations;

      rotationAngleMapToJoint = Math.signum(actuatorToActuatorRodEndVector.getX()); // FIXME

      // This is the length of side AB
      actuatorToRodEndLengthInDatum = Math.sqrt(MathTools.square(actuatorToActuatorRodEndVector.getX()) + MathTools.square(actuatorToActuatorRodEndVector.getZ()));

      // This is the length of side AD
      FramePoint3D secondJointPosition = new FramePoint3D(frameAfterSecondJoint);
      secondJointPosition.changeFrame(actuatorFrame);
      secondJointToActuatorInDatum = Math.sqrt(MathTools.square(secondJointPosition.getX()) + MathTools.square(secondJointPosition.getZ()));

      frameAfterActuatorRotation = new ReferenceFrame("frameAfterActuator", actuatorFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.getRotation().set(actuatorRotationMatrix);
         }
      };
      updateFrameAfterActuator(0.0);
      actuatorAngleAtZero = computeActuatorConfigurationForCurrentJointState();
   }

   public double computeActuatorConfigurationForCurrentJointState()
   {
      // This is point C in the above diagram
      baseRodEndAttachment.setIncludingFrame(ReferenceFrame.getWorldFrame(), firstJointToBaseRodEndVector);
      baseRodEndAttachment.changeFrame(actuatorFrame);
      // This is point D in the above diagram
      secondJoint.setToZero(frameAfterSecondJoint);
      secondJoint.changeFrame(actuatorFrame);

      // This is the distance out of the actuator plane frame. When computing the length of the four bar inside the data frame, we want to remove this distance
      double transverseRodEndLength = baseRodEndAttachment.getY() + actuatorToActuatorRodEndVector.getY();

      // This is the length of side BC in the above diagram.
      double rodEndLengthInDatum = Math.sqrt(MathTools.square(tieRodLength) - MathTools.square(transverseRodEndLength));
      if (Double.isNaN(rodEndLengthInDatum))
      {
         //         LogTools.warn("Warning: Rolled too much.");
         rodEndLengthInDatum = 1e-3;
      }
      // ASSUMES ALL ROTATION IS ABOUT Y ON THE ACTUATOR
      // this is the distance from the actuator to the rod end in the actuator projected into the plane on the face of the actuator. This is side AC in the above
      // diagram.
      double actuatorToBaseRodEndInDatum = EuclidCoreTools.norm(baseRodEndAttachment.getX(), baseRodEndAttachment.getZ());
      // This is the distance from the second joint to the rod end in the base projected into the plane on the face of the actuator. This is side CD in the
      // above diagram.
      double secondToBaseRodEndInDatum = EuclidCoreTools.norm(secondJoint.getX() - baseRodEndAttachment.getX(),
                                                              secondJoint.getZ() - baseRodEndAttachment.getZ());

      double sideAC = actuatorToBaseRodEndInDatum;
      double sideAB = actuatorToRodEndLengthInDatum;
      double sideBC = rodEndLengthInDatum;
      double sideAD = secondJointToActuatorInDatum;
      double sideCD = secondToBaseRodEndInDatum;

      // this is the cos of the angle in the actuator datum plane between the vectors that go to the two rod-ends from the actuator
      double cosX = FourBarTools.cosineAngleWithCosineLaw(sideAB, sideAC, sideBC);
      // this is the cos of the angle in the actuator datum plane between the vector that goes to the actuator rod end and the pitch joint.
      double cosY = FourBarTools.cosineAngleWithCosineLaw(sideAD, sideAC, sideCD);
      // This is angle a in the above diagram. It is the sum of angles X and Y that we just computed. It can be fast-added from angles x and y.
      double cosA = TrigonometricApproximation.approximateArgumentForAddedArcCosAngles(cosX, cosY);
      actuatorAngle = approximateArcCos(cosA);
      if (Double.isNaN(actuatorAngle))
         throw new IllegalArgumentException("Invalid actuator angle " + actuatorAngle);

      updateFrameAfterActuator(getActuatorRotation() * rotationAngleMapToJoint);
      return actuatorAngle;
   }

   private double approximateArcCos(double cosTheta)
   {
      if (useTrigonometricApproximations)
         return TrigonometricApproximation.taylorArcTan2ApproximationOfArcCos(cosTheta,
                                                                              RotaryActuatorDifferentialForwardKinematics.trigonometricApproximationInverseDepth);
      else
         return TrigonometricApproximation.aTan2ApproximationOfArcCos(cosTheta);
   }

   private void updateFrameAfterActuator(double actuatorAngle)
   {
      if (Double.isNaN(actuatorAngle))
         throw new IllegalArgumentException("Invalid actuator rotation " + actuatorAngle);
      if (useTrigonometricApproximations)
         TrigonometricApproximation.computePitchMatrix(actuatorAngle,
                                                       actuatorRotationMatrix,
                                                       RotaryActuatorDifferentialForwardKinematics.trigonometricApproximationForwardDepth);
      else
         actuatorRotationMatrix.setToPitchOrientation(actuatorAngle);
      frameAfterActuatorRotation.update();
      actuatorRodEndAttachment.setIncludingFrame(frameAfterActuatorRotation, actuatorToActuatorRodEndVector);
   }

   /**
    * This returns the angle between the rod end attached to the actuator and the pitch joint, in the plane of rotation of the actuator.
    *
    * @return angle in radians
    */
   public double getActuatorAngle()
   {
      return actuatorAngle;
   }

   /**
    * This returns the angle of the rod end attached to the actuator and the second joint, in the plane of rotation of the actuator, when the actuator
    * rotation is zero.
    *
    * @return angle in radians
    */
   public double getActuatorAngleAtZero()
   {
      return actuatorAngleAtZero;
   }

   /**
    * This returns the necessary actuator rotation that achieves the current joint configuration.
    *
    * @return rotation in radians
    */
   public double getActuatorRotation()
   {
      return getActuatorAngleAtZero() - getActuatorAngle();
   }

   public double getRotationAngleMapToJoint()
   {
      return rotationAngleMapToJoint;
   }

   public FramePoint3DReadOnly getBaseRodEndAttachment()
   {
      return baseRodEndAttachment;
   }

   public FramePoint3DReadOnly getActuatorRodEndAttachment()
   {
      return actuatorRodEndAttachment;
   }

   public ReferenceFrame getFrameAfterActuatorRotation()
   {
      return frameAfterActuatorRotation;
   }
}