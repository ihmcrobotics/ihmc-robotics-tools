package us.ihmc.robotics.math.trajectories.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface PositionTrajectoryGenerator extends TrajectoryGenerator
{
   Point3DReadOnly getPosition();

   Vector3DReadOnly getVelocity();

   Vector3DReadOnly getAcceleration();

   default void getLinearData(Point3DBasics positionToPack, Vector3DBasics velocityToPack, Vector3DBasics accelerationToPack)
   {
      positionToPack.set(getPosition());
      velocityToPack.set(getVelocity());
      accelerationToPack.set(getAcceleration());
   }

   default void showVisualization()
   {
      // Override to do something
   }

   default void hideVisualization()
   {
      // Override to do something
   }
}
