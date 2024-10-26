package us.ihmc.robotics.trajectories;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.trajectories.interfaces.DoubleTrajectoryGenerator;
import us.ihmc.robotics.trajectories.yoVariables.YoPolynomial;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ConstantVelocityTrajectoryGenerator implements DoubleTrajectoryGenerator
{
   private final YoRegistry registry;
   private final DoubleProvider initialPositionProvider;
   private final DoubleProvider velocityProvider;
   private final YoPolynomial polynomial;
   private final YoDouble trajectoryTime;
   private final DoubleProvider trajectoryTimeProvider;
   private final YoDouble currentTime;
   private final int numberOfCoefficients = 2;

   public ConstantVelocityTrajectoryGenerator(String namePrefix, DoubleProvider initialPositionProvider,
           DoubleProvider velocityProvider, DoubleProvider trajectoryTimeProvider, YoRegistry parentRegistry)
   {
      this.registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      this.initialPositionProvider = initialPositionProvider;
      this.velocityProvider = velocityProvider;
      this.polynomial = new YoPolynomial(namePrefix + "Polynomial", numberOfCoefficients, registry);
      this.trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);
      this.currentTime = new YoDouble(namePrefix + "CurrentTime", registry);
      this.trajectoryTimeProvider = trajectoryTimeProvider;
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      currentTime.set(0.0);
      this.trajectoryTime.set(trajectoryTimeProvider.getValue());
      polynomial.setLinear(0.0, initialPositionProvider.getValue(), velocityProvider.getValue());
   }

   public void compute(double time)
   {
      this.currentTime.set(time);
      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());
      polynomial.compute(time);
   }

   public boolean isDone()
   {
      return currentTime.getDoubleValue() > trajectoryTime.getDoubleValue();
   }

   public double getValue()
   {
      return polynomial.getValue();
   }

   public double getVelocity()
   {
      return polynomial.getVelocity();
   }

   public double getAcceleration()
   {
      return 0.0;
   }

   public String toString()
   {
      return getClass().getSimpleName() + ": x = " + getValue() + ", xdot = " + getVelocity() + "xddot = " + getAcceleration();
   }
}
