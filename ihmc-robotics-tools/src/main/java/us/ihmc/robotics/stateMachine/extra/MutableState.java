package us.ihmc.robotics.stateMachine.extra;

import us.ihmc.robotics.stateMachine.core.State;

import java.util.function.DoubleConsumer;
import java.util.function.DoublePredicate;

/**
 * Allows the user to ignore inapplicable methods and utilize lambdas if desired.
 */
public class MutableState implements State
{
   private Runnable onEntry;
   private DoubleConsumer doAction;
   private Runnable onExit;
   private DoublePredicate isDone;

   /** {@inheritDoc} */
   @Override
   public void onEntry()
   {
      if (onEntry != null) onEntry.run();
   }

   /** {@inheritDoc} */
   @Override
   public void doAction(double timeInState)
   {
      if (doAction != null) doAction.accept(timeInState);
   }

   /** {@inheritDoc} */
   @Override
   public void onExit(double timeInState)
   {
      if (onExit != null) onExit.run();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isDone(double timeInState)
   {
      if (isDone != null) return isDone.test(timeInState);

      return false;
   }

   public void setOnEntry(Runnable onEntry)
   {
      this.onEntry = onEntry;
   }

   public void setDoAction(DoubleConsumer doAction)
   {
      this.doAction = doAction;
   }

   public void setOnExit(Runnable onExit)
   {
      this.onExit = onExit;
   }

   public void setIsDone(DoublePredicate isDone)
   {
      this.isDone = isDone;
   }
}