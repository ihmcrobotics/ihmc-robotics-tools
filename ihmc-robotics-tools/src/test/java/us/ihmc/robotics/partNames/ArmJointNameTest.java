package us.ihmc.robotics.partNames;

import com.google.common.base.CaseFormat;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class ArmJointNameTest
{
   @Test
   public void testNamesMatch()
   {
      for (ArmJointName jointName : ArmJointName.values)
      {
         assertEquals(CaseFormat.UPPER_UNDERSCORE.to(CaseFormat.LOWER_CAMEL, jointName.name()), jointName.getCamelCaseNameForStartOfExpression());
      }
   }

}
