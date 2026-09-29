package com.aembot.frc2026.constants;

import com.aembot.frc2026.config.RobotConfiguration;
import com.aembot.frc2026.config.RobotIDYearly;
import com.aembot.frc2026.constants.field.FieldBB2026;
import com.aembot.lib.constants.RuntimeConstants;
import com.aembot.lib.constants.fields.YearFieldConstantable;

public class RobotRuntimeConstants extends RuntimeConstants {
  public static final RobotIDYearly ROBOT_ID = (RobotIDYearly) RobotIDYearly.getIdentification();

  public static final RobotConfiguration ROBOT_CONFIG =
      RobotConfiguration.getRobotConstants((RobotIDYearly) ROBOT_ID);

  /** Field we are playing on. Swap to Field2026.get() for the FRC field */
  public static final YearFieldConstantable FIELD = FieldBB2026.get();
}
