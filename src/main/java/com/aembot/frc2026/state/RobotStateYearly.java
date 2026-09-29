package com.aembot.frc2026.state;

import com.aembot.lib.core.logging.AEMLogger;
import com.aembot.lib.state.RobotState;
import edu.wpi.first.math.geometry.Pose3d;

public class RobotStateYearly extends RobotState {
  // Ppl on the interwebs say this is good & thread safe
  private static final RobotStateYearly INSTANCE = new RobotStateYearly();

  public static RobotStateYearly get() {
    return INSTANCE;
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    super.updateLog(standardPrefix, inputPrefix);

    // Log mechanism positions here. In AScope, this will let us see
    // the positions of things like intakes, turrets, and arms as if
    // we're looking at a real robot!
    AEMLogger.recordOutput("SensorRobotState/MechanismPositions", new Pose3d[] {});
  }
}
