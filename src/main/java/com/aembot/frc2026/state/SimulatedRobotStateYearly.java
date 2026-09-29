package com.aembot.frc2026.state;

import com.aembot.frc2026.constants.field.Field2026;
import com.aembot.lib.core.logging.AEMLogger;
import com.aembot.lib.state.SimulatedRobotState;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.Arrays;
import org.ironmaple.simulation.SimulatedArena;

public class SimulatedRobotStateYearly extends SimulatedRobotState {
  private SimulatedRobotStateYearly() {
    visionSimulation.addAprilTags(Field2026.get().getFieldLayout());

    SimulatedArena.getInstance().placeGamePiecesOnField();
  }

  // Ppl on the interwebs say this is good & thread safe
  private static final SimulatedRobotStateYearly INSTANCE = new SimulatedRobotStateYearly();

  public static SimulatedRobotStateYearly get() {
    return INSTANCE;
  }

  @Override
  public void updateState() {
    super.updateState();
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    super.updateLog(standardPrefix, inputPrefix);

    // Get the positions of the fuel (both on the field and in the air)
    // Replace this with the new season's game piece type when updating :3
    ArrayList<Pose3d> fuelPoses =
        new ArrayList<>(
            Arrays.asList(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel")));

    // Publish to telemetry using AdvantageKit
    AEMLogger.recordOutput("SimulatedRobotState/FuelPositions", fuelPoses.toArray(new Pose3d[0]));
  }
}
