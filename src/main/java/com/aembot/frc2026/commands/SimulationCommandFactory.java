package com.aembot.frc2026.commands;

import com.aembot.frc2026.simulation.intake.SimulatedConeIntakeState;
import com.aembot.frc2026.state.SimulatedRobotStateYearly;
import com.aembot.lib.constants.RuntimeConstants;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Commands that only exist in sim, for poking at the Cone Zone before any real mechanisms exist.
 * Once there is an intake subsystem its sim IO should drive SimulatedConeIntakeState directly and
 * this whole class can go
 */
public final class SimulationCommandFactory {
  private final SimulatedRobotStateYearly simulatedState;

  public SimulationCommandFactory(SimulatedRobotStateYearly simulatedState) {
    this.simulatedState = simulatedState;
  }

  private SimulatedConeIntakeState intake() {
    return simulatedState.getConeIntakeState();
  }

  /** Runs the intake collider while held, drive into a cone to grab it */
  public Command runIntake() {
    return Commands.startEnd(() -> intake().startIntake(), () -> intake().stopIntake())
        .withName("SimRunIntake");
  }

  /** Flicks the held cone onto whatever station the bumper is against */
  public Command lightToss() {
    return Commands.runOnce(() -> intake().lightToss()).withName("SimLightToss");
  }

  /** Drops the held cone in front of the robot */
  public Command ejectCone() {
    return Commands.runOnce(() -> intake().eject()).withName("SimEjectCone");
  }

  /** Nearest human player of that alliance drops a cone at their corner */
  public Command spawnHumanPlayerCone(Alliance alliance) {
    return Commands.runOnce(
            () ->
                simulatedState
                    .getArena()
                    .spawnHumanPlayerCone(
                        alliance, simulatedState.getLatestFieldRobotPose().getTranslation()))
        .withName("SimSpawnCone" + alliance);
  }

  /** Our nearest human player throws in the bunny */
  public Command spawnHumanPlayerBunny() {
    return Commands.runOnce(
            () ->
                simulatedState
                    .getArena()
                    .spawnHumanPlayerBunny(
                        RuntimeConstants.isRedAlliance() ? Alliance.Red : Alliance.Blue,
                        simulatedState.getLatestFieldRobotPose().getTranslation()))
        .withName("SimSpawnBunny");
  }

  /** Puts the 42 starting cones back and clears the stacks */
  public Command resetField() {
    return Commands.runOnce(simulatedState::resetField)
        .ignoringDisable(true)
        .withName("SimResetField");
  }
}
