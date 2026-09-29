// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.aembot.frc2026;

import com.aembot.frc2026.commands.CommandFactory;
import com.aembot.frc2026.commands.SimulationCommandFactory;
import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.constants.field.FieldBB2026;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.state.SimulatedRobotStateYearly;
import com.aembot.frc2026.subsystems.SubsystemFactory;
import com.aembot.frc2026.util.AutoHelper;
import com.aembot.lib.constants.RuntimeConstants.RuntimeMode;
import com.aembot.lib.core.RobotContainerBase;
import com.aembot.lib.subsystems.aprilvision.AprilVisionSubsystem;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.LoggedRobot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here. Controllers, alliance
 * triggers and the logger boilerplate live in {@link RobotContainerBase}.
 */
public class RobotContainer extends RobotContainerBase {
  /** Between the blue starting line and the arena, placeholder until there are autos */
  private static final Pose2d BLUE_START_POSE =
      new Pose2d(FieldBB2026.BLUE_STARTING_LINE_X + 0.5, 1.8, Rotation2d.kZero);

  /* ---- DRIVETRAIN ---- */
  private final DriveSubsystem driveSubsystem = SubsystemFactory.createDriveSubsystem();

  private final CommandFactory commandFactory;

  /* ---- VISION ---- */
  private final AprilVisionSubsystem visionSubsystem =
      SubsystemFactory.createAprilVisionSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(LoggedRobot robot) {
    super(robot);

    this.commandFactory = new CommandFactory(driveSubsystem);

    if (RobotRuntimeConstants.MODE == RuntimeMode.SIM) {
      SimulatedRobotStateYearly.get()
          .setDriveSim(driveSubsystem.getSimDrivetrain().mapleSimSwerveDrivetrain);
      configureSimulationBindings();
    }

    configureBindings();

    driveSubsystem.resetPose(BLUE_START_POSE);
  }

  /** Use this method to define your controller button -> command mappings */
  private void configureBindings() {
    setupAutos().schedule();

    /* ---- DEFAULT COMMANDS ---- */

    // Use left bumper for slow mode
    driveSubsystem.setDefaultCommand(
        commandFactory.createDriveJoystickCmd(driverController, driverController.leftBumper()));

    /* ---- PRIMARY DRIVER COMMANDS ---- */

    driverController.start().onTrue(commandFactory.resetOdometryHeading());

    /* ---- SECONDARY CONTROLLER BINDINGS ---- */

    secondaryController.leftBumper().onTrue(visionSubsystem.createKillVisionCommand());

    // rest is unused

    /* ---- ADDITIONAL TRIGGER BINDINGS ---- */

    robotEnabled
        .onTrue(visionSubsystem.updateNTEnabledCommand())
        .onFalse(visionSubsystem.updateNTDisabledCommand());

    // allianceIsBlue.onChange(setupAutos());
    // allianceIsRed.onChange(setupAutos());
  }

  /** Sim only bindings on the secondary controller for messing with the cone sim */
  private void configureSimulationBindings() {
    SimulationCommandFactory simulationCommands =
        new SimulationCommandFactory(SimulatedRobotStateYearly.get());

    secondaryController.a().whileTrue(simulationCommands.runIntake());
    secondaryController.y().onTrue(simulationCommands.lightToss());
    secondaryController.b().onTrue(simulationCommands.ejectCone());
    secondaryController.x().onTrue(simulationCommands.spawnHumanPlayerBunny());

    // Human players, left is blue right is red
    secondaryController.leftBumper().onTrue(simulationCommands.spawnHumanPlayerCone(Alliance.Blue));
    secondaryController.rightBumper().onTrue(simulationCommands.spawnHumanPlayerCone(Alliance.Red));

    secondaryController.back().onTrue(simulationCommands.resetField());
  }

  @Override
  public Command getAutonomousCommand() {
    return AutoHelper.autoChooser.selectedCommandScheduler();
  }

  @Override
  public Command getTeleopInitCommand() {
    return new InstantCommand();
  }

  @Override
  public void logCommands() {
    super.logCommands();
    commandFactory.logCommands();
    logFieldPose(RobotStateYearly.get().getLatestFieldRobotPose());
  }

  private Command setupAutos() {
    return new InstantCommand(
            () -> {
              AutoHelper.setupAutoFactory(driveSubsystem);

              AutoHelper.registerAutoCommands(commandFactory);

              AutoHelper.setupAutoChooser(commandFactory);

              SmartDashboard.putData("Choose Auto Routine", AutoHelper.autoChooser);

              System.out.println("Setup Autos");
            })
        .ignoringDisable(true);
  }
}
