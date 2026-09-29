package com.aembot.frc2026.commands;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.drive.commands.JoystickDriveCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class CommandFactory {

  private final DriveSubsystem driveSubsystem;

  public CommandFactory(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  public void logCommands() {}

  public JoystickDriveCommand createDriveJoystickCmd(
      CommandXboxController driverController, Trigger slowModeButton) {
    return DriveCommands.createDriveJoystickCmd(
        driveSubsystem, driverController.getHID(), () -> slowModeButton.getAsBoolean());
  }

  public Command resetOdometryHeading() {
    return new InstantCommand(
        () -> {
          Translation2d robotTranslation =
              RobotStateYearly.get().getLatestFieldRobotPose().getTranslation();
          Rotation2d robotRotation =
              RobotRuntimeConstants.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg;
          driveSubsystem.resetPose(new Pose2d(robotTranslation, robotRotation));
        });
  }
}
