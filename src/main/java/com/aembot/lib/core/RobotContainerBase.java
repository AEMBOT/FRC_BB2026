package com.aembot.lib.core;

import com.aembot.lib.core.logging.Loggerable;
import com.aembot.lib.core.logging.log_entries.LogEntry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.LoggedRobot;

/**
 * Season agnostic part of the robot container. Controllers, driver station triggers, logger setup
 * and alliance logging. The season's RobotContainer extends this and declares the actual robot
 */
public abstract class RobotContainerBase implements Loggerable {
  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int SECONDARY_CONTROLLER_PORT = 1;

  /** Loops between alliance log pushes */
  private static final int ALLIANCE_LOG_THROTTLE = 20;

  /* ---- CONTROLLERS ---- */
  // Replace with CommandPS4Controller or CommandJoystick if needed
  protected final CommandXboxController driverController =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  protected final CommandXboxController secondaryController =
      new CommandXboxController(SECONDARY_CONTROLLER_PORT);

  /* ---- DRIVER STATION TRIGGERS ---- */
  protected final Trigger robotEnabled = new Trigger(() -> DriverStation.isEnabled());

  protected final Trigger allianceInitialized =
      new Trigger(() -> DriverStation.getAlliance().isPresent());

  protected final Trigger allianceIsRed =
      new Trigger(
          () ->
              allianceInitialized.getAsBoolean()
                  && DriverStation.getAlliance().get().equals(Alliance.Red));

  protected final Trigger allianceIsBlue =
      new Trigger(
          () ->
              allianceInitialized.getAsBoolean()
                  && DriverStation.getAlliance().get().equals(Alliance.Blue));

  /* ---- LOGGING ---- */
  protected final Field2d field = new Field2d();

  protected final LogEntry<Alliance> allianceLogEntry =
      new LogEntry<>("Alliance", Alliance.class, ALLIANCE_LOG_THROTTLE);

  protected final LogEntry<Boolean> allianceSetLogEntry =
      new LogEntry<>("AllianceSet", Boolean.class, ALLIANCE_LOG_THROTTLE);

  /** Sets up the logger. Subsystems get built by the subclass */
  protected RobotContainerBase(LoggedRobot robot) {
    setupLogger(robot);
  }

  /** Logs the alliance entries. Subclasses should call super then log their own commands */
  public void logCommands() {
    if (DriverStation.getAlliance().isPresent()) {
      allianceLogEntry.pushValue(DriverStation.getAlliance().get());
    }
    allianceSetLogEntry.pushValue(DriverStation.getAlliance().isPresent());
  }

  /** Puts the robot pose on the dashboard field widget */
  protected void logFieldPose(Pose2d pose) {
    field.setRobotPose(pose);
    SmartDashboard.putData("FieldData/Field2d", field);
  }

  /**
   * Use this to pass the autonomous command to the main Robot class
   *
   * @return The command to run in autonomous
   */
  public abstract Command getAutonomousCommand();

  /**
   * Use this to pass the teleop init command to the main Robot class
   *
   * @return The command to run at the start of teleop
   */
  public abstract Command getTeleopInitCommand();
}
