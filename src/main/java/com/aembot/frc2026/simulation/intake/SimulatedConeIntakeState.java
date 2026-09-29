package com.aembot.frc2026.simulation.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.aembot.frc2026.config.subsystems.intake.ConeIntakeConfiguration;
import com.aembot.frc2026.constants.field.FieldBB2026;
import com.aembot.frc2026.constants.field.FieldBB2026.TowerStation;
import com.aembot.frc2026.simulation.arena.ConeZoneArena;
import com.aembot.frc2026.simulation.arena.SimulatedTowerStations;
import com.aembot.frc2026.simulation.gamepieces.ConeColor;
import com.aembot.frc2026.simulation.gamepieces.LargeConeOnField;
import com.aembot.frc2026.simulation.gamepieces.LargeConeOnFly;
import com.aembot.lib.core.logging.AEMLogger;
import com.aembot.lib.core.logging.Loggable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

/**
 * Sim of the robot picking up and letting go of large cones, independent of whatever intake we end
 * up building. Same shape as SimulatedOverBumperIntakeState but it also remembers the color of each
 * cone it grabs, and since the intake is what scores, the toss lives here too. A toss flies the
 * cone as a maplesim projectile and seats it on whichever tower station the arc passes closest to.
 * The collider is created once the drivetrain sim is supplied through {@link #setDriveSim}.
 */
public class SimulatedConeIntakeState implements Loggable {
  /** Half size of the box a tossed cone must pass through, around a stack's tip */
  public static final Translation3d TARGET_TOLERANCE = new Translation3d(0.12, 0.12, 0.18);

  /** How far above the stack tip a light toss releases the cone */
  public static final double LIGHT_TOSS_CLEARANCE_METERS = Inches.of(3).in(Meters);

  /** A light toss only makes sense at a station this close to the release point */
  public static final double LIGHT_TOSS_MAX_STATION_DISTANCE_METERS = 1.0;

  private final ConeIntakeConfiguration kConfig;

  private final AtomicReference<AbstractDriveTrainSimulation> driveSim = new AtomicReference<>();
  private final AtomicReference<IntakeSimulation> intakeSim = new AtomicReference<>();

  /** Colors of the held cones, oldest first */
  private final Deque<ConeColor> held = new ArrayDeque<>();

  private List<Pose3d> loggedTrajectory = List.of();

  public SimulatedConeIntakeState(ConeIntakeConfiguration config) {
    this.kConfig = config;
  }

  private void init() {
    if (intakeSim.get() == null && driveSim.get() != null) {
      intakeSim.set(
          IntakeSimulation.OverTheBumperIntake(
              LargeConeOnField.TYPE,
              driveSim.get(),
              Meters.of(kConfig.kWidthMeters),
              Meters.of(kConfig.kExtensionMeters),
              kConfig.kSide,
              kConfig.kCapacity));

      // Runs once per cone actually collected, so this is where we remember the color
      intakeSim
          .get()
          .setCustomIntakeCondition(
              (gamePiece) -> {
                if (gamePiece instanceof LargeConeOnField cone) {
                  held.addLast(cone.getColor());
                  return true;
                }
                return false;
              });
    }
  }

  /** Supply a maplesim drivetrain to the intake sim. Required for initialization. */
  public void setDriveSim(AbstractDriveTrainSimulation driveSim) {
    if (this.driveSim.get() != null) {
      throw new IllegalStateException("Drive sim already set");
    }
    this.driveSim.set(driveSim);

    init();
  }

  public boolean isInitialized() {
    return intakeSim.get() != null;
  }

  /* ---- INTAKE ---- */
  /** Extend the intake collider. Any cone it touches gets collected */
  public void startIntake() {
    if (intakeSim.get() != null) {
      intakeSim.get().startIntake();
    }
  }

  public void stopIntake() {
    if (intakeSim.get() != null) {
      intakeSim.get().stopIntake();
    }
  }

  public boolean hasCone() {
    return intakeSim.get() != null && intakeSim.get().getGamePiecesAmount() > 0;
  }

  /** Color of the cone the robot is holding, if any */
  public Optional<ConeColor> getHeldColor() {
    return Optional.ofNullable(held.peekFirst());
  }

  /** Preload a cone, robots are allowed to start with one */
  public void preload(ConeColor color) {
    if (intakeSim.get() != null && intakeSim.get().addGamePieceToIntake()) {
      held.addLast(color);
    }
  }

  /**
   * Pull a cone out of the intake
   *
   * @return The color of the cone pulled, empty if the intake was empty
   * @see IntakeSimulation#obtainGamePieceFromIntake() obtainGamePieceFromIntake(); MapleSim method
   *     wrapped by this
   */
  public Optional<ConeColor> pullCone() {
    if (intakeSim.get() == null || !intakeSim.get().obtainGamePieceFromIntake()) {
      return Optional.empty();
    }
    return Optional.ofNullable(held.pollFirst());
  }

  /* ---- TOSS ---- */
  /** Field relative position of the release point right now */
  public Translation2d getReleasePosition() {
    Pose2d robot = driveSim.get().getSimulatedDriveTrainPose();
    return robot.getTranslation().plus(kConfig.kReleasePoint.rotateBy(robot.getRotation()));
  }

  /** Nearest station to the release point, if one is within light toss distance */
  public Optional<Integer> getNearestStationForLightToss() {
    Translation2d from = getReleasePosition();

    int nearest = -1;
    double nearestDistance = LIGHT_TOSS_MAX_STATION_DISTANCE_METERS;
    for (TowerStation station : FieldBB2026.TOWER_STATIONS) {
      double distance = station.center().getDistance(from);
      if (distance < nearestDistance) {
        nearestDistance = distance;
        nearest = station.tagId();
      }
    }

    return nearest < 0 ? Optional.empty() : Optional.of(nearest);
  }

  /**
   * Release height for a light toss. Just above the nearby stack, or the default if none is near
   */
  public double getLightTossReleaseHeight() {
    Optional<Integer> station = getNearestStationForLightToss();
    if (station.isEmpty()) {
      return kConfig.kReleaseHeightMeters;
    }

    // TODO clamp to whatever the real mechanism can actually reach
    double tip = getArena().getTowerStations().getTipHeightMeters(station.get());
    return Math.max(kConfig.kReleaseHeightMeters, tip + LIGHT_TOSS_CLEARANCE_METERS);
  }

  /**
   * Tosses the held cone onto the station in front of the robot. Drive up to it first.
   *
   * @return The projectile, or empty if nothing was held
   */
  public Optional<LargeConeOnFly> lightToss() {
    return toss(
        kConfig.kLightTossSpeedMps, kConfig.kLightTossAngleDeg, getLightTossReleaseHeight());
  }

  /** Toss straight ahead at the given speed and angle from the default release height */
  public Optional<LargeConeOnFly> toss(double launchSpeedMps, double launchAngleDeg) {
    return toss(launchSpeedMps, launchAngleDeg, kConfig.kReleaseHeightMeters);
  }

  /**
   * Toss the held cone straight ahead along the robot heading, chassis velocity included. Whichever
   * station tip the arc passes closest to is the target, hitting it seats the cone.
   *
   * @param launchSpeedMps Speed relative to the robot
   * @param launchAngleDeg Angle above horizontal
   * @param releaseHeightMeters Height the cone leaves the robot at
   * @return The projectile, or empty if nothing was held
   */
  public Optional<LargeConeOnFly> toss(
      double launchSpeedMps, double launchAngleDeg, double releaseHeightMeters) {
    Optional<ConeColor> pulled = pullCone();
    if (pulled.isEmpty()) {
      return Optional.empty();
    }
    ConeColor color = pulled.get();
    ConeZoneArena arena = getArena();

    Pose2d robot = driveSim.get().getSimulatedDriveTrainPose();
    ChassisSpeeds chassisSpeeds = driveSim.get().getDriveTrainSimulatedChassisSpeedsFieldRelative();

    // Launch velocity is robot relative, so add the chassis motion on top
    double angle = Math.toRadians(launchAngleDeg);
    Translation2d start = getReleasePosition();
    Translation2d velocity =
        new Translation2d(launchSpeedMps * Math.cos(angle), robot.getRotation())
            .plus(
                new Translation2d(
                    chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));
    double verticalSpeed = launchSpeedMps * Math.sin(angle);

    LargeConeOnFly projectile =
        new LargeConeOnFly(
            color, start, velocity, releaseHeightMeters, verticalSpeed, robot.getRotation());
    int target =
        closestStationToTrajectory(
            arena.getTowerStations(), start, velocity, releaseHeightMeters, verticalSpeed);

    // Seating the cone is the arena's job, we just tell the projectile where to aim
    projectile
        .withTargetPosition(() -> arena.getTowerStations().getTarget(target))
        .withTargetTolerance(TARGET_TOLERANCE)
        .withHitTargetCallBack(() -> arena.scoreCone(target, color))
        .withProjectileTrajectoryDisplayCallBack(
            (trajectory) -> loggedTrajectory = trajectory,
            (trajectory) -> loggedTrajectory = trajectory);
    arena.addGamePieceProjectile(projectile);

    return Optional.of(projectile);
  }

  /**
   * Launch speed that carries a cone through a point under maplesim's projectile physics
   *
   * @param rangeMeters Horizontal distance from the release point
   * @param riseMeters Height above the release point
   * @param launchAngleDeg Angle above horizontal
   * @return Speed in m/s, NaN if that angle cannot get there
   */
  public static double getLaunchSpeedFor(
      double rangeMeters, double riseMeters, double launchAngleDeg) {
    double gravity = LargeConeOnFly.getGravity();
    double angle = Math.toRadians(launchAngleDeg);

    double denominator =
        2 * Math.cos(angle) * Math.cos(angle) * (rangeMeters * Math.tan(angle) - riseMeters);
    if (denominator <= 0) {
      return Double.NaN;
    }

    return Math.sqrt(gravity * rangeMeters * rangeMeters / denominator);
  }

  /** Drop the held cone on the carpet just past the bumper, robot velocity plus a push */
  public Optional<LargeConeOnField> eject() {
    Optional<ConeColor> pulled = pullCone();
    if (pulled.isEmpty()) {
      return Optional.empty();
    }
    ConeColor color = pulled.get();
    Pose2d robot = driveSim.get().getSimulatedDriveTrainPose();

    // Just clear of the bumper
    double ahead =
        driveSim.get().config.bumperLengthX.in(Meters) / 2.0
            + Inches.of(8).in(Meters)
            + FieldBB2026.LARGE_CONE_BASE_WIDTH_METERS / 2.0;
    Translation2d spot = robot.getTranslation().plus(new Translation2d(ahead, robot.getRotation()));

    ChassisSpeeds chassisSpeeds = driveSim.get().getDriveTrainSimulatedChassisSpeedsFieldRelative();
    Translation2d velocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .plus(new Translation2d(kConfig.kEjectSpeedMps, robot.getRotation()));

    LargeConeOnField cone = new LargeConeOnField(color, spot, velocity);
    SimulatedArena.getInstance().addGamePiece(cone);

    return Optional.of(cone);
  }

  /** Station whose stack tip the trajectory passes closest to. Drag free, maplesim gravity */
  static int closestStationToTrajectory(
      SimulatedTowerStations stations,
      Translation2d start,
      Translation2d velocity,
      double initialHeight,
      double verticalSpeed) {
    double gravity = LargeConeOnFly.getGravity();

    int best = 1;
    double bestDistance = Double.MAX_VALUE;
    for (int id = 1; id <= SimulatedTowerStations.STATION_COUNT; id++) {
      Translation3d target = stations.getTarget(id);

      // Step along the arc until it hits the floor
      for (double t = 0; t < 3.0; t += 0.01) {
        double z = initialHeight + verticalSpeed * t - 0.5 * gravity * t * t;
        if (z < 0) {
          break;
        }

        Translation2d position = start.plus(velocity.times(t));
        double distance =
            Math.hypot(position.getDistance(target.toTranslation2d()), z - target.getZ());
        if (distance < bestDistance) {
          bestDistance = distance;
          best = id;
        }
      }
    }

    return best;
  }

  private static ConeZoneArena getArena() {
    return (ConeZoneArena) SimulatedArena.getInstance();
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    AEMLogger.recordOutput(standardPrefix + "/Initialized", isInitialized());
    AEMLogger.recordOutput(standardPrefix + "/HasCone", hasCone());
    AEMLogger.recordOutput(
        standardPrefix + "/HeldCone", getHeldColor().map(Enum::name).orElse("NONE"));
    AEMLogger.recordOutput(
        standardPrefix + "/TossTrajectory", loggedTrajectory.toArray(new Pose3d[0]));
  }
}
