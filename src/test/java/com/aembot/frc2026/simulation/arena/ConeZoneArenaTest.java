package com.aembot.frc2026.simulation.arena;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.aembot.frc2026.config.robots.ProductionConeConfig;
import com.aembot.frc2026.constants.field.FieldBB2026;
import com.aembot.frc2026.simulation.gamepieces.ConeColor;
import com.aembot.frc2026.simulation.gamepieces.LargeConeOnField;
import com.aembot.frc2026.simulation.gamepieces.LargeConeOnFly;
import com.aembot.frc2026.simulation.intake.SimulatedConeIntakeState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ConeZoneArenaTest {
  private ConeZoneArena arena;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0));
    // Projectiles run off the FPGA clock so time is stepped by hand
    SimHooks.pauseTiming();

    arena = new ConeZoneArena();
    SimulatedArena.overrideInstance(arena);
  }

  @AfterEach
  void tearDown() {
    SimHooks.resumeTiming();
    arena.shutDown();
    HAL.shutdown();
  }

  private void runSeconds(double seconds) {
    for (double t = 0; t < seconds; t += 0.02) {
      SimHooks.stepTiming(0.02);
      arena.simulationPeriodic();
    }
  }

  private static Pose2d againstStation(int id) {
    double halfBumper = DriveTrainSimulationConfig.Default().bumperLengthX.in(Units.Meters) / 2.0;
    return FieldBB2026.getTowerStation(id).approachPose(halfBumper + 0.01);
  }

  private SwerveDriveSimulation addDrive(Pose2d start) {
    SwerveDriveSimulation drive =
        new SwerveDriveSimulation(DriveTrainSimulationConfig.Default(), start);
    arena.addDriveTrainSimulation(drive);
    return drive;
  }

  @Test
  void placesFortyTwoAlternatingConesOutsideTheArena() {
    arena.placeGamePiecesOnField();

    assertEquals(21, arena.getConesOnField(ConeColor.BLUE).size());
    assertEquals(21, arena.getConesOnField(ConeColor.RED).size());

    double halfX = FieldBB2026.ARENA_LENGTH_METERS / 2.0;
    double halfY = FieldBB2026.ARENA_WIDTH_METERS / 2.0;
    for (Translation2d cone : ConeZoneArena.getStartingConeRing()) {
      Translation2d relative = cone.minus(FieldBB2026.FIELD_CENTER);
      assertTrue(
          Math.abs(relative.getX()) > halfX || Math.abs(relative.getY()) > halfY,
          "cone at " + relative + " is inside the arena");
    }
  }

  @Test
  void arenaBlockStopsTheRobot() {
    Pose2d start = new Pose2d(FieldBB2026.fromCenter(-120, 0), Rotation2d.kZero);
    SwerveDriveSimulation drive = addDrive(start);

    for (int i = 0; i < 250; i++) {
      drive.setRobotSpeeds(new ChassisSpeeds(2.0, 0.0, 0.0)); // this is field relative
      runSeconds(0.02);
    }

    Pose2d end = drive.getSimulatedDriveTrainPose();
    double arenaFaceX = FieldBB2026.FIELD_CENTER.getX() - FieldBB2026.ARENA_LENGTH_METERS / 2.0;
    assertTrue(end.getX() > start.getX() + 0.5, "robot should have moved forward");
    assertTrue(end.getX() < arenaFaceX, "robot center passed the arena face");
  }

  @Test
  void intakeCollectsAConeAndALightTossSeatsIt() {
    SwerveDriveSimulation drive = addDrive(againstStation(5));
    SimulatedConeIntakeState intake =
        new SimulatedConeIntakeState(new ProductionConeConfig().intakeConfiguration);

    assertFalse(intake.isInitialized());
    intake.setDriveSim(drive);
    assertTrue(intake.isInitialized());

    // Back up, grab a cone off the carpet
    Pose2d back = FieldBB2026.getTowerStation(5).approachPose(2.0);
    drive.setSimulationWorldPose(back);
    arena.spawnCone(ConeColor.BLUE, back.getTranslation().plus(new Translation2d(0.8, 0.0)));

    intake.startIntake();
    for (int i = 0; i < 100 && !intake.hasCone(); i++) {
      drive.setRobotSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0));
      runSeconds(0.02);
    }
    intake.stopIntake();

    assertTrue(intake.hasCone(), "intake should have collected the cone");
    assertEquals(ConeColor.BLUE, intake.getHeldColor().orElseThrow());
    assertEquals(0, arena.getConesOnField().size());

    // Back at the station, toss it on
    drive.setRobotSpeeds(new ChassisSpeeds());
    drive.setSimulationWorldPose(againstStation(5));
    runSeconds(0.1);

    Optional<LargeConeOnFly> projectile = intake.lightToss();
    assertTrue(projectile.isPresent());
    assertTrue(projectile.get().willHitTarget(), "short arc should pass through the stack tip");

    runSeconds(3.0);
    assertEquals("B", arena.getTowerStations().getStack(5).getCode());
    assertEquals(1, arena.getGamePiecesArrayByType(LargeConeOnField.TYPE_BLUE).length);
  }

  @Test
  void lightTossFromTooFarLandsAsAColoredFloorCone() {
    SwerveDriveSimulation drive = addDrive(FieldBB2026.getTowerStation(5).approachPose(1.5));
    SimulatedConeIntakeState intake =
        new SimulatedConeIntakeState(new ProductionConeConfig().intakeConfiguration);
    intake.setDriveSim(drive);
    intake.preload(ConeColor.RED);

    Optional<LargeConeOnFly> projectile = intake.lightToss();
    assertTrue(projectile.isPresent());
    assertFalse(projectile.get().willHitTarget());

    runSeconds(3.0);
    assertEquals("", arena.getTowerStations().getStack(5).getCode());
    assertEquals(1, arena.getConesOnField().size(), "missed cone should be on the carpet");
    assertEquals(ConeColor.RED, arena.getConesOnField().get(0).getColor());
  }

  @Test
  void launchSpeedSolverMatchesProjectilePhysics() {
    double range = 2.0;
    double rise = 0.5;
    double angle = 45.0;
    double speed = SimulatedConeIntakeState.getLaunchSpeedFor(range, rise, angle);

    // Fly it by hand and see if it gets there
    double gravity = LargeConeOnFly.getGravity();
    double t = range / (speed * Math.cos(Math.toRadians(angle)));
    double z = speed * Math.sin(Math.toRadians(angle)) * t - 0.5 * gravity * t * t;
    assertEquals(rise, z, 1e-9);

    assertTrue(Double.isNaN(SimulatedConeIntakeState.getLaunchSpeedFor(2.0, 5.0, 10.0)));
  }
}
