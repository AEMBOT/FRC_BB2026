package com.aembot.frc2026.state;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.simulation.arena.ConeZoneArena;
import com.aembot.frc2026.simulation.arena.SimulatedTowerStations;
import com.aembot.frc2026.simulation.intake.SimulatedConeIntakeState;
import com.aembot.frc2026.simulation.vision.ConeCameraSim;
import com.aembot.frc2026.simulation.vision.SimulatedConeLimelight;
import com.aembot.lib.state.SimulatedRobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Notifier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

/**
 * BunnyBots 2026 sim state. Owns the Cone Zone arena, the cone intake sim and the fake cone camera
 * + Limelight. get() has to be called before the drivetrain is built (see Robot) because the
 * drivetrain sim registers with whatever SimulatedArena exists at that moment and this constructor
 * is what installs the Cone Zone one.
 */
public class SimulatedRobotStateYearly extends SimulatedRobotState {
  private static final String CONE_CAMERA_STREAM_NAME = "ConeCam";
  private static final double CONE_CAMERA_MAX_RANGE_METERS = 6.0;
  private static final double CONE_PIPELINE_LATENCY_MS = 25.0;

  /** Cone camera frame period. A real pipeline is nowhere near 50 fps anyway */
  /** 15 fps, the software renderer takes about 50 ms a frame at the Limelight 3A's 1280x960 */
  private static final double CONE_CAMERA_PERIOD_SECONDS = 1.0 / 15.0;

  /** Who fills in the cone limelight's network table */
  public enum ConeLimelightSource {
    /** The OpenCV pipeline (java port of the Limelight python) run on the rendered frame */
    PIPELINE,
    /** Ground truth from the renderer, no vision noise at all */
    GROUND_TRUTH,
    /**
     * limelight_sim.py from Field_BB2026 running the python pipeline on the ConeCam stream. Robot
     * code still renders the stream but leaves the table alone
     */
    EXTERNAL
  }

  private static final ConeLimelightSource CONE_LIMELIGHT_SOURCE = ConeLimelightSource.PIPELINE;

  // Rendering and the pipeline take ~15ms together, way too much for the main loop, so they run
  // on their own thread like an actual coprocessor would
  private Notifier coneCameraThread = null;

  private final ConeZoneArena arena;
  private final SimulatedConeIntakeState coneIntakeState;
  private final ConeCameraSim coneCamera;
  private final SimulatedConeLimelight coneLimelight;

  private SimulatedRobotStateYearly() {
    arena = new ConeZoneArena();
    SimulatedArena.overrideInstance(arena);
    arena.placeGamePiecesOnField();

    visionSimulation.addAprilTags(RobotRuntimeConstants.FIELD.getFieldLayout());

    coneIntakeState =
        new SimulatedConeIntakeState(
            RobotRuntimeConstants.ROBOT_CONFIG.getConeIntakeConfiguration());

    coneCamera =
        new ConeCameraSim(
                RobotRuntimeConstants.ROBOT_CONFIG.getConeCameraConfiguration(),
                CONE_CAMERA_MAX_RANGE_METERS)
            .withCameraServerStream(CONE_CAMERA_STREAM_NAME);
    coneLimelight =
        new SimulatedConeLimelight(
            RobotRuntimeConstants.ROBOT_CONFIG.getConeCameraConfiguration(),
            CONE_PIPELINE_LATENCY_MS);
  }

  // Ppl on the interwebs say this is good & thread safe
  private static final SimulatedRobotStateYearly INSTANCE = new SimulatedRobotStateYearly();

  public static SimulatedRobotStateYearly get() {
    return INSTANCE;
  }

  public ConeZoneArena getArena() {
    return arena;
  }

  public SimulatedTowerStations getTowerStations() {
    return arena.getTowerStations();
  }

  public SimulatedConeIntakeState getConeIntakeState() {
    return coneIntakeState;
  }

  public ConeCameraSim getConeCamera() {
    return coneCamera;
  }

  public SimulatedConeLimelight getConeLimelight() {
    return coneLimelight;
  }

  /** Supply the maplesim drivetrain to the sims that need a chassis to attach to */
  public void setDriveSim(AbstractDriveTrainSimulation driveSim) {
    coneIntakeState.setDriveSim(driveSim);
  }

  /** Put the 42 starting cones back and clear the stacks */
  public void resetField() {
    arena.resetFieldForAuto();
  }

  @Override
  public void updateState() {
    super.updateState();

    // Started here rather than in the constructor so the HAL is definitely up
    if (coneCameraThread == null) {
      coneCameraThread = new Notifier(this::coneCameraFrame);
      coneCameraThread.setName("ConeCameraSim");
      coneCameraThread.startPeriodic(CONE_CAMERA_PERIOD_SECONDS);
    }
  }

  /** One frame of the cone camera, runs on the ConeCameraSim thread */
  private void coneCameraFrame() {
    Pose2d pose = getLatestFieldRobotPose();
    if (pose == null) {
      return;
    }

    ConeCameraSim.Frame frame = coneCamera.render(pose);

    switch (CONE_LIMELIGHT_SOURCE) {
      case PIPELINE:
        coneLimelight.updateFromPipeline(frame);
        break;
      case GROUND_TRUTH:
        coneLimelight.updateFromGroundTruth(frame);
        break;
      default:
        break;
    }

    // After the pipeline so its boxes end up on the stream
    coneCamera.stream(frame);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    super.updateLog(standardPrefix, inputPrefix);

    // Field side things (cones, stacks, score) live under the arena, robot side things under here
    arena.updateLog("SimulatedArenaState", inputPrefix);

    coneIntakeState.updateLog("SimulatedRobotState/ConeIntake", inputPrefix);
    coneLimelight.updateLog("SimulatedRobotState/ConeLimelight", inputPrefix);
  }
}
