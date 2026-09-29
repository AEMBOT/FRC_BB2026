package com.aembot.frc2026.config.robots;

import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.config.subsystems.vision.SimulatedCameraConfiguration;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

// TODO camera positions are placeholders until the robot has cameras on it
public class ProductionCameraConfig {
  // LL4 runs at ~90fps, but MT2 processing reduces effective rate
  private static final double SIM_CAMERA_FPS = 50;

  // MT2 typical latency is ~25-40ms capture + ~10-15ms pipeline
  private static final double SIM_CAMERA_LATENCY_MS = 35;
  private static final double SIM_CAMERA_LATENCY_STDDEV_MS = 8;

  // Additional frame-to-frame latency jitter
  private static final double SIM_LATENCY_VARIATION_MS = 5;

  // Pose noise at 1m distance (scales with distance²)
  private static final double SIM_POSE_NOISE_TRANSLATION_M = 0.015; // 1.5cm
  private static final double SIM_POSE_NOISE_ROTATION_RAD = Units.degreesToRadians(0.5);

  private static final int DISABLED_THROTTLE = 100;

  private static final int ENABLED_THROTTLE = 1;

  private static final int DISABLED_IMU_MODE = 1;

  private static final int ENABLED_IMU_MODE = 1;

  /* ---- TAG CAM ---- */
  // Placeholder apriltag limelight so pose estimation can be simmed. The BunnyBots tags are only
  // 12.75" off the ground so keep this low and level
  public final CameraConfiguration cameraConfigTags =
      CameraConfiguration.makeLimelight4Config("limelight-tags")
          .withCameraOffset(
              new Transform3d(
                  new Translation3d(Units.inchesToMeters(-12), 0, Units.inchesToMeters(12)),
                  new Rotation3d(0, 0, Units.degreesToRadians(180))))
          .withDisabledThrottleValue(DISABLED_THROTTLE)
          .withEnabledThrottleValue(ENABLED_THROTTLE)
          .withDisabledIMUMode(DISABLED_IMU_MODE)
          .withEnabledIMUMode(ENABLED_IMU_MODE);

  public final SimulatedCameraConfiguration simConfigTags =
      new SimulatedCameraConfiguration(cameraConfigTags)
          .withFramerate(SIM_CAMERA_FPS)
          .withCalibrationError(0, 0)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS)
          .withPoseNoise(SIM_POSE_NOISE_TRANSLATION_M, SIM_POSE_NOISE_ROTATION_RAD)
          .withLatencyVariation(SIM_LATENCY_VARIATION_MS);

  /* ---- CONE CAM ---- */
  // Limelight 3A running the cone stack pipeline, not part of the apriltag vision subsystem
  public final CameraConfiguration cameraConfigCones =
      CameraConfiguration.makeLimelight3AConfig("limelight-cones")
          .withCameraOffset(
              new Transform3d(
                  new Translation3d(Units.inchesToMeters(12), 0, Units.inchesToMeters(12)),
                  new Rotation3d()));

  /** Apriltag cameras in the order their IO gets created */
  public final List<CameraConfiguration> cameraConfigurations = List.of(cameraConfigTags);

  /** Simulated apriltag cameras, same order as {@link #cameraConfigurations} */
  public final List<SimulatedCameraConfiguration> simConfigurations = List.of(simConfigTags);
}
