package com.aembot.frc2026.config.robots;

import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.config.subsystems.vision.SimulatedCameraConfiguration;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

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

  /* ---- RIGHT CAM ---- */
  public final CameraConfiguration cameraConfigRight =
      CameraConfiguration.makeLimelight4Config("right")
          .withCameraOffset(
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-3),
                      Units.inchesToMeters(-13.5),
                      Units.inchesToMeters(15.132)),
                  new Rotation3d(
                      Units.degreesToRadians(180),
                      Units.degreesToRadians(-27),
                      Units.degreesToRadians(-90))))
          .withDisabledThrottleValue(DISABLED_THROTTLE)
          .withEnabledThrottleValue(ENABLED_THROTTLE)
          .withDisabledIMUMode(DISABLED_IMU_MODE)
          .withEnabledIMUMode(ENABLED_IMU_MODE);

  public final SimulatedCameraConfiguration simConfigRight =
      new SimulatedCameraConfiguration(cameraConfigRight)
          .withFramerate(SIM_CAMERA_FPS)
          .withCalibrationError(0, 0)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS)
          .withPoseNoise(SIM_POSE_NOISE_TRANSLATION_M, SIM_POSE_NOISE_ROTATION_RAD)
          .withLatencyVariation(SIM_LATENCY_VARIATION_MS);

  /* ---- LEFT CAM ---- */
  public final CameraConfiguration cameraConfigLeft =
      CameraConfiguration.makeLimelight4Config("left")
          .withCameraOffset(
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-3),
                      Units.inchesToMeters(13.5),
                      Units.inchesToMeters(15.132)),
                  new Rotation3d(
                      Units.degreesToRadians(180),
                      Units.degreesToRadians(-27),
                      Units.degreesToRadians(90))))
          .withDisabledThrottleValue(DISABLED_THROTTLE)
          .withEnabledThrottleValue(ENABLED_THROTTLE)
          .withDisabledIMUMode(DISABLED_IMU_MODE)
          .withEnabledIMUMode(ENABLED_IMU_MODE);

  public final SimulatedCameraConfiguration simConfigLeft =
      new SimulatedCameraConfiguration(cameraConfigLeft)
          .withFramerate(SIM_CAMERA_FPS)
          .withCalibrationError(0, 0)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS)
          .withPoseNoise(SIM_POSE_NOISE_TRANSLATION_M, SIM_POSE_NOISE_ROTATION_RAD)
          .withLatencyVariation(SIM_LATENCY_VARIATION_MS);

  /* ---- BACK CAM ---- */
  public final CameraConfiguration cameraConfigBack =
      CameraConfiguration.makeLimelight4Config("back")
          .withCameraOffset(
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-13),
                      Units.inchesToMeters(0),
                      Units.inchesToMeters(9.18)),
                  new Rotation3d(
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(-27),
                      Units.degreesToRadians(180))))
          .withDisabledThrottleValue(DISABLED_THROTTLE)
          .withEnabledThrottleValue(ENABLED_THROTTLE)
          .withDisabledIMUMode(DISABLED_IMU_MODE)
          .withEnabledIMUMode(ENABLED_IMU_MODE);

  public final SimulatedCameraConfiguration simConfigBack =
      new SimulatedCameraConfiguration(cameraConfigBack)
          .withFramerate(SIM_CAMERA_FPS)
          .withCalibrationError(0, 0)
          .withCameraLatency(SIM_CAMERA_LATENCY_MS, SIM_CAMERA_LATENCY_STDDEV_MS)
          .withPoseNoise(SIM_POSE_NOISE_TRANSLATION_M, SIM_POSE_NOISE_ROTATION_RAD)
          .withLatencyVariation(SIM_LATENCY_VARIATION_MS);

  /** List of configurations in FL, FR, BL, BR order */
  public final List<CameraConfiguration> cameraConfigurations =
      List.of(cameraConfigRight, cameraConfigLeft, cameraConfigBack);

  /** List of configurations in FL, FR, BL, BR order */
  public final List<SimulatedCameraConfiguration> simConfigurations =
      List.of(simConfigRight, simConfigLeft, simConfigBack);
}
