package com.aembot.frc2026.simulation.vision;

import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.core.logging.AEMLogger;
import com.aembot.lib.core.logging.Loggable;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Optional;
import org.opencv.core.Rect;

/**
 * Fake Limelight for the cone stack pipeline. Publishes the same NT keys the real one would (tv,
 * tx, ty, ta, tl, cl, hb, llpython) so robot code doesn't care whether it is in sim. Fed either by
 * the OpenCV pipeline run on the rendered frame or by the renderer's ground truth.
 */
public class SimulatedConeLimelight implements Loggable {
  public static final int LLPYTHON_LENGTH = 8;

  private final NetworkTable networkTable;
  private final NetworkTableEntry validTargetEntry;
  private final NetworkTableEntry xOffsetEntry;
  private final NetworkTableEntry yOffsetEntry;
  private final NetworkTableEntry targetAreaEntry;
  private final NetworkTableEntry pipelineLatencyEntry;
  private final NetworkTableEntry captureLatencyEntry;
  private final NetworkTableEntry heartbeatEntry;
  private final NetworkTableEntry pythonOutputEntry;

  private final double kPipelineLatencyMs;

  // Pinhole intrinsics from the camera config, the real Limelight gets these from calibration
  private final int kWidth;
  private final int kHeight;
  private final double kFx;
  private final double kFy;

  /** Heartbeat value of the fake limelight. Resets at 2 billion */
  private int heartbeat = 0;

  // Written from the camera thread, read by updateLog on the main loop
  private volatile double[] lastPythonOutput = new double[LLPYTHON_LENGTH];
  private volatile double lastPipelineMs = 0;

  /**
   * Constructor. Publishes to the table named after the camera like a real Limelight would
   *
   * @param cameraConfiguration Config for the cone camera, name, resolution and fov are used
   * @param pipelineLatencyMs Latency to report on "tl" when publishing ground truth
   */
  public SimulatedConeLimelight(CameraConfiguration cameraConfiguration, double pipelineLatencyMs) {
    networkTable = NetworkTableInstance.getDefault().getTable(cameraConfiguration.cameraName);
    validTargetEntry = networkTable.getEntry("tv");
    xOffsetEntry = networkTable.getEntry("tx");
    yOffsetEntry = networkTable.getEntry("ty");
    targetAreaEntry = networkTable.getEntry("ta");
    pipelineLatencyEntry = networkTable.getEntry("tl");
    captureLatencyEntry = networkTable.getEntry("cl");
    heartbeatEntry = networkTable.getEntry("hb");
    pythonOutputEntry = networkTable.getEntry("llpython");

    this.kPipelineLatencyMs = pipelineLatencyMs;

    kWidth = cameraConfiguration.cameraResolution.widthPixels;
    kHeight = cameraConfiguration.cameraResolution.heightPixels;
    kFx =
        (kWidth / 2.0)
            / Math.tan(Math.toRadians(cameraConfiguration.cameraFOV.horizontalDegrees / 2.0));
    kFy =
        (kHeight / 2.0)
            / Math.tan(Math.toRadians(cameraConfiguration.cameraFOV.verticalDegrees / 2.0));
  }

  /**
   * Run the OpenCV pipeline on the frame and publish what it found. The frame image gets the
   * pipeline's boxes drawn on it, same as the real Limelight's stream
   *
   * @return What the pipeline found, for tests and logging
   */
  public ConeStackPipeline.Result updateFromPipeline(ConeCameraSim.Frame frame) {
    long start = System.nanoTime();
    ConeStackPipeline.Result result = ConeStackPipeline.runPipeline(frame.image());
    lastPipelineMs = (System.nanoTime() - start) / 1e6;

    tick(lastPipelineMs);

    if (result.llpython()[0] > 0) {
      // tx/ty from the hull's box with the real intrinsics, the way the Limelight does it from the
      // returned contour. llpython keeps the pipeline's own guess
      Rect box = ConeStackPipeline.boundingRect(result.largestContour());
      double cx = box.x + box.width / 2.0;
      double cy = box.y + box.height / 2.0;

      validTargetEntry.setInteger(1);
      xOffsetEntry.setDouble(Math.toDegrees(Math.atan((cx - kWidth / 2.0) / kFx)));
      yOffsetEntry.setDouble(Math.toDegrees(Math.atan((kHeight / 2.0 - cy) / kFy)));
      targetAreaEntry.setDouble(100.0 * box.area() / (kWidth * kHeight));
    } else {
      noTarget();
    }

    lastPythonOutput = result.llpython();
    pythonOutputEntry.setDoubleArray(lastPythonOutput);

    return result;
  }

  /** Publish the largest station in view straight from the renderer's ground truth */
  public void updateFromGroundTruth(ConeCameraSim.Frame frame) {
    tick(kPipelineLatencyMs);

    Optional<ConeCameraSim.Detection> best = frame.getLargestStack();

    if (best.isPresent()) {
      ConeCameraSim.Detection detection = best.get();

      validTargetEntry.setInteger(1);
      xOffsetEntry.setDouble(detection.txDegrees());
      yOffsetEntry.setDouble(detection.tyDegrees());
      targetAreaEntry.setDouble(detection.areaPercent());

      // Bare station has no scored cones and no top color
      String colors = detection.colors();
      char top = colors.isEmpty() ? ' ' : colors.charAt(colors.length() - 1);

      /*
       * Same layout as llpython from cone_stack_pipeline.py: [tv, tx, ty, ta, station id, scored
       * cones, top color (1 red, 2 blue, 3 white, 0 bare), distance in meters]. Station id and
       * distance are only known here, a real camera reports 0 for both
       */
      lastPythonOutput =
          new double[] {
            1, // tv
            detection.txDegrees(),
            detection.tyDegrees(),
            detection.areaPercent(),
            detection.stationId(),
            detection.scoredCones(),
            ConeStackPipeline.topColorCode(top),
            detection.distanceMeters()
          };
    } else {
      noTarget();
      lastPythonOutput = new double[LLPYTHON_LENGTH];
    }

    pythonOutputEntry.setDoubleArray(lastPythonOutput);
  }

  /** Heartbeat and latency, every frame whether or not there is a target */
  private void tick(double pipelineMs) {
    heartbeat++;
    if (heartbeat > 2e9) heartbeat = 0;
    heartbeatEntry.setDouble(heartbeat);

    pipelineLatencyEntry.setDouble(pipelineMs);
    // Nothing reads the capture latency yet so this is made up
    captureLatencyEntry.setDouble(10.0);
  }

  private void noTarget() {
    validTargetEntry.setInteger(0);
    xOffsetEntry.setDouble(0);
    yOffsetEntry.setDouble(0);
    targetAreaEntry.setDouble(0);
  }

  public NetworkTable getNetworkTable() {
    return networkTable;
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    AEMLogger.recordOutput(standardPrefix + "/PythonOutput", lastPythonOutput);
    AEMLogger.recordOutput(standardPrefix + "/PipelineMs", lastPipelineMs);
  }
}
