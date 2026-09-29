package com.aembot.frc2026.simulation.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.aembot.frc2026.constants.field.FieldBB2026;
import com.aembot.frc2026.simulation.arena.ConeZoneArena;
import com.aembot.frc2026.simulation.gamepieces.ConeColor;
import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.nio.file.Files;
import java.nio.file.Path;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

class ConeCameraSimTest {
  private ConeZoneArena arena;
  private CameraConfiguration cameraConfiguration;
  private ConeCameraSim camera;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0));
    arena = new ConeZoneArena();
    SimulatedArena.overrideInstance(arena);

    cameraConfiguration =
        CameraConfiguration.makeLimelight3AConfig("limelight-cones-test")
            .withCameraOffset(
                new Transform3d(
                    new Translation3d(0.3, 0, Units.inchesToMeters(12)), new Rotation3d()));
    camera = new ConeCameraSim(cameraConfiguration, 6.0);
  }

  @AfterEach
  void tearDown() {
    arena.shutDown();
    HAL.shutdown();
  }

  @Test
  void stackInFrontOfTheCameraIsRenderedWhereTheGroundTruthSaysItIs() {
    arena.getTowerStations().scoreCone(5, ConeColor.RED, false);
    arena.getTowerStations().scoreCone(5, ConeColor.BLUE, false);

    Pose2d robot = FieldBB2026.getTowerStation(5).approachPose(2.0);
    ConeCameraSim.Frame frame = camera.render(robot);
    assertEquals(1, frame.stacks().size(), "only station 5 within range and view");

    ConeCameraSim.Detection detection = frame.stacks().get(0);
    assertEquals(5, detection.stationId());
    assertEquals("WRB", detection.colors());
    assertEquals(2, detection.scoredCones());
    assertTrue(Math.abs(detection.txDegrees()) < 3.0, "stack should be centered");

    // Top of the stack is blue. Shading changes brightness not hue, so compare channels in BGR
    Rect bbox = detection.bbox();
    Scalar top =
        Core.mean(new Mat(frame.image(), new Rect(bbox.x + bbox.width / 2 - 2, bbox.y + 6, 4, 4)));
    assertTrue(
        top.val[0] > 2 * top.val[2] && top.val[0] > 2 * top.val[1],
        "top pixel should be blue, got " + top);
  }

  @Test
  void dumpFrameSetForPipelineBench() throws Exception {
    // Not really a test, this writes frames to build/conesim-frames for the Field_BB2026 bench,
    // plus what the java port of the pipeline said about each one so the two can be diffed
    Path directory = Path.of("build", "conesim-frames");
    StringBuilder javaResults = new StringBuilder();

    // A few stacks of different heights and colors
    arena.placeGamePiecesOnField();
    arena.getTowerStations().scoreCone(5, ConeColor.RED, false);
    arena.getTowerStations().scoreCone(5, ConeColor.RED, false);
    arena.getTowerStations().scoreCone(5, ConeColor.BLUE, false);
    arena.getTowerStations().scoreCone(4, ConeColor.BLUE, false);
    arena.getTowerStations().scoreCone(1, ConeColor.RED, false);
    arena.getTowerStations().scoreCone(1, ConeColor.WHITE, false);
    arena.getTowerStations().scoreCone(3, ConeColor.BLUE, false);
    arena.getTowerStations().scoreCone(3, ConeColor.BLUE, false);
    arena.getTowerStations().scoreCone(3, ConeColor.BLUE, false);
    arena.getTowerStations().scoreCone(3, ConeColor.RED, false);

    // Walk up to each one at a few distances and angles
    int index = 0;
    for (int id : new int[] {5, 4, 1, 3, 7, 2}) {
      for (double standoff : new double[] {1.0, 2.0, 3.5}) {
        for (double yawOffset : new double[] {0, 15}) {
          Pose2d pose = FieldBB2026.getTowerStation(id).approachPose(standoff);
          pose =
              new Pose2d(
                  pose.getTranslation(),
                  pose.getRotation().plus(Rotation2d.fromDegrees(yawOffset)));
          dumpWithPipeline(directory, index++, camera.render(pose), pose, javaResults);
        }
      }
    }

    // And some wide shots with lots of floor cones
    for (Pose2d pose :
        new Pose2d[] {
          new Pose2d(FieldBB2026.fromCenter(-200, 0), Rotation2d.kZero),
          new Pose2d(FieldBB2026.fromCenter(0, -120), Rotation2d.fromDegrees(90)),
          new Pose2d(FieldBB2026.fromCenter(150, 100), Rotation2d.fromDegrees(-135)),
        }) {
      dumpWithPipeline(directory, index++, camera.render(pose), pose, javaResults);
    }

    Files.writeString(directory.resolve("java_results.txt"), javaResults.toString());
    assertTrue(Files.exists(directory.resolve("frame_0000.json")));
  }

  // Same line format as bench.py prints, one per frame: name tv bbox cones top
  private void dumpWithPipeline(
      Path directory, int index, ConeCameraSim.Frame frame, Pose2d pose, StringBuilder results)
      throws Exception {
    camera.dump(directory, index, frame, pose);

    ConeStackPipeline.Result result = ConeStackPipeline.runPipeline(frame.image());
    Rect box = ConeStackPipeline.boundingRect(result.largestContour());
    results.append(
        String.format(
            "frame_%04d tv=%d bbox=[%d, %d, %d, %d] cones=%d top=%d%n",
            index,
            (int) result.llpython()[0],
            box.x,
            box.y,
            box.width,
            box.height,
            (int) result.llpython()[5],
            (int) result.llpython()[6]));
  }

  @Test
  void rendersFastEnoughForTheLoop() {
    arena.placeGamePiecesOnField();
    Pose2d pose = new Pose2d(FieldBB2026.fromCenter(-200, 0), Rotation2d.kZero);

    // Warm up so the JIT does not eat the first few frames
    for (int i = 0; i < 5; i++) {
      camera.render(pose);
    }

    long start = System.nanoTime();
    int frames = 20;
    for (int i = 0; i < frames; i++) {
      camera.render(pose);
    }
    double msPerFrame = (System.nanoTime() - start) / 1e6 / frames;

    System.out.println("ConeCameraSim: " + msPerFrame + " ms per frame");
    // Has to fit inside the 15 fps camera thread period at 1280x960
    assertTrue(msPerFrame < 65, "render took " + msPerFrame + " ms");
  }

  @Test
  void coneLimelightPublishesTheGroundTruthStack() {
    arena.getTowerStations().scoreCone(1, ConeColor.RED, false);
    SimulatedConeLimelight limelight = new SimulatedConeLimelight(cameraConfiguration, 25.0);

    Pose2d robot = FieldBB2026.getTowerStation(1).approachPose(1.5);
    limelight.updateFromGroundTruth(camera.render(robot));

    NetworkTable table = NetworkTableInstance.getDefault().getTable(cameraConfiguration.cameraName);
    assertEquals(1, table.getEntry("tv").getInteger(0));
    assertTrue(Math.abs(table.getEntry("tx").getDouble(99)) < 3.0);

    double[] python = table.getEntry("llpython").getDoubleArray(new double[0]);
    assertEquals(SimulatedConeLimelight.LLPYTHON_LENGTH, python.length);
    assertEquals(1, (int) python[4], "station id");
    assertEquals(1, (int) python[5], "scored cones");
    assertEquals(1, (int) python[6], "top color red");

    // Turn away so no station is in view at all
    limelight.updateFromGroundTruth(
        camera.render(new Pose2d(robot.getTranslation(), Rotation2d.kZero)));
    assertEquals(0, table.getEntry("tv").getInteger(1));
  }
}
