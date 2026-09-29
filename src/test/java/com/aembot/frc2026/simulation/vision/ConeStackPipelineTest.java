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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opencv.core.Rect;

class ConeStackPipelineTest {
  private ConeZoneArena arena;
  private ConeCameraSim camera;
  private CameraConfiguration cameraConfiguration;

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
  void findsTheStackInFrontOfTheRobot() {
    arena.getTowerStations().scoreCone(5, ConeColor.RED, false);
    arena.getTowerStations().scoreCone(5, ConeColor.RED, false);
    arena.getTowerStations().scoreCone(5, ConeColor.BLUE, false);

    ConeCameraSim.Frame frame = camera.render(FieldBB2026.getTowerStation(5).approachPose(1.5));
    ConeStackPipeline.Result result = ConeStackPipeline.runPipeline(frame.image());

    assertEquals(1, (int) result.llpython()[0], "tv");
    assertEquals(2, (int) result.llpython()[6], "top should be blue");
    // Nested same color cones are counted from blob aspect ratio, so give it one either way
    int cones = (int) result.llpython()[5];
    assertTrue(cones >= 2 && cones <= 4, "three scored cones, pipeline said " + cones);

    // Same box the renderer says the stack is in
    Rect truth = frame.getLargestScoredStack().orElseThrow().bbox();
    Rect found = ConeStackPipeline.boundingRect(result.largestContour());
    assertTrue(iou(truth, found) > 0.5, "pipeline box " + found + " vs truth " + truth);
  }

  @Test
  void bareStationIsATargetWithNothingScored() {
    ConeCameraSim.Frame frame = camera.render(FieldBB2026.getTowerStation(5).approachPose(1.5));
    ConeStackPipeline.Result result = ConeStackPipeline.runPipeline(frame.image());

    // The robot has to be able to line up on an empty station too
    assertEquals(1, (int) result.llpython()[0], "tv");
    assertEquals(0, (int) result.llpython()[5], "no scored cones");
    assertEquals(0, (int) result.llpython()[6], "no top color");
  }

  @Test
  void coneOnTheCarpetIsNotATarget() {
    Pose2d robot = new Pose2d(FieldBB2026.fromCenter(-200, 60), Rotation2d.kZero);
    arena.spawnCone(ConeColor.RED, robot.getTranslation().plus(new Translation2d(1.5, 0)));

    ConeCameraSim.Frame frame = camera.render(robot);
    ConeStackPipeline.Result result = ConeStackPipeline.runPipeline(frame.image());

    // Stations further down the field are fair game, the cone right in front of us is not
    Rect floorCone = frame.floorCones().get(0).bbox();
    for (ConeStackPipeline.Stack stack : result.stacks()) {
      assertTrue(iou(floorCone, stack.bbox()) < 0.1, "floor cone got picked up as " + stack.bbox());
      assertEquals("", stack.cones(), "anything found should be a bare station");
    }
  }

  @Test
  void fakeLimelightPublishesWhatThePipelineFound() {
    arena.getTowerStations().scoreCone(1, ConeColor.BLUE, false);
    SimulatedConeLimelight limelight = new SimulatedConeLimelight(cameraConfiguration, 25.0);

    ConeCameraSim.Frame frame = camera.render(FieldBB2026.getTowerStation(1).approachPose(1.5));
    limelight.updateFromPipeline(frame);

    assertEquals(1, limelight.getNetworkTable().getEntry("tv").getInteger(0));
    double[] llpython =
        limelight.getNetworkTable().getEntry("llpython").getDoubleArray(new double[0]);
    assertEquals(SimulatedConeLimelight.LLPYTHON_LENGTH, llpython.length);
    assertEquals(2, (int) llpython[6], "blue on top");

    // Station is dead ahead, tx should be near zero either way
    double tx = limelight.getNetworkTable().getEntry("tx").getDouble(99);
    assertTrue(Math.abs(tx) < 5, "tx " + tx);
  }

  @Test
  void runsFastEnoughForTheLoop() {
    arena.placeGamePiecesOnField();
    arena.getTowerStations().scoreCone(5, ConeColor.RED, false);
    ConeCameraSim.Frame frame = camera.render(FieldBB2026.getTowerStation(5).approachPose(2.0));

    // Warm up so the JIT does not eat the first few frames
    for (int i = 0; i < 5; i++) {
      ConeStackPipeline.runPipeline(frame.image().clone());
    }

    long start = System.nanoTime();
    int frames = 20;
    for (int i = 0; i < frames; i++) {
      ConeStackPipeline.runPipeline(frame.image().clone());
    }
    double msPerFrame = (System.nanoTime() - start) / 1e6 / frames;

    System.out.println("ConeStackPipeline: " + msPerFrame + " ms per frame");
    assertTrue(msPerFrame < 30, "pipeline took " + msPerFrame + " ms");
  }

  private static double iou(Rect a, Rect b) {
    int ix = Math.max(0, Math.min(a.x + a.width, b.x + b.width) - Math.max(a.x, b.x));
    int iy = Math.max(0, Math.min(a.y + a.height, b.y + b.height) - Math.max(a.y, b.y));
    double inter = ix * iy;
    return inter == 0 ? 0 : inter / (a.area() + b.area() - inter);
  }
}
