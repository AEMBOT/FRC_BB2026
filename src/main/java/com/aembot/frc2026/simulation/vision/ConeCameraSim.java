package com.aembot.frc2026.simulation.vision;

import com.aembot.frc2026.constants.field.FieldBB2026;
import com.aembot.frc2026.simulation.arena.ConeZoneArena;
import com.aembot.frc2026.simulation.gamepieces.ConeColor;
import com.aembot.frc2026.simulation.gamepieces.LargeConeOnField;
import com.aembot.frc2026.simulation.gamepieces.LargeConeOnFly;
import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.RawFrame;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;

/**
 * Renders what the cone camera sees of the simulated field so the pipeline has frames to chew on
 * before the real field exists. Tiny z-buffer rasterizer with flat shading and textured apriltags,
 * no OpenGL needed. Every frame comes with ground truth boxes for the stacks and floor cones in
 * view. The stream is at http://localhost:1181 (first free port from 1181 up) and shows up as a
 * camera source in the sim GUI and AdvantageScope.
 */
public class ConeCameraSim {
  static {
    // cscore only loads the OpenCV natives on demand, so poke it
    OpenCvLoader.forceStaticLoad();
  }

  /**
   * One thing the camera can see
   *
   * @param stationId 1-8 for a stack, 0 for a floor cone
   * @param colors Bottom to top color codes, station stacks start with the white base cone
   * @param scoredCones Cones on top of the base cone, 0 for a floor cone
   * @param bbox Whole object, clipped to the image
   * @param distanceMeters Horizontal distance from the camera
   * @param txDegrees Limelight style horizontal angle to the bbox center, positive right
   * @param tyDegrees Limelight style vertical angle to the bbox center, positive up
   * @param areaPercent Bbox area as a percent of the image
   */
  public record Detection(
      int stationId,
      String colors,
      int scoredCones,
      Rect bbox,
      double distanceMeters,
      double txDegrees,
      double tyDegrees,
      double areaPercent) {}

  public record Frame(Mat image, List<Detection> stacks, List<Detection> floorCones) {
    /** Largest station in view, bare or stacked: what the cone stack pipeline targets */
    public Optional<Detection> getLargestStack() {
      return stacks.stream().max(Comparator.comparingDouble(detection -> detection.bbox().area()));
    }

    /** Largest stack that actually has scored cones on it */
    public Optional<Detection> getLargestScoredStack() {
      return stacks.stream()
          .filter(detection -> detection.scoredCones() > 0)
          .max(Comparator.comparingDouble(detection -> detection.bbox().area()));
    }
  }

  /* ---- MATERIALS ---- */
  // BGR 0-1 plus alpha, same channel order OpenCV uses
  private static final float[] CARPET = {0.38f, 0.36f, 0.36f, 1f};
  private static final float[] WALL = {0.75f, 0.72f, 0.72f, 1f};
  private static final float[] WOOD = {0.24f, 0.44f, 0.62f, 1f};
  private static final float[] HARDBOARD = {0.22f, 0.38f, 0.52f, 1f};
  private static final float[] STAGE = {0.18f, 0.18f, 0.18f, 1f};
  private static final float[] WHITE = {0.95f, 0.95f, 0.95f, 1f};
  private static final float[] RED = {0.10f, 0.10f, 0.85f, 1f};
  private static final float[] BLUE = {0.90f, 0.25f, 0.10f, 1f};
  private static final float[] POLYCARB = {1.0f, 0.9f, 0.8f, 0.3f};
  private static final float[] TAPE_BLUE = {0.9f, 0.3f, 0.15f, 1f};
  private static final float[] TAPE_RED = {0.15f, 0.15f, 0.85f, 1f};
  private static final float[] TAPE_WHITE = {0.95f, 0.95f, 0.95f, 1f};
  private static final float[] BACKGROUND = {0.98f, 0.98f, 0.98f, 1f};

  /* ---- END MATERIALS ---- */

  /* ---- LIGHTING AND CAMERA ---- */
  /** Direction toward the light. Overhead but tilted so cone sides are not all in half shadow */
  private static final float[] LIGHT = normalize(-0.55f, 0.35f, 0.75f);

  private static final float AMBIENT = 0.6f;
  private static final float DIFFUSE = 0.4f;

  /** Anything closer than this to the camera gets clipped */
  private static final float NEAR_PLANE_METERS = 0.05f;

  /** Segments around a cone body. 16 looks round enough at 640x480 */
  private static final int CONE_SEGMENTS = 16;

  /* ---- END LIGHTING AND CAMERA ---- */

  /** A triangle in the field frame with a flat color or a grayscale texture */
  private static final class Triangle {
    final float[] a;
    final float[] b;
    final float[] c;
    final float[] normal;
    final float[] material;
    final Texture texture; // null for flat color
    final float[] uvA;
    final float[] uvB;
    final float[] uvC;

    Triangle(float[] a, float[] b, float[] c, float[] material) {
      this(a, b, c, material, null, null, null, null);
    }

    Triangle(
        float[] a,
        float[] b,
        float[] c,
        float[] material,
        Texture texture,
        float[] uvA,
        float[] uvB,
        float[] uvC) {
      this.a = a;
      this.b = b;
      this.c = c;
      this.material = material;
      this.texture = texture;
      this.uvA = uvA;
      this.uvB = uvB;
      this.uvC = uvC;
      this.normal = normalize(cross(sub(b, a), sub(c, a)));
    }
  }

  private record Texture(int width, int height, byte[] gray) {}

  private final int kWidth;
  private final int kHeight;
  private final double kFx;
  private final double kFy;
  private final double kCx;
  private final double kCy;
  private final double kMaxRangeMeters;
  private final CameraConfiguration kCameraConfiguration;

  private final List<Triangle> staticScene = new ArrayList<>();
  private final float[] depthBuffer;
  private final byte[] pixels;

  private CvSource streamSource = null;
  private Path recordDirectory = null;
  private int recordEvery = 1;
  private int renderCount = 0;
  private int recordedFrames = 0;

  /**
   * Builds the pinhole model from the camera config. The camera position is read from the config
   * every frame so a camera on a moving mechanism is tracked.
   *
   * @param cameraConfiguration Resolution, FOV and mount position of the camera
   * @param maxRangeMeters Detections further than this are not reported
   */
  public ConeCameraSim(CameraConfiguration cameraConfiguration, double maxRangeMeters) {
    this.kCameraConfiguration = cameraConfiguration;
    this.kMaxRangeMeters = maxRangeMeters;

    this.kWidth = cameraConfiguration.cameraResolution.widthPixels;
    this.kHeight = cameraConfiguration.cameraResolution.heightPixels;

    // Pinhole intrinsics from the configured FOV
    this.kFx =
        (kWidth / 2.0)
            / Math.tan(Math.toRadians(cameraConfiguration.cameraFOV.horizontalDegrees / 2.0));
    this.kFy =
        (kHeight / 2.0)
            / Math.tan(Math.toRadians(cameraConfiguration.cameraFOV.verticalDegrees / 2.0));
    this.kCx = kWidth / 2.0;
    this.kCy = kHeight / 2.0;

    this.depthBuffer = new float[kWidth * kHeight];
    this.pixels = new byte[kWidth * kHeight * 3];

    buildStaticScene();
  }

  /** Also push frames to CameraServer so the sim GUI and AdvantageScope can show them */
  public ConeCameraSim withCameraServerStream(String name) {
    // putVideo also registers the MJPEG server, that is where the localhost:1181 stream comes from
    streamSource = CameraServer.putVideo(name, kWidth, kHeight);
    return this;
  }

  /** Start writing every nth rendered frame and its ground truth json to a folder */
  public void startRecording(Path directory, int everyNth) {
    recordDirectory = directory;
    recordEvery = Math.max(1, everyNth);
    recordedFrames = 0;
  }

  public void stopRecording() {
    recordDirectory = null;
  }

  public int getRecordedFrames() {
    return recordedFrames;
  }

  public int getWidth() {
    return kWidth;
  }

  public int getHeight() {
    return kHeight;
  }

  /* ---- SCENE ---- */

  /** Everything that never moves. Carpet, walls, tape, arena, stations, base cones and tags */
  private void buildStaticScene() {
    float length = (float) FieldBB2026.FIELD_LENGTH_METERS;
    float width = (float) FieldBB2026.FIELD_WIDTH_METERS;
    float wallHeight = (float) inches(20);
    float wallThickness = (float) inches(1.5);

    /* ---- FIELD ---- */
    // Carpet is a slab sunk just below z 0 so the tape and cone plates sit on top of it
    addBox(staticScene, CARPET, 0, 0, -0.02f, length, width, 0);

    // 20" walls around the outside, blue wall at x 0 and red wall at x length
    addBox(
        staticScene, WALL, -wallThickness, -wallThickness, 0, 0, width + wallThickness, wallHeight);
    addBox(
        staticScene,
        WALL,
        length,
        -wallThickness,
        0,
        length + wallThickness,
        width + wallThickness,
        wallHeight);
    addBox(staticScene, WALL, 0, -wallThickness, 0, length, 0, wallHeight);
    addBox(staticScene, WALL, 0, width, 0, length, width + wallThickness, wallHeight);

    // 2" tape for the two starting lines and the center line, barely raised so they win the depth
    // test against the carpet
    float tape = (float) inches(2);
    float centerX = (float) FieldBB2026.FIELD_CENTER.getX();
    float blueLine = (float) FieldBB2026.BLUE_STARTING_LINE_X;
    float redLine = (float) FieldBB2026.RED_STARTING_LINE_X;

    addBox(staticScene, TAPE_BLUE, blueLine - tape / 2, 0, 0, blueLine + tape / 2, width, 0.002f);
    addBox(staticScene, TAPE_RED, redLine - tape / 2, 0, 0, redLine + tape / 2, width, 0.002f);
    addBox(staticScene, TAPE_WHITE, centerX - tape / 2, 0, 0, centerX + tape / 2, width, 0.002f);

    /* ---- MINIBOT ARENA ---- */
    // 10' x 8' box in the middle of the field, 16" walls, robots never go in
    float arenaHalfX = (float) FieldBB2026.ARENA_LENGTH_METERS / 2;
    float arenaHalfY = (float) FieldBB2026.ARENA_WIDTH_METERS / 2;
    float wall = (float) FieldBB2026.ARENA_WALL_THICKNESS_METERS;
    float arenaWallHeight = (float) FieldBB2026.ARENA_WALL_HEIGHT_METERS;
    float woodHeight = (float) inches(3.5);
    float centerY = (float) FieldBB2026.FIELD_CENTER.getY();

    // Outside corners of the arena
    float x0 = centerX - arenaHalfX;
    float x1 = centerX + arenaHalfX;
    float y0 = centerY - arenaHalfY;
    float y1 = centerY + arenaHalfY;

    // Hardboard floor, then 3.5" of wood on the bottom of each wall and see through polycarb above
    // it up to the full wall height
    addBox(staticScene, HARDBOARD, x0, y0, 0, x1, y1, (float) inches(0.125));
    float[][][] arenaWalls = {
      {{x0, y0}, {x0 + wall, y1}}, {{x1 - wall, y0}, {x1, y1}},
      {{x0, y0}, {x1, y0 + wall}}, {{x0, y1 - wall}, {x1, y1}}
    };
    for (float[][] w : arenaWalls) {
      addBox(staticScene, WOOD, w[0][0], w[0][1], 0, w[1][0], w[1][1], woodHeight);
      addBox(
          staticScene, POLYCARB, w[0][0], w[0][1], woodHeight, w[1][0], w[1][1], arenaWallHeight);
    }

    // Raised stage in the middle of the arena, 48" x 36", the minibots' problem not ours but it
    // is visible through the polycarb
    float stageHalfX = (float) inches(24);
    float stageHalfY = (float) inches(18);
    float stageHeight = (float) inches(0.93 + 0.5);
    addBox(
        staticScene,
        STAGE,
        centerX - stageHalfX,
        centerY - stageHalfY,
        0,
        centerX + stageHalfX,
        centerY + stageHalfY,
        stageHeight);

    /* ---- TOWER STATIONS ---- */
    // 15" square wooden box, 18" tall, with the white base cone bolted on top. Scored cones are
    // dynamic so they come from buildDynamicScene
    float half = (float) FieldBB2026.TOWER_STATION_SIZE_METERS / 2;
    float stationHeight = (float) FieldBB2026.TOWER_STATION_HEIGHT_METERS;

    for (FieldBB2026.TowerStation station : FieldBB2026.TOWER_STATIONS) {
      float sx = (float) station.center().getX();
      float sy = (float) station.center().getY();

      addBox(staticScene, WOOD, sx - half, sy - half, 0, sx + half, sy + half, stationHeight);
      addCone(staticScene, WHITE, sx, sy, stationHeight);
    }

    /* ---- APRIL TAGS ---- */
    // All 12 tags off the season layout, 8 on the stations and 4 inside the arena
    AprilTagFieldLayout layout = FieldBB2026.get().getFieldLayout();
    for (AprilTag tag : layout.getTags()) {
      addTag(staticScene, tag.pose, tag.ID);
    }
  }

  private static double inches(double inches) {
    return Units.inchesToMeters(inches);
  }

  /** Axis aligned box, all six faces */
  private static void addBox(
      List<Triangle> scene,
      float[] material,
      float x0,
      float y0,
      float z0,
      float x1,
      float y1,
      float z1) {
    // Corner naming is pXYZ with 0 for the min side and 1 for the max side
    float[] p000 = {x0, y0, z0};
    float[] p100 = {x1, y0, z0};
    float[] p010 = {x0, y1, z0};
    float[] p110 = {x1, y1, z0};
    float[] p001 = {x0, y0, z1};
    float[] p101 = {x1, y0, z1};
    float[] p011 = {x0, y1, z1};
    float[] p111 = {x1, y1, z1};

    // Winding doesn't matter for the shading since it uses abs(normal . light)
    addQuad(scene, material, p001, p101, p111, p011); // top
    addQuad(scene, material, p000, p010, p110, p100); // bottom
    addQuad(scene, material, p000, p100, p101, p001); // -y
    addQuad(scene, material, p010, p011, p111, p110); // +y
    addQuad(scene, material, p000, p001, p011, p010); // -x
    addQuad(scene, material, p100, p110, p111, p101); // +x
  }

  private static void addQuad(
      List<Triangle> scene, float[] material, float[] a, float[] b, float[] c, float[] d) {
    // Two triangles sharing the a-c diagonal
    scene.add(new Triangle(a, b, c, material));
    scene.add(new Triangle(a, c, d, material));
  }

  /** An 18" traffic cone standing on baseZ, square base plate plus a tapered body */
  private static void addCone(
      List<Triangle> scene, float[] material, float cx, float cy, float baseZ) {
    // 10.5" square base plate, 3/4" thick. The pipeline keys off this flat foot so it matters
    float plate = (float) FieldBB2026.LARGE_CONE_BASE_WIDTH_METERS / 2;
    float plateHeight = (float) inches(0.75);
    addBox(
        scene,
        material,
        cx - plate,
        cy - plate,
        baseZ,
        cx + plate,
        cy + plate,
        baseZ + plateHeight);

    // Tapered body from the plate to the tip, 4.25" radius at the bottom down to a 1" flat top
    float r0 = (float) inches(4.25);
    float r1 = (float) inches(1.0);
    float z0 = baseZ + plateHeight;
    float z1 = baseZ + (float) FieldBB2026.LARGE_CONE_HEIGHT_METERS;

    // One wedge per segment: a quad on the side (two triangles) plus a cap triangle on top
    for (int i = 0; i < CONE_SEGMENTS; i++) {
      double a0 = 2 * Math.PI * i / CONE_SEGMENTS;
      double a1 = 2 * Math.PI * (i + 1) / CONE_SEGMENTS;
      float[] b0 = {cx + r0 * (float) Math.cos(a0), cy + r0 * (float) Math.sin(a0), z0}; // bottom
      float[] b1 = {cx + r0 * (float) Math.cos(a1), cy + r0 * (float) Math.sin(a1), z0};
      float[] t0 = {cx + r1 * (float) Math.cos(a0), cy + r1 * (float) Math.sin(a0), z1}; // top
      float[] t1 = {cx + r1 * (float) Math.cos(a1), cy + r1 * (float) Math.sin(a1), z1};

      scene.add(new Triangle(b0, b1, t1, material));
      scene.add(new Triangle(b0, t1, t0, material));
      scene.add(new Triangle(new float[] {cx, cy, z1}, t0, t1, material)); // cap
    }
  }

  /** A textured apriltag quad on the tag's pose, +X out of the face */
  private static void addTag(List<Triangle> scene, Pose3d pose, int id) {
    // No texture means WPILib couldn't generate the tag image, just leave it off the box
    Texture texture = tagTexture(id);
    if (texture == null) {
      return;
    }

    // WPILib's tag image includes the white border cells but the 6.5" is just the black square, so
    // the quad has to grow by the border. Walking the diagonal to the first dark cell finds it.
    // Walking the top row does not, that row is all white and you get a 10x tag
    int margin = 0;
    while (margin < texture.width() / 2
        && (texture.gray()[margin * texture.width() + margin] & 0xFF) > 128) {
      margin++;
    }
    double size = inches(6.5) * texture.width() / Math.max(1, texture.width() - 2 * margin);
    double h = size / 2;

    // Tag frame corners, y left and z up when looking at the face. Stood off 3mm so it does not
    // z-fight with the station box
    Translation3d[] corners = {
      new Translation3d(0.003, h, h), // top left seen from the front
      new Translation3d(0.003, -h, h), // top right
      new Translation3d(0.003, -h, -h), // bottom right
      new Translation3d(0.003, h, -h) // bottom left
    };

    // Rotate the corners onto the tag pose and drop them into the field frame
    float[][] world = new float[4][];
    for (int i = 0; i < 4; i++) {
      Translation3d p = pose.getTranslation().plus(corners[i].rotateBy(pose.getRotation()));
      world[i] = new float[] {(float) p.getX(), (float) p.getY(), (float) p.getZ()};
    }

    // Texture coords, u right and v down like the image, so top left of the tag is (0, 0)
    float[] uvTl = {0, 0};
    float[] uvTr = {1, 0};
    float[] uvBr = {1, 1};
    float[] uvBl = {0, 1};

    // Material is only used for alpha here, the texture supplies the gray
    scene.add(new Triangle(world[0], world[1], world[2], WHITE, texture, uvTl, uvTr, uvBr));
    scene.add(new Triangle(world[0], world[2], world[3], WHITE, texture, uvTl, uvBr, uvBl));
  }

  private static Texture tagTexture(int id) {
    // WPILib hands back a tiny 8 bit gray image, 10x10 for 36h11 including the white border
    try (RawFrame frame = AprilTag.generate36h11AprilTagImage(id)) {
      int width = frame.getWidth();
      int height = frame.getHeight();
      ByteBuffer data = frame.getData();

      // Rows are padded out to the stride so copy them out one at a time
      byte[] gray = new byte[width * height];
      int stride = frame.getStride() > 0 ? frame.getStride() : width;
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          gray[y * width + x] = data.get(y * stride + x);
        }
      }

      return new Texture(width, height, gray);
    } catch (Exception e) {
      // Native lib not loaded or a bad id, either way no tag
      return null;
    }
  }

  /** Cones on the carpet, in flight and seated on the stations, rebuilt every frame */
  private List<Triangle> buildDynamicScene(ConeZoneArena arena) {
    List<Triangle> scene = new ArrayList<>();

    for (ConeColor color : ConeColor.values()) {
      float[] material = materialOf(color);

      // Cones sitting on the carpet, maplesim gives their pose at floor level
      for (LargeConeOnField cone : arena.getConesOnField(color)) {
        Translation3d center = cone.getPose3d().getTranslation();
        addCone(scene, material, (float) center.getX(), (float) center.getY(), 0f);
      }

      // Flying and seated poses are the cone center, addCone wants the base
      for (LargeConeOnFly cone : arena.getConesInFlight(color)) {
        Translation3d center = cone.getPose3d().getTranslation();
        addCone(
            scene,
            material,
            (float) center.getX(),
            (float) center.getY(),
            (float) (center.getZ() - FieldBB2026.LARGE_CONE_HEIGHT_METERS / 2));
      }

      // Cones seated on stations, one pose per cone from the stack tracker
      for (Pose3d pose : arena.getTowerStations().getConePoses(color)) {
        addCone(
            scene,
            material,
            (float) pose.getX(),
            (float) pose.getY(),
            (float) (pose.getZ() - FieldBB2026.LARGE_CONE_HEIGHT_METERS / 2));
      }
    }

    return scene;
  }

  private static float[] materialOf(ConeColor color) {
    switch (color) {
      case RED:
        return RED;
      case BLUE:
        return BLUE;
      case WHITE:
      default:
        return WHITE;
    }
  }

  /* ---- CAMERA ---- */

  private Pose3d getCameraPose(Pose2d robotPose) {
    // Read from the config every frame in case the camera is on something that moves
    Pose3d cameraOnRobot = kCameraConfiguration.getCameraPosition();
    return new Pose3d(robotPose)
        .plus(new Transform3d(cameraOnRobot.getTranslation(), cameraOnRobot.getRotation()));
  }

  /** Row major rotation plus translation that take field points into the camera frame */
  private static float[] viewMatrix(Pose3d camera) {
    // Inverse of the camera rotation takes field vectors into the camera frame
    double[][] r = toRows(camera.getRotation().unaryMinus());
    Translation3d t = camera.getTranslation();

    // Rotation rows first then the camera position, toCamera knows the layout
    return new float[] {
      (float) r[0][0], (float) r[0][1], (float) r[0][2],
      (float) r[1][0], (float) r[1][1], (float) r[1][2],
      (float) r[2][0], (float) r[2][1], (float) r[2][2],
      (float) t.getX(), (float) t.getY(), (float) t.getZ()
    };
  }

  private static double[][] toRows(Rotation3d rotation) {
    Matrix<N3, N3> m = rotation.toMatrix();
    double[][] rows = new double[3][3];

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        rows[i][j] = m.get(i, j);
      }
    }

    return rows;
  }

  /** Field point to camera frame, x forward y left z up */
  private static float[] toCamera(float[] view, float[] p) {
    // Translate so the camera is the origin, then rotate. Keeps WPILib's axes, no OpenGL flip
    float x = p[0] - view[9];
    float y = p[1] - view[10];
    float z = p[2] - view[11];

    return new float[] {
      view[0] * x + view[1] * y + view[2] * z,
      view[3] * x + view[4] * y + view[5] * z,
      view[6] * x + view[7] * y + view[8] * z
    };
  }

  /* ---- RENDERING ---- */

  /** Render the field as seen from the camera at the simulated robot pose */
  public Frame render(Pose2d robotPose) {
    ConeZoneArena arena = (ConeZoneArena) SimulatedArena.getInstance();
    Pose3d camera = getCameraPose(robotPose);
    float[] view = viewMatrix(camera);

    // Clear the depth buffer and paint the background. Near white so the sky reads like a lit
    // gym wall instead of black
    Arrays.fill(depthBuffer, Float.MAX_VALUE);
    byte bgB = (byte) (BACKGROUND[0] * 255);
    byte bgG = (byte) (BACKGROUND[1] * 255);
    byte bgR = (byte) (BACKGROUND[2] * 255);
    for (int i = 0; i < pixels.length; i += 3) {
      pixels[i] = bgB;
      pixels[i + 1] = bgG;
      pixels[i + 2] = bgR;
    }

    // Opaque first, then the polycarb on top without writing depth so it blends over whatever is
    // behind it. Only the static scene has translucent stuff
    List<Triangle> dynamicScene = buildDynamicScene(arena);
    List<Triangle> transparent = new ArrayList<>();

    for (Triangle triangle : staticScene) {
      if (triangle.material[3] < 1f) {
        transparent.add(triangle);
      } else {
        drawTriangle(triangle, view, true);
      }
    }
    for (Triangle triangle : dynamicScene) {
      drawTriangle(triangle, view, true);
    }

    for (Triangle triangle : transparent) {
      drawTriangle(triangle, view, false);
    }

    // Wrap the pixel buffer in a Mat. Streaming is a separate call so the pipeline can draw on
    // the frame first, like the real Limelight's stream shows its detections
    Mat image = new Mat(kHeight, kWidth, CvType.CV_8UC3);
    image.put(0, 0, pixels);

    // Ground truth goes out alongside the image so the fake limelight has something to publish
    Frame frame =
        new Frame(image, findStacks(arena, camera, view), findFloorCones(arena, camera, view));

    // Every nth frame to disk while recording, for the pipeline bench
    if (recordDirectory != null && renderCount % recordEvery == 0) {
      try {
        dump(recordDirectory, recordedFrames++, frame, robotPose);
      } catch (IOException e) {
        System.err.println("ConeCameraSim: could not write frame: " + e.getMessage());
      }
    }
    renderCount++;

    return frame;
  }

  /** Push a frame to the CameraServer stream, if there is one */
  public void stream(Frame frame) {
    if (streamSource != null) {
      streamSource.putFrame(frame.image());
    }
  }

  /** Shade, clip against the near plane, project and rasterize one triangle */
  private void drawTriangle(Triangle triangle, float[] view, boolean writeDepth) {
    // Flat Lambert shading per triangle, abs so back faces light the same as front faces
    float lambert = Math.max(0f, Math.abs(dot(triangle.normal, LIGHT)));
    float shade = AMBIENT + DIFFUSE * lambert;

    // Into the camera frame, uvs ride along only for textured tags
    float[][] verts = {
      toCamera(view, triangle.a), toCamera(view, triangle.b), toCamera(view, triangle.c)
    };
    float[][] uvs =
        triangle.texture == null ? null : new float[][] {triangle.uvA, triangle.uvB, triangle.uvC};

    // Sutherland-Hodgman clip against x = NEAR_PLANE_METERS, camera x is depth here. Without this
    // anything crossing behind the camera projects to garbage
    List<float[]> polygon = new ArrayList<>(4);
    List<float[]> polygonUv = new ArrayList<>(4);
    for (int i = 0; i < 3; i++) {
      float[] current = verts[i];
      float[] next = verts[(i + 1) % 3];
      float[] currentUv = uvs == null ? null : uvs[i];
      float[] nextUv = uvs == null ? null : uvs[(i + 1) % 3];

      boolean currentIn = current[0] >= NEAR_PLANE_METERS;
      boolean nextIn = next[0] >= NEAR_PLANE_METERS;

      // Keep the vertex if it is in front, and add the crossing point if the edge crosses the plane
      if (currentIn) {
        polygon.add(current);
        polygonUv.add(currentUv);
      }
      if (currentIn != nextIn) {
        float t = (NEAR_PLANE_METERS - current[0]) / (next[0] - current[0]);
        polygon.add(lerp(current, next, t));
        polygonUv.add(currentUv == null ? null : lerp(currentUv, nextUv, t));
      }
    }
    if (polygon.size() < 3) {
      return; // Entirely behind the near plane
    }

    // Fan the clipped polygon back into triangles, a clipped triangle is at most a quad
    for (int i = 1; i + 1 < polygon.size(); i++) {
      rasterize(
          polygon.get(0),
          polygon.get(i),
          polygon.get(i + 1),
          polygonUv.get(0),
          polygonUv.get(i),
          polygonUv.get(i + 1),
          triangle,
          shade,
          writeDepth);
    }
  }

  private void rasterize(
      float[] a,
      float[] b,
      float[] c,
      float[] uvA,
      float[] uvB,
      float[] uvC,
      Triangle triangle,
      float shade,
      boolean writeDepth) {
    // Camera frame to pixels, image right is -y and image down is -z
    float ax = (float) (kCx - kFx * a[1] / a[0]);
    float ay = (float) (kCy - kFy * a[2] / a[0]);
    float bx = (float) (kCx - kFx * b[1] / b[0]);
    float by = (float) (kCy - kFy * b[2] / b[0]);
    float cx = (float) (kCx - kFx * c[1] / c[0]);
    float cy = (float) (kCy - kFy * c[2] / c[0]);

    // Twice the signed area, zero means edge on or degenerate
    float area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    if (Math.abs(area) < 1e-6f) {
      return;
    }

    // Screen bounding box, clipped to the image
    int minX = Math.max(0, (int) Math.floor(Math.min(ax, Math.min(bx, cx))));
    int maxX = Math.min(kWidth - 1, (int) Math.ceil(Math.max(ax, Math.max(bx, cx))));
    int minY = Math.max(0, (int) Math.floor(Math.min(ay, Math.min(by, cy))));
    int maxY = Math.min(kHeight - 1, (int) Math.ceil(Math.max(ay, Math.max(by, cy))));
    if (minX > maxX || minY > maxY) {
      return;
    }

    // 1/depth is what interpolates linearly in screen space, not depth itself
    float invA = 1f / a[0];
    float invB = 1f / b[0];
    float invC = 1f / c[0];
    float invArea = 1f / area;

    float[] material = triangle.material;
    float alpha = material[3];
    boolean textured = triangle.texture != null && uvA != null && uvB != null && uvC != null;

    // Walk every pixel center in the bounding box
    for (int y = minY; y <= maxY; y++) {
      float py = y + 0.5f;
      for (int x = minX; x <= maxX; x++) {
        float px = x + 0.5f;
        // Edge functions give the barycentric weights, a negative one means outside
        float w0 = ((bx - px) * (cy - py) - (by - py) * (cx - px)) * invArea;
        float w1 = ((cx - px) * (ay - py) - (cy - py) * (ax - px)) * invArea;
        float w2 = 1f - w0 - w1;
        if (w0 < 0 || w1 < 0 || w2 < 0) {
          continue;
        }

        // Depth test, translucent triangles test but don't write so they can overlap each other
        float invDepth = w0 * invA + w1 * invB + w2 * invC;
        float depth = 1f / invDepth;
        int index = y * kWidth + x;
        if (depth >= depthBuffer[index]) {
          continue;
        }
        if (writeDepth) {
          depthBuffer[index] = depth;
        }

        // Flat color unless there is a texture, then sample the tag's gray
        float bC = material[0];
        float gC = material[1];
        float rC = material[2];
        if (textured) {
          // Perspective correct uv, same 1/depth trick
          float u = (w0 * uvA[0] * invA + w1 * uvB[0] * invB + w2 * uvC[0] * invC) * depth;
          float v = (w0 * uvA[1] * invA + w1 * uvB[1] * invB + w2 * uvC[1] * invC) * depth;
          int tx =
              Math.min(
                  triangle.texture.width() - 1, Math.max(0, (int) (u * triangle.texture.width())));
          int ty =
              Math.min(
                  triangle.texture.height() - 1,
                  Math.max(0, (int) (v * triangle.texture.height())));
          float g = (triangle.texture.gray()[ty * triangle.texture.width() + tx] & 0xFF) / 255f;
          bC = g;
          gC = g;
          rC = g;
        }

        // Write the shaded color, blending with what is already there for the polycarb
        int p = index * 3;
        if (alpha >= 1f) {
          pixels[p] = toByte(bC * shade);
          pixels[p + 1] = toByte(gC * shade);
          pixels[p + 2] = toByte(rC * shade);
        } else {
          pixels[p] = toByte((1 - alpha) * ((pixels[p] & 0xFF) / 255f) + alpha * bC * shade);
          pixels[p + 1] =
              toByte((1 - alpha) * ((pixels[p + 1] & 0xFF) / 255f) + alpha * gC * shade);
          pixels[p + 2] =
              toByte((1 - alpha) * ((pixels[p + 2] & 0xFF) / 255f) + alpha * rC * shade);
        }
      }
    }
  }

  /* ---- GROUND TRUTH ---- */

  /** Bounding boxes of the station stacks in range, occlusion ignored */
  private List<Detection> findStacks(ConeZoneArena arena, Pose3d camera, float[] view) {
    List<Detection> stacks = new ArrayList<>();
    Translation2d cameraXY = camera.getTranslation().toTranslation2d();

    for (FieldBB2026.TowerStation station : FieldBB2026.TOWER_STATIONS) {
      // Too far to be worth reporting, the real pipeline won't see it either
      Translation2d center = station.center();
      double range = center.getDistance(cameraXY);
      if (range > kMaxRangeMeters) {
        continue;
      }

      // Skip stations behind the camera, projecting those gives mirrored nonsense
      float baseZ = (float) FieldBB2026.TOWER_STATION_HEIGHT_METERS;
      float[] baseCenter = {
        (float) center.getX(),
        (float) center.getY(),
        baseZ + (float) FieldBB2026.LARGE_CONE_HEIGHT_METERS / 2
      };
      if (toCamera(view, baseCenter)[0] <= 0) {
        continue;
      }

      // Box of the white base cone grown by each scored cone on top of it
      Rect bbox = coneBounds(view, (float) center.getX(), (float) center.getY(), baseZ);
      StringBuilder codes = new StringBuilder("W");
      List<ConeColor> scored = arena.getTowerStations().getStack(station.tagId()).getCones();

      for (int k = 0; k < scored.size(); k++) {
        Pose3d pose = arena.getTowerStations().getConePose(station.tagId(), k);
        codes.append(scored.get(k).getLetter());
        bbox =
            union(
                bbox,
                coneBounds(
                    view,
                    (float) pose.getX(),
                    (float) pose.getY(),
                    (float) (pose.getZ() - FieldBB2026.LARGE_CONE_HEIGHT_METERS / 2)));
      }

      // Nothing in the image if the box clipped to zero
      if (bbox != null && bbox.area() > 0) {
        stacks.add(detection(station.tagId(), codes.toString(), scored.size(), bbox, range));
      }
    }

    return stacks;
  }

  /** Bounding boxes of the loose cones in range, same rules as the stacks */
  private List<Detection> findFloorCones(ConeZoneArena arena, Pose3d camera, float[] view) {
    List<Detection> cones = new ArrayList<>();
    Translation2d cameraXY = camera.getTranslation().toTranslation2d();

    for (ConeColor color : ConeColor.values()) {
      for (LargeConeOnField cone : arena.getConesOnField(color)) {
        Translation3d center = cone.getPose3d().getTranslation();
        double range = center.toTranslation2d().getDistance(cameraXY);
        if (range > kMaxRangeMeters) {
          continue;
        }

        // Behind the camera
        float[] c = {(float) center.getX(), (float) center.getY(), (float) center.getZ()};
        if (toCamera(view, c)[0] <= 0) {
          continue;
        }

        // Floor cones always stand on the carpet so the base is at z 0
        Rect bbox = coneBounds(view, c[0], c[1], 0f);
        if (bbox != null && bbox.area() > 0) {
          cones.add(detection(0, "" + color.getLetter(), 0, bbox, range));
        }
      }
    }

    return cones;
  }

  /** Image bounding box of a cone's base ring and tip, clipped to the image. Null if behind us */
  private Rect coneBounds(float[] view, float cx, float cy, float baseZ) {
    float r = (float) FieldBB2026.LARGE_CONE_BASE_WIDTH_METERS / 2;
    float tipZ = baseZ + (float) FieldBB2026.LARGE_CONE_HEIGHT_METERS;

    double x0 = Double.MAX_VALUE;
    double y0 = Double.MAX_VALUE;
    double x1 = -Double.MAX_VALUE;
    double y1 = -Double.MAX_VALUE;

    // 12 points around the base ring and then the tip
    for (int i = 0; i <= 12; i++) {
      float[] p =
          i < 12
              ? new float[] {
                cx + r * (float) Math.cos(2 * Math.PI * i / 12),
                cy + r * (float) Math.sin(2 * Math.PI * i / 12),
                baseZ
              }
              : new float[] {cx, cy, tipZ};
      float[] c = toCamera(view, p);
      if (c[0] < NEAR_PLANE_METERS) {
        return null; // Part of the cone is behind us, no sane box for that
      }

      // Same projection as rasterize, grow the box to cover the point
      double u = kCx - kFx * c[1] / c[0];
      double v = kCy - kFy * c[2] / c[0];
      x0 = Math.min(x0, u);
      y0 = Math.min(y0, v);
      x1 = Math.max(x1, u);
      y1 = Math.max(y1, v);
    }

    // Clip to the image
    int ix0 = (int) Math.max(0, Math.floor(x0));
    int iy0 = (int) Math.max(0, Math.floor(y0));
    int ix1 = (int) Math.min(kWidth, Math.ceil(x1));
    int iy1 = (int) Math.min(kHeight, Math.ceil(y1));

    if (ix1 <= ix0 || iy1 <= iy0) {
      return new Rect(0, 0, 0, 0); // Fully off screen
    }
    return new Rect(ix0, iy0, ix1 - ix0, iy1 - iy0);
  }

  private static Rect union(Rect a, Rect b) {
    if (a == null || a.area() == 0) {
      return b;
    }
    if (b == null || b.area() == 0) {
      return a;
    }

    int x0 = Math.min(a.x, b.x);
    int y0 = Math.min(a.y, b.y);
    int x1 = Math.max(a.x + a.width, b.x + b.width);
    int y1 = Math.max(a.y + a.height, b.y + b.height);

    return new Rect(x0, y0, x1 - x0, y1 - y0);
  }

  private Detection detection(
      int stationId, String colors, int scored, Rect bbox, double distance) {
    // tx and ty are angles to the box center like a limelight reports, ta is percent of the image
    double u = bbox.x + bbox.width / 2.0;
    double v = bbox.y + bbox.height / 2.0;

    return new Detection(
        stationId,
        colors,
        scored,
        bbox,
        distance,
        Math.toDegrees(Math.atan((u - kCx) / kFx)),
        Math.toDegrees(Math.atan((kCy - v) / kFy)),
        100.0 * bbox.area() / (kWidth * kHeight));
  }

  /** Writes frame_NNNN.png and the ground truth frame_NNNN.json to a folder */
  public void dump(Path directory, int index, Frame frame, Pose2d robotPose) throws IOException {
    // frame_0000.png and frame_0000.json side by side, bench.py pairs them up by name
    Files.createDirectories(directory);
    String base = String.format("frame_%04d", index);
    Imgcodecs.imwrite(directory.resolve(base + ".png").toString(), frame.image());

    // Hand rolled json, not worth a library for this. Intrinsics and the robot pose go first so a
    // frame can be reprojected without the sim
    StringBuilder json = new StringBuilder();
    json.append("{\n  \"width\": ").append(kWidth).append(", \"height\": ").append(kHeight);
    json.append(
        String.format(
            ",\n  \"fx\": %.3f, \"fy\": %.3f, \"cx\": %.3f, \"cy\": %.3f", kFx, kFy, kCx, kCy));
    json.append(
        String.format(
            ",\n  \"robotPose\": [%.4f, %.4f, %.4f]",
            robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees()));

    // Then the two detection lists, same fields as the pipeline's own output
    json.append(",\n  \"stacks\": [");
    appendDetections(json, frame.stacks());
    json.append("],\n  \"floorCones\": [");
    appendDetections(json, frame.floorCones());
    json.append("]\n}\n");

    Files.writeString(directory.resolve(base + ".json"), json.toString());
  }

  private static void appendDetections(StringBuilder json, List<Detection> detections) {
    for (int i = 0; i < detections.size(); i++) {
      Detection d = detections.get(i);

      // coloredBbox is null here, only the real pipeline fills it in
      json.append(i == 0 ? "\n" : ",\n");
      json.append(
          String.format(
              "    {\"station\": %d, \"colors\": \"%s\", \"scoredCones\": %d, \"bbox\": [%d, %d, %d, %d], \"coloredBbox\": null, \"distance\": %.3f, \"tx\": %.2f, \"ty\": %.2f, \"ta\": %.3f}",
              d.stationId(),
              d.colors(),
              d.scoredCones(),
              d.bbox().x,
              d.bbox().y,
              d.bbox().width,
              d.bbox().height,
              d.distanceMeters(),
              d.txDegrees(),
              d.tyDegrees(),
              d.areaPercent()));
    }
    if (!detections.isEmpty()) {
      json.append("\n  ");
    }
  }

  /* ---- VECTOR HELPERS ---- */

  private static float[] sub(float[] a, float[] b) {
    return new float[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
  }

  private static float[] cross(float[] a, float[] b) {
    return new float[] {
      a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]
    };
  }

  private static float dot(float[] a, float[] b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  private static float[] normalize(float x, float y, float z) {
    float m = (float) Math.sqrt(x * x + y * y + z * z);
    return m < 1e-9f ? new float[] {0, 0, 1} : new float[] {x / m, y / m, z / m};
  }

  private static float[] normalize(float[] v) {
    return normalize(v[0], v[1], v[2]);
  }

  private static float[] lerp(float[] a, float[] b, float t) {
    float[] out = new float[a.length];
    for (int i = 0; i < a.length; i++) {
      out[i] = a[i] + (b[i] - a[i]) * t;
    }
    return out;
  }

  private static byte toByte(float value) {
    return (byte) Math.max(0, Math.min(255, Math.round(value * 255f)));
  }
}
