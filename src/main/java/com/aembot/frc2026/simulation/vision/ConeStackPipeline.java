package com.aembot.frc2026.simulation.vision;

import edu.wpi.first.cscore.OpenCvLoader;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * Java port of tools/limelight/cone_stack_pipeline.py from Field_BB2026, the OpenCV pipeline that
 * runs on the cone Limelight, so the sim can run the real detection without a python process. The
 * python is still what gets pasted into the Limelight so keep the two in sync, same functions in
 * the same order, a frame turns into a stack as masks, blobs, stacks, station check, output. The
 * bench dump test writes java_results.txt next to the frames to diff against the python bench
 */
public final class ConeStackPipeline {
  static {
    OpenCvLoader.forceStaticLoad();
  }

  /* ---- CAMERA ---- */
  // Limelight 3A, 54.5 by 42 degree fov, run at its full 1280x960. Pixel thresholds are given at
  // REFERENCE_WIDTH and scaled by frame width so 640x480 frames still work. The python has a slot
  // for measured intrinsics, the sim camera is a perfect pinhole so there is none here
  private static final int REFERENCE_WIDTH = 1280;
  private static final double HFOV_DEG = 54.5;
  private static final double VFOV_DEG = 42.0;
  /* ---- END CAMERA ---- */

  /* ---- COLOR THRESHOLDS ---- */
  // HSV, OpenCV ranges (H 0-180, S/V 0-255). Tuned on the lit renderer and on AdvantageScope
  // footage, will need a re-tune on real footage

  // Red wraps around hue 0 so it needs a range at each end
  private static final Scalar RED_LO_1 = new Scalar(0, 110, 70);
  private static final Scalar RED_HI_1 = new Scalar(8, 255, 255);
  private static final Scalar RED_LO_2 = new Scalar(172, 110, 70);
  private static final Scalar RED_HI_2 = new Scalar(180, 255, 255);

  private static final Scalar BLUE_LO = new Scalar(100, 110, 50);
  private static final Scalar BLUE_HI = new Scalar(130, 255, 255);

  /** White cones including the shaded side, V above 247 is sky or ceiling */
  private static final Scalar WHITE_LO = new Scalar(0, 0, 150);

  private static final Scalar WHITE_HI = new Scalar(180, 40, 247);

  /** The wooden station box a stack sits on, hue about 10-25 */
  private static final Scalar WOOD_LO = new Scalar(5, 40, 50);

  private static final Scalar WOOD_HI = new Scalar(30, 255, 255);
  /* ---- END COLOR THRESHOLDS ---- */

  /* ---- BLOB FILTERS ---- */
  // Pixel sizes at REFERENCE_WIDTH, pxScale() shrinks them for smaller frames
  private static final double MIN_CONE_AREA_PX = 600;
  private static final double MIN_CONE_HEIGHT_PX = 28;

  /** A blob wider than this fraction of the image is sky, ceiling or wall, not a cone */
  private static final double MAX_BLOB_WIDTH_FRAC = 0.4;

  /* ---- END BLOB FILTERS ---- */

  /* ---- STACKING ---- */
  /** Fraction of width two blobs must overlap horizontally to be one stack */
  private static final double STACK_X_OVERLAP = 0.4;

  /** Sanity cap on the nested cone estimate */
  private static final int MAX_CONES_PER_BLOB = 8;

  /** A white blob with nothing on it has to be taller than wide to pass as a base cone */
  private static final double BARE_CONE_MIN_ASPECT = 1.2;

  // A large cone is 18" tall on a 10.5" base and each nested cone adds about 3". A blob of N nested
  // same color cones has height/width of about SINGLE_CONE_ASPECT + (N - 1) * NESTED_ASPECT_STEP
  private static final double SINGLE_CONE_ASPECT = 18.0 / 10.5;
  private static final double NESTED_ASPECT_STEP = 3.0 / 10.5;

  /* ---- END STACKING ---- */

  /* ---- STATION CHECK ---- */
  /** Look this many stack widths below the colored cones for the box, past the base cone */
  private static final double STATION_SEARCH_DEPTH = 2.6;

  /** Wood under a stack wider than this times the stack width is the arena wall, not a station */
  private static final double STATION_MAX_WIDTH_RATIO = 2.5;

  /* ---- END STATION CHECK ---- */

  private static final Scalar YELLOW = new Scalar(0, 255, 255);
  private static final Scalar GREEN = new Scalar(0, 255, 0);
  private static final Mat OPEN_KERNEL = Mat.ones(3, 3, CvType.CV_8U);

  /** One connected blob of a single color */
  private static final class Blob {
    final char color;
    final int x;
    final int y;
    final int w;
    final int h;
    final MatOfPoint contour;

    Blob(char color, Rect rect, MatOfPoint contour) {
      this.color = color;
      this.x = rect.x;
      this.y = rect.y;
      this.w = rect.width;
      this.h = rect.height;
      this.contour = contour;
    }

    int bottom() {
      return y + h;
    }

    int right() {
      return x + w;
    }
  }

  /** Blobs grouped into one vertical stack while it is being built, box kept as x y x2 y2 */
  private static final class Group {
    final List<Blob> members = new ArrayList<>();
    int x;
    int y;
    int x2;
    int y2;

    Group(Blob first) {
      members.add(first);
      x = first.x;
      y = first.y;
      x2 = first.right();
      y2 = first.bottom();
    }

    /** Puts the blob on top of the stack and grows the box to take it in */
    void add(Blob blob) {
      members.add(blob);
      x = Math.min(x, blob.x);
      y = Math.min(y, blob.y);
      x2 = Math.max(x2, blob.right());
      y2 = Math.max(y2, blob.bottom());
    }

    Blob top() {
      return members.get(members.size() - 1);
    }

    int width() {
      return x2 - x;
    }
  }

  /**
   * A detected stack
   *
   * @param bbox Whole stack in pixels, base cone included
   * @param cones Bottom to top codes of the scored cones, one letter per estimated cone, R B or W
   * @param top Code of the topmost scored cone, a space for a bare station
   * @param contour Convex hull of every blob in the stack, what tx/ty latch onto on the Limelight
   */
  public record Stack(Rect bbox, String cones, char top, MatOfPoint contour) {}

  /**
   * What runPipeline hands back, same three things the python returns
   *
   * @param largestContour Hull of the chosen stack, empty if none
   * @param llpython [tv, tx, ty, ta, 0, scoredCones, topColor 1 red 2 blue 3 white 0 bare, 0]
   * @param stacks Every stack found, largest first is not guaranteed
   */
  public record Result(MatOfPoint largestContour, double[] llpython, List<Stack> stacks) {}

  private ConeStackPipeline() {}

  /* ---- OUTPUT ---- */

  /**
   * Same as the python runPipeline. Picks the biggest stack in view, fills in llpython for it and
   * draws every stack onto the image like the Limelight stream would show
   *
   * @param image BGR frame, annotated in place
   * @return The largest stack, its llpython and every stack found
   */
  public static Result runPipeline(Mat image) {
    List<Stack> stacks = findStacks(image);
    if (stacks.isEmpty()) {
      return new Result(new MatOfPoint(), new double[8], stacks);
    }

    Stack best = stacks.get(0);
    for (Stack stack : stacks) {
      if (stack.bbox().area() > best.bbox().area()) {
        best = stack;
      }
    }

    double[] angles = targetAngles(best.bbox(), image.cols(), image.rows());
    double[] llpython =
        new double[] {
          1, // tv
          angles[0], // tx
          angles[1], // ty
          angles[2], // ta
          0, // station id, sim ground truth only
          best.cones().length(),
          topColorCode(best.top()),
          0 // distance, sim ground truth only
        };

    draw(image, stacks, best);
    return new Result(best.contour(), llpython, stacks);
  }

  /** 1 red, 2 blue, 3 white, 0 anything else, the llpython convention */
  public static int topColorCode(char color) {
    switch (color) {
      case 'R':
        return 1;
      case 'B':
        return 2;
      case 'W':
        return 3;
      default:
        return 0;
    }
  }

  /** How much to shrink the reference pixel thresholds for a frame this wide, 1 at 1280 */
  private static double pxScale(int imgWidth) {
    return imgWidth / (double) REFERENCE_WIDTH;
  }

  /** Pinhole {fx, fy, cx, cy} for a frame of this size, derived from the Limelight 3A's fov */
  private static double[] intrinsics(int w, int h) {
    double fx = (w / 2.0) / Math.tan(Math.toRadians(HFOV_DEG) / 2);
    double fy = (h / 2.0) / Math.tan(Math.toRadians(VFOV_DEG) / 2);
    return new double[] {fx, fy, w / 2.0, h / 2.0};
  }

  /**
   * {tx, ty, ta} for the chosen stack. The Limelight computes tx/ty from the contour itself, these
   * are for consumers of llpython that only get the array. Degrees right and up of the crosshair,
   * ta in percent of the frame
   */
  private static double[] targetAngles(Rect bbox, int w, int h) {
    double[] k = intrinsics(w, h);
    double cx = bbox.x + bbox.width / 2.0;
    double cy = bbox.y + bbox.height / 2.0;
    double tx = Math.toDegrees(Math.atan((cx - k[2]) / k[0]));
    double ty = Math.toDegrees(Math.atan((k[3] - cy) / k[1]));
    double ta = 100.0 * bbox.width * bbox.height / (double) (w * h);
    return new double[] {tx, ty, ta};
  }

  /** Yellow box and cone code on every stack, a thicker green box on the one we picked */
  private static void draw(Mat image, List<Stack> stacks, Stack best) {
    double font = 0.8 * pxScale(image.cols());
    for (Stack stack : stacks) {
      Imgproc.rectangle(image, stack.bbox(), YELLOW, 1);
      Imgproc.putText(
          image,
          stack.cones(),
          new Point(stack.bbox().x, Math.max(12, stack.bbox().y - 4)),
          Imgproc.FONT_HERSHEY_SIMPLEX,
          font,
          YELLOW,
          1);
    }
    Imgproc.rectangle(image, best.bbox(), GREEN, 2);
  }

  /* ---- END OUTPUT ---- */

  /* ---- STACKS ---- */

  /**
   * Groups colored blobs that sit vertically on top of each other into stacks. A stack is reported
   * if it sits on something station like, even a bare station with nothing scored yet so the robot
   * can line up on it. The station's white base cone is an anchor, not counted
   */
  public static List<Stack> findStacks(Mat image) {
    Mat hsv = new Mat();
    Imgproc.cvtColor(image, hsv, Imgproc.COLOR_BGR2HSV);

    List<Stack> out = new ArrayList<>();
    for (Group group : groupIntoStacks(allBlobs(hsv))) {
      Stack stack = describeStack(hsv, group);
      if (stack != null) {
        out.add(stack);
      }
    }

    hsv.release();
    return out;
  }

  /** Bottom up, each blob either nests on the top of an existing stack or starts a new one */
  private static List<Group> groupIntoStacks(List<Blob> blobs) {
    List<Group> groups = new ArrayList<>();
    for (Blob blob : blobs) {
      boolean placed = false;
      for (Group group : groups) {
        if (xOverlap(group, blob) >= STACK_X_OVERLAP && nestsOn(blob, group.top())) {
          group.add(blob);
          placed = true;
          break;
        }
      }
      if (!placed) {
        groups.add(new Group(blob));
      }
    }
    return groups;
  }

  /**
   * Index of the first colored blob in bottom to top members, members.size() if there is none.
   * Everything from there up is scored, the white below it is the station's base cone
   */
  private static int firstColorIndex(List<Blob> members) {
    for (int i = 0; i < members.size(); i++) {
      if (members.get(i).color != 'W') {
        return i;
      }
    }
    return members.size();
  }

  /** Turns a group into a Stack, or null if it is not sitting on a station */
  private static Stack describeStack(Mat hsv, Group group) {
    List<Blob> members = new ArrayList<>(group.members);
    members.sort(Comparator.comparingInt(Blob::bottom).reversed());

    // The bottom most white blob is the station's base cone. Any white above a colored cone is a
    // bonus bunny and counts as a scored cone
    int firstColor = firstColorIndex(members);
    boolean hasWhiteBase = firstColor > 0;
    List<Blob> scored = members.subList(firstColor, members.size());

    // A bare station is just its white base cone, so white alone has to look like a cone (the
    // gray walls pass the white threshold too) and prove it is sitting on the wooden box
    if (scored.isEmpty() && (group.y2 - group.y) < BARE_CONE_MIN_ASPECT * group.width()) {
      return null;
    }
    if ((!hasWhiteBase || scored.isEmpty()) && !onAStation(hsv, group)) {
      return null;
    }

    // One letter per estimated cone, the hull of every member is what the Limelight gets
    Blob top = members.get(members.size() - 1);
    StringBuilder cones = new StringBuilder();
    for (Blob blob : scored) {
      cones.append(String.valueOf(blob.color).repeat(conesInBlob(blob, top)));
    }

    return new Stack(
        new Rect(group.x, group.y, group.width(), group.y2 - group.y),
        cones.toString(),
        scored.isEmpty() ? ' ' : scored.get(scored.size() - 1).color,
        hull(members));
  }

  /* ---- END STACKS ---- */

  /* ---- MASKS AND BLOBS ---- */

  /** inRange on one or two hue bands then a 3x3 open to knock the specks off */
  private static Mat mask(Mat hsv, Scalar lo1, Scalar hi1, Scalar lo2, Scalar hi2) {
    Mat mask = new Mat();
    Core.inRange(hsv, lo1, hi1, mask);
    if (lo2 != null) {
      Mat second = new Mat();
      Core.inRange(hsv, lo2, hi2, second);
      Core.bitwise_or(mask, second, mask);
      second.release();
    }
    Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, OPEN_KERNEL);
    return mask;
  }

  /** False for specks, very short blobs and anything wide enough to be the wall or the sky */
  private static boolean couldBeACone(MatOfPoint contour, int imgWidth) {
    double scale = pxScale(imgWidth);
    if (Imgproc.contourArea(contour) < MIN_CONE_AREA_PX * scale * scale) {
      return false;
    }
    Rect rect = Imgproc.boundingRect(contour);
    return rect.height >= MIN_CONE_HEIGHT_PX * scale
        && rect.width <= MAX_BLOB_WIDTH_FRAC * imgWidth;
  }

  /** Bounding boxes of the contours in one mask that could plausibly be a cone */
  private static List<Blob> blobs(Mat mask, char color) {
    List<MatOfPoint> contours = new ArrayList<>();
    Imgproc.findContours(
        mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

    List<Blob> out = new ArrayList<>();
    for (MatOfPoint contour : contours) {
      if (couldBeACone(contour, mask.cols())) {
        out.add(new Blob(color, Imgproc.boundingRect(contour), contour));
      }
    }

    mask.release();
    return out;
  }

  /** Every plausible cone blob in the frame, bottom most first so stacks build from the base up */
  private static List<Blob> allBlobs(Mat hsv) {
    List<Blob> blobs = new ArrayList<>();
    blobs.addAll(blobs(mask(hsv, RED_LO_1, RED_HI_1, RED_LO_2, RED_HI_2), 'R'));
    blobs.addAll(blobs(mask(hsv, BLUE_LO, BLUE_HI, null, null), 'B'));
    blobs.addAll(blobs(mask(hsv, WHITE_LO, WHITE_HI, null, null), 'W'));
    blobs.sort(Comparator.comparingInt(Blob::bottom).reversed());
    return blobs;
  }

  /* ---- END MASKS AND BLOBS ---- */

  /* ---- BOX GEOMETRY ---- */

  /** Horizontal overlap as a fraction of the narrower one's width, 0 to 1 */
  private static double xOverlap(Group group, Blob blob) {
    int lo = Math.max(group.x, blob.x);
    int hi = Math.min(group.x2, blob.right());
    int narrower = Math.max(1, Math.min(group.width(), blob.w));
    return Math.max(0, hi - lo) / (double) narrower;
  }

  /**
   * True if upper looks like a cone nested on lower: similar width, centers roughly aligned, and
   * the upper blob's bottom sitting on or just inside the lower blob's top
   */
  private static boolean nestsOn(Blob upper, Blob lower) {
    double widthRatio = upper.w / (double) Math.max(1, lower.w);
    if (widthRatio < 0.55 || widthRatio > 1.45) {
      return false;
    }

    double upperCenter = upper.x + upper.w / 2.0;
    double lowerCenter = lower.x + lower.w / 2.0;
    if (Math.abs(upperCenter - lowerCenter) > 0.3 * Math.max(upper.w, lower.w)) {
      return false;
    }

    int gap = upper.bottom() - lower.y;
    return -0.25 * upper.h <= gap && gap <= 0.6 * upper.h;
  }

  /* ---- END BOX GEOMETRY ---- */

  /* ---- COUNTING CONES ---- */

  /**
   * Pixel height of one nested cone's visible band. Taken from the top cone's height, but never
   * less than the blob's width says because the top cone may be clipped by the edge of the image
   */
  private static double bandPx(Blob blob, Blob top) {
    double topHeight = Math.max(1, top.h);
    return Math.max(
        topHeight * NESTED_ASPECT_STEP / SINGLE_CONE_ASPECT,
        Math.max(1, blob.w) * NESTED_ASPECT_STEP);
  }

  /**
   * One whole cone plus whatever extra height is left over. A lone cone seen from below already
   * looks taller than 18/10.5 so only clear extras are counted
   */
  private static int conesInTopBlob(Blob blob) {
    double aspect = blob.h / (double) Math.max(1, blob.w);
    int extras = (int) ((aspect - SINGLE_CONE_ASPECT - 0.35) / NESTED_ASPECT_STEP);
    return 1 + Math.max(0, Math.min(MAX_CONES_PER_BLOB - 1, extras));
  }

  /**
   * Same color nested cones merge into one blob, estimate how many. The topmost blob shows a whole
   * cone plus a band per extra nested cone under it, a blob with a cone nested on top only shows
   * its bands. Bands are measured against the top cone's own height which cancels most of the
   * perspective from a camera looking up at the stack
   */
  private static int conesInBlob(Blob blob, Blob top) {
    if (blob == top) {
      return conesInTopBlob(blob);
    }

    // rint not round, python's round() is half to even
    int bands = (int) Math.rint(blob.h / bandPx(blob, top));
    return Math.max(1, Math.min(MAX_CONES_PER_BLOB, bands));
  }

  /* ---- END COUNTING CONES ---- */

  /* ---- STATION CHECK ---- */

  /**
   * The column to search for the station box under a stack as {x0, y0, x1, y1}, a bit wider than
   * the stack and reaching down past the white base cone. Null if it is cut off by the bottom of
   * the image
   */
  private static int[] stationWindow(Mat hsv, Group group) {
    int h = hsv.rows();
    int w = hsv.cols();
    int stackWidth = group.width();

    int y0 = Math.min(h - 1, group.y2);
    int y1 = Math.min(h, (int) (group.y2 + STATION_SEARCH_DEPTH * stackWidth));
    int x0 = Math.max(0, group.x - stackWidth / 4);
    int x1 = Math.min(w, group.x2 + stackWidth / 4);
    if (y1 - y0 < 4 || x1 <= x0) {
      return null;
    }
    return new int[] {x0, y0, x1, y1};
  }

  /**
   * Image row of the first mostly wood row inside the window, or -1 if there is too little wood.
   * Enough rows in the window need some wood in them, otherwise the stack is sitting on carpet
   */
  private static int firstWoodRow(Mat hsv, int[] window) {
    int x0 = window[0];
    int y0 = window[1];
    int x1 = window[2];
    int y1 = window[3];

    Mat column = hsv.submat(y0, y1, x0, x1);
    Mat wood = new Mat();
    Core.inRange(column, WOOD_LO, WOOD_HI, wood);

    int woodRows = 0;
    int firstWoodRow = -1;
    for (int row = 0; row < wood.rows(); row++) {
      double mean = Core.mean(wood.row(row)).val[0];
      if (mean > 0.15 * 255) {
        woodRows++;
        if (firstWoodRow < 0) {
          firstWoodRow = row;
        }
      }
    }
    wood.release();

    if (woodRows < Math.max(3, 0.1 * (y1 - y0))) {
      return -1;
    }
    return y0 + firstWoodRow;
  }

  /** Width of the unbroken run of wood through column cx on row y, walking out both ways */
  private static int woodRunWidth(Mat hsv, int y, int cx) {
    int w = hsv.cols();
    Mat rowMask = new Mat();
    Core.inRange(hsv.row(y), WOOD_LO, WOOD_HI, rowMask);
    byte[] row = new byte[w];
    rowMask.get(0, 0, row);
    rowMask.release();

    int left = cx;
    while (left > 0 && row[left - 1] != 0) {
      left--;
    }
    int right = cx;
    while (right < w - 1 && row[right + 1] != 0) {
      right++;
    }
    return right - left;
  }

  /**
   * True if a wooden station box sits below the stack, which is what separates a stack from a cone
   * lying on the carpet. The box is at most about 1.4 cone widths wide, a much wider run of wood is
   * the arena wall behind a floor cone. The front face has an apriltag on it so wood is looked for
   * across 1.5 stack widths, not just the middle
   */
  private static boolean onAStation(Mat hsv, Group group) {
    int[] window = stationWindow(hsv, group);
    if (window == null) {
      // Cut off by the bottom of the image, can't tell so keep it
      return true;
    }

    int yWood = firstWoodRow(hsv, window);
    if (yWood < 0) {
      return false;
    }

    // Station box width is fine, the arena wall is not
    int cx = (group.x + group.x2) / 2;
    return woodRunWidth(hsv, yWood, cx) <= STATION_MAX_WIDTH_RATIO * group.width();
  }

  /* ---- END STATION CHECK ---- */

  /** Convex hull around every point of every member blob */
  private static MatOfPoint hull(List<Blob> members) {
    List<Point> points = new ArrayList<>();
    for (Blob blob : members) {
      points.addAll(blob.contour.toList());
    }
    MatOfPoint all = new MatOfPoint();
    all.fromList(points);

    MatOfInt indices = new MatOfInt();
    Imgproc.convexHull(all, indices);

    List<Point> hullPoints = new ArrayList<>();
    for (int index : indices.toArray()) {
      hullPoints.add(points.get(index));
    }
    MatOfPoint hull = new MatOfPoint();
    hull.fromList(hullPoints);
    return hull;
  }

  /** Bounding box of a hull, what the bench compares against ground truth */
  public static Rect boundingRect(MatOfPoint contour) {
    return contour.empty() ? new Rect() : Imgproc.boundingRect(contour);
  }
}
