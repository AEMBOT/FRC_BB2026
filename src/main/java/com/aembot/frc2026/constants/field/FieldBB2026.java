package com.aembot.frc2026.constants.field;

import com.aembot.lib.constants.fields.YearFieldConstantable;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;

/**
 * Field constants for BunnyBots 2026 (Cone Zone). The field drawings dimension everything from the
 * center of the field in inches, so most of the raw numbers below go through {@link #fromCenter}.
 * The apriltag layout is built here since this isn't an official field.
 */
public class FieldBB2026 implements YearFieldConstantable {
  /* ---- FIELD ---- */
  /** 54'-8 3/4" wall to wall */
  public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(54 * 12 + 8.75);

  /** 26'-4" wall to wall */
  public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(26 * 12 + 4);

  public static final Translation2d FIELD_CENTER =
      new Translation2d(FIELD_LENGTH_METERS / 2.0, FIELD_WIDTH_METERS / 2.0);

  /**
   * Starting lines are 10' either side of the centerline. Not dimensioned, scaled off the drawing
   */
  public static final double STARTING_LINE_FROM_CENTER_METERS = Units.inchesToMeters(120);

  public static final double BLUE_STARTING_LINE_X =
      FIELD_CENTER.getX() - STARTING_LINE_FROM_CENTER_METERS;
  public static final double RED_STARTING_LINE_X =
      FIELD_CENTER.getX() + STARTING_LINE_FROM_CENTER_METERS;

  /* ---- APRIL TAGS ---- */
  public static final int TAG_COUNT = 12;

  /** Tags are 6.5" square with the top edge 16" off the floor */
  public static final double TAG_CENTER_HEIGHT_METERS = Units.inchesToMeters(16.0 - 6.5 / 2.0);

  /* ---- TOWER STATIONS ---- */
  /** 15" square wooden box with a white cone bolted on top */
  public static final double TOWER_STATION_SIZE_METERS = Units.inchesToMeters(15);

  public static final double TOWER_STATION_HEIGHT_METERS = Units.inchesToMeters(18);

  /**
   * One tower station
   *
   * @param tagId Apriltag id, 1-8
   * @param center Center of the footprint on the floor
   * @param tagFacing Direction the tag points, which is the front of the station
   */
  public record TowerStation(int tagId, Translation2d center, Rotation2d tagFacing) {
    /** Center of the front face, rotation pointing out of the face */
    public Pose2d frontFace() {
      return new Pose2d(
          center.plus(new Translation2d(TOWER_STATION_SIZE_METERS / 2.0, tagFacing)), tagFacing);
    }

    /**
     * A robot pose in front of the station looking at the tag
     *
     * @param standoffMeters Distance from the front face to the robot
     */
    public Pose2d approachPose(double standoffMeters) {
      Pose2d face = frontFace();
      return new Pose2d(
          face.getTranslation().plus(new Translation2d(standoffMeters, tagFacing)),
          tagFacing.plus(Rotation2d.k180deg));
    }
  }

  /** Indexed by tag id - 1. Tags are on whichever face is nearest the field center */
  public static final List<TowerStation> TOWER_STATIONS =
      List.of(
          station(1, -288.0, 0.0, 0.0),
          station(2, -180.0, 140.0, 0.0),
          station(3, 0.0, 140.0, -90.0),
          station(4, 180.0, 140.0, 180.0),
          station(5, 288.0, 0.0, 180.0),
          station(6, 180.0, -140.0, 180.0),
          station(7, 0.0, -140.0, 90.0),
          station(8, -180.0, -140.0, 0.0));

  public static TowerStation getTowerStation(int tagId) {
    return TOWER_STATIONS.get(tagId - 1);
  }

  /* ---- MINIBOT ARENA ---- */
  /** Outside of the wooden frame, polycarb goes on the outside of this */
  public static final double ARENA_LENGTH_METERS = Units.inchesToMeters(120);

  public static final double ARENA_WIDTH_METERS = Units.inchesToMeters(96);

  /** 2x4s laid flat */
  public static final double ARENA_WALL_THICKNESS_METERS = Units.inchesToMeters(3.5);

  public static final double ARENA_WALL_HEIGHT_METERS = Units.inchesToMeters(16);
  public static final double ARENA_POLYCARB_THICKNESS_METERS = Units.inchesToMeters(0.125);

  /** Robots can't go in here */
  public static final Pose2d ARENA_CENTER = new Pose2d(FIELD_CENTER, Rotation2d.kZero);

  /* ---- HUMAN PLAYERS ---- */
  /** Guess at how far in from the corner a human player cone ends up */
  public static final double HUMAN_PLAYER_DROP_INSET_METERS = Units.inchesToMeters(24);

  public record HumanPlayerStation(
      Alliance alliance, Translation2d corner, Translation2d dropPoint) {}

  /** Red human players are at the blue wall +Y corner and red wall -Y corner, blue the other two */
  public static final List<HumanPlayerStation> HUMAN_PLAYER_STATIONS =
      List.of(
          humanPlayer(Alliance.Red, 0.0, FIELD_WIDTH_METERS),
          humanPlayer(Alliance.Red, FIELD_LENGTH_METERS, 0.0),
          humanPlayer(Alliance.Blue, FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
          humanPlayer(Alliance.Blue, 0.0, 0.0));

  public static HumanPlayerStation getNearestHumanPlayer(Alliance alliance, Translation2d from) {
    HumanPlayerStation nearest = null;
    for (HumanPlayerStation station : HUMAN_PLAYER_STATIONS) {
      if (station.alliance() != alliance) {
        continue;
      }
      if (nearest == null
          || station.corner().getDistance(from) < nearest.corner().getDistance(from)) {
        nearest = station;
      }
    }
    return nearest;
  }

  /* ---- GAME PIECES ---- */
  /** Lakeside 18" PVC traffic cone */
  public static final double LARGE_CONE_HEIGHT_METERS = Units.inchesToMeters(18);

  public static final double LARGE_CONE_BASE_WIDTH_METERS = Units.inchesToMeters(10.5);
  public static final double LARGE_CONE_MASS_POUNDS = 3.0;

  /* ---- APRIL TAG LAYOUT ---- */
  public final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
      new AprilTagFieldLayout(
          List.of(
              // Tower stations, on the front face so 7.5" in from the station center
              tag(1, -280.5, 0.0, 0.0),
              tag(2, -172.5, 140.0, 0.0),
              tag(3, 0.0, 132.5, -90.0),
              tag(4, 172.5, 140.0, 180.0),
              tag(5, 280.5, 0.0, 180.0),
              tag(6, 172.5, -140.0, 180.0),
              tag(7, 0.0, -132.5, 90.0),
              tag(8, -172.5, -140.0, 0.0),
              // Arena, inside face of each wall facing in
              tag(9, -56.5, 0.0, 0.0),
              tag(10, 0.0, 44.5, -90.0),
              tag(11, 56.5, 0.0, 180.0),
              tag(12, 0.0, -44.5, 90.0)),
          FIELD_LENGTH_METERS,
          FIELD_WIDTH_METERS);

  @Override
  public AprilTagFieldLayout getFieldLayout() {
    return APRIL_TAG_FIELD_LAYOUT;
  }

  // Has to come after the constants it uses or they're still null when this runs
  private static final FieldBB2026 INSTANCE = new FieldBB2026();

  public static FieldBB2026 get() {
    return INSTANCE;
  }

  /* ---- HELPERS ---- */
  /** Center relative inches off the drawings to a field translation */
  public static Translation2d fromCenter(double xInches, double yInches) {
    return FIELD_CENTER.plus(
        new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches)));
  }

  private static TowerStation station(int id, double xInches, double yInches, double facingDeg) {
    return new TowerStation(id, fromCenter(xInches, yInches), Rotation2d.fromDegrees(facingDeg));
  }

  private static HumanPlayerStation humanPlayer(Alliance alliance, double cornerX, double cornerY) {
    double inset = HUMAN_PLAYER_DROP_INSET_METERS;
    Translation2d dropPoint =
        new Translation2d(
            cornerX == 0 ? inset : cornerX - inset, cornerY == 0 ? inset : cornerY - inset);
    return new HumanPlayerStation(alliance, new Translation2d(cornerX, cornerY), dropPoint);
  }

  private static AprilTag tag(int id, double xInches, double yInches, double facingDeg) {
    Translation2d position = fromCenter(xInches, yInches);
    return new AprilTag(
        id,
        new Pose3d(
            new Translation3d(position.getX(), position.getY(), TAG_CENTER_HEIGHT_METERS),
            new Rotation3d(0.0, 0.0, Math.toRadians(facingDeg))));
  }
}
