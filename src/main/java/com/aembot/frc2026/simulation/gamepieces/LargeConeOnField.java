package com.aembot.frc2026.simulation.gamepieces;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import com.aembot.frc2026.constants.field.FieldBB2026;
import com.aembot.frc2026.simulation.arena.ConeZoneArena;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.dyn4j.geometry.Circle;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/**
 * An 18" traffic cone upright on the carpet in maple-sim. All colors share one physics type so an
 * intake can grab any of them, the color is just a property of the cone.
 */
public class LargeConeOnField extends GamePieceOnFieldSimulation {
  /** The maple-sim game piece type. This is what an intake sim targets */
  public static final String TYPE = "LargeCone";

  /** Per color types, only used for logging poses through {@link ConeZoneArena} */
  public static final String TYPE_RED = "LargeConeRed";

  public static final String TYPE_BLUE = "LargeConeBlue";
  public static final String TYPE_WHITE = "LargeConeWhite";

  /** Square base modeled as a 10.5" circle so it slides cleanly off bumpers */
  public static final GamePieceInfo CONE_INFO =
      new GamePieceInfo(
          TYPE,
          new Circle(FieldBB2026.LARGE_CONE_BASE_WIDTH_METERS / 2.0),
          Meters.of(FieldBB2026.LARGE_CONE_HEIGHT_METERS),
          Pounds.of(FieldBB2026.LARGE_CONE_MASS_POUNDS),
          3.0, // linear damping
          5.0, // angular damping
          0.2); // coefficient of restitution

  private final ConeColor kColor;

  public LargeConeOnField(ConeColor color, Translation2d position) {
    this(color, position, new Translation2d());
  }

  /**
   * @param velocityMps Initial field relative velocity
   */
  public LargeConeOnField(ConeColor color, Translation2d position, Translation2d velocityMps) {
    super(
        CONE_INFO,
        () -> FieldBB2026.LARGE_CONE_HEIGHT_METERS / 2.0,
        new Pose2d(position, Rotation2d.kZero),
        velocityMps);
    this.kColor = color;
  }

  public ConeColor getColor() {
    return kColor;
  }

  public static String typeOf(ConeColor color) {
    switch (color) {
      case RED:
        return TYPE_RED;
      case BLUE:
        return TYPE_BLUE;
      case WHITE:
      default:
        return TYPE_WHITE;
    }
  }

  /** Color for one of the per color types, null for anything else */
  public static ConeColor colorOfType(String type) {
    switch (type) {
      case TYPE_RED:
        return ConeColor.RED;
      case TYPE_BLUE:
        return ConeColor.BLUE;
      case TYPE_WHITE:
        return ConeColor.WHITE;
      default:
        return null;
    }
  }
}
