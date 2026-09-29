package com.aembot.frc2026.simulation.gamepieces;

import com.aembot.frc2026.constants.field.FieldBB2026;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

/**
 * A tossed cone. maple-sim flies it on a parabola with no drag, scores it if it passes through the
 * target and otherwise turns it back into a floor cone when it lands.
 */
public class LargeConeOnFly extends GamePieceProjectile {
  private final ConeColor kColor;

  public LargeConeOnFly(
      ConeColor color,
      Translation2d initialPosition,
      Translation2d initialVelocityMps,
      double initialHeightMeters,
      double initialVerticalSpeedMps,
      Rotation2d facing) {
    super(
        LargeConeOnField.CONE_INFO,
        initialPosition,
        initialVelocityMps,
        initialHeightMeters,
        initialVerticalSpeedMps,
        new Rotation3d(0, 0, facing.getRadians()));
    this.kColor = color;

    // Counts as landed once the center is at resting height
    enableBecomesGamePieceOnFieldAfterTouchGround();
    withTouchGroundHeight(FieldBB2026.LARGE_CONE_HEIGHT_METERS / 2.0);
  }

  public ConeColor getColor() {
    return kColor;
  }

  /** Overridden so the cone keeps its color when it lands */
  @Override
  public void addGamePieceAfterTouchGround(SimulatedArena arena) {
    if (!becomesGamePieceOnGroundAfterTouchGround) {
      return;
    }
    Translation3d landing = getPositionAtTime(launchedTimer.get());
    arena.addGamePiece(
        new LargeConeOnField(kColor, landing.toTranslation2d(), initialLaunchingVelocityMPS));
  }

  /** maple-sim uses 11 for gravity for some reason, exposed so tests can match it */
  public static double getGravity() {
    return GRAVITY;
  }
}
