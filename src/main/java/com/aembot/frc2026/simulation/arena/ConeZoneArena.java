package com.aembot.frc2026.simulation.arena;

import com.aembot.frc2026.constants.field.FieldBB2026;
import com.aembot.frc2026.constants.field.FieldBB2026.HumanPlayerStation;
import com.aembot.frc2026.constants.field.FieldBB2026.TowerStation;
import com.aembot.frc2026.simulation.gamepieces.ConeColor;
import com.aembot.frc2026.simulation.gamepieces.LargeConeOnField;
import com.aembot.frc2026.simulation.gamepieces.LargeConeOnFly;
import com.aembot.lib.core.logging.AEMLogger;
import com.aembot.lib.core.logging.Loggable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

/**
 * maple-sim arena for the BunnyBots 2026 Cone Zone field. Walls, tower stations and the minibot
 * arena are obstacles, large cones are physics bodies. Install it with
 * SimulatedArena.overrideInstance before anything grabs the default arena.
 */
public class ConeZoneArena extends SimulatedArena implements Loggable {

  /** Collision geometry, WPILib field frame. The minibot arena is one solid block */
  public static final class ConeZoneObstacleMap extends FieldMap {
    public ConeZoneObstacleMap() {
      double length = FieldBB2026.FIELD_LENGTH_METERS;
      double width = FieldBB2026.FIELD_WIDTH_METERS;

      /* ---- FIELD PERIMETER ---- */
      addBorderLine(new Translation2d(0, 0), new Translation2d(0, width));
      addBorderLine(new Translation2d(length, 0), new Translation2d(length, width));
      addBorderLine(new Translation2d(0, 0), new Translation2d(length, 0));
      addBorderLine(new Translation2d(0, width), new Translation2d(length, width));

      /* ---- TOWER STATIONS ---- */
      for (TowerStation station : FieldBB2026.TOWER_STATIONS) {
        addRectangularObstacle(
            FieldBB2026.TOWER_STATION_SIZE_METERS,
            FieldBB2026.TOWER_STATION_SIZE_METERS,
            new Pose2d(station.center(), station.tagFacing()));
      }

      /* ---- MINIBOT ARENA ---- */
      addRectangularObstacle(
          FieldBB2026.ARENA_LENGTH_METERS + 2 * FieldBB2026.ARENA_POLYCARB_THICKNESS_METERS,
          FieldBB2026.ARENA_WIDTH_METERS + 2 * FieldBB2026.ARENA_POLYCARB_THICKNESS_METERS,
          FieldBB2026.ARENA_CENTER);
    }
  }

  public static final int STARTING_LARGE_CONES = 42;

  private final SimulatedTowerStations towerStations = new SimulatedTowerStations();

  public ConeZoneArena() {
    super(new ConeZoneObstacleMap());
  }

  /** 42 cones in alternating colors around the outside of the arena wall */
  @Override
  public void placeGamePiecesOnField() {
    List<Translation2d> ring = getStartingConeRing();
    for (int i = 0; i < ring.size(); i++) {
      spawnCone(i % 2 == 0 ? ConeColor.BLUE : ConeColor.RED, ring.get(i));
    }
  }

  @Override
  public synchronized void clearGamePieces() {
    super.clearGamePieces();
    towerStations.reset();
  }

  public SimulatedTowerStations getTowerStations() {
    return towerStations;
  }

  /** Seat a cone on a station's stack. This is what a successful toss ends up calling */
  public void scoreCone(int stationId, ConeColor color) {
    towerStations.scoreCone(stationId, color, DriverStation.isAutonomousEnabled());
  }

  /* ---- SPAWNING CONES ---- */

  /** Drop a cone at rest at a field position */
  public LargeConeOnField spawnCone(ConeColor color, Translation2d position) {
    LargeConeOnField cone = new LargeConeOnField(color, position);
    addGamePiece(cone);
    return cone;
  }

  /** Human player puts a cone of their color in at their corner */
  public LargeConeOnField spawnHumanPlayerCone(HumanPlayerStation station) {
    return spawnCone(ConeColor.of(station.alliance()), station.dropPoint());
  }

  /** Same, from whichever of the alliance's human players is closest to the robot */
  public LargeConeOnField spawnHumanPlayerCone(Alliance alliance, Translation2d from) {
    return spawnHumanPlayerCone(FieldBB2026.getNearestHumanPlayer(alliance, from));
  }

  /** Closest human player of the alliance puts in their bonus bunny */
  public LargeConeOnField spawnHumanPlayerBunny(Alliance alliance, Translation2d from) {
    return spawnCone(
        ConeColor.WHITE, FieldBB2026.getNearestHumanPlayer(alliance, from).dropPoint());
  }

  /* ---- QUERIES ---- */

  /** Cones on the carpet. Not held, not in the air, not on a stack */
  public synchronized List<LargeConeOnField> getConesOnField() {
    List<LargeConeOnField> cones = new ArrayList<>();
    for (GamePiece piece : gamePieces) {
      if (piece instanceof LargeConeOnField cone) {
        cones.add(cone);
      }
    }
    return cones;
  }

  public synchronized List<LargeConeOnField> getConesOnField(ConeColor color) {
    List<LargeConeOnField> cones = new ArrayList<>();
    for (LargeConeOnField cone : getConesOnField()) {
      if (cone.getColor() == color) {
        cones.add(cone);
      }
    }
    return cones;
  }

  public synchronized List<LargeConeOnFly> getConesInFlight(ConeColor color) {
    List<LargeConeOnFly> cones = new ArrayList<>();
    for (GamePieceProjectile projectile : gamePieceLaunched()) {
      if (projectile instanceof LargeConeOnFly cone && cone.getColor() == color) {
        cones.add(cone);
      }
    }
    return cones;
  }

  /**
   * Also answers the per color types from LargeConeOnField so AdvantageScope gets one entry per
   * color that covers carpet, in flight and stacked cones
   */
  @Override
  public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
    ConeColor color = LargeConeOnField.colorOfType(type);
    if (color == null) {
      return super.getGamePiecesPosesByType(type);
    }

    List<Pose3d> poses = new ArrayList<>();
    for (LargeConeOnField cone : getConesOnField(color)) {
      poses.add(cone.getPose3d());
    }
    for (LargeConeOnFly cone : getConesInFlight(color)) {
      poses.add(cone.getPose3d());
    }
    poses.addAll(towerStations.getConePoses(color));

    return poses;
  }

  /**
   * Centers of the 42 starting cones going clockwise from the blue side +Y corner. Long sides get
   * six per half with a gap at the centerline, short sides get nine between the corners. Real
   * placement is by hand so don't trust this too much.
   */
  public static List<Translation2d> getStartingConeRing() {
    double coneRadius = FieldBB2026.LARGE_CONE_BASE_WIDTH_METERS / 2.0;
    double pitch = FieldBB2026.LARGE_CONE_BASE_WIDTH_METERS;
    double halfX =
        FieldBB2026.ARENA_LENGTH_METERS / 2.0
            + FieldBB2026.ARENA_POLYCARB_THICKNESS_METERS
            + coneRadius;
    double halfY =
        FieldBB2026.ARENA_WIDTH_METERS / 2.0
            + FieldBB2026.ARENA_POLYCARB_THICKNESS_METERS
            + coneRadius;

    List<Translation2d> ring = new ArrayList<>();

    // +Y long side, six from each corner in toward the centerline
    for (int i = 0; i < 6; i++) {
      ring.add(FieldBB2026.FIELD_CENTER.plus(new Translation2d(-halfX + i * pitch, halfY)));
    }
    for (int i = 5; i >= 0; i--) {
      ring.add(FieldBB2026.FIELD_CENTER.plus(new Translation2d(halfX - i * pitch, halfY)));
    }

    // Red side short end
    for (int i = 1; i <= 9; i++) {
      ring.add(
          FieldBB2026.FIELD_CENTER.plus(new Translation2d(halfX, halfY - i * (2 * halfY / 10.0))));
    }

    // -Y long side
    for (int i = 0; i < 6; i++) {
      ring.add(FieldBB2026.FIELD_CENTER.plus(new Translation2d(halfX - i * pitch, -halfY)));
    }
    for (int i = 5; i >= 0; i--) {
      ring.add(FieldBB2026.FIELD_CENTER.plus(new Translation2d(-halfX + i * pitch, -halfY)));
    }

    // Blue side short end
    for (int i = 1; i <= 9; i++) {
      ring.add(
          FieldBB2026.FIELD_CENTER.plus(
              new Translation2d(-halfX, -halfY + i * (2 * halfY / 10.0))));
    }

    if (ring.size() != STARTING_LARGE_CONES) {
      throw new IllegalStateException("Expected 42 starting cones, built " + ring.size());
    }
    return ring;
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    // Every cone of each color, on the carpet, in flight and seated on stations
    for (ConeColor color : ConeColor.values()) {
      AEMLogger.recordOutput(
          standardPrefix + "/Cones/" + color.name(),
          getGamePiecesArrayByType(LargeConeOnField.typeOf(color)));
    }

    // Stacks as letter codes, bottom to top
    SimulatedTowerStations stations = getTowerStations();
    for (int id = 1; id <= SimulatedTowerStations.STATION_COUNT; id++) {
      AEMLogger.recordOutput(
          standardPrefix + "/Stacks/Station" + id, stations.getStack(id).getCode());
    }
    AEMLogger.recordOutput(
        standardPrefix + "/Stacks/Center",
        stations.getStack(SimulatedTowerStations.CENTER).getCode());

    AEMLogger.recordOutput(standardPrefix + "/Score/Red", stations.getTotalPoints(Alliance.Red));
    AEMLogger.recordOutput(standardPrefix + "/Score/Blue", stations.getTotalPoints(Alliance.Blue));
  }
}
