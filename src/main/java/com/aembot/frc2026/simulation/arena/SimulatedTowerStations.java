package com.aembot.frc2026.simulation.arena;

import com.aembot.frc2026.constants.field.FieldBB2026;
import com.aembot.frc2026.simulation.gamepieces.ConeColor;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * The eight tower station stacks plus the center stack in the minibot arena, and the tic-tac-toe
 * scoring across them. Stacks are indexed by the station's apriltag id, CENTER is the arena one.
 */
public class SimulatedTowerStations {
  public static final int CENTER = 0;
  public static final int STATION_COUNT = 8;
  public static final int POINTS_PER_TIC_TAC_TOE = 20;
  public static final int AUTO_POINTS_PER_CONE = 5;

  /** How much each nested cone adds to the stack height. TODO measure on the real cones */
  public static final double NESTED_CONE_PITCH_METERS = Units.inchesToMeters(3.0);

  /** Tip of the white cone bolted to every station. 18" box plus 18" cone */
  public static final double BASE_CONE_TIP_HEIGHT_METERS =
      FieldBB2026.TOWER_STATION_HEIGHT_METERS + FieldBB2026.LARGE_CONE_HEIGHT_METERS;

  /** Tic-tac-toe grid of station ids, [row][col]. Row 0 is the +Y side, col 0 the blue side */
  public static final int[][] GRID = {{2, 3, 4}, {1, CENTER, 5}, {8, 7, 6}};

  private final ConeStack[] stacks = new ConeStack[STATION_COUNT + 1];
  private int redAutoPoints = 0;
  private int blueAutoPoints = 0;

  public SimulatedTowerStations() {
    for (int i = 0; i < stacks.length; i++) {
      stacks[i] = new ConeStack();
    }
  }

  public ConeStack getStack(int index) {
    return stacks[index];
  }

  /**
   * Seats a cone on a station
   *
   * @param duringAuto Whether to add the auto bonus for this cone
   */
  public synchronized void scoreCone(int stationId, ConeColor color, boolean duringAuto) {
    stacks[stationId].add(color);
    if (duringAuto) {
      color
          .getAlliance()
          .ifPresent(
              alliance -> {
                if (alliance == Alliance.Red) {
                  redAutoPoints += AUTO_POINTS_PER_CONE;
                } else {
                  blueAutoPoints += AUTO_POINTS_PER_CONE;
                }
              });
    }
  }

  /** The center stack is minibot territory, so it is only ever set by hand */
  public synchronized void setCenterStack(List<ConeColor> bottomToTop) {
    stacks[CENTER].clear();
    bottomToTop.forEach(stacks[CENTER]::add);
  }

  public synchronized void reset() {
    for (ConeStack stack : stacks) {
      stack.clear();
    }

    redAutoPoints = 0;
    blueAutoPoints = 0;
  }

  /* ---- GEOMETRY ---- */

  /** Tip of the topmost cone on a station. A tossed cone has to clear this to seat */
  public double getTipHeightMeters(int stationId) {
    return BASE_CONE_TIP_HEIGHT_METERS + stacks[stationId].size() * NESTED_CONE_PITCH_METERS;
  }

  /** Point a tossed cone has to pass through to seat on the station */
  public Translation3d getTarget(int stationId) {
    Translation2d center = FieldBB2026.getTowerStation(stationId).center();
    return new Translation3d(center.getX(), center.getY(), getTipHeightMeters(stationId));
  }

  /** Center pose of the k-th scored cone on a station, 0 based */
  public Pose3d getConePose(int stationId, int k) {
    Translation2d center = FieldBB2026.getTowerStation(stationId).center();
    double z =
        FieldBB2026.TOWER_STATION_HEIGHT_METERS
            + FieldBB2026.LARGE_CONE_HEIGHT_METERS / 2.0
            + (k + 1) * NESTED_CONE_PITCH_METERS;
    return new Pose3d(new Translation3d(center.getX(), center.getY(), z), new Rotation3d());
  }

  /** Poses of every scored cone of one color across all stations, for logging */
  public synchronized List<Pose3d> getConePoses(ConeColor color) {
    List<Pose3d> poses = new ArrayList<>();
    for (int id = 1; id <= STATION_COUNT; id++) {
      List<ConeColor> cones = stacks[id].getCones();
      for (int k = 0; k < cones.size(); k++) {
        if (cones.get(k) == color) {
          poses.add(getConePose(id, k));
        }
      }
    }
    return poses;
  }

  /* ---- SCORING ---- */

  /** 20 per line of three controlled stacks, doubled once if any stack in the line has a bunny */
  public synchronized int getTicTacToePoints(Alliance alliance) {
    int points = 0;
    for (int[] line : getLines()) {
      boolean controlsAll = true;
      boolean hasBunny = false;

      for (int index : line) {
        Optional<Alliance> controller = stacks[index].getController();
        if (controller.isEmpty() || controller.get() != alliance) {
          controlsAll = false;
          break;
        }
        hasBunny |= stacks[index].hasBunny();
      }

      if (controlsAll) {
        points += hasBunny ? 2 * POINTS_PER_TIC_TAC_TOE : POINTS_PER_TIC_TAC_TOE;
      }
    }
    return points;
  }

  public synchronized int getTicTacToes(Alliance alliance) {
    int count = 0;
    for (int[] line : getLines()) {
      boolean controlsAll = true;
      for (int index : line) {
        Optional<Alliance> controller = stacks[index].getController();
        if (controller.isEmpty() || controller.get() != alliance) {
          controlsAll = false;
        }
      }

      if (controlsAll) {
        count++;
      }
    }
    return count;
  }

  public synchronized int getStackPoints(Alliance alliance) {
    int points = 0;
    for (ConeStack stack : stacks) {
      points += stack.getPoints(alliance);
    }
    return points;
  }

  public synchronized int getAutoPoints(Alliance alliance) {
    return alliance == Alliance.Red ? redAutoPoints : blueAutoPoints;
  }

  /** Cones, runs, tic-tac-toes and auto bonus all together */
  public synchronized int getTotalPoints(Alliance alliance) {
    return getStackPoints(alliance) + getTicTacToePoints(alliance) + getAutoPoints(alliance);
  }

  /** Rows, columns and diagonals of GRID */
  public static List<int[]> getLines() {
    List<int[]> lines = new ArrayList<>();
    for (int row = 0; row < 3; row++) {
      lines.add(new int[] {GRID[row][0], GRID[row][1], GRID[row][2]});
    }
    for (int col = 0; col < 3; col++) {
      lines.add(new int[] {GRID[0][col], GRID[1][col], GRID[2][col]});
    }

    lines.add(new int[] {GRID[0][0], GRID[1][1], GRID[2][2]});
    lines.add(new int[] {GRID[0][2], GRID[1][1], GRID[2][0]});

    return lines;
  }
}
