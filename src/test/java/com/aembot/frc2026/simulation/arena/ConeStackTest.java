package com.aembot.frc2026.simulation.arena;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.aembot.frc2026.simulation.gamepieces.ConeColor;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import org.junit.jupiter.api.Test;

class ConeStackTest {
  private static ConeStack stack(String code) {
    ConeStack stack = new ConeStack();
    for (char c : code.toCharArray()) {
      stack.add(c == 'R' ? ConeColor.RED : c == 'B' ? ConeColor.BLUE : ConeColor.WHITE);
    }
    return stack;
  }

  @Test
  void controlIsTopmostNonBunny() {
    assertTrue(stack("").getController().isEmpty());
    assertEquals(Alliance.Red, stack("BR").getController().orElseThrow());
    assertEquals(
        Alliance.Blue, stack("RBW").getController().orElseThrow(), "bunny on top does not control");
    assertTrue(stack("W").getController().isEmpty());
  }

  @Test
  void verticalRunsScoreCorrectly() {
    // only two runs count, extra cones are still a point each
    assertEquals(2, stack("RRRRRRR").getVerticalRuns(Alliance.Red));
    assertEquals(7 + 2 * 5, stack("RRRRRRR").getPoints(Alliance.Red));

    // opposing cone breaks the run
    assertEquals(0, stack("RRBRR").getVerticalRuns(Alliance.Red));
    assertEquals(4, stack("RRBRR").getPoints(Alliance.Red));
    assertEquals(1, stack("RRBRR").getPoints(Alliance.Blue));

    // so does a bunny, but it doubles the stack
    assertEquals(0, stack("RRWR").getVerticalRuns(Alliance.Red));
    assertEquals(2 * 3, stack("RRWR").getPoints(Alliance.Red));
    assertEquals(2 * (3 + 5), stack("RRRW").getPoints(Alliance.Red));
  }

  @Test
  void ticTacToeAcrossTheGrid() {
    SimulatedTowerStations stations = new SimulatedTowerStations();

    // red takes the top row
    for (int id : new int[] {2, 3, 4}) {
      stations.scoreCone(id, ConeColor.RED, false);
    }
    assertEquals(1, stations.getTicTacToes(Alliance.Red));
    assertEquals(20, stations.getTicTacToePoints(Alliance.Red));
    assertEquals(0, stations.getTicTacToes(Alliance.Blue));

    // blue caps station 3, red loses control of it and the line
    stations.scoreCone(3, ConeColor.BLUE, false);
    assertEquals(0, stations.getTicTacToes(Alliance.Red));

    // a bunny in one of the three stacks doubles the line once
    for (int id : new int[] {1, 5}) {
      stations.scoreCone(id, ConeColor.BLUE, false);
    }
    stations.setCenterStack(List.of(ConeColor.BLUE));
    stations.scoreCone(5, ConeColor.WHITE, false);

    assertEquals(1, stations.getTicTacToes(Alliance.Blue));
    assertEquals(40, stations.getTicTacToePoints(Alliance.Blue));
    // blue on 3, 1 and center are 1 each, 5 is doubled by its bunny, plus the line
    assertEquals(1 + 1 + 1 + 2 * 1 + 40, stations.getTotalPoints(Alliance.Blue));
  }

  @Test
  void autoConesAreWorthFiveExtra() {
    SimulatedTowerStations stations = new SimulatedTowerStations();
    stations.scoreCone(1, ConeColor.RED, true);
    stations.scoreCone(1, ConeColor.RED, false);

    assertEquals(5, stations.getAutoPoints(Alliance.Red));
    assertEquals(2 + 5, stations.getTotalPoints(Alliance.Red));
  }

  @Test
  void stackGeometryGrowsWithNestedCones() {
    SimulatedTowerStations stations = new SimulatedTowerStations();
    double emptyTip = stations.getTipHeightMeters(5);

    stations.scoreCone(5, ConeColor.RED, false);
    stations.scoreCone(5, ConeColor.BLUE, false);

    assertEquals(
        emptyTip + 2 * SimulatedTowerStations.NESTED_CONE_PITCH_METERS,
        stations.getTipHeightMeters(5),
        1e-9);

    assertEquals(1, stations.getConePoses(ConeColor.RED).size());
    assertEquals(1, stations.getConePoses(ConeColor.BLUE).size());
    assertTrue(
        stations.getConePoses(ConeColor.BLUE).get(0).getZ()
            > stations.getConePoses(ConeColor.RED).get(0).getZ());
  }
}
