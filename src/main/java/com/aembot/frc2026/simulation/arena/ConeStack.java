package com.aembot.frc2026.simulation.arena;

import com.aembot.frc2026.simulation.gamepieces.ConeColor;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

/** The cones seated on one tower station, bottom to top, plus the scoring rules for one stack */
public class ConeStack {
  public static final int POINTS_PER_CONE = 1;
  public static final int POINTS_PER_VERTICAL_RUN = 5;
  public static final int MAX_SCORED_RUNS_PER_STACK = 2;
  public static final int RUN_LENGTH = 3;

  private final List<ConeColor> cones = new ArrayList<>();

  public void add(ConeColor color) {
    cones.add(color);
  }

  public void clear() {
    cones.clear();
  }

  public int size() {
    return cones.size();
  }

  public boolean isEmpty() {
    return cones.isEmpty();
  }

  /** Bottom to top */
  public List<ConeColor> getCones() {
    return Collections.unmodifiableList(cones);
  }

  public Optional<ConeColor> getTop() {
    return cones.isEmpty() ? Optional.empty() : Optional.of(cones.get(cones.size() - 1));
  }

  /**
   * Gets the alliance controlling this stack
   *
   * @return The alliance of the topmost non-bunny cone, empty if there isn't one
   */
  public Optional<Alliance> getController() {
    for (int i = cones.size() - 1; i >= 0; i--) {
      if (!cones.get(i).isBunny()) {
        return cones.get(i).getAlliance();
      }
    }
    return Optional.empty();
  }

  public boolean hasBunny() {
    return cones.stream().anyMatch(ConeColor::isBunny);
  }

  public int count(Alliance alliance) {
    ConeColor color = ConeColor.of(alliance);
    return (int) cones.stream().filter(cone -> cone == color).count();
  }

  /**
   * Counts non-overlapping runs of 3 same color cones. A bunny or an opposing cone breaks a run.
   *
   * @return Number of runs, capped at MAX_SCORED_RUNS_PER_STACK
   */
  public int getVerticalRuns(Alliance alliance) {
    ConeColor color = ConeColor.of(alliance);
    int runs = 0;
    int streak = 0;
    for (ConeColor cone : cones) {
      if (cone == color) {
        streak++;
        if (streak == RUN_LENGTH) {
          runs++;
          streak = 0;
        }
      } else {
        streak = 0;
      }
    }
    return Math.min(runs, MAX_SCORED_RUNS_PER_STACK);
  }

  /** Cone points plus run bonuses, doubled if there is a bunny anywhere in the stack */
  public int getPoints(Alliance alliance) {
    int base =
        count(alliance) * POINTS_PER_CONE + getVerticalRuns(alliance) * POINTS_PER_VERTICAL_RUN;
    return hasBunny() ? base * 2 : base;
  }

  /** Stack as a string of R/B/W letters, bottom to top. Empty string for an empty stack */
  public String getCode() {
    StringBuilder builder = new StringBuilder();
    for (ConeColor cone : cones) {
      builder.append(cone.getLetter());
    }
    return builder.toString();
  }

  @Override
  public String toString() {
    return "ConeStack[" + getCode() + "]";
  }
}
