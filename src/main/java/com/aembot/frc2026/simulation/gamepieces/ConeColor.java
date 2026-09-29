package com.aembot.frc2026.simulation.gamepieces;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/** Color of a large cone. White cones are bonus bunnies */
public enum ConeColor {
  RED,
  BLUE,
  WHITE;

  public static ConeColor of(Alliance alliance) {
    return alliance == Alliance.Red ? RED : BLUE;
  }

  public boolean isBunny() {
    return this == WHITE;
  }

  public Optional<Alliance> getAlliance() {
    switch (this) {
      case RED:
        return Optional.of(Alliance.Red);
      case BLUE:
        return Optional.of(Alliance.Blue);
      case WHITE:
      default:
        return Optional.empty();
    }
  }

  /** R, B or W. Used for logging stacks */
  public char getLetter() {
    return name().charAt(0);
  }
}
