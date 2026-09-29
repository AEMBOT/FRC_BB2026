package com.aembot.frc2026.config.subsystems.intake;

import edu.wpi.first.math.geometry.Translation2d;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

/**
 * Geometry of the cone intake and where it lets go of a cone. Only used by sim until there is a
 * real one
 */
public class ConeIntakeConfiguration {
  public final String kName;

  /** Width of the intake opening in meters */
  public double kWidthMeters;

  /** How far the intake reaches past the bumper in meters when running */
  public double kExtensionMeters;

  /** Side of the robot the intake is on */
  public IntakeSide kSide = IntakeSide.FRONT;

  /** How many cones the robot can hold at once */
  public int kCapacity = 1;

  /** Robot relative point cones leave the robot from, meters */
  public Translation2d kReleasePoint = new Translation2d();

  /** Height cones leave the robot from in meters */
  public double kReleaseHeightMeters;

  /** Speed of a light toss relative to the robot, m/s */
  public double kLightTossSpeedMps;

  /** Angle of a light toss above horizontal in degrees */
  public double kLightTossAngleDeg;

  /** How hard an ejected cone gets shoved out the front, m/s */
  public double kEjectSpeedMps;

  public ConeIntakeConfiguration(String name) {
    this.kName = name;
  }

  /**
   * @param widthMeters Width of the intake opening in meters
   * @return A reference to this object for chaining
   */
  public ConeIntakeConfiguration withWidthMeters(double widthMeters) {
    this.kWidthMeters = widthMeters;
    return this;
  }

  /**
   * @param extensionMeters Reach past the bumper in meters
   * @return A reference to this object for chaining
   */
  public ConeIntakeConfiguration withExtensionMeters(double extensionMeters) {
    this.kExtensionMeters = extensionMeters;
    return this;
  }

  /**
   * @param side Side of the robot the intake is on
   * @return A reference to this object for chaining
   */
  public ConeIntakeConfiguration withSide(IntakeSide side) {
    this.kSide = side;
    return this;
  }

  /**
   * @param capacity How many cones the robot can hold at once
   * @return A reference to this object for chaining
   */
  public ConeIntakeConfiguration withCapacity(int capacity) {
    this.kCapacity = capacity;
    return this;
  }

  /**
   * @param releasePoint Robot relative point cones leave the robot from
   * @param releaseHeightMeters Height cones leave the robot from
   * @return A reference to this object for chaining
   */
  public ConeIntakeConfiguration withReleasePoint(
      Translation2d releasePoint, double releaseHeightMeters) {
    this.kReleasePoint = releasePoint;
    this.kReleaseHeightMeters = releaseHeightMeters;
    return this;
  }

  /**
   * @param speedMps Speed of a light toss relative to the robot
   * @param angleDeg Angle of a light toss above horizontal
   * @return A reference to this object for chaining
   */
  public ConeIntakeConfiguration withLightToss(double speedMps, double angleDeg) {
    this.kLightTossSpeedMps = speedMps;
    this.kLightTossAngleDeg = angleDeg;
    return this;
  }

  /**
   * @param ejectSpeedMps How hard an ejected cone gets shoved
   * @return A reference to this object for chaining
   */
  public ConeIntakeConfiguration withEjectSpeedMps(double ejectSpeedMps) {
    this.kEjectSpeedMps = ejectSpeedMps;
    return this;
  }
}
