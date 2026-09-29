package com.aembot.frc2026.config.robots;

import com.aembot.frc2026.config.subsystems.intake.ConeIntakeConfiguration;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

// TODO all placeholders, there is no cone intake yet
/** Cone intake config for the production robot */
public class ProductionConeConfig {
  public final ConeIntakeConfiguration intakeConfiguration =
      new ConeIntakeConfiguration("ConeIntake")
          .withWidthMeters(Units.inchesToMeters(24))
          .withExtensionMeters(Units.inchesToMeters(8))
          .withSide(IntakeSide.FRONT)
          .withCapacity(1)
          .withReleasePoint(
              new Translation2d(Units.inchesToMeters(12), 0), Units.inchesToMeters(44))
          // Barely more than letting go of the cone
          .withLightToss(1.2, 30.0)
          .withEjectSpeedMps(1.0);
}
