package com.aembot.frc2026.constants.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

class FieldBB2026Test {
  private static final double EPSILON = 1e-6;

  @Test
  void hasTwelveTags() {
    assertEquals(FieldBB2026.TAG_COUNT, FieldBB2026.get().getFieldLayout().getTags().size());
    assertEquals(FieldBB2026.TAG_COUNT, FieldBB2026.get().getNumTags());
  }

  // Field is 180 degree symmetric so each tag has a twin on the other side
  @Test
  void tagsAreRotationallySymmetric() {
    int[][] pairs = {{1, 5}, {2, 6}, {3, 7}, {4, 8}, {9, 11}, {10, 12}};
    for (int[] pair : pairs) {
      Pose3d a = FieldBB2026.get().getAprilTagPose3d(pair[0]);
      Pose3d b = FieldBB2026.get().getAprilTagPose3d(pair[1]);

      Pose3d mirrored =
          new Pose3d(
              FieldBB2026.FIELD_LENGTH_METERS - a.getX(),
              FieldBB2026.FIELD_WIDTH_METERS - a.getY(),
              a.getZ(),
              a.getRotation().rotateBy(new Rotation3d(0, 0, Math.PI)));

      assertEquals(mirrored.getX(), b.getX(), EPSILON, "tags " + pair[0] + "/" + pair[1] + " x");
      assertEquals(mirrored.getY(), b.getY(), EPSILON, "tags " + pair[0] + "/" + pair[1] + " y");
      assertEquals(
          0.0,
          Math.abs(MathUtil.angleModulus(mirrored.getRotation().minus(b.getRotation()).getAngle())),
          EPSILON,
          "tags " + pair[0] + "/" + pair[1] + " rotation");
    }
  }

  @Test
  void towerStationTagsSitOnFrontFaces() {
    for (FieldBB2026.TowerStation station : FieldBB2026.TOWER_STATIONS) {
      Pose3d tag = FieldBB2026.get().getAprilTagPose3d(station.tagId());
      Translation2d face = station.frontFace().getTranslation();

      assertEquals(face.getX(), tag.getX(), EPSILON, "station " + station.tagId() + " x");
      assertEquals(face.getY(), tag.getY(), EPSILON, "station " + station.tagId() + " y");
      assertEquals(
          station.tagFacing().getRadians(),
          tag.getRotation().getZ(),
          EPSILON,
          "station " + station.tagId() + " facing");
      assertEquals(FieldBB2026.TAG_CENTER_HEIGHT_METERS, tag.getZ(), EPSILON);
    }
  }

  @Test
  void arenaTagsFaceInward() {
    double halfLengthInside =
        FieldBB2026.ARENA_LENGTH_METERS / 2 - FieldBB2026.ARENA_WALL_THICKNESS_METERS;
    double halfWidthInside =
        FieldBB2026.ARENA_WIDTH_METERS / 2 - FieldBB2026.ARENA_WALL_THICKNESS_METERS;

    for (int id = 9; id <= 12; id++) {
      Pose3d tag = FieldBB2026.get().getAprilTagPose3d(id);
      Translation2d relative =
          tag.getTranslation().toTranslation2d().minus(FieldBB2026.FIELD_CENTER);
      double facingDeg = Math.toDegrees(tag.getRotation().getZ());
      double towardCenterDeg = Math.toDegrees(Math.atan2(-relative.getY(), -relative.getX()));

      assertEquals(
          0.0,
          Math.abs(MathUtil.inputModulus(facingDeg - towardCenterDeg, -180, 180)),
          EPSILON,
          "tag " + id + " facing");
      assertTrue(
          Math.abs(Math.abs(relative.getX()) - halfLengthInside) < EPSILON
              || Math.abs(Math.abs(relative.getY()) - halfWidthInside) < EPSILON,
          "tag " + id + " should be on an inside wall face");
    }
  }

  @Test
  void tagCenterHeightIsRight() {
    assertEquals(Units.inchesToMeters(12.75), FieldBB2026.TAG_CENTER_HEIGHT_METERS, EPSILON);
  }
}
