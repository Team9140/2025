package org.team9140.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

import java.util.List;

import static edu.wpi.first.units.Units.*;

public class Constants {
    public static class Drive {
        public static final LinearVelocity MIN_TRANSLATE_MPS = MetersPerSecond.of(0.01);
        public static final AngularVelocity MIN_ROTATE_RPS = DegreesPerSecond.of(1);
    }

    public static class FieldItemPoses {
        public static final Translation2d REEF_BLUE = new Translation2d(4.48945, 4.0259);
        public static final Translation2d REEF_RED = new Translation2d(13.058775, 4.0259);
        public static final List<Translation2d> REEFS = List.of(REEF_BLUE, REEF_RED);
    }
}
