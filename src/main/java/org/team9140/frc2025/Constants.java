package org.team9140.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.team9140.frc2025.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

public class Constants {
    public static class Drive {
        public static final LinearVelocity SPEED_AT_12_VOLTS = TunerConstants.kSpeedAt12Volts.times(0.8);
        public static final LinearVelocity MIN_TRANSLATIONAL_SPEED = MetersPerSecond.of(0.0375);
        public static final LinearVelocity MIN_TRANSLATIONAL_SPEED_TELEOP = MetersPerSecond.of(0.01);
        public static final AngularVelocity MIN_ROTATIONAL_SPEED = DegreesPerSecond.of(1);
        public static final AngularVelocity MIN_ROTATIONAL_SPEED_TELEOP = DegreesPerSecond.of(0.1);
        public static final AngularVelocity MAX_ROTATIONAL_RATE = RotationsPerSecond.of(2);
        public static final double X_CONTROLLER_P = 10.0;
        public static final double X_CONTROLLER_I = 0.0;
        public static final double X_CONTROLLER_D = 0.0;
        public static final double Y_CONTROLLER_P = 10.0;
        public static final double Y_CONTROLLER_I = 0.0;
        public static final double Y_CONTROLLER_D = 0.0;
        public static final double HEADING_CONTROLLER_P = 11.0;
        public static final double HEADING_CONTROLLER_I = 0.0;
        public static final double HEADING_CONTROLLER_D = 0.25;
    }

    public static class FieldItemPoses {
        public static final Pose2d REEF_BLUE = new Pose2d(4.48945, 4.0259, new Rotation2d());
        public static final Pose2d REEF_RED = new Pose2d(13.058775, 4.0259, new Rotation2d());
    }

    public static final Distance REEF_RADIUS = Meters.of(1.5);
    public static final Transform2d HORIZONTAL_BRANCH_DISTANCE_FROM_CENTER = new Transform2d(Meters.of(0), Meters.of(0.25), new Rotation2d());
}
