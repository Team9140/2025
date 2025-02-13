package org.team9140.frc2025;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.*;

public class Constants {
    public static class Drive {
        public static final LinearVelocity MIN_TRANSLATE_MPS = MetersPerSecond.of(0.01);
        public static final AngularVelocity MIN_ROTATE_RPS = DegreesPerSecond.of(1);
    }

    public static class ArmPositions{
        // Positions in radians
        public static final double INTAKE = -1.7;
        // Motion Magic Specific Limit
        public static final double FEED_FORWARD = 0.0;

        public static final double MAX_CURRENT = 48.0;
        // Radian rotations of arm
        public static final double SENSOR_TO_MECHANISM_RATIO = 80.0 / 9.0 * 58.0 / 11.0 / (2 * Math.PI);
    }

    public static class Elevator {
        public static final double GEAR_RATIO = 1.0;
        public static final Distance SPOOL_RADIUS = Inches.of(1);
        public static final double METERS_PER_MOTOR_ROTATION = SPOOL_RADIUS.in(Meters) * Math.PI * 2.0 / GEAR_RATIO;
        public static final InvertedValue LEFT_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
    }
}
