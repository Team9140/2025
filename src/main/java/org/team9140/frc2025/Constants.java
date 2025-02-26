package org.team9140.frc2025;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

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

    public static class Manipulator {
        public static final double HOLD_VOLTAGE_ALGAE = 0;

        public static final double INTAKE_VOLTAGE_CORAL = 0;
        public static final double INTAKE_VOLTAGE_ALGAE = 0;

        public static final double OUTTAKE_VOLTAGE_CORAL = 0;
        public static final double OUTTAKE_VOLTAGE_ALGAE = 0;

        public static final Measure<DistanceUnit> CORAL_DISTANCE = Centimeters.of(10);
        public static final Time INTAKE_CORAL_TIME = Seconds.of(1);

        public static final int MANIPULATOR_MOTOR_DEVICE_NUM = 0;
        public static final int FUNNEL_MOTOR_DEVICE_NUM = 52;
        public static final int CAN_RANGE_DEVICE_NUM = 0;

        public static final int MIN_SIGNAL_STRENGTH = 5000;
        public static final double PROXIMITY_HYSTERESIS = 0.05;
        public static final double PROXIMITY_THRESHOLD = 0.4;
        public static final double FORWARD_AUTOSET = 0.0;

        public static final int MANIPULATOR_PEAK_CURRENT_LIMIT = 30;
        public static final int MANIPULATOR_PEAK_CURRENT_DURATION = 100;
        public static final int MANIPULATOR_CONTINUOUS_CURRENT_LIMIT = 20;

        public static final AngularVelocity FUNNEL_CONTROLLER_VELOCITY = RotationsPerSecond.of(0);
    }
}
