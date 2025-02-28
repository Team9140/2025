package org.team9140.frc2025;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.math.util.Units;
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

    public static final class Elevator {
        public static final int MOTOR_ID = 5;
        public static final double GEARING = 5.0;
        public static final double MASS_KG = Units.lbsToKilograms(20);
        public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.32) / 2.0; // TODO
        public static final double MIN_HEIGHT_METERS = 0.005; // TODO
        public static final double MAX_HEIGHT_METERS = 1.57; // TODO

        public static final int CURRENT_LIMIT = 60;

        public static final double GEAR_RATIO = 1.0;
        public static final Distance SPOOL_RADIUS = Inches.of(2);
        public static final double METERS_PER_MOTOR_ROTATION = SPOOL_RADIUS.in(Meters) * Math.PI * 2.0 / GEAR_RATIO;

        public static final double P = 0.2; // TODO
        public static final double I = 0; // TODO
        public static final double D = 5; // TODO
        public static final double S = 0.095388; // TODO
        public static final double V = 2.0; // TODO
        public static final double A = 1.0; // TODO
        public static final double TOLERANCE = 0.02;
        public static final double CRUISE_VELOCITY = 1.3; // TODO
        public static final double ACCELERATION = 3; // TODO

        public static Distance BOTTOM = Meters.of(0.1);
        public static Distance L1 = Meters.of(0.1);
        public static Distance L2 = Meters.of(0.1);
        public static Distance L3 = Meters.of(0.1);
        public static Distance L4 = Meters.of(0.1);
    }
}
