package org.team9140.frc2025;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.*;

public class Constants {
    public static class Drive {
        public static final LinearVelocity MIN_TRANSLATE_MPS = MetersPerSecond.of(0.01);
        public static final AngularVelocity MIN_ROTATE_RPS = RadiansPerSecond.of(5);
    }
}
