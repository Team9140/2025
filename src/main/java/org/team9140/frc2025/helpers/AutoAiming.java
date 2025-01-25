package org.team9140.frc2025.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.team9140.frc2025.Constants;

import java.util.Arrays;

public class AutoAiming {
    static abstract class Reef {
        public enum Branches {
            A,
            B,
            C,
            D,
            E,
            F,
            G,
            H,
            I,
            J,
            K,
            L
        }
    }

    static class Red extends Reef {
        public static Branches nearestBranch(Translation2d robotPose) {
            double angle = robotPose.minus(Constants.FieldItemPoses.REEF_BLUE).getAngle().getRadians();
            Branches[] branches = Branches.values();

            return branches[(int) (Math.TAU / branches.length * angle / (Math.TAU))];
        }
    }

    static class Blue extends Reef {
        public static Branches nearestBranch(Translation2d robotPose) {
            double angle = -robotPose.minus(Constants.FieldItemPoses.REEF_BLUE).getAngle().getRadians();
            Branches[] branches = Branches.values();

            return branches[(int) (Math.TAU / branches.length * angle / (Math.TAU))];
        }
    }

    public static Reef.Branches getBranch(Translation2d pose) {
        if (pose.getX() > 17.548225) {
            return Blue.nearestBranch(pose);
        } else {
            return Red.nearestBranch(pose);
        }
    }
}
