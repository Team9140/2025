package org.team9140.frc2025.helpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import org.team9140.frc2025.Constants;

public class AutoAiming {
    // DO NOT CHANGE THE ORDER OF THESE UNLESS YOU KNOW WHAT YOU'RE DOING
    public enum Branches {
        BlueH,
        BlueI,
        BlueJ,
        BlueK,
        BlueL,
        BlueA,
        BlueB,
        BlueC,
        BlueD,
        BlueE,
        BlueF,
        BlueG,
        RedB,
        RedC,
        RedD,
        RedE,
        RedF,
        RedG,
        RedH,
        RedI,
        RedJ,
        RedK,
        RedL,
        RedA,
    }

    public static Branches getBranch(Translation2d pose) {
        // Before Middle
        if (pose.getX() < 17.548225 / 2) {
            double angle = MathUtil.inputModulus(pose.minus(Constants.FieldItemPoses.REEF_BLUE).getAngle().getDegrees(), 0, 360);
            Branches[] branches = Branches.values();

            return branches[(int) (angle / 30)];
        } else {
            double angle = MathUtil.inputModulus(pose.minus(Constants.FieldItemPoses.REEF_RED).getAngle().getDegrees(), 0, 360);
            Branches[] branches = Branches.values();

            return branches[12 + (int) (angle / 30)];
        }
    }
}
