package org.team9140.frc2025.helpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import org.team9140.frc2025.Constants;

public class AutoAiming {
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
        RedA,
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
        RedL
    }

    public static Branches getBranch(Translation2d pose) {
        // Past Middle
        if (pose.getX() < 17.548225 / 2) {
            double angle = MathUtil.angleModulus(pose.minus(Constants.FieldItemPoses.REEF_BLUE).getAngle().unaryMinus().getRadians());
            Branches[] branches = Branches.values();

            return branches[branches.length / 4 + (int) (branches.length / 2 * angle / (Math.TAU)) - 1];
        } else {
            double angle = pose.minus(Constants.FieldItemPoses.REEF_BLUE).getAngle().getRadians();
            Branches[] branches = Branches.values();

            return branches[24 - (int) (Math.TAU / branches.length * angle / (Math.TAU))];
        }
    }
}
