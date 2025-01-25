package org.team9140.frc2025.limelight;

import org.team9140.frc2025.Constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class LimelightConstantsFactory {
    public static LimelightConstants getConstantsForId(String id) {
        switch (id) {
            default: // Intentional fall through
            case "back2024":
                // Limelight 3G because they dont know how to publish software
                return new LimelightConstants(
                        "back2024",
                        "limelight-back",
                        1.33296, //random
                        Rotation2d.fromDegrees(0.0), //random
                        Rotation2d.fromDegrees(-35.0), //random
                        new UndistortConstants(
                                new double[]{0.12554005287488895,-0.2134498176970171,0.0006899014186974819,-0.0001416329438772633,0.06191709384284909},
                                new double[][]{{737.2322486244727, 0.0, 638.9525014798731},
                                        {0.0, 736.6301969232578, 390.8846601731128},
                                        {0.0, 0.0, 1.0}}
                        )
                );
        }
    }

}