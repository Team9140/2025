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
                        0.5,
                        Rotation2d.fromDegrees(0.0),
                        Rotation2d.fromDegrees(0.0),
                        new UndistortConstants(
                                new double[]{0.1394220917285037,-0.24654889674484518,-0.0005996899249218598,8.247478330939335e-05,0.08289921803271944},
                                new double[][]{{745.5524377236715,0.0,655.1438911369394},
                                        {0.0,745.1406890491054,413.06128159642583},
                                        {0.0, 0.0, 1.0}}
                        )
                );
        }
    }

}