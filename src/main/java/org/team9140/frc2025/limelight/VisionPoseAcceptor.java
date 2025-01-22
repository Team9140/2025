package org.team9140.frc2025.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;


public class VisionPoseAcceptor {
    public boolean shouldAcceptVision(double timestamp, Pose2d visionFieldToVehicle, Pose2d vehicleToTag, Twist2d robotVelocity) {
        if (DriverStation.isAutonomous()) {
            double kMaxRange = 4.0;
            if (vehicleToTag.getTranslation().norm() > kMaxRange) {
                return false;
            }
        }

        boolean rotatingTooFast = Math.abs(robotVelocity.dtheta) >= 1.0;
        if (!rotatingTooFast) {
            return true;
        } else {
            return false;
        }
    }

}