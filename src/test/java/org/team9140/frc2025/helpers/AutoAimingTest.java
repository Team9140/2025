package org.team9140.frc2025.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.team9140.frc2025.Constants;
import org.team9140.lib.Util;

import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class AutoAimingTest {
    @Test
    @DisplayName("AutoAiming.getClosestFace returns the closest face's Pose2d when given the side right and height L1")
    public void getRightL1BlueReefPose() {
        assertTrue(Util.epsilonEquals(new Pose2d(5.63891, 5.68662, new Rotation2d(Radians.of(-2.0*Math.PI / 3.0))), AutoAiming.getClosestFace(new Translation2d(6.5, 7.25)).getRight(Constants.ElevatorSetbacks.L1)));
        assertTrue(Util.epsilonEquals(new Pose2d(3.33999, 5.68662, new Rotation2d(Radians.of(-Math.PI / 3.0))), AutoAiming.getClosestFace(new Translation2d(2.6, 7.6)).getRight(Constants.ElevatorSetbacks.L1)));
        assertTrue(Util.epsilonEquals(new Pose2d(2.4765, 3.8608, new Rotation2d(Radians.of(0))), AutoAiming.getClosestFace(new Translation2d(0.6, 4.3)).getRight(Constants.ElevatorSetbacks.L1)));
        assertTrue(Util.epsilonEquals(new Pose2d(3.62595, 2.20008, new Rotation2d(Radians.of(Math.PI / 3.0))), AutoAiming.getClosestFace(new Translation2d(2.5, 1.3)).getRight(Constants.ElevatorSetbacks.L1)));
        assertTrue(Util.epsilonEquals(new Pose2d(5.35295, 2.20008, new Rotation2d(Radians.of(2.0*Math.PI / 3.0))), AutoAiming.getClosestFace(new Translation2d(6, 1.25)).getRight(Constants.ElevatorSetbacks.L1)));
        assertTrue(Util.epsilonEquals(new Pose2d(6.5024, 3.8608, new Rotation2d(Radians.of(Math.PI))), AutoAiming.getClosestFace(new Translation2d(7.5, 4.3)).getRight(Constants.ElevatorSetbacks.L1)));
    }

    @Test
    @DisplayName("AutoAiming.getClosestFace returns the closest face's Pose2d when given the side left and height L1")
    public void getLeftL1BlueReefPose() {
        assertTrue(Util.epsilonEquals(new Pose2d(5.35295, 5.85172, new Rotation2d(Radians.of(-2.0*Math.PI / 3.0))), AutoAiming.getClosestFace(new Translation2d(6.5, 7.25)).getLeft(Constants.ElevatorSetbacks.L1)));
        assertTrue(Util.epsilonEquals(new Pose2d(3.62595, 5.85172, new Rotation2d(Radians.of(-Math.PI / 3.0))), AutoAiming.getClosestFace(new Translation2d(2.6, 7.6)).getLeft(Constants.ElevatorSetbacks.L1)));
        assertTrue(Util.epsilonEquals(new Pose2d(2.4765, 4.191, new Rotation2d(Radians.of(0))), AutoAiming.getClosestFace(new Translation2d(0.6, 4.3)).getLeft(Constants.ElevatorSetbacks.L1)));
        assertTrue(Util.epsilonEquals(new Pose2d(3.33999, 2.36518, new Rotation2d(Radians.of(Math.PI / 3.0))), AutoAiming.getClosestFace(new Translation2d(2.5, 1.3)).getLeft(Constants.ElevatorSetbacks.L1)));
        assertTrue(Util.epsilonEquals(new Pose2d(5.63891, 2.36518, new Rotation2d(Radians.of(2.0*Math.PI / 3.0))), AutoAiming.getClosestFace(new Translation2d(6, 1.25)).getLeft(Constants.ElevatorSetbacks.L1)));
        assertTrue(Util.epsilonEquals(new Pose2d(6.5024, 4.191, new Rotation2d(Radians.of(Math.PI))), AutoAiming.getClosestFace(new Translation2d(7.5, 4.3)).getLeft(Constants.ElevatorSetbacks.L1)));
    }
}
