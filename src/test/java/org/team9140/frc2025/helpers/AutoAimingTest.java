package org.team9140.frc2025.helpers;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.team9140.lib.Util;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class AutoAimingTest {
    @Test
    @DisplayName("AutoAiming.getClosestFace returns the closest face's position attribute when accessed.")
    public void getClosestFaceTranslationBlueReef() {
        assertTrue(Util.epsilonEquals(new Translation2d(5.26733, 5.37322), AutoAiming.getClosestFace(new Translation2d(6.5, 7.25)).getPose().getTranslation(), 0.01));
        assertTrue(Util.epsilonEquals(new Translation2d(3.71157, 5.37322), AutoAiming.getClosestFace(new Translation2d(2.6, 7.6)).getPose().getTranslation(), 0.01));
        assertTrue(Util.epsilonEquals(new Translation2d(2.9337, 4.0259), AutoAiming.getClosestFace(new Translation2d(0.6, 4.3)).getPose().getTranslation(), 0.01));
        assertTrue(Util.epsilonEquals(new Translation2d(3.71157, 2.67858), AutoAiming.getClosestFace(new Translation2d(2.5, 1.3)).getPose().getTranslation(), 0.01));
        assertTrue(Util.epsilonEquals(new Translation2d(5.26733, 2.67858), AutoAiming.getClosestFace(new Translation2d(6, 1.25)).getPose().getTranslation(), 0.01));
        assertTrue(Util.epsilonEquals(new Translation2d(6.0452, 4.0259), AutoAiming.getClosestFace(new Translation2d(7.5, 4.3)).getPose().getTranslation(), 0.01));
    }
}
