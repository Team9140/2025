package org.team9140.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.*;

public class UtilTest {
    @Test
    @DisplayName("For values not greater than the threshold, Util.applyDeadband should return zero")
    public void applyDeadbandBelowThreshold() {
        assertEquals(0.0, Util.applyDeadband(0.0, 0.0));
        assertEquals(0.0, Util.applyDeadband(-0.7, 0.7));
        assertEquals(0.0, Util.applyDeadband(-0.1, 0.5));
        assertEquals(0.0, Util.applyDeadband(0.2, 0.25));
        assertEquals(0.0, Util.applyDeadband(-0.15, 0.25));
    }

    @Test
    @DisplayName("For values greater than the threshold, Util.applyDeadband should appropriately scale the value")
    public void applyDeadbandAboveThresholdScaled() {
        assertEquals(1.0, Util.applyDeadband(1.0, 0.12));
        assertTrue(Util.epsilonEquals(0.6875, Util.applyDeadband(0.8, 0.36)));
        assertTrue(Util.epsilonEquals(-0.5, Util.applyDeadband(-0.6, 0.2)));
        assertTrue(Util.epsilonEquals(1.0 / 17.0, Util.applyDeadband(0.2, 0.15)));
        assertEquals(-5.0 / 9.0, Util.applyDeadband(-0.6, 0.1));
    }

    @Test
    @DisplayName("For values within epsilon radians, Util.rotationEpsilonEquals should be true")
    public void rotationEpsilonEqualsForSimilarAngles() {
        assertTrue(Util.rotationEpsilonEquals(new Rotation2d(Radians.of(Math.PI)), new Rotation2d(Radians.of(Math.PI)), 0.0001));
        assertTrue(Util.rotationEpsilonEquals(new Rotation2d(Radians.of(Math.PI)), new Rotation2d(Radians.of(Math.PI+0.01)), 0.1));
        assertTrue(Util.rotationEpsilonEquals(new Rotation2d(Radians.of(Math.PI)), new Rotation2d(Radians.of(Math.PI+0.05)), 0.1));
        assertTrue(Util.rotationEpsilonEquals(new Rotation2d(Radians.of(Math.PI)), new Rotation2d(Radians.of(Math.PI+0.0000001)), 0.0001));
    }

    @Test
    @DisplayName("For angles with multiple revolutions but equal positions, Util.rotationEpsilonEquals should be true")
    public void rotationEpsilonEqualsForDifferentRevolutions() {
        assertTrue(Util.rotationEpsilonEquals(new Rotation2d(Radians.of(Math.PI)), new Rotation2d(Radians.of(3*Math.PI)), 0.0001));
        assertTrue(Util.rotationEpsilonEquals(new Rotation2d(Radians.of(3*Math.PI/2)), new Rotation2d(Radians.of(-Math.PI/2)), 0.0001));
        assertTrue(Util.rotationEpsilonEquals(new Rotation2d(Radians.of(Math.PI*2)), new Rotation2d(Radians.of(Math.PI/2)), Math.PI*2));
        assertTrue(Util.rotationEpsilonEquals(new Rotation2d(Radians.of(5*Math.PI/6)), new Rotation2d(Radians.of(-7*Math.PI/6+0.01)), 0.1));
        assertTrue(Util.rotationEpsilonEquals(new Rotation2d(Degrees.of(-1.0)), new Rotation2d(Radians.of(0)), Math.toRadians(5)));
    }

    @Test
    @DisplayName("For angles that are not equal, Util.rotationEpsilonEquals should be false")
    public void rotationEpsilonEqualsForDifferentAngles() {
        assertFalse(Util.rotationEpsilonEquals(new Rotation2d(Radians.of(Math.PI/2)), new Rotation2d(Radians.of(Math.PI/4)), Math.PI/4 - 0.1));
        assertFalse(Util.rotationEpsilonEquals(new Rotation2d(Radians.of(Math.PI/2)), new Rotation2d(Radians.of(7*Math.PI/3)), 0.001));
        assertFalse(Util.rotationEpsilonEquals(new Rotation2d(Radians.of(Math.PI)), new Rotation2d(Radians.of(Math.PI+0.01)), 0.0001));
        assertFalse(Util.rotationEpsilonEquals(new Rotation2d(Radians.of(Math.PI/2)), new Rotation2d(Radians.of(Math.PI/3)), 0.0000001));

    }
}
