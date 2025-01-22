package org.team9140.lib;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

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
}
