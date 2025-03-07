package org.team9140.lib;

import org.team9140.frc2025.helpers.LimelightHelpers.PoseEstimate;

import edu.wpi.first.units.measure.Time;

public class VisionMeasurement {
    public enum Kind {
        MT1,
        MT2;
    }

    public final Kind kind;
    public final Time timestamp;
    public final PoseEstimate measurement;
    
    public VisionMeasurement(Kind k, Time t, PoseEstimate m) {
        this.kind = k;
        this.timestamp = t;
        this.measurement = m;
    }
}
