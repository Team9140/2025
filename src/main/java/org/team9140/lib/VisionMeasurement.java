package org.team9140.lib;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionMeasurement {
    // in CTRE timebase
    double timestamp;

    public enum Kind {
        MT1,
        MT2,
        pinhole
    }

    Kind type;

    // in blue alliance coordinate system
    Pose2d measurement;

    // add more fields if needed
    // might need avg tag size/area?
    public VisionMeasurement(double ts, Pose2d pmo, Kind ong)
    {
        this.timestamp = ts;
        this.measurement = pmo;
        this.type = ong;

    }

}
