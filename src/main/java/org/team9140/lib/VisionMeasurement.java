package org.team9140.lib;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionMeasurement {
    // in CTRE timebase
    double timestamp;

    enum Kind {
        MT1,
        MT2,
        pinhole
    }

    // in blue alliance coordinate system
    Pose2d measurement;

    // add more fields if needed
    // might need avg tag size/area?
}
