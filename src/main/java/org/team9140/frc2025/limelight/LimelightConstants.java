package org.team9140.frc2025.limelight;
import edu.wpi.first.math.geometry.Rotation2d;

public class LimelightConstants {
    private String id;
    private String name;
    private double height;
    private Rotation2d yawOffset;
    private Rotation2d horizontalPlaneToLens;

    private UndistortConstants undistortConstants;



    public LimelightConstants(String id, String name, double height, Rotation2d yawOffset, Rotation2d horizontalPlaneToLens,  UndistortConstants undistortConstants) {
        this.id = id;
        this.name = name;
        this.height = height;
        this.yawOffset = yawOffset;
        this.horizontalPlaneToLens = horizontalPlaneToLens;
        this.undistortConstants = undistortConstants;
    }

    public String getId() {
        return id;
    }

    public String getName() {
        return name;
    }

    public double getHeight() {
        return height;
    }

    public Rotation2d getYawOffset() {
        return yawOffset;
    }

    public Rotation2d getHorizontalPlaneToLens() {
        return horizontalPlaneToLens;
    }

    public UndistortConstants getUndistortConstants() {
        return undistortConstants;
    }

}