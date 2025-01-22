package org.team9140.frc2025.limelight;

public class UndistortConstants {
    private double[][] cameraMatrix;
    private double[] cameraDistortion;

    public UndistortConstants(double[] cameraDistortion, double[][] cameraMatrix) {
        this.cameraDistortion = cameraDistortion;
        this.cameraMatrix = cameraMatrix;
    }

    public double[][] getCameraMatrix() {
        return cameraMatrix;
    }

    public double[] getCameraDistortion() {
        return cameraDistortion;
    }
}