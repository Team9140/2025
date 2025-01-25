package org.team9140.frc2025.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.io.IOException;

public class TheMatrix {

    private VisionSystemSim visionSystem;
    private SimCameraProperties simProperties;
    private PhotonCamera camera;
    private PhotonCameraSim cameraSim;

    private void setupVision() {
        this.visionSystem = visionSim();
        this.simProperties = simProperties();
        this.camera = new PhotonCamera("MinjaeKim");
        this.cameraSim = new PhotonCameraSim(camera, this.simProperties);
        this.addCameraSim(this.cameraSim);
    }

    private VisionSystemSim visionSim() {
        VisionSystemSim visionSim = new VisionSystemSim("main");
        try {
            AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);

            visionSim.addAprilTags(tagLayout);
        } catch (IOException e){
            return null;
        }

        return visionSim;
    }

    private SimCameraProperties simProperties() {
        SimCameraProperties cameraProp = new SimCameraProperties();

        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        return cameraProp;
    }

    private void addCameraSim(PhotonCameraSim camera) {
        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        // Add this camera to the vision system simulation with the given robot-to-camera transform.
        this.visionSystem.addCamera(camera, robotToCamera);
    }

}
