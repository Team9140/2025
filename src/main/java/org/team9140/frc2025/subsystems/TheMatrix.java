package org.team9140.frc2025.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.core.Point;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team9140.frc2025.Constants;
import org.team9140.frc2025.generated.TunerConstants;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TheMatrix extends SubsystemBase {

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private VisionSystemSim visionSystem;
    private SimCameraProperties simProperties;
    private PhotonCamera camera;
    private PhotonCameraSim cameraSim;
    private CommandSwerveDrivetrain drivetrain;
    Limelight light = Limelight.getInstance();

    public void setupVision(CommandSwerveDrivetrain drivetrain) {
        this.visionSystem = visionSim();
        this.simProperties = simProperties();
        this.camera = new PhotonCamera("MinjaeKim");
        this.cameraSim = new PhotonCameraSim(camera, this.simProperties);
        this.addCameraSim(this.cameraSim);
        this.drivetrain = drivetrain;
        cameraSim.enableDrawWireframe(true);


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
        cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(98.355661));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(60);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        return cameraProp;
    }

    private void addCameraSim(PhotonCameraSim camera) {
        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
        Translation3d robotToCameraTrl = new Translation3d(0.0, 0, Constants.Camera.kLensHeight);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(0), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        // Add this camera to the vision system simulation with the given robot-to-camera transform.
        this.visionSystem.addCamera(camera, robotToCamera);
    }
    private void writeToTable(PhotonPipelineResult result, NetworkTable table) {
        if (result.getTargets().size() <= 0) {
            return;
        }
        PhotonTrackedTarget target = result.getTargets().get(0);

        Double[] ret = {target.detectedCorners.get(0).x, target.detectedCorners.get(0).y, target.detectedCorners.get(1).x, target.detectedCorners.get(1).y,
                target.detectedCorners.get(2).x, target.detectedCorners.get(2).y, target.detectedCorners.get(3).x, target.detectedCorners.get(3).y
        };
        NetworkTableValue val = NetworkTableValue.makeDoubleArray(ret);
        table.putValue("tcornxy", val);
        NetworkTableValue Joseph = NetworkTableValue.makeDouble(target.fiducialId);
        table.putValue("tid", Joseph);



//            table.getEntry("botpose_wpiblue")
//                    .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
//            table.getEntry("botpose_orb_wpiblue")
//                    .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
    }

    @Override
    public void simulationPeriodic() {
        this.visionSystem.update(drivetrain.getState().Pose);

        writeToTable(camera.getLatestResult(), inst.getTable("limelight-b"));
    }

}
