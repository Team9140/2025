package org.team9140.frc2025.subsystems;

import java.util.EnumSet;

import com.ctre.phoenix6.SignalLogger;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team9140.frc2025.generated.TunerConstants;
import org.team9140.frc2025.helpers.LimelightHelpers;

import static edu.wpi.first.units.Units.DegreesPerSecond;

public class LimeLight extends SubsystemBase {
    public static final LimeLight LIME_B = new LimeLight("limelight-b");
    private static CommandSwerveDrivetrain drivetrain;

    private String name;

    private LimeLight(String nm) {
        this.name = nm;
        this.drivetrain = TunerConstants.getDrivetrain();
    }

    public static class VisionResult {
        public double timestamp;
        public boolean valid;
    }

    private volatile VisionResult latestResult = new VisionResult();

    public VisionResult getLatest() {
        return this.latestResult;
    }

    public void setRobotOrientation(Rotation2d direction) {
        LimelightHelpers.SetRobotOrientation(
                this.name, direction.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    Field2d field = new Field2d();

    private class Listener implements TableEventListener {
        private static int mode = -1;

        public void accept(NetworkTable table, String key, NetworkTableEvent event) {
            if (key.equals("json")) {
                double before = Timer.getFPGATimestamp();
                LimelightHelpers.LimelightResults llResult = LimelightHelpers.getLatestResults(LimeLight.this.name);
                VisionResult vr = new VisionResult();

                vr.timestamp = before - llResult.latency_pipeline / 1000.0 - llResult.latency_capture / 1000.0;

                if (llResult.targets_Fiducials.length > 0) {
                    vr.valid = true;
                }

                latestResult = vr;

                if (DriverStation.isDisabled()) {
                    if (mode != 1) {
                        LimelightHelpers.SetIMUMode(LimeLight.this.name, 1);
                        mode = 1;
                        System.out.println("IMU Mode set to 1");
                    }

                    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimeLight.this.name);

                    if (mt1 != null) {
                        boolean reject = false;

                        reject |= mt1.avgTagArea <= 0.05;
//                        reject |= mt1.avgTagDist >= 4.0;

                        if (!reject) {
                            double thetaStdDev = 5.0;
                            drivetrain.addVisionMeasurement(mt1.pose, Utils.fpgaToCurrentTime(vr.timestamp),
                                    VecBuilder.fill(5.0, 5.0, thetaStdDev));

                            field.setRobotPose(mt1.pose);
                            SmartDashboard.putData(field);

                            SignalLogger.writeDoubleArray(LimeLight.this.name + " pose",
                                    new double[]{mt1.pose.getX(), mt1.pose.getY(), mt1.pose.getRotation().getDegrees()});

                            setRobotOrientation(drivetrain.getState().Pose.getRotation());
                        }
                    }
                } else {
                    if (mode != 2) {
                        LimelightHelpers.SetIMUMode(LimeLight.this.name, 2);
                        mode = 2;
                        System.out.println("IMU Mode set to 2");
                    }

                    setRobotOrientation(drivetrain.getState().Pose.getRotation());

                    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimeLight.this.name);

                    boolean reject = false;
                    reject |= (Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValue().in(DegreesPerSecond)) >= 360.0);
                    reject |= mt2.avgTagArea <= 0.05;
//                    reject |= mt2.avgTagDist >= 4.0;

                    if (!reject) {
                        double thetaStdDev = 999.0;
                        drivetrain.addVisionMeasurement(mt2.pose, Utils.fpgaToCurrentTime(vr.timestamp),
                                VecBuilder.fill(5.0, 5.0, thetaStdDev));
                    }
                }
            }
        }
    }

    private int m_listenerID = -1;

    public synchronized void start() {
        if (m_listenerID < 0) {
            m_listenerID = NetworkTableInstance.getDefault().getTable(this.name).addListener("json",
                    EnumSet.of(Kind.kValueAll), new Listener());
        }
    }
}