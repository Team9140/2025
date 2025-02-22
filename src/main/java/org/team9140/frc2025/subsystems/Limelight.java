package org.team9140.frc2025.subsystems;

import java.util.EnumSet;
import java.util.function.Consumer;

import com.ctre.phoenix6.SignalLogger;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team9140.lib.LimelightHelpers;
import org.team9140.lib.VisionMeasurement;

import static edu.wpi.first.units.Units.DegreesPerSecond;

public class Limelight extends SubsystemBase {
    private final Consumer<VisionMeasurement> addVisionMeasurement;
    private Pose2d pose;
    private AngularVelocity angularVelocity;
    private int mode = -1;


    // instead of holding reference to drivetrain, pass in a consumer<VisionMeasurement>


    //private static CommandSwerveDrivetrain drivetrain;

    public String name;

    public Limelight(String nm, Consumer<VisionMeasurement> visionMeasurement, Pose2d pose, AngularVelocity angularVelocity) {
        this.name = nm;
        this.addVisionMeasurement = visionMeasurement;
        this.pose = pose;
        this.angularVelocity = angularVelocity;

        //this.drivetrain = TunerConstants.getDrivetrain();
    }

    public static class VisionResult {
        public double timestamp;
        public boolean valid;
    }

    private volatile VisionResult latestResult = new VisionResult();

    public VisionResult getLatest() {
        return this.latestResult;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public void setAngularVelocity(AngularVelocity angularVelocity) {
        this.angularVelocity = angularVelocity;
    }

    // call in robotcontainer instead of in listener or smth
    public void setRobotOrientation(Rotation2d direction) {
        LimelightHelpers.SetRobotOrientation(
                this.name, direction.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    Field2d field = new Field2d();

    private class LimelightListener implements TableEventListener {
        private static int mode = -1;

        public void accept(NetworkTable table, String key, NetworkTableEvent event) {
            if (key.equals("json")) {
                double before = Timer.getFPGATimestamp();
                LimelightHelpers.LimelightResults llResult = LimelightHelpers.getLatestResults(Limelight.this.name);
                VisionResult vr = new VisionResult();

                vr.timestamp = before - llResult.latency_pipeline / 1000.0 - llResult.latency_capture / 1000.0;

                if (llResult.targets_Fiducials.length > 0) {
                    vr.valid = true;
                }

                latestResult = vr;


                if(mode == 1){

                    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(Limelight.this.name);

                    if (mt1 != null) {
                        boolean reject = false;

                        reject |= mt1.avgTagArea <= 0.05;
//                        reject |= mt1.avgTagDist >= 4.0;

                        if (!reject) {
                            double thetaStdDev = 5.0;
//                          VecBuilder.fill(5.0, 5.0, thetaStdDev)
                            addVisionMeasurement.accept(new VisionMeasurement(Utils.fpgaToCurrentTime(vr.timestamp), mt1.pose, VisionMeasurement.Kind.MT1));

                            field.setRobotPose(mt1.pose);
                            SmartDashboard.putData(field);

                            SignalLogger.writeDoubleArray(Limelight.this.name + " pose",
                                    new double[]{mt1.pose.getX(), mt1.pose.getY(), mt1.pose.getRotation().getDegrees()});

                            setRobotOrientation(pose.getRotation());
                        }
                    }
                } else {
                    // find a way to delete this dependency on drivetrain
                    setRobotOrientation(pose.getRotation());

                    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Limelight.this.name);

                    // all this logic to use or reject measurement belongs in drivetrain
                    boolean reject = false;
                    reject |= Math.abs(angularVelocity.in(DegreesPerSecond)) >= 360.0;
                    reject |= mt2.avgTagArea <= 0.05;
//                    reject |= mt2.avgTagDist >= 4.0;

                    if (!reject) {
                        double thetaStdDev = 999.0;
                        // instead of adding measurement directly, give a VisionMeasurement to the consumer
                        addVisionMeasurement.accept(new VisionMeasurement((Utils.fpgaToCurrentTime(vr.timestamp)), mt2.pose,
                                VisionMeasurement.Kind.MT2));
                    }
                }
            }
        }
    }

    private class TcornxyListener implements TableEventListener {
        public void accept(NetworkTable table, String key, NetworkTableEvent event) {
            if (key.equals("tcornxy")) {
                Number[] corners = table.getEntry(key).getNumberArray(new Number[] {0, 0, 0, 0, 0});
            }
        }
    }

    private class TIDListener implements TableEventListener {
        public void accept(NetworkTable table, String key, NetworkTableEvent event) {
            if (key.equals("tid")) {
                Number id = table.getEntry(key).getNumber(-1);
                System.out.println(id + "\n \n \n");
            }
        }
    }

    private class CameraSpaceListener implements TableEventListener {
        double x;
        double y;
        double z;
        double pitch;
        double yaw;
        double roll;
        public void accept(NetworkTable table, String key, NetworkTableEvent event) {
            if (key.equals("targetpose_cameraspace")) {
                double[] values = table.getEntry(key).getDoubleArray(new double[] {0, 0, 0, 0, 0});
                x = values[0];
                y = values[1];
                z = values[2];
                pitch = values[3];
                yaw = values[4];
                roll = values[5];
            }
        }
    }

    public void setIMU(int i) {
        if(i != mode) {
            LimelightHelpers.SetIMUMode(Limelight.this.name, i);
            mode = i;
            System.out.println("IMU Mode set to " + i);
        }
    }

    private int m_listenerID = -1;
    private int n_listenerID = -1;
    private int o_listenerID = -1;
    private int p_listenerID = -1;

    public synchronized void start() {
        if (m_listenerID < 0) {
            m_listenerID = NetworkTableInstance.getDefault().getTable(name).addListener("json",
                    EnumSet.of(Kind.kValueAll), new LimelightListener());
        }
        if(n_listenerID < 0) {
            n_listenerID = NetworkTableInstance.getDefault().getTable(name).addListener("tcornxy",
                    EnumSet.of(Kind.kValueAll), new TcornxyListener());
        }
        if(o_listenerID < 0) {
            o_listenerID = NetworkTableInstance.getDefault().getTable(name).addListener("tid",
                    EnumSet.of(Kind.kValueAll), new TIDListener());
        }if(p_listenerID < 0) {
            p_listenerID = NetworkTableInstance.getDefault().getTable(name).addListener("targetpose_cameraspace",
                    EnumSet.of(Kind.kValueAll), new CameraSpaceListener());
        }
    }
}