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
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team9140.lib.LimelightHelpers;
import org.team9140.lib.VisionMeasurement;

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
                SmartDashboard.putNumber("fit", llResult.Fiducial.length);
                SmartDashboard.putNumber("saulgoodman", llResult.Fiducial[0].corner_points[0][0]);
                if (llResult.Fiducial.length > 0) {
                    vr.valid = true;
                    System.out.println(llResult.Fiducial[0]);
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
                    // setRobotOrientation(pose.getRotation());

                    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Limelight.this.name);

                    // all this logic to use or reject measurement belongs in drivetrain
                    boolean reject = false;
                    // reject |= Math.abs(angularVelocity.in(DegreesPerSecond)) >= 360.0;
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
//        double[] corners = {728.9579467773438, 503.0422058105469, 763.508056640625, 503.43731689453125, 763.1161499023438, 469.2093811035156, 729.0219116210938, 468.1083984375};
//        double[] distortionCoefficients = {0.1394220917285037,-0.24654889674484518,-0.0005996899249218598,8.247478330939335e-05,0.08289921803271944};
//        double[][] intrinsics_matrix = {{745.5524377236715,0.0,655.1438911369394}, {0.0,745.1406890491054,413.06128159642583}, {0.0,0.0,1.0}};
//        Tupl
//        double reproerror = 0.548062243424268;

        double[]corners;
        Number tid;
        Time tl;
        Time cl;
        Time now;
        Time when;
        public void accept(NetworkTable table, String key, NetworkTableEvent event) {
            if (key.equals("tcornxy")) {
                corners = table.getEntry(key).getDoubleArray(new double[0]);
                if(corners.length > 1){
                    SmartDashboard.putNumberArray("tcornxy", corners);
                }
            }
        }

        public void doMath(){

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

    public synchronized void start() {
        if (m_listenerID < 0) {
            m_listenerID = NetworkTableInstance.getDefault().getTable(name).addListener("json",
                    EnumSet.of(Kind.kValueAll), new LimelightListener());
        }
        if(n_listenerID < 0) {
            n_listenerID = NetworkTableInstance.getDefault().getTable(name).addListener("tcornxy",
                    EnumSet.of(Kind.kValueAll), new TcornxyListener());
        }
//        if(o_listenerID < 0) {
//            o_listenerID = NetworkTableInstance.getDefault().getTable(name).addListener("targetpose_cameraspace",
//                    EnumSet.of(Kind.kValueAll), new CameraSpaceListener());
//        }
    }
}