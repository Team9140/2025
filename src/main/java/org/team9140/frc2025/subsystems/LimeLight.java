package org.team9140.frc2025.subsystems;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.EnumSet;
import java.util.function.Consumer;

import org.team9140.frc2025.helpers.LimelightHelpers;
import org.team9140.lib.VisionMeasurement;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {

    private String name;
    private Consumer<VisionMeasurement> measurementConsumer;

    public LimeLight(String nm, Consumer<VisionMeasurement> cvm) {
        this.name = nm;
        this.measurementConsumer = cvm;
    }

    public void setRobotOrientation(Rotation2d direction) {
        LimelightHelpers.SetRobotOrientation(
                this.name, direction.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    public void setIMUMode(int mode) {
        LimelightHelpers.SetIMUMode(this.name, mode);
    }

    private class Listener implements TableEventListener {
        public void accept(NetworkTable table, String key, NetworkTableEvent event) {
            if (key.equals("json")) {
                Time timestamp = Seconds.of(Utils.getCurrentTimeSeconds());
                LimelightHelpers.LimelightResults llResult = LimelightHelpers.getLatestResults(LimeLight.this.name);

                timestamp = timestamp.minus(Milliseconds.of(llResult.latency_capture))
                        .minus(Milliseconds.of(llResult.latency_pipeline));

                if (DriverStation.isDisabled()) {

                    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers
                            .getBotPoseEstimate_wpiBlue(LimeLight.this.name);

                    if (mt1 != null) {
                        LimeLight.this.measurementConsumer
                                .accept(new VisionMeasurement(VisionMeasurement.Kind.MT1, timestamp, mt1));
                    }

                } else {

                    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                            .getBotPoseEstimate_wpiBlue_MegaTag2(LimeLight.this.name);

                    if (mt2 != null) {
                        LimeLight.this.measurementConsumer
                                .accept(new VisionMeasurement(VisionMeasurement.Kind.MT2, timestamp, mt2));
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