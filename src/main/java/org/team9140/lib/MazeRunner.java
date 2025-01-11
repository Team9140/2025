// TODO: Rename to FollowPath

package org.team9140.lib;

import choreo.Choreo;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.ChoreoAlert;
import choreo.util.ChoreoAllianceFlipUtil;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team9140.frc2025.subsystems.CommandSwerveDrivetrain;

import java.util.Optional;
import java.util.TreeMap;
import java.util.function.BooleanSupplier;

public class MazeRunner {
    private final TreeMap<String, Trigger> eventtimes;
    private Trajectory<SwerveSample> trajectory;
    private final EventLoop loop;
    private final Timer timer;
    private final CommandSwerveDrivetrain drive;

    private final boolean autoFlip;
    private boolean active = false;
    private final Trigger activeTrigger;

    public MazeRunner(String name, CommandSwerveDrivetrain drivetrain, boolean autoMirror) {
        Choreo.<SwerveSample>loadTrajectory(name).ifPresent(trajectory -> this.trajectory = trajectory);
        this.drive = drivetrain;
        this.autoFlip = autoMirror;

        this.loop = new EventLoop();
        this.timer = new Timer();
        this.eventtimes = new TreeMap<>();

        this.activeTrigger = new Trigger(loop, () -> this.active);

        for(EventMarker e : this.trajectory.events()) {
            if (this.eventtimes.containsKey(e.event))
                ChoreoAlert.alert("YO MR. HICE SPICE SAYS NOT TO HAVE TWO EVENTS WITH THE SAME NAME, goofus.", Alert.AlertType.kWarning);

            this.eventtimes.put(e.event, atTime(e.timestamp));
        }
    }

    public Trigger atEventTime(String eventName) {
        return this.eventtimes.get(eventName);
    }

    public Trigger atTime(double timestamp) {
        return new Trigger(
                loop,
                new BooleanSupplier() {
                    double lastTimestamp = timer.get();

                    public boolean getAsBoolean() {
                        double nowTimestamp = timer.get();
                        try {
                            return lastTimestamp < nowTimestamp && nowTimestamp >= timestamp;
                        } finally {
                            lastTimestamp = nowTimestamp;
                        }
                    }
                })
                .and(activeTrigger);
    }

    private boolean withinTolerance(Rotation2d lhs, Rotation2d rhs, double toleranceRadians) {
        if (Math.abs(toleranceRadians) > Math.PI) {
            return true;
        }
        double dot = lhs.getCos() * rhs.getCos() + lhs.getSin() * rhs.getSin();
        // cos(θ) >= cos(tolerance) means |θ| <= tolerance, for tolerance in [-pi, pi], as pre-checked
        // above.
        return dot > Math.cos(toleranceRadians);
    }

    public Trigger atPose(Pose2d pose, double toleranceMeters, double toleranceRadians) {
        Pose2d flippedPose = ChoreoAllianceFlipUtil.flip(pose);
        return new Trigger(
                loop,
                () -> {
                    final Pose2d currentPose = this.drive.getState().Pose;
                    if (autoFlip && ChoreoAllianceFlipUtil.shouldFlip()) {
                        boolean transValid =
                                currentPose.getTranslation().getDistance(flippedPose.getTranslation())
                                        < toleranceMeters;
                        boolean rotValid =
                                withinTolerance(
                                        currentPose.getRotation(), flippedPose.getRotation(), toleranceRadians);
                        return transValid && rotValid;
                    } else {
                        boolean transValid =
                                currentPose.getTranslation().getDistance(pose.getTranslation())
                                        < toleranceMeters;
                        boolean rotValid =
                                withinTolerance(
                                        currentPose.getRotation(), pose.getRotation(), toleranceRadians);
                        return transValid && rotValid;
                    }
                })
                .and(activeTrigger);
    }

    public Trigger atTranslation(Translation2d translation, double toleranceMeters) {
        Translation2d flippedTranslation = ChoreoAllianceFlipUtil.flip(translation);
        return new Trigger(
                loop,
                () -> {
                    final Translation2d currentTrans = this.drive.getState().Pose.getTranslation();
                    if (autoFlip && ChoreoAllianceFlipUtil.shouldFlip()) {
                        return currentTrans.getDistance(flippedTranslation) < toleranceMeters;
                    } else {
                        return currentTrans.getDistance(translation) < toleranceMeters;
                    }
                })
                .and(activeTrigger);
    }

    public Command gimmeCommand() {
        return new FunctionalCommand(
                () -> {
                    this.timer.restart();
                    this.drive.resetPose(new Pose2d());
                    this.active = true;
                },
                () -> {
                    this.loop.poll();
                    Optional<SwerveSample> sample = this.trajectory.sampleAt(this.timer.get(), autoFlip && ChoreoAllianceFlipUtil.shouldFlip());

                    sample.ifPresent(this.drive::followPath);
                },
                interrupted -> {
                    if (interrupted) {
                        this.drive.setControl(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds()));
                    } else {
                        Optional<SwerveSample> sample = this.trajectory.getFinalSample(autoFlip && ChoreoAllianceFlipUtil.shouldFlip());
                        sample.ifPresent(this.drive::followPath);
                    }

                    this.active = false;
                },
                () -> this.timer.hasElapsed(this.trajectory.getTotalTime()),
                this.drive
        );
    }
}
