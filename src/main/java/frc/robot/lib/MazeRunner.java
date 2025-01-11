// TODO: Rename to FollowPath

package frc.robot.lib;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Optional;
import java.util.TreeMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class MazeRunner {
    private TreeMap<String, Trigger> triggers;
    private Trajectory<SwerveSample> trajectory;
    private EventLoop loop;
    private Timer timer;
    private CommandSwerveDrivetrain drive;

    private boolean autoFlip;

    MazeRunner(String name, CommandSwerveDrivetrain drivetrain, boolean autoMirror) throws Exception {
        Choreo.<SwerveSample>loadTrajectory(name).ifPresent((trajectory) -> this.trajectory = trajectory);
        this.drive = drivetrain;
        for(EventMarker e : this.trajectory.events()) {
            if (triggers.containsKey(e.event))
                ChoreoAlert.alert("YO MR. HICE SPICE SAYS NOT TO HAVE TWO EVENTS WITH THE SAME NAME, goofus.", Alert.AlertType.kWarning);

            triggers.put(e.event, atTime(e.timestamp));
        }
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
                });
    }

    public Trigger atPose(
            Supplier<Optional<Pose2d>> pose, double toleranceMeters, double toleranceRadians) {
        return new Trigger(
                loop,
                () -> {
                    Optional<Pose2d> checkedPoseOpt = pose.get();
                    return checkedPoseOpt
                            .map(
                                    (checkedPose) -> {
                                        Translation2d currentTrans = this.drive.getState().Pose.getTranslation();
                                        Rotation2d currentRot = this.drive.getState().Pose.getRotation();
                                        return currentTrans.getDistance(checkedPose.getTranslation())
                                                < toleranceMeters
                                                && Math.abs(currentRot.minus(checkedPose.getRotation()).getRadians())
                                                < toleranceRadians;
                                    })
                            .orElse(false);
                });
//                .and(active());
    }

    public Trigger atPose(Optional<Pose2d> pose, double toleranceMeters, double toleranceRadians) {
        return atPose(
                ChoreoAllianceFlipUtil.optionalFlippedPose2d(pose, DriverStation::getAlliance, () -> autoFlip),
                toleranceMeters,
                toleranceRadians);
    }



    public Command gimmeCommand() {
        return new FunctionalCommand(
                () -> {
                    this.timer.restart();
                    this.drive.resetPose(new Pose2d());
                },
                () -> {
                    this.loop.poll();
                    Optional<SwerveSample> sample = this.trajectory.sampleAt(this.timer.get(), autoFlip && ChoreoAllianceFlipUtil.shouldFlip());

                    sample.ifPresent(swerveSample -> this.drive.followPath(swerveSample));
                },
                (boolean interrupted) -> {
                    if (interrupted) {
                        this.drive.setControl(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds()));
                    } else {
                        Optional<SwerveSample> sample = this.trajectory.getFinalSample(autoFlip && ChoreoAllianceFlipUtil.shouldFlip());
                        sample.ifPresent(swerveSample -> this.drive.followPath(swerveSample));
                    }
                },
                () -> this.timer.hasElapsed(this.trajectory.getTotalTime()),
                this.drive
        );
    }
}
