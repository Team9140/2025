// TODO: Rename to FollowPath

package org.team9140.lib;

import static org.team9140.lib.Util.rotationEpsilonEquals;

import java.util.Optional;
import java.util.TreeMap;

import org.team9140.frc2025.subsystems.CommandSwerveDrivetrain;

import choreo.Choreo;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.ChoreoAlert;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class MazeRunner {
    private final TreeMap<String, Trigger> eventtimes;
    private final StructPublisher<Pose2d> posePublisher;
    private final EventLoop loop;
    private final Timer timer;
    private final CommandSwerveDrivetrain drive;
    private final DriverStation.Alliance alliance;
    private final Trigger activeTrigger;

    private Trajectory<SwerveSample> trajectory;
    private Pose2d targetPose;
    private boolean active = false;

    public MazeRunner(String name, CommandSwerveDrivetrain drivetrain, DriverStation.Alliance alliance) {
        Choreo.<SwerveSample>loadTrajectory(name).ifPresent(trajectory -> this.trajectory = alliance.equals(DriverStation.Alliance.Blue) ? trajectory : trajectory.flipped());
        this.drive = drivetrain;

        this.loop = new EventLoop();
        this.timer = new Timer();
        this.eventtimes = new TreeMap<>();
        this.alliance = alliance;
        this.posePublisher = NetworkTableInstance.getDefault().getStructTopic("expected_pose", Pose2d.struct).publish();

        this.activeTrigger = new Trigger(loop, () -> this.active);

        NetworkTableInstance.getDefault().getStructArrayTopic("trajectory", Pose2d.struct).publish().set(this.trajectory.getPoses());

        for(EventMarker e : this.trajectory.events()) {
            if (this.eventtimes.containsKey(e.event))
                ChoreoAlert.alert("YO MR. HICE SPICE SAYS NOT TO HAVE TWO EVENTS WITH THE SAME NAME, goofus.", Alert.AlertType.kWarning);

            this.eventtimes.put(e.event, atTime(e.timestamp));
            System.out.println("Added event " + e.event + " at time " + e.timestamp);
        }
    }

    public Trigger atEventTime(String eventName) {
        return this.eventtimes.get(eventName);
    }

    public Trigger atTime(double timestamp) {
        return new Trigger(
                loop,
                () -> timer.get() >= timestamp)
                .and(activeTrigger);
    }

    public Trigger atPose(Pose2d pose, double toleranceMeters, double toleranceRadians) {
        Pose2d flippedPose = ChoreoAllianceFlipUtil.flip(pose);

        if (this.alliance.equals(DriverStation.Alliance.Red)) {
            return positionTrigger(flippedPose, toleranceMeters, toleranceRadians);
        }
        return positionTrigger(pose, toleranceMeters, toleranceRadians);
    }

    private Trigger positionTrigger(Pose2d pose, double toleranceMeters, double toleranceRadians) {
        return new Trigger(
                loop,
                () -> {
                    final Pose2d currentPose = this.drive.getState().Pose;
                    boolean transValid =
                            currentPose.getTranslation().getDistance(pose.getTranslation())
                                    < toleranceMeters;
                    boolean rotValid =
                            rotationEpsilonEquals(
                                    currentPose.getRotation(), pose.getRotation(), toleranceRadians);
                    return transValid && rotValid;
                })
                .and(activeTrigger);
    }

    public Trigger atTranslation(Translation2d translation, double toleranceMeters) {
        Translation2d flippedTranslation = ChoreoAllianceFlipUtil.flip(translation);

        if (this.alliance.equals(DriverStation.Alliance.Red)) {
            return new Trigger(
                    loop,
                    () -> {
                        final Translation2d currentTrans = this.drive.getState().Pose.getTranslation();
                        return currentTrans.getDistance(flippedTranslation) < toleranceMeters;
                    })
                    .and(activeTrigger);
        }

        return new Trigger(
                loop,
                () -> {
                    final Translation2d currentTrans = this.drive.getState().Pose.getTranslation();
                    return currentTrans.getDistance(translation) < toleranceMeters;
                })
                .and(activeTrigger);
    }

    public Pose2d getInitialPose() {
        return this.trajectory.getInitialPose(false).get();
    }

    public Command gimmeCommand() {
        return this.drive.goToPose(() -> this.targetPose).raceWith(new FunctionalCommand(
                () -> {
                    this.timer.restart();
                    this.active = true;
                },
                () -> {
                    this.loop.poll();
                    Optional<SwerveSample> sample = this.trajectory.sampleAt(this.timer.get(), false);
                    sample.ifPresent((swerveSample) -> {
                        this.targetPose = swerveSample.getPose();
                        this.posePublisher.set(this.targetPose);
                    });
                },
                interrupted -> {
                    this.targetPose = this.drive.getState().Pose;
                    this.active = false;
                },
                () -> this.timer.hasElapsed(this.trajectory.getTotalTime())
        ));
    }
}
