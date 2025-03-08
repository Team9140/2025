package org.team9140.frc2025.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.team9140.frc2025.Constants.Drive.MAX_teleop_rotation;
import static org.team9140.frc2025.Constants.Drive.MAX_teleop_velocity;
import static org.team9140.frc2025.Constants.Drive.MIN_ROTATIONAL_SPEED_TELEOP;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.team9140.frc2025.Constants;
import org.team9140.frc2025.generated.TunerConstants;
import org.team9140.frc2025.generated.TunerConstants.TunerSwerveDrivetrain;
import org.team9140.frc2025.helpers.AutoAiming;
import org.team9140.lib.SysIdRoutineTorqueCurrent;
import org.team9140.lib.Util;
import org.team9140.lib.VisionMeasurement;
import org.team9140.lib.swerve.SwerveRequests9140;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final Time kSimLoopPeriod = Milliseconds.of(5);
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // move magic numbers to constants
    private final PhoenixPIDController m_pathXController = new PhoenixPIDController(TunerConstants.X_CONTROLLER_P,
            TunerConstants.X_CONTROLLER_I, TunerConstants.X_CONTROLLER_D);
    private final PhoenixPIDController m_pathYController = new PhoenixPIDController(TunerConstants.Y_CONTROLLER_P,
            TunerConstants.Y_CONTROLLER_I, TunerConstants.Y_CONTROLLER_D);
    private final PhoenixPIDController headingController = new PhoenixPIDController(TunerConstants.HEADING_CONTROLLER_P,
            TunerConstants.HEADING_CONTROLLER_I, TunerConstants.HEADING_CONTROLLER_D);// 11.0, 0.0, 0.25

    Field2d dashField2d = new Field2d();

    private Pose2d targetPose = new Pose2d();

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.centric.HeadingController = headingController;

        SmartDashboard.putData("field", dashField2d);

        SmartDashboard.putNumberArray("drive target pose", new double[] { 0, 0, 0 });
    }

    @Override
    public void periodic() {
        dashField2d.setRobotPose(this.getState().Pose);

        SignalLogger.writeString("drivetrain command", this.getCurrentCommand().toString());
        SmartDashboard.putString("drivetrain command", this.getCurrentCommand().toString());

        if (this.targetPose != null) {
            SmartDashboard.putNumberArray("drive target pose", new double[] { this.targetPose.getX(),
                    this.targetPose.getY(), this.targetPose.getRotation().getRadians() });
            SignalLogger.writeDoubleArray("drive target pose", new double[] { this.targetPose.getX(),
                    this.targetPose.getY(), this.targetPose.getRotation().getRadians() });
        } else {
            SmartDashboard.putNumberArray("drive target pose", new double[] { -1, -1, -1 });
            SignalLogger.writeDoubleArray("drive target pose", new double[] { -1, -1, -1 });
        }
    }

    public void acceptVisionMeasurement(VisionMeasurement vm) {
        double xyStdDev = 9999;
        double thetaStdDev = 9999;
        if (vm.kind.equals(VisionMeasurement.Kind.MT1)) {
            boolean reject = false;

            reject |= vm.measurement.avgTagDist >= 4;
            reject |= vm.measurement.avgTagArea < 0.1;

            xyStdDev = 5.0;
            thetaStdDev = 5.0;

            if (reject) {
                return;
            }

            this.addVisionMeasurement(vm.measurement.pose, vm.timestamp.in(Seconds),
                    VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
        } else if (vm.kind.equals(VisionMeasurement.Kind.MT2)) {
            boolean reject = false;

            reject |= this.getPigeon2().getAngularVelocityZWorld().getValue().abs(RotationsPerSecond) >= 0.5;
            reject |= vm.measurement.avgTagDist >= 4;
            reject |= vm.measurement.avgTagArea < 0.1;

            if (reject) {
                return;
            }

            if (Math.abs(this.getState().Speeds.vxMetersPerSecond) <= 0.25
                    && Math.abs(this.getState().Speeds.vxMetersPerSecond) <= 0.25) {
                xyStdDev = 2.0;
            } else {
                xyStdDev = 5.0;
            }

            this.addVisionMeasurement(vm.measurement.pose, vm.timestamp.in(Seconds),
                    VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
        }
    }

    private final SwerveRequest.FieldCentricFacingAngle centric = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withTargetDirection(new Rotation2d())
            .withRotationalDeadband(Constants.Drive.MIN_ROTATIONAL_SPEED)
            .withDeadband(Constants.Drive.MIN_TRANSLATIONAL_SPEED);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drive.MIN_TRANSLATIONAL_SPEED_TELEOP)
            .withRotationalDeadband(MIN_ROTATIONAL_SPEED_TELEOP)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
            .withDriveRequestType(DriveRequestType.Velocity);

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param poses Provides poses to go to at any given time.
     */
    public Command goToPose(Supplier<Pose2d> poses) {
        return this.run(() -> {
            this.targetPose = poses.get();

            if (this.targetPose == null)
                return;

            Pose2d pose = getState().Pose;
            double currentTime = Utils.getCurrentTimeSeconds();
            this.setControl(this.centric
                    .withTargetDirection(this.targetPose.getRotation())
                    .withVelocityX(m_pathXController.calculate(pose.getX(), this.targetPose.getX(), currentTime))
                    .withVelocityY(m_pathYController.calculate(pose.getY(), this.targetPose.getY(), currentTime)));
        });
    }

    public final Trigger reachedPose = new Trigger(
            () -> this.targetPose != null
                    && !this.targetPose.equals(new Pose2d())
                    && Util.epsilonEquals(this.targetPose, this.getState().Pose));

    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public Command coralReefDrive(int level, boolean lefty) {
        return this.goToPose(() -> {
            boolean left = lefty;
            int closestTag = AutoAiming.getClosestFace(this.getState().Pose.getTranslation()).getTagId();

            if (closestTag >= 9 && closestTag <= 11 || closestTag >= 20 && closestTag <= 22) {
                left = !left;
            }

            Pose2d tagXY = layout.getTagPose(closestTag).orElse(new Pose3d()).toPose2d();

            Translation2d offset = new Translation2d();

            if (left) {
                offset = switch (level) {
                    case 1 -> Constants.AutoAlign.leftBranchOffset_L1;
                    case 2 -> Constants.AutoAlign.leftBranchOffset_L2;
                    case 3 -> Constants.AutoAlign.leftBranchOffset_L3;
                    case 4 -> Constants.AutoAlign.leftBranchOffset_L4;
                    default -> offset;
                };
            } else {
                offset = switch (level) {
                    case 1 -> Constants.AutoAlign.rightBranchOffset_L1;
                    case 2 -> Constants.AutoAlign.rightBranchOffset_L2;
                    case 3 -> Constants.AutoAlign.rightBranchOffset_L3;
                    case 4 -> Constants.AutoAlign.rightBranchOffset_L4;
                    default -> offset;
                };
            }

            return tagXY.plus(new Transform2d(offset, new Rotation2d(Math.PI)));
        });
    }

    public Command algaeReefDrive(int level) {
        return null;
    }

    private double multiplier = 1.0;

    public Command engageSlowMode() {
        return this.runOnce(() -> this.multiplier = 0.5);
    }

    public Command disengageSlowMode() {
        return this.runOnce(() -> this.multiplier = 1.0);
    }

    public Command teleopDrive(DoubleSupplier leftStickX, DoubleSupplier leftStickY, DoubleSupplier rightStickX) {
        return this.run(() -> {
            var vX = MAX_teleop_velocity.times(Util.applyDeadband(-leftStickY.getAsDouble())).times(this.multiplier);
            var vY = MAX_teleop_velocity.times(Util.applyDeadband(-leftStickX.getAsDouble())).times(this.multiplier);
            var omega = MAX_teleop_rotation.times(Util.applyDeadband(-rightStickX.getAsDouble()))
                    .times(this.multiplier);

            if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
                vX = vX.unaryMinus();
                vY = vY.unaryMinus();
            }

            this.setControl(this.drive
                    .withVelocityX(vX)
                    .withVelocityY(vY)
                    .withRotationalRate(omega));
        }).withName("regular drive");
    }

    public Command resetGyroCommand() {
        return this.runOnce(this::seedFieldCentric);
    }

    // move all sysid stuff to a new file in lib

    private final SwerveRequests9140.SysIdSwerveSteerTorqueCurrentFOC m_steerSysID = new SwerveRequests9140.SysIdSwerveSteerTorqueCurrentFOC();

    // for tuning steer motors
    private final SysIdRoutineTorqueCurrent m_steerRoutine = new SysIdRoutineTorqueCurrent(
            new SysIdRoutineTorqueCurrent.Config(
                    Amps.of(1.0).per(Second),
                    Amps.of(5.0),
                    Seconds.of(15),
                    state -> SignalLogger.writeString("sysIdSteer_state", state.toString())),
            new SysIdRoutineTorqueCurrent.Mechanism(
                    output -> setControl(m_steerSysID.withAmps(output)),
                    null,
                    this));

    // put back routine for tuning drive motors

    // @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        this.setControl(this.drive.withVelocityX(output.in(Volts)));
                        SignalLogger.writeDouble("Target_Velocity", output.in(Volts));
                        SignalLogger.writeDouble("Actual_Velocity", this.getState().Speeds.vxMetersPerSecond);
                        SignalLogger.writeDouble("Drive_Position", this.getState().Pose.getX());
                    },
                    null,
                    this));

    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    // @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    Seconds.of(30), // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Target_Rotational_Rate_RadPerSec", output.in(Volts));
                        SignalLogger.writeDouble("Actual_Rotational_Rate_RadPerSec",
                                this.getPigeon2().getAngularVelocityZWorld().getValue().in(RadiansPerSecond));
                        SignalLogger.writeDouble("Actual_Rotational_Position_Radians",
                                this.getPigeon2().getRotation2d().getRadians());
                    },
                    null,
                    this));

    public Command sysIdSteerQ(SysIdRoutine.Direction direction) {
        return m_steerRoutine.quasistatic(direction);
    }

    public Command sysIdSteerD(SysIdRoutine.Direction direction) {
        return m_steerRoutine.dynamic(direction);
    }

    public Command sysIdRotateQ(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineRotation.quasistatic(direction);
    }

    public Command sysIdRotateD(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineRotation.dynamic(direction);
    }

    public Command sysIdDriveQ(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTranslation.quasistatic(direction);
    }

    public Command sysIdDriveD(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTranslation.dynamic(direction);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod.in(Seconds));
    }

}