package org.team9140.frc2025.subsystems;

import static edu.wpi.first.units.Units.*;
import static org.team9140.frc2025.Constants.Drive.MAX_teleop_rotation;
import static org.team9140.frc2025.Constants.Drive.MAX_teleop_velocity;
import static org.team9140.frc2025.Constants.Drive.MIN_ROTATIONAL_SPEED;
import static org.team9140.frc2025.Constants.Drive.MIN_ROTATIONAL_SPEED_TELEOP;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import org.team9140.frc2025.Constants.ElevatorSetbacks;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.*;
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
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final Time kSimLoopPeriod = Milliseconds.of(5);
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final ADIS16470_IMU green_gyro = new ADIS16470_IMU();

    // move magic numbers to constants
    private final PhoenixPIDController m_pathXController = new PhoenixPIDController(TunerConstants.X_CONTROLLER_P,
            TunerConstants.X_CONTROLLER_I, TunerConstants.X_CONTROLLER_D);
    private final PhoenixPIDController m_pathYController = new PhoenixPIDController(TunerConstants.Y_CONTROLLER_P,
            TunerConstants.Y_CONTROLLER_I, TunerConstants.Y_CONTROLLER_D);
    private final PhoenixPIDController headingController = new PhoenixPIDController(TunerConstants.HEADING_CONTROLLER_P,
            TunerConstants.HEADING_CONTROLLER_I, TunerConstants.HEADING_CONTROLLER_D);// 11.0, 0.0, 0.25

    Field2d dashField2d = new Field2d();

    private Pose2d targetPose = new Pose2d();
    private final double[] targetPoseDecomposed = new double[] { 0, 0, 0 };

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

        this.getPigeon2().getConfigurator().apply(new GyroTrimConfigs().withGyroScalarZ(-2.94));

        SmartDashboard.putData("field", dashField2d);

        SmartDashboard.putNumberArray("drive target pose", targetPoseDecomposed);
    }

    @Override
    public void periodic() {
        dashField2d.setRobotPose(this.getState().Pose);

        SmartDashboard.putNumber("pigeon yaw", this.getPigeon2().getYaw().getValueAsDouble());
        SmartDashboard.putNumber("green yaw", this.green_gyro.getAngle());
        SmartDashboard.putNumber("x error", this.m_pathXController.getPositionError());
        SmartDashboard.putNumber("y error", this.m_pathYController.getPositionError());
        SmartDashboard.putNumber("angle error", this.headingController.getPositionError());

        if (this.getCurrentCommand() != null) {
            SignalLogger.writeString("drivetrain command", this.getCurrentCommand().getName());
            SmartDashboard.putString("drivetrain command", this.getCurrentCommand().getName());
        } else {
            SignalLogger.writeString("drivetrain command", "N/A");
            SmartDashboard.putString("drivetrain command", "N/A");
        }

        if (this.targetPose != null) {
            targetPoseDecomposed[0] = this.targetPose.getX();
            targetPoseDecomposed[1] = this.targetPose.getY();
            targetPoseDecomposed[2] = this.targetPose.getRotation().getRadians();
        } else {
            targetPoseDecomposed[0] = -1;
            targetPoseDecomposed[1] = -1;
            targetPoseDecomposed[2] = -1;
        }

        SmartDashboard.putNumberArray("drive target pose", targetPoseDecomposed);
        SignalLogger.writeDoubleArray("drive target pose", targetPoseDecomposed);
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

    // TODO: Potentially use SwerveRequest.ApplyFieldSpeeds or SwerveRequest.ApplyRobotSpeeds
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drive.MIN_TRANSLATIONAL_SPEED_TELEOP)
            .withRotationalDeadband(MIN_ROTATIONAL_SPEED_TELEOP)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.FieldCentric auton = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drive.MIN_TRANSLATIONAL_SPEED)
            .withRotationalDeadband(MIN_ROTATIONAL_SPEED)
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
     * Follows the given field-centric path sample with PID and applies any velocities as feed forwards.
     *
     * @param sample Provides sample to execute.
     */
    public void followSample(SwerveSample sample) {
        Pose2d currPose = getState().Pose;
        Pose2d target = sample.getPose();

        double currentTime = Utils.getCurrentTimeSeconds();

        this.setControl(this.auton
                .withRotationalRate(sample.omega + this.headingController.calculate(currPose.getRotation().getRadians(), target.getRotation().getRadians(), currentTime))
                .withVelocityX(sample.vx + this.m_pathXController.calculate(currPose.getX(), target.getX(), currentTime))
                .withVelocityY(sample.vy + this.m_pathYController.calculate(currPose.getY(), target.getY(), currentTime)));
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
            this.setControl(this.auton
                    .withRotationalRate(this.headingController.calculate(pose.getRotation().getRadians(), this.targetPose.getRotation().getRadians(), currentTime))
                    .withVelocityX(m_pathXController.calculate(pose.getX(), this.targetPose.getX(), currentTime))
                    .withVelocityY(m_pathYController.calculate(pose.getY(), this.targetPose.getY(), currentTime)));
        });
    }

//    public final Trigger reachedPose = new Trigger(
//            () -> this.targetPose != null
//                    && !this.targetPose.equals(new Pose2d())
//                    && Util.epsilonEquals(this.targetPose, this.getState().Pose)).debounce(REACHEDPOSE_DEBOUNCE.in(Seconds), Debouncer.DebounceType.kBoth);

    public Command coralReefDrive(ElevatorSetbacks level, boolean lefty) {
        return this.goToPose(() -> {
            AutoAiming.ReefFaces closestReef = AutoAiming.getClosestFace(this.getState().Pose.getTranslation());

            return lefty ? closestReef.getLeft(level) : closestReef.getRight(level);
        }).withName("coral drive");
    }

    public Command algaeReefDrive(ElevatorSetbacks level) {
        return this.goToPose(() -> AutoAiming.getClosestFace(this.getState().Pose.getTranslation()).getCenter(level)).withName("coral drive");
    }

    private double multiplier = 1.0;

    public Command engageSlowMode() {
        // intentionally don't require subsystem
        return Commands.runOnce(() -> this.multiplier = 0.5);
    }

    public Command disengageSlowMode() {
        // intentionally don't require subsystem
        return Commands.runOnce(() -> this.multiplier = 1.0);
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

    public Command stop() {
        return this.runOnce(() -> this.setControl(new SwerveRequest.Idle()));
    }

    public Command resetGyroCommand() {
        return this.runOnce(this::seedFieldCentric);
    }

    public Command resetGyroCommand(Angle angle) {
        return this.runOnce(() -> {
            this.green_gyro.setGyroAngleZ(angle.in(Degrees));
            this.getPigeon2().setYaw(angle);
        });
    }

    // TODO:
    //  move all sysid stuff to a new file in lib

//    private final SwerveRequests9140.SysIdSwerveSteerTorqueCurrentFOC m_steerSysID = new SwerveRequests9140.SysIdSwerveSteerTorqueCurrentFOC();
//
//    // for tuning steer motors
//    private final SysIdRoutineTorqueCurrent m_steerRoutine = new SysIdRoutineTorqueCurrent(
//            new SysIdRoutineTorqueCurrent.Config(
//                    Amps.of(1.0).per(Second),
//                    Amps.of(5.0),
//                    Seconds.of(15),
//                    state -> SignalLogger.writeString("sysIdSteer_state", state.toString())),
//            new SysIdRoutineTorqueCurrent.Mechanism(
//                    output -> setControl(m_steerSysID.withAmps(output)),
//                    null,
//                    this));
//
//    private final SwerveRequest.SysIdSwerveTranslation m_driveSysID = new SwerveRequest.SysIdSwerveTranslation();
//
//    private final SysIdRoutine m_driveRoutine = new SysIdRoutine(
//            new SysIdRoutine.Config(
//                    Volts.of(1.0).per(Second),
//                    Volts.of(4.0),
//                    Seconds.of(10),
//                    state -> SignalLogger.writeString("sysIdDrive_state", state.toString())),
//            new SysIdRoutine.Mechanism(output -> setControl(m_driveSysID.withVolts(output)), null, this));
//
//    @SuppressWarnings("unused")
//    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
//            new SysIdRoutine.Config(
//                    null, // Use default ramp rate (1 V/s)
//                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
//                    null, // Use default timeout (10 s)
//                    // Log state with SignalLogger class
//                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
//            new SysIdRoutine.Mechanism(
//                    output -> {
//                        this.setControl(this.drive.withVelocityX(output.in(Volts)));
//                        SignalLogger.writeDouble("Target_Velocity", output.in(Volts));
//                        SignalLogger.writeDouble("Actual_Velocity", this.getState().Speeds.vxMetersPerSecond);
//                        SignalLogger.writeDouble("Drive_Position", this.getState().Pose.getX());
//                    },
//                    null,
//                    this));
//
//    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
//
//    // @SuppressWarnings("unused")
//    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
//            new SysIdRoutine.Config(
//                    /* This is in radians per second², but SysId only supports "volts per second" */
//                    Volts.of(Math.PI / 6).per(Second),
//                    /* This is in radians per second, but SysId only supports "volts" */
//                    Volts.of(Math.PI),
//                    Seconds.of(30), // Use default timeout (10 s)
//                    // Log state with SignalLogger class
//                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
//            new SysIdRoutine.Mechanism(
//                    output -> {
//                        /* output is actually radians per second, but SysId only supports "volts" */
//                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
//                        /* also log the requested output for SysId */
//                        SignalLogger.writeDouble("Target_Rotational_Rate_RadPerSec", output.in(Volts));
//                        SignalLogger.writeDouble("Actual_Rotational_Rate_RadPerSec",
//                                this.getPigeon2().getAngularVelocityZWorld().getValue().in(RadiansPerSecond));
//                        SignalLogger.writeDouble("Actual_Rotational_Position_Radians",
//                                this.getPigeon2().getRotation2d().getRadians());
//                    },
//                    null,
//                    this));
//
//    public Command sysIdSteerQ(SysIdRoutine.Direction direction) {
//        return m_steerRoutine.quasistatic(direction);
//    }
//
//    public Command sysIdSteerD(SysIdRoutine.Direction direction) {
//        return m_steerRoutine.dynamic(direction);
//    }
//
//    public Command sysIdRotateQ(SysIdRoutine.Direction direction) {
//        return m_sysIdRoutineRotation.quasistatic(direction);
//    }
//
//    public Command sysIdRotateD(SysIdRoutine.Direction direction) {
//        return m_sysIdRoutineRotation.dynamic(direction);
//    }
//
//    public Command sysIdDriveQ(SysIdRoutine.Direction direction) {
//        return m_driveRoutine.quasistatic(direction);
//    }
//
//    public Command sysIdDriveD(SysIdRoutine.Direction direction) {
//        return m_driveRoutine.dynamic(direction);
//    }

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