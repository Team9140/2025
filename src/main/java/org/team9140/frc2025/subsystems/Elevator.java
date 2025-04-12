package org.team9140.frc2025.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import org.team9140.frc2025.Constants;
import org.team9140.frc2025.Constants.Ports;
import org.team9140.lib.SysIdRoutineTorqueCurrent;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

public class Elevator extends SubsystemBase {
    private final TalonFX rightMotor = new TalonFX(Ports.ELEVATOR_MOTOR_RIGHT, "sigma");
    private final TalonFX leftMotor = new TalonFX(Ports.ELEVATOR_MOTOR_LEFT, "sigma");

    private final MotionMagicTorqueCurrentFOC motionMagic;

    private Distance targetPosition;

    private static final Time kSimLoopPeriod = Milliseconds.of(5);
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final ElevatorSim elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60Foc(2),
            Constants.Elevator.GEAR_RATIO,
            Constants.Elevator.mass.in(Kilograms),
            Constants.Elevator.SPOOL_RADIUS.in(Meters),
            Constants.Elevator.MIN_HEIGHT.in(Meters),
            Constants.Elevator.MAX_HEIGHT.in(Meters),
            true,
            Constants.Elevator.STOW_height.in(Meters));

    private Elevator() {
        Slot0Configs elevatorGains = new Slot0Configs()
                .withKP(150.0)
                .withKI(0.0)
                .withKD(12.0)
                .withKS(0.88302)
                .withKV(0.7863)
                .withKA(0.44435)
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKG(9.6);

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Elevator.STATOR_LIMIT)
                .withStatorCurrentLimitEnable(true);

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.Elevator.CRUISE_VELOCITY)
                .withMotionMagicAcceleration(Constants.Elevator.ACCELERATION);

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.Elevator.GEAR_RATIO);

        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Constants.Elevator.SOFT_LIMIT
                        .div(Constants.Elevator.SPOOL_CIRCUMFERENCE).magnitude())
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(0.0);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withSlot0(elevatorGains)
                .withCurrentLimits(currentLimitsConfigs)
                .withFeedback(feedbackConfigs)
                .withMotionMagic(motionMagicConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withSoftwareLimitSwitch(softLimits);

        this.rightMotor.getConfigurator().apply(motorConfig);
        this.rightMotor.getConfigurator().apply(feedbackConfigs);
        // this.rightMotor.setPosition(0.0);

        this.motionMagic = new MotionMagicTorqueCurrentFOC(0)
                .withSlot(0);

        this.targetPosition = Constants.Elevator.STOW_height;

        this.rightMotor.setControl(this.motionMagic.withPosition(
                this.targetPosition.div(Constants.Elevator.SPOOL_CIRCUMFERENCE).magnitude()));
        this.leftMotor.setControl(new Follower(this.rightMotor.getDeviceID(), true));

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    TorqueCurrentFOC testOutput = new TorqueCurrentFOC(0);
    SysIdRoutineTorqueCurrent routine = new SysIdRoutineTorqueCurrent(
            new SysIdRoutineTorqueCurrent.Config(Amps.of(5.0).per(Second), Amps.of(20.0), Seconds.of(10.0),
                    state -> SignalLogger.writeString("elevator sysid", state.toString())),
            new SysIdRoutineTorqueCurrent.Mechanism(output -> this.rightMotor.setControl(testOutput.withOutput(output)),
                    null, this));

    public Command qForward() {
        return this.routine.quasistatic(Direction.kForward);
    }

    public Command qReverse() {
        return this.routine.quasistatic(Direction.kReverse);
    }

    public Command dForward() {
        return this.routine.dynamic(Direction.kForward);
    }

    public Command dReverse() {
        return this.routine.dynamic(Direction.kReverse);
    }

    private static Elevator instance;

    public static Elevator getInstance() {
        return (instance == null) ? instance = new Elevator() : instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Current Position Inch", getPosition().in(Inches));
        SmartDashboard.putNumber("Elevator Target Position Inch", targetPosition.in(Inches));
        // SmartDashboard.putNumber("Elevator Voltage",
        // rightMotor.getMotorVoltage(false).getValueAsDouble());
        // SmartDashboard.putNumber("Elevator Current",
        // rightMotor.getStatorCurrent(false).getValueAsDouble());
        // SmartDashboard.putNumber("Elevator raw position",
        // rightMotor.getPosition(false).getValueAsDouble());
        SmartDashboard.putBoolean("Elevator at target", this.atPosition.getAsBoolean());
        SmartDashboard.putBoolean("Elevator algae", this.isAlgaeing.getAsBoolean());
        // SmartDashboard.putNumber("error",
        // this.leftMotor.getClosedLoopError().getValueAsDouble());

        // refresh once instead of letting every trigger do it every loop
        this.rightMotor.getPosition().refresh();
    }

    public void updateSimState(double deltatime) {
        double simvVolts = this.rightMotor.getSimState().getMotorVoltage();
        this.elevatorSim.setInputVoltage(simvVolts);
        this.elevatorSim.update(deltatime);

        Distance simPosition = Meters.of(this.elevatorSim.getPositionMeters());
        LinearVelocity simVelocity = MetersPerSecond.of(this.elevatorSim.getVelocityMetersPerSecond());

        this.rightMotor.getSimState().setRawRotorPosition(
                Rotations.of(simPosition.div(Constants.Elevator.SPOOL_CIRCUMFERENCE).magnitude()
                        * Constants.Elevator.GEAR_RATIO));

        this.rightMotor.getSimState().setRotorVelocity(
                RotationsPerSecond
                        .of(simVelocity.div(Constants.Elevator.SPOOL_CIRCUMFERENCE).magnitude()
                                * Constants.Elevator.GEAR_RATIO));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime);
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod.in(Seconds));
    }

    public Distance getPosition() {
        return Constants.Elevator.SPOOL_CIRCUMFERENCE
                .times(this.rightMotor.getPosition(false).getValue().in(Rotations));
    }

    public Command moveToPosition(Distance goalPosition) {
        return this.runOnce(() -> {
            this.targetPosition = goalPosition;
            this.rightMotor.setControl(this.motionMagic.withPosition(
                    this.targetPosition.div(Constants.Elevator.SPOOL_CIRCUMFERENCE).magnitude()));
        }).andThen(new WaitUntilCommand(atPosition));
    }

    public Command moveToPosition(Supplier<Distance> goalPosition) {
        return this.runOnce(() -> {
            this.targetPosition = goalPosition.get();
            this.rightMotor.setControl(this.motionMagic.withPosition(
                    this.targetPosition.div(Constants.Elevator.SPOOL_CIRCUMFERENCE).magnitude()));
        }).andThen(new WaitUntilCommand(atPosition));
    }

    public final Trigger isUp = new Trigger(() -> this.getPosition().gt(Feet.of(3)));
    private final Distance algaeingCenter = Constants.Elevator.L3_ALGAE_height.plus(Constants.Elevator.L2_ALGAE_height)
            .div(2.0);
    public final Trigger isAlgaeing = new Trigger(() -> this.getPosition().isNear(algaeingCenter, Inches.of(18.0)));
    public final Trigger atPosition = new Trigger(
            () -> this.getPosition().isNear(this.targetPosition, Constants.Elevator.POSITION_epsilon));
    public final Trigger isStowed = new Trigger(
            () -> this.getPosition().isNear(Constants.Elevator.STOW_height, Inches.of(0.75)));
}