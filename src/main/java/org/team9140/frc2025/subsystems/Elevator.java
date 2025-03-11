package org.team9140.frc2025.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.team9140.frc2025.Constants;
import org.team9140.frc2025.Robot;
import org.team9140.frc2025.Constants.Ports;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Elevator extends SubsystemBase {
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    private final MotionMagicVoltage motionMagic;

    private Distance targetPosition;

    private final ElevatorSim elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(20),
            Constants.Elevator.GEAR_RATIO * 10,
            Constants.Elevator.mass.in(Kilograms),
            Constants.Elevator.SPOOL_RADIUS.in(Meters),
            Constants.Elevator.MIN_HEIGHT.in(Meters),
            Constants.Elevator.MAX_HEIGHT.in(Meters),
            false,
            Constants.Elevator.STOW_height.in(Meters));

    private Elevator() {
        leftMotor = new TalonFX(Ports.ELEVATOR_MOTOR_LEFT, "sigma");
        rightMotor = new TalonFX(Ports.ELEVATOR_MOTOR_RIGHT, "sigma");

        Slot0Configs elevatorGains = new Slot0Configs()
                .withKP(20.0)
                .withKI(0)
                .withKD(0)
                .withKS(0)
                .withKV(0)
                .withKA(0);

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Elevator.STATOR_LIMIT)
                .withStatorCurrentLimitEnable(true);

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.Elevator.CRUISE_VELOCITY)
                .withMotionMagicAcceleration(Constants.Elevator.ACCELERATION);

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.Elevator.GEAR_RATIO);

        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Constants.Elevator.L4_coral_height
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

        this.leftMotor.getConfigurator().apply(motorConfig);
        this.leftMotor.getConfigurator().apply(feedbackConfigs);
        this.leftMotor.setPosition(0.0);

        this.motionMagic = new MotionMagicVoltage(0)
                .withEnableFOC(true)
                .withSlot(0);

        this.targetPosition = Constants.Elevator.MIN_HEIGHT;

        this.rightMotor.setControl(new Follower(this.leftMotor.getDeviceID(), true));
    }

    private static Elevator instance;

    public static Elevator getInstance() {
        return (instance == null) ? instance = new Elevator() : instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Current Position Inch", getPosition().in(Inches));
        SmartDashboard.putNumber("Elevator Target Position Inch", targetPosition.in(Inches));
        SmartDashboard.putNumber("Elevator Voltage", leftMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Current", leftMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator raw position", leftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Elevator at target", this.atPosition.getAsBoolean());
        // SmartDashboard.putNumber("error",
        // this.leftMotor.getClosedLoopError().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        double simvVolts = this.leftMotor.getSimState().getMotorVoltage();
        this.elevatorSim.setInputVoltage(simvVolts);
        this.elevatorSim.update(Constants.LOOP_PERIOD.in(Seconds));

        Distance simPosition = Meters.of(this.elevatorSim.getPositionMeters());
        LinearVelocity simVelocity = MetersPerSecond.of(this.elevatorSim.getVelocityMetersPerSecond());

        this.leftMotor.getSimState().setRawRotorPosition(
                Rotations.of(simPosition.div(Constants.Elevator.SPOOL_CIRCUMFERENCE).magnitude()
                        * Constants.Elevator.GEAR_RATIO));

        this.leftMotor.getSimState().setRotorVelocity(
                RotationsPerSecond
                        .of(simVelocity.div(Constants.Elevator.SPOOL_CIRCUMFERENCE).magnitude()
                                * Constants.Elevator.GEAR_RATIO));
    }

    public Distance getPosition() {
        return Constants.Elevator.SPOOL_CIRCUMFERENCE
                .times(this.leftMotor.getPosition().getValue().in(Rotations));
    }

    public Command moveToPosition(Distance goalPosition) {
        return this.runOnce(() -> {
            this.targetPosition = goalPosition;
            this.leftMotor.setControl(this.motionMagic.withPosition(
                    this.targetPosition.div(Constants.Elevator.SPOOL_CIRCUMFERENCE).magnitude()));
        });
    }

    public final Trigger isUp = new Trigger(() -> this.getPosition().gt(Inches.of(6)));
    public final Trigger atPosition = new Trigger(() -> this.getPosition().isNear(this.targetPosition, Constants.Elevator.POSITION_epsilon) || Robot.isSimulation());
}