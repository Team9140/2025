package org.team9140.frc2025.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team9140.frc2025.Constants;

import javax.swing.text.StyleContext;

import static edu.wpi.first.units.Units.*;
import static org.team9140.frc2025.Constants.Elevator.*;

public class Elevator extends SubsystemBase {
    private final TalonFX motor;
    private final MotionMagicVoltage motionMagic;
    private Distance targetPosition;
    private final ElevatorSim elevatorSim = new ElevatorSim(
            DCMotor.getFalcon500(1), GEAR_RATIO, MASS_KG, DRUM_RADIUS_METERS,
            MIN_HEIGHT_METERS, MAX_HEIGHT_METERS, true, BOTTOM.in(Meters));
    private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
    private final MechanismLigament2d m_elevatorMech2d =
            m_mech2dRoot.append(
                    new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), ElevatorAngle.in(Degrees)));

    public static Elevator instance;

    private Elevator() {
        motor = new TalonFX(Constants.Ports.ELEVATOR_MOTOR);
        SmartDashboard.putData("Elevator Sim", m_mech2d);

        Slot0Configs elevatorGains = new Slot0Configs()
                .withKP(Constants.Elevator.P)
                .withKI(Constants.Elevator.I)
                .withKD(Constants.Elevator.D)
                .withKS(Constants.Elevator.S)
                .withKV(Constants.Elevator.V)
                .withKA(Constants.Elevator.A);

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.Elevator.CRUISE_VELOCITY)
                .withMotionMagicAcceleration(Constants.Elevator.ACCELERATION);

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(METERS_PER_MOTOR_ROTATION);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withSlot0(elevatorGains)
                .withCurrentLimits(currentLimitsConfigs)
                .withFeedback(feedbackConfigs)
                .withMotionMagic(motionMagicConfigs)
                .withMotorOutput(motorOutputConfigs);

        this.motor.getConfigurator().apply(motorConfig);
        this.motor.setNeutralMode(NeutralModeValue.Brake);

        this.motionMagic = new MotionMagicVoltage(0)
                .withEnableFOC(true)
                .withSlot(0);

        this.targetPosition = BOTTOM;
        if (Math.abs(this.getPosition().in(Meters)) < Constants.Elevator.INITIAL_VARIANCE.in(Radians)) this.motor.setPosition(BOTTOM.in(Meters));
    }

    public static Elevator getInstance() {
        return (instance == null) ? instance = new Elevator() : instance;
    }

    @Override
    public void periodic() {
        motor.setControl(motionMagic);
        SmartDashboard.putNumber("Elevator Current Position", getPosition().in(Meters));
        SmartDashboard.putNumber("Elevator Target Position", targetPosition.in(Meters));
        SmartDashboard.putNumber("Elevator Voltage", motor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {}

    public Distance getPosition() {
        return Meters.of(motor.getPosition().getValueAsDouble());
    }

    public Command moveToPosition(Distance goalPosition) {
        return this.runOnce(() -> {
            targetPosition = goalPosition;
            motionMagic.withPosition(targetPosition.in(Meters));
        });
    }
}