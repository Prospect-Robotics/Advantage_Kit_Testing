package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Revolution;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ElevatorIOReal implements ElevatorIO {

    private TalonFX motor;
    private TalonFXConfiguration motorConfig;

    private final PositionVoltage positionControl = new PositionVoltage(0);

    public ElevatorIOReal() {
        motor = new TalonFX(Constants.ELEVATOR_ID);
        motor.setNeutralMode(NeutralModeValue.Brake);

        motorConfig = new TalonFXConfiguration();

        // Config PID
        var slot0PIDConfig = motorConfig.Slot0;
        slot0PIDConfig.kP = ELEVATOR_kP;
        slot0PIDConfig.kI = ELEVATOR_kI;
        slot0PIDConfig.kD = ELEVATOR_kD;

        // Config motor inversion.
        motorConfig.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateState(ElevatorIOInputs inputs) {
        // TODO: This conversion code might be a major thorn later, or maybe its fine, and the sim code needs rework.
        inputs.carriagePositionInches = motorRotationToCarriagePosition(motor.getPosition().getValue()).in(Inches);
        inputs.motorRotations = motor.getPosition().getValueAsDouble();
        inputs.motorVelocityRotsPerSecond = motor.getVelocity().getValueAsDouble();
        inputs.motorCurrent = motor.getStatorCurrent().getValueAsDouble();
        inputs.motorVoltage = motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setMotorSetpoint(Angle setpoint) {
        motor.setControl(positionControl.withPosition(setpoint));
    }

    @Override
    public void setMotorVoltage(Voltage voltage) {
        motor.setVoltage(voltage.magnitude());
    }

    @Override
    public Angle getMotorPosition() {
        return motor.getPosition().getValue();
    }

    @Override
    public Distance getCarriagePosition() {
        return
    }

    private static Distance motorRotationToCarriagePosition(Angle motorPosition) {
        return Inches.of(motorPosition.in(Revolution) * ELEVATOR_HEIGHT_CHANGE_PER_MOTOR_ROTATION);
    }
}