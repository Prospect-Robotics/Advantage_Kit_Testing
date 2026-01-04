package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOReal implements ElevatorIO {

    // TODO(vdikov): Looks like IOReal and IOSim could share the motor instance.
    private TalonFX motor;

    private final PositionVoltage positionControl = new PositionVoltage(Rotations.of(0));

    public ElevatorIOReal() {}

    @Override
    public void setMotor(TalonFX motor) {
        this.motor = motor;
    }

    @Override
    public void updateState(ElevatorIOInputs inputs) {
        // TODO: This conversion code might be a major thorn later, or maybe its fine, and the sim code needs rework.
        inputs.carriagePositionInches = getCarriagePosition().in(Inches);
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
        return motorRotationToCarriagePosition(motor.getPosition().getValue());
    }

    private static Distance motorRotationToCarriagePosition(Angle motorPosition) {
        return Inches.of(motorPosition.in(Rotations) * ELEVATOR_HEIGHT_CHANGE_PER_MOTOR_ROTATION)
                .times(2);
    }
}
