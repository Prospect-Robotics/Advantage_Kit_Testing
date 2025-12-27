package frc.robot.subsystems.groundintake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class GroundIntakePivotIOReal implements GroundIntakePivotIO {

    TalonFX groundIntakePivotMotor;
    TalonFXConfiguration groundIntakeMotorPivotConfig;

    private PositionVoltage positionVoltage = new PositionVoltage(0);

    public GroundIntakePivotIOReal() {

        groundIntakePivotMotor = new TalonFX(Constants.GROUND_INTAKE_PIVOT_ID);
    }

    @Override
    public void updateState(GroundIntakePivotIOInputs inputs) {
        inputs.groundIntakePivotAngleDegrees = 0.0;

        inputs.pivotMotorRotations = groundIntakePivotMotor.getPosition().getValueAsDouble();
        inputs.pivotMotorVelocityRotsPerSecond =
                groundIntakePivotMotor.getVelocity().getValueAsDouble();
        inputs.pivotMotorCurrent = groundIntakePivotMotor.getStatorCurrent().getValueAsDouble();
        inputs.pivotMotorVoltage = groundIntakePivotMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setPivotMotorSetpoint(Angle setpoint) {
        groundIntakePivotMotor.setControl(positionVoltage.withPosition(setpoint));
    }

    @Override
    public void setPivotMotorVoltage(Voltage voltage) {
        groundIntakePivotMotor.setVoltage(voltage.magnitude());
    }

    @Override
    public Angle getPivotMotorPosition() {
        return groundIntakePivotMotor.getPosition().getValue();
    }

    @Override
    public Angle getGroundIntakePivotPosition() {
        return groundIntakePivotMotor.getPosition().getValue().div(MOTOR_TO_GROUND_INTAKE_PIVOT);
    }
}
