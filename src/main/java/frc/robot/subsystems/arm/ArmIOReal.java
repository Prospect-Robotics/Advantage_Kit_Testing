package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

import static frc.robot.subsystems.arm.ArmConstants.*;

// TODO: Test this class on LePrawn, ensure the arm won't rotate more than once in any direction.
public class ArmIOReal implements ArmIO {

    // Pivot motors & configurations
    TalonFX pivotMotor;
    TalonFXConfiguration pivotMotorConfig;

    PositionVoltage pivotPositionControl = new PositionVoltage(0);

    // Intake/Outtake motors & configurations
    TalonFX intakeMotor;
    TalonFXConfiguration intakeMotorConfig;

    public ArmIOReal() {
        // Set up arm pivot motor.
        pivotMotor = new TalonFX(Constants.ARM_PIVOT_ID);

        // Start motor config.
        pivotMotorConfig = new TalonFXConfiguration();

        // PID
        pivotMotorConfig.withSlot0(
                new Slot0Configs().withKP(ARM_PIVOT_kP).withKI(ARM_PIVOT_kI).withKD(ARM_PIVOT_kD));

        // Motor Rotation Direction - Positive: arm moves clockwise.
        pivotMotorConfig.withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

        // End motor config.
        pivotMotor.getConfigurator().apply(pivotMotorConfig);

        // Set up arm intake motor.
        intakeMotor = new TalonFX(Constants.ARM_INTAKE_ID);

        // Start motor config.
        intakeMotorConfig = new TalonFXConfiguration();

        // Motor Rotation Direction - Positive: intake. Negative: outtake.
        intakeMotorConfig.withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

        // End motor config.
        intakeMotor.getConfigurator().apply(intakeMotorConfig);
    }

    @Override
    public void updateState(ArmIOInputs inputs) {
        inputs.armPositionDegrees = 0.0;

        inputs.pivotMotorRotations = pivotMotor.getPosition().getValueAsDouble();
        inputs.pivotMotorVelocityRotsPerSecond = pivotMotor.getVelocity().getValueAsDouble();
        inputs.pivotMotorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();
        inputs.pivotMotorVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();

        inputs.intakeMotorVelocityRotsPerSecond = intakeMotor.getVelocity().getValueAsDouble();
        inputs.intakeMotorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setPivotMotorSetpoint(Angle setpoint) {
        pivotMotor.setControl(pivotPositionControl.withPosition(setpoint));
    }

    @Override
    public void setPivotMotorVoltage(Voltage voltage) {
        pivotMotor.setVoltage(voltage.magnitude());
    }

    @Override
    public Angle getPivotMotorPosition() {
        return pivotMotor.getPosition().getValue();
    }

    // TODO & NOTE: Test on bot, if it doesn't work move to the Arm.java (this and sim impl) and fix.
    @Override
    public Angle getArmPosition() {
        return pivotMotor.getPosition().getValue().div(MOTOR_TO_ARM_GEARING);
    }

    @Override
    public void setIntakeMotorVoltage(Voltage voltage) {
        intakeMotor.setVoltage(voltage.magnitude());
    }

}
