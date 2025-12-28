package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.arm.ArmConstants.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

// TODO: Test this class on LePrawn, ensure the arm won't rotate more than once in any direction.
public class ArmIOReal implements ArmIO {

    // Pivot motors & configurations
    TalonFX pivotMotor;
    TalonFXConfiguration pivotMotorConfig;

    CANcoder armEncoder;
    CANcoderConfiguration armEncoderConfig;

    // Intake/Outtake motors & configurations
    TalonFX intakeMotor;
    TalonFXConfiguration intakeMotorConfig;

    public ArmIOReal() {
        // Set up the arm encoder (records the arm position)
        armEncoder = new CANcoder(ARM_ENCODER_ID);

        armEncoderConfig = new CANcoderConfiguration();
        armEncoderConfig.withMagnetSensor(
                new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.Clockwise_Positive));

        armEncoder.getConfigurator().apply(armEncoderConfig);

        // Set up arm pivot motor.
        pivotMotor = new TalonFX(ARM_PIVOT_ID);

        // Set the current position of the arm to what the position of the absolute encoder.
        // Negates the encoder position because the encoder and motor are on different sides of the arm carriage.
        Angle zeroPos = armEncoder
                .getAbsolutePosition()
                .getValue()
                .plus(Rotations.of(0.408691).plus(Degrees.of(90)));
        pivotMotor.setPosition(zeroPos.times(MOTOR_TO_ARM_GEARING));
        Logger.recordOutput("Arm/Encoder/ABS_VAL_Deg", zeroPos.in(Degrees));
        Logger.recordOutput("Arm/Encoder/ABS_VAL_Rot", zeroPos.in(Rotations));

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
        intakeMotor = new TalonFX(ARM_INTAKE_ID);

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
        inputs.armPositionDegrees = pivotMotor.getPosition().getValue().in(Degrees) / MOTOR_TO_ARM_GEARING;

        inputs.pivotMotorRotations = pivotMotor.getPosition().getValue().in(Rotations);
        inputs.pivotMotorVelocityRotsPerSecond =
                pivotMotor.getVelocity().getValue().in(RotationsPerSecond);
        inputs.pivotMotorCurrent = pivotMotor.getStatorCurrent().getValue().in(Amp);
        inputs.pivotMotorVoltage = pivotMotor.getMotorVoltage().getValue().in(Volts);

        inputs.intakeMotorVelocityRotsPerSecond =
                intakeMotor.getVelocity().getValue().in(RotationsPerSecond);
        inputs.intakeMotorVoltage = intakeMotor.getMotorVoltage().getValue().in(Volts);

        inputs.encoderPositionDegrees = armEncoder.getPosition().getValue().in(Degrees);
    }

    @Override
    public void setPivotMotorSetpoint(Angle setpoint) {
        // "0" in sim is facing to the east, while in real robot, it is north. No me gusta, but thus must be done.
        Logger.recordOutput("Arm/PivotMotor/SetpointDeg", setpoint.in(Degrees));
        pivotMotor.setControl(new PositionVoltage(setpoint));
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
