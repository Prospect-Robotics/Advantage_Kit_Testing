package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
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
import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {

    // TODO(vdikov): Looks like IOReal and IOSim could share the motor instance.
    private TalonFX motor;
    // TODO(vdikov): Ideally, IOSim and IOReal work off the same motorConfig
    // (though PID values might have to be kept separately)
    private TalonFXConfiguration motorConfig;

    private final PositionVoltage positionControl = new PositionVoltage(0);

    // TODO(vdikov): this definitely should be moved to Elevator.
    private Angle currentMotorSetpoint = Rotations.of(0);

    public ElevatorIOReal() {
        motor = new TalonFX(Constants.ELEVATOR_ID);
        motor.setNeutralMode(NeutralModeValue.Brake);

        motorConfig = new TalonFXConfiguration();

        // Config PID
        var slot0PIDConfig = motorConfig.Slot0;
        slot0PIDConfig.kP = ELEVATOR_kP;
        slot0PIDConfig.kI = ELEVATOR_kI;
        slot0PIDConfig.kD = ELEVATOR_kD;
        slot0PIDConfig.kS = ELEVATOR_kS;
        slot0PIDConfig.kV = ELEVATOR_kV;
        slot0PIDConfig.kA = ELEVATOR_kA;
        slot0PIDConfig.kG = ELEVATOR_kG;

        // Config motor inversion.
        motorConfig.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        // TODO(vdikov): Discuss with Stefan if we should adopt this as a more
        // self-contained way to configure the motor.
        // motorConfig = new TalonFXConfiguration()
        //         .withSlot0(
        //                 new Slot0Configs() // Motor PID and gain values.
        //                         .withKP(ELEVATOR_kP)
        //                         .withKI(ELEVATOR_kI)
        //                         .withKD(ELEVATOR_kD)
        //                         .withKS(ELEVATOR_kS)
        //                         .withKV(ELEVATOR_kV)
        //                         .withKA(ELEVATOR_kA)
        //                         .withKG(ELEVATOR_kG))
        //         .withMotorOutput(
        //                 new MotorOutputConfigs()
        //                         .withInverted(InvertedValue.Clockwise_Positive) // Invert motor rotation.
        //                 );

        motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateState(ElevatorIOInputs inputs) {
        // TODO: This conversion code might be a major thorn later, or maybe its fine, and the sim code needs rework.
        inputs.carriagePositionInches = getCarriagePosition().in(Inches);
        inputs.motorRotations = motor.getPosition().getValueAsDouble();
        inputs.motorVelocityRotsPerSecond = motor.getVelocity().getValueAsDouble();
        inputs.motorCurrent = motor.getStatorCurrent().getValueAsDouble();
        inputs.motorVoltage = motor.getMotorVoltage().getValueAsDouble();

        // Remember to explicitly state the unit.
        Logger.recordOutput("Elevator/motor/SetpointRotations", currentMotorSetpoint.in(Rotations));
    }

    @Override
    public void setMotorSetpoint(Angle setpoint) {
        currentMotorSetpoint = setpoint;
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
