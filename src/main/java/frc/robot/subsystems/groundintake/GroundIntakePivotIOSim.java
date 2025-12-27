package frc.robot.subsystems.groundintake;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class GroundIntakePivotIOSim implements GroundIntakePivotIO {

    private final SingleJointedArmSim groundIntakePivotSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            GroundIntakePivotConstants.MOTOR_TO_PIVOT_GEARING,
            SingleJointedArmSim.estimateMOI(
                    GroundIntakePivotConstants.GROUND_INTAKE_LENGTH.in(Units.Meters),
                    GroundIntakePivotConstants.GROUND_INTAKE_WEIGHT.in(Units.Kilograms)),
            GroundIntakePivotConstants.GROUND_INTAKE_LENGTH.in(Units.Meters),
            GroundIntakePivotConstants.MIN_PIVOT_ANGLE.in(Radians),
            GroundIntakePivotConstants.MAX_PIVOT_ANGLE.in(Radians),
            true,
            Units.Degrees.of(0).in(Radians));

    TalonFX pivotMotor;
    TalonFXSimState pivotMotorSimState;
    TalonFXConfiguration pivotMotorConfig;

    PositionVoltage pivotPositionControl = new PositionVoltage(0);

    public GroundIntakePivotIOSim() {
        pivotMotor = new TalonFX(Constants.GROUND_INTAKE_PIVOT_ID);
        pivotMotorSimState = pivotMotor.getSimState();

        pivotMotorConfig = new TalonFXConfiguration();

        pivotMotorConfig.withSlot0(new Slot0Configs()
                .withKP(GroundIntakePivotConstants.kP_PIVOT)
                .withKI(GroundIntakePivotConstants.kI_PIVOT)
                .withKD(GroundIntakePivotConstants.kD_PIVOT));

        pivotMotorConfig.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        pivotMotor.getConfigurator().apply(pivotMotorConfig);
    }

    @Override
    public void updateState(GroundIntakePivotIOInputs inputs) {
        updateSim();

        inputs.groundIntakePivotAngleDegrees = getPivotMotorPosition().in(Units.Degrees);

        inputs.pivotMotorRotations = pivotMotor.getPosition().getValueAsDouble();
        inputs.pivotMotorVelocityRotsPerSecond = pivotMotor.getVelocity().getValueAsDouble();
        inputs.pivotMotorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();
        inputs.pivotMotorVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
    }

    private void updateSim() {
        pivotMotorSimState.setSupplyVoltage(Units.Volts.of(12));

        groundIntakePivotSim.setInputVoltage(pivotMotorSimState.getMotorVoltage());
        groundIntakePivotSim.update(0.02);

        Logger.recordOutput(
                "Simulated Ground Intake Pivot/pivotMotorSimState/Voltage", pivotMotorSimState.getMotorVoltage());

        pivotMotorSimState.setRawRotorPosition(
                edu.wpi.first.math.util.Units.radiansToRotations(groundIntakePivotSim.getAngleRads())
                        * GroundIntakePivotConstants.MOTOR_TO_PIVOT_GEARING);
        pivotMotorSimState.setRotorVelocity(groundIntakePivotSim.getVelocityRadPerSec()
                / (2.0 * Math.PI)
                * GroundIntakePivotConstants.MOTOR_TO_PIVOT_GEARING);
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

    @Override
    public Angle getGroundIntakePivotPosition() {
        return Radians.of(groundIntakePivotSim.getAngleRads());
    }
}
