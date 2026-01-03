package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SimulationVisualizer;
import org.littletonrobotics.junction.Logger;

/**
 * Class that holds control logic and public interface for the elevator.
 */
public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged replayedInputs = new ElevatorIOInputsAutoLogged();

    private ElevatorHeight currentElevatorSetpoint = ElevatorHeight.DOWN;

    /**
     * @param io The hardware implementation for the elevator, either sim or real.
     */
    public Elevator(ElevatorIO io) {
        var motorConfig = new TalonFXConfiguration()
                .withSlot0(
                        new Slot0Configs() // Motor PID and gain values.
                                .withKP(ElevatorConstants.ELEVATOR_kP)
                                .withKI(ElevatorConstants.ELEVATOR_kI)
                                .withKD(ElevatorConstants.ELEVATOR_kD)
                                .withKS(ElevatorConstants.ELEVATOR_kS)
                                .withKV(ElevatorConstants.ELEVATOR_kV)
                                .withKA(ElevatorConstants.ELEVATOR_kA)
                                .withKG(ElevatorConstants.ELEVATOR_kG))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive) // Invert motor rotation.
                        );
        var motor = new TalonFX(Constants.ELEVATOR_ID);
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.getConfigurator().apply(motorConfig);

        io.setMotor(motor);
        this.io = io;
    }

    public void stopElevator() {
        io.setMotorVoltage(Volts.of(0));
    }

    @Override
    public void periodic() {
        io.updateState(replayedInputs);
        // `processInputs` must be called every periodic after updating hardware state.
        // In `REPLAY` mode, `updateState` does nothing, and the `replayedInputs` are populated from the replayed logs
        // instead.
        Logger.processInputs("Elevator", replayedInputs);
        Logger.recordOutput(
                "Elevator/Carriage Setpoint (inches)",
                currentElevatorSetpoint.getPosition().in(Inches));
        Logger.recordOutput(
                "Elevator/Motor Setpoint (rotations)",
                currentElevatorSetpoint.getPositionAngle().in(Rotations));
    }

    @Override
    public void simulationPeriodic() {
        SimulationVisualizer.getInstance().updateElevatorHeight(Inches.of(replayedInputs.carriagePositionInches));
    }

    public void setElevatorPosition(ElevatorHeight heightSetpoint) {
        currentElevatorSetpoint = heightSetpoint;
        io.setMotorSetpoint(heightSetpoint.getPositionAngle());
    }

    public Command setElevatorPositionCommand(ElevatorHeight height) {
        return new InstantCommand(() -> setElevatorPosition(height));
    }

    public enum ElevatorHeight {
        // Positions taken from offseason bot code, inturn taken from onshape.
        UP(Inches.of(56.0)),
        MIDDLE(Inches.of(28.0)),
        DOWN(Inches.of(0.0));

        public final Distance position;

        ElevatorHeight(Distance position) {
            this.position = position;
        }

        public Distance getPosition() {
            return position;
        }

        public Angle getPositionAngle() {
            // NOTE: Divide by 2 because the motor controls the first stage only, not the second stage
            return Rotations.of(
                    (getPosition().in(Inches) / 2) / ElevatorConstants.ELEVATOR_HEIGHT_CHANGE_PER_MOTOR_ROTATION);
        }
    }
}
