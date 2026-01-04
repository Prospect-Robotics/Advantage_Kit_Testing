package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {

    // Physics sim for the elevator.
    private final ElevatorSim elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            ElevatorConstants.MOTOR_TO_ELEVATOR_GEARING,
            ElevatorConstants.ELEVATOR_CARRIAGE_WEIGHT.in(Kilograms),
            ElevatorConstants.ELEVATOR_SPOOL_RADIUS.in(Meter),
            ElevatorConstants.ELEVATOR_MIN_HEIGHT.in(Meter),
            ElevatorConstants.ELEVATOR_MAX_HEIGHT.in(Meter),
            true,
            ElevatorConstants.ELEVATOR_MIN_HEIGHT.in(Meter));

    private TalonFX motor;
    private TalonFXSimState motorSim;

    // Used for actually moving the motor to a given position with PID applied to a voltage input.
    private final PositionVoltage positionControl = new PositionVoltage(Rotations.of(0));

    public ElevatorIOSim() {}

    @Override
    public void setMotor(TalonFX motor) {
        this.motorSim = motor.getSimState();
        this.motor = motor;
    }

    @Override
    public void updateState(ElevatorIOInputs inputs) {
        updateSim();

        inputs.carriagePositionInches =
                Meters.of(elevatorSim.getPositionMeters()).in(Inches);
        inputs.motorCurrent = motor.getStatorCurrent().getValueAsDouble();
        inputs.motorRotations = motor.getPosition().getValueAsDouble();
        inputs.motorVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.motorVelocityRotsPerSecond = motor.getVelocity().getValueAsDouble();
    }

    private void updateSim() {
        motorSim.setSupplyVoltage(Volts.of(12));
        double motorInverted = -1.0; // -1 for inverted, 1 for forward motor.

        // Apply the voltage to the sim elevator that we apply to the sim motor.
        // Negating the sim motor value since it is set to use negative value when pushing
        // the cartrage UP.
        elevatorSim.setInputVoltage(motorInverted * motorSim.getMotorVoltage());
        elevatorSim.update(0.02); // Same update cycle as an actual robot, 20 ms.

        // Logs to "Real Outputs" NT
        Logger.recordOutput("Simulated Elevator/motorSim/Voltage", motorSim.getMotorVoltage());
        Logger.recordOutput("Simulated Elevator/elevatorSim/position (meters)", elevatorSim.getPositionMeters());
        Logger.recordOutput("Simulated Elevator/elevatorSim/hitsUpperLimit", elevatorSim.hasHitUpperLimit());
        Logger.recordOutput("Simulated Elevator/elevatorSim/hitsLowerLimit", elevatorSim.hasHitLowerLimit());

        motorSim.setRawRotorPosition(motorInverted * getMotorRotations(elevatorSim.getPositionMeters()));

        // angular velocity = linear velocity / radius, taken also from 5414
        motorSim.setRotorVelocity(motorInverted
                * ((elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.ELEVATOR_SPOOL_RADIUS.in(Meters))
                        // radians/sec to rotations/sec
                        / (2.0 * Math.PI))
                * ElevatorConstants.MOTOR_TO_ELEVATOR_GEARING);
    }

    @Override
    public void setMotorSetpoint(Angle setpoint) {
        motor.setControl(positionControl.withPosition(setpoint));
    }

    @Override
    public void setMotorVoltage(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public Angle getMotorPosition() {
        return motor.getPosition().getValue();
    }

    /**
     * @return The height of the first stage of the elevator.
     */
    @Override
    public Distance getCarriagePosition() {
        return Meters.of(elevatorSim.getPositionMeters());
    }

    /**
     * Source: 5414 Pearadox
     * Converts the elevators position (meters) to motor rotations based on the elevator spool radius and motor gearing.
     * @param elevatorPosition
     * @return
     */
    private static double getMotorRotations(double elevatorPosition) {
        // angular displacement in radians = linear displacement / radius
        return Units.radiansToRotations(elevatorPosition / ElevatorConstants.ELEVATOR_SPOOL_RADIUS.in(Meters))
                // multiply by gear ratio
                * ElevatorConstants.MOTOR_TO_ELEVATOR_GEARING;
    }
}
