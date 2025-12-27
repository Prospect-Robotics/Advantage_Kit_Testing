package frc.robot.subsystems.groundintake;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakePivotIO {

    final double MOTOR_TO_GROUND_INTAKE_PIVOT = 66.0 / 14.0; // 14.0 / 66.0;

    @AutoLog
    class GroundIntakePivotIOInputs {
        public double groundIntakePivotAngleDegrees;

        public double pivotMotorRotations;
        public double pivotMotorVelocityRotsPerSecond;
        public double pivotMotorCurrent;
        public double pivotMotorVoltage;
    }

    default void updateState(GroundIntakePivotIOInputs inputs) {}

    default void setPivotMotorSetpoint(Angle setpoint) {}

    default void setPivotMotorVoltage(Voltage voltage) {}

    default Angle getPivotMotorPosition() {
        return Units.Rotations.of(0);
    }

    default Angle getGroundIntakePivotPosition() {
        return Units.Degrees.of(0);
    }
}
