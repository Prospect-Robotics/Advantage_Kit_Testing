package frc.robot.subsystems.groundintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntake extends SubsystemBase {
    private final GroundIntakeIO io;
    private final GroundIntakePivotIOInputsAutoLogged replayedInputs = new GroundIntakePivotIOInputsAutoLogged();

    public GroundIntake(GroundIntakeIO io) {
        this.io = io;
    }
}
