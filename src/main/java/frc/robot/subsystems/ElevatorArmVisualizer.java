package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorArmVisualizer {

    private static final ElevatorArmVisualizer instance = new ElevatorArmVisualizer();

    // Used for articulation of a 3d mechanism, I think.
    private Distance elevatorHeight = Inches.of(0.0);

    private ElevatorArmVisualizer() {}

    public static ElevatorArmVisualizer getInstance() {
        return instance;
    }

    LoggedMechanism2d mechanismCanvas = new LoggedMechanism2d(56, 84, new Color8Bit("#3b83bd"));
    LoggedMechanismRoot2d elevatorRoot = mechanismCanvas.getRoot("ElevatorRoot", 28, 0);
    LoggedMechanismLigament2d elevatorLigament =
            elevatorRoot.append(new LoggedMechanismLigament2d("Elevator", 56, 90, 5, new Color8Bit("#cf6c30")));

    public void periodic() {
        SmartDashboard.putData("ElevatorArm Visualization", mechanismCanvas);
        Logger.recordOutput("ElevatorArm Visualization", mechanismCanvas);
    }

    public void updateElevatorHeight(Distance newHeight) {
        elevatorHeight = newHeight;
        elevatorLigament.setLength(newHeight.in(Inches));
    }
}
