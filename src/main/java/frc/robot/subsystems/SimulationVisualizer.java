package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SimulationVisualizer {

    private static final SimulationVisualizer instance = new SimulationVisualizer();

    // Used for articulation of the 3d mechanism.
    private Distance elevatorHeight = Inches.of(0.0);

    private SimulationVisualizer() {}

    public static SimulationVisualizer getInstance() {
        return instance;
    }

    // Where the elevator and arm will be "painted" to
    LoggedMechanism2d elevatorArmMechanismCanvas = new LoggedMechanism2d(56, 84, new Color8Bit("#3b83bd"));

    // Sets up the elevator in the mechanism 2d sim
    LoggedMechanismRoot2d elevatorRoot = elevatorArmMechanismCanvas.getRoot("ElevatorRoot", 28, 0);
    LoggedMechanismLigament2d elevatorLigament =
            elevatorRoot.append(new LoggedMechanismLigament2d("Elevator", 56, 90, 5, new Color8Bit("#cf6c30")));

    public void periodic() {
        // Mechanism 2D for Elevator Arm
        SmartDashboard.putData("ElevatorArm Visualization", elevatorArmMechanismCanvas);
        Logger.recordOutput("ElevatorArm Visualization", elevatorArmMechanismCanvas);

        // Component Simulation for the 3D robot.
        Logger.recordOutput("Component Positions", new Pose3d[] {
            new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
            new Pose3d(0, 0, elevatorHeight.times(.5).in(Meters), new Rotation3d(0, 0, 0)),
            new Pose3d(0, 0, elevatorHeight.in(Meters), new Rotation3d(0, 0, 0))
        });
    }

    public void updateElevatorHeight(Distance newHeight) {
        elevatorLigament.setLength(newHeight.in(Inches));
        elevatorHeight = Inches.of(elevatorLigament.getLength());
    }
}
