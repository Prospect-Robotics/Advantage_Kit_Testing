package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SimulationVisualizer {

    private static final SimulationVisualizer instance = new SimulationVisualizer();

    // In mech 2d, the arm is rotated 90 degrees off of the actual subsystem.
    private static final Angle ARM_OFFSET = Degrees.of(90);

    // Used for articulation of the 3d mechanism.
    private Distance elevatorHeight = Inches.of(0.0);
    private Angle armAngle = Degrees.of(0);

    private SimulationVisualizer() {}

    public static SimulationVisualizer getInstance() {
        return instance;
    }

    // Where the elevator and arm will be "painted" to
    LoggedMechanism2d elevatorArmMechanismCanvas = new LoggedMechanism2d(56, 84, new Color8Bit("#4d226e"));

    // Sets up the elevator in the mechanism 2d sim
    LoggedMechanismRoot2d elevatorRoot = elevatorArmMechanismCanvas.getRoot("ElevatorRoot", 28, 0);
    LoggedMechanismLigament2d elevatorLigament =
            elevatorRoot.append(new LoggedMechanismLigament2d("Elevator", 56, 90, 5, new Color8Bit("#cf6c30")));

    // Sets up the arm in the mechanism 2d sim
    LoggedMechanismLigament2d armLigament =
            elevatorLigament.append(new LoggedMechanismLigament2d("Arm", 23, 0, 3, new Color8Bit("#30cfaa")));

    public void periodic() {
        // Mechanism 2D for Elevator Arm
        SmartDashboard.putData("ElevatorArm Visualization", elevatorArmMechanismCanvas);
        Logger.recordOutput("ElevatorArm Visualization", elevatorArmMechanismCanvas);

        // Component Simulation for the 3D robot.
        Logger.recordOutput("Component Positions", new Pose3d[] {
            // Elevator base.
            new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),

            // Elevator 1st stage.
            new Pose3d(0, 0, elevatorHeight.in(Meters), new Rotation3d(0, 0, 0)),

            // Elevator 2nd stage (carriage) so it is multiplied by 2.
            new Pose3d(0, 0, elevatorHeight.times(2).in(Meters), new Rotation3d(0, 0, 0)),

            // Arm, attached to the second stage of the elevator.
            new Pose3d(
                    0,
                    -0.125,
                    elevatorHeight.times(2).plus(Meters.of(0.37)).in(Meters),
                    new Rotation3d(0, armAngle.in(Radians), 0))
        });

        // Used for testing new components models added to the robot.
        //        Logger.recordOutput("Zeroed Positions", new Pose3d[] {
        //            new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
        //            new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
        //            new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
        //            new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))
        //        });
    }

    // TODO: Do we need to get the length/angle of the Mech2d, or just set it to the input?

    public void updateElevatorHeight(Distance newHeight) {
        // The elevator height is of the second stage of the elevator, so double the given height.
        elevatorLigament.setLength(newHeight.in(Inches) * 2);
        elevatorHeight = newHeight;
    }

    public void updateArmRotation(Angle newRotation) {
        armLigament.setAngle(newRotation.minus(ARM_OFFSET).in(Degrees));
        armAngle = Degrees.of(armLigament.getAngle());
    }
}
