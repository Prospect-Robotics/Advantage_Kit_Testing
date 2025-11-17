package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxController implements Controller {
    private CommandXboxController xboxController;

    public XboxController(int port) {
        xboxController = new CommandXboxController(port);
    }

    @Override
    public Trigger faceTop() {
        return xboxController.y();
    }
    @Override
    public Trigger faceLeft() {
        return xboxController.x();
    }
    @Override
    public Trigger faceDown() {
        return xboxController.a();
    }
    @Override
    public Trigger faceRight() {
        return xboxController.b();
    }
    @Override
    public Trigger povUp() {
        return xboxController.povUp();
    }
    @Override
    public Trigger povRight() {
        return xboxController.povRight();
    }
    @Override
    public Trigger povDown() {
        return xboxController.povDown();
    }
    @Override
    public Trigger povLeft() {
        return xboxController.povLeft();
    }
    @Override
    public Trigger shoulderLeft() {
        return xboxController.leftBumper();
    }
    @Override
    public Trigger shoulderRight() {
        return xboxController.rightBumper();
    }
    @Override
    public Trigger triggerLeft() {
        return xboxController.leftTrigger();
    }
    @Override
    public Trigger triggerRight() {
    return xboxController.rightTrigger();
    }
    @Override
    public Trigger leftMenuButtons() {
        return xboxController.back();
    }
    @Override
    public Trigger rightMenuButtons() {
        return xboxController.start();
    }
    @Override
    public Trigger joystickLeftPress() {
        return xboxController.leftStick();
    }
    @Override
    public Trigger joystickRightPress() {
        return xboxController.rightStick();
    }
    @Override
    public double leftXAxis() {
        return xboxController.getLeftX();
    }
    @Override
    public double leftYAxis() {
        return xboxController.getLeftY();
    }
    @Override
    public double rightXAxis() {
        return xboxController.getRightX();
    }
    @Override
    public double rightYAxis() {
        return xboxController.getLeftY();
    }

}
