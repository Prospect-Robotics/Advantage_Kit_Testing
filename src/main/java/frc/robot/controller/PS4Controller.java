package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS4Controller implements Controller {
    private CommandPS4Controller ps4Controller;
    @Override
    public Trigger faceTop() {
        return ps4Controller.triangle();
    }
    @Override
    public Trigger faceLeft() {
        return ps4Controller.square();
    }
    @Override
    public Trigger faceDown() {
        return ps4Controller.cross();
    }
    @Override
    public Trigger faceRight() {
        return ps4Controller.circle();
    }
    @Override
    public Trigger povUp() {
        return ps4Controller.povUp();
    }
    @Override
    public Trigger povRight() {
        return ps4Controller.povRight();
    }
    @Override
    public Trigger povDown() {
        return ps4Controller.povDown();
    }
    @Override
    public Trigger povLeft() {
        return ps4Controller.povLeft();
    }
    @Override
    public Trigger shoulderLeft() {
        return ps4Controller.L1();
    }
    @Override
    public Trigger shoulderRight() {
        return ps4Controller.R1();
    }
    @Override
    public Trigger triggerLeft() {
        return ps4Controller.L2();
    }
    @Override
    public Trigger triggerRight() {
    return ps4Controller.R2();
    }
    @Override
    public Trigger leftMenuButtons() {
        return ps4Controller.share();
    }
    @Override
    public Trigger rightMenuButtons() {
        return ps4Controller.options();
    }
    @Override
    public Trigger joystickLeftPress() {
        return ps4Controller.L3();
    }
    @Override
    public Trigger joystickRightPress() {
        return ps4Controller.R3();
    }
    @Override
    public double leftXAxis() {
        return ps4Controller.getLeftX();
    }
    @Override
    public double leftYAxis() {
        return ps4Controller.getLeftY();
    }
    @Override
    public double rightXAxis() {
        return ps4Controller.getRightX();
    }
    @Override
    public double rightYAxis() {
        return ps4Controller.getLeftY();
    }
}
