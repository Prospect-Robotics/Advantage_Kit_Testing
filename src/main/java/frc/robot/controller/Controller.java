package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controller {
    /** Top button on the face pad (not POV/DPAD). */
    public Trigger faceTop(); 

    public Trigger faceRight();

    public Trigger faceDown();

    public Trigger faceLeft();
    /** DPAD controles */
    public Trigger povUp();

    public Trigger povRight();

    public Trigger povDown();

    public Trigger povLeft();
    /** shoulder top of controler */
    public Trigger shoulderLeft();

    public Trigger shoulderRight();
    /** Trigger under sholder */
    public Trigger triggerLeft();

    public Trigger triggerRight();
    /** setting buttons */
    public Trigger leftMenuButtons();

    public Trigger rightMenuButtons();

    /** joystick button */
    public Trigger joystickLeftPress(); 

    public Trigger joystickRightPress();
    /** joystick */
    public double leftXAxis();

    public double leftYAxis();

    public double rightXAxis();

    public double rightYAxis(); 

} 
