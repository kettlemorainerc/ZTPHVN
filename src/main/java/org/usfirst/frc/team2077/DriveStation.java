/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.button.*;
import org.usfirst.frc.team2077.common.command.*;
import org.usfirst.frc.team2077.common.control.DriveJoystick;
import org.usfirst.frc.team2077.common.control.DriveStick;
import org.usfirst.frc.team2077.common.control.DriveXboxController;
import org.usfirst.frc.team2077.common.subsystem.InputMap;
import org.usfirst.frc.team2077.subsystem.Claw;
import org.usfirst.frc.team2077.subsystem.ScissorArm;

/**
 * This class is intended to be the center point of defining actions that can be utilized during teleop segments of
 * control. This is where we should define what USB port joysticks should be registered as in `FRC Driver Station`'s usb
 * menu. As well as define what buttons on primary/technical driver's controllers should do what.
 * */
public class DriveStation {
    // Common controller port numbers
    // Joysticks that support rotation
    private static final int DRIVE_JOYSTICK_PORT = 0;
    private static final int DRIVE_XBOX_PORT = 1;
    private static final int FLYSKY_PORT = 2;

    private static final int TECHNICAL_XBOX_PORT = 2;

    // Joysticks that do not support rotation
    private static final int TECHNICAL_JOYSTICK_PORT = 4;
    private static final int NUMPAD_PORT = 5;

    private final DriveXboxController driveStick;
    private final DriveXboxController technicalStick;

    public DriveStation(RobotHardware hardware) {
        /** Set the driver's control method this MUST be a {@link DriveStick} implementation */
//        driveStick = getFlysky();
//        driveStick = getJoystick();
        driveStick = getXbox(DRIVE_XBOX_PORT);

        /** Set the technical control method. This can be any {@link Joystick} implementation */
//        technicalStick = getTechnicalJoystick();
        technicalStick = getXbox(TECHNICAL_XBOX_PORT);

        bind(hardware);
    }

    /**
     * This method binds any subsystem's default command and bind commands to a user's chosen
     * control method.
     */
    public void bind(RobotHardware hardware) {
        // Setup basic robot movement commands
        hardware.getPosition().setDefaultCommand(new CardinalMovement(hardware, driveStick));
        hardware.getHeading().setDefaultCommand(new RotationMovement(hardware, driveStick));

//        useCommand(technicalStick,12, new RaiseArm(hardware, RaiseArm.PistonDirection.UP,0.2));
//        useCommand(technicalStick,16, new RaiseArm(hardware, RaiseArm.PistonDirection.DOWN,0.2));
//        useCommand(technicalStick,11, new CloseClaw(hardware, CloseClaw.ClawDirection.OPEN, 0.05));
//        useCommand(technicalStick,15, new CloseClaw(hardware, CloseClaw.ClawDirection.CLOSE, 0.05));
//        //useCommand(technicalStick, 15, new OpenClaw(hardware, 1));
//        useCommand(technicalStick, 10, new ExtendArm(hardware, ExtendArm.ArmDirection.EXTEND, 0.15));
//        useCommand(technicalStick, 14, new ExtendArm(hardware, ExtendArm.ArmDirection.RETRACT, 0.15));

        bindDriverControl(hardware, driveStick);
        bindTechnicalControl(hardware, technicalStick);
    }

    /** Bind primary driver's button commands here */
    private static void bindDriverControl(RobotHardware hardware, DriveXboxController primary) {
//        primary.getRightTriggerAxis()
    }

    /** Bind technical driver button commands here */
    private void bindTechnicalControl(RobotHardware hardware, DriveXboxController secondary) {

        InputMap.bindAxis(Claw.Input.CLOSE, secondary::getRightTriggerAxis);
//        InputMap.bindAxis("Open", secondary::getLeftTriggerAxis);

        InputMap.bindAxis(ScissorArm.Input.EXTEND, secondary::getLeftY);
//        InputMap.bindAxis("Pivot", secondary::getRightY);

        // Claw open directions: top row on num pad, it is a scale of --, -. +, ++ where +- is direction and the number is speed
//        useCommand(secondary,1, new CloseClaw(hardware, CloseClaw.ClawDirection.OPEN, 0.4));
//        useCommand(secondary,2, new CloseClaw(hardware, CloseClaw.ClawDirection.OPEN, 0.2));
//        useCommand(secondary,3, new CloseClaw(hardware, CloseClaw.ClawDirection.CLOSE, 0.2));
//        useCommand(secondary,4, new CloseClaw(hardware, CloseClaw.ClawDirection.CLOSE, 0.4));
//
//        // Raising arm: rotates the arm around the pivot
//        useCommand(secondary,5, new RaiseArm(hardware, RaiseArm.PistonDirection.UP,0.5));
//        useCommand(secondary,9, new RaiseArm(hardware, RaiseArm.PistonDirection.DOWN,0.5));
//
//        // Extending arm: extending the scissor arm
//        useCommand(secondary, 8, new ExtendArm(hardware, ExtendArm.ArmDirection.EXTEND, "extend speed", 0.5));
//        useCommand(secondary, 12, new ExtendArm(hardware, ExtendArm.ArmDirection.RETRACT, "extend speed", 0.5));

//        useCommand(secondary, 1, new RotateWheel(hardware.getWheel(WheelPosition.FRONT_LEFT)));
//        useCommand(secondary, 5, new RotateWheel(hardware.getWheel(WheelPosition.BACK_LEFT)));
//        useCommand(secondary, 2, new RotateWheel(hardware.getWheel(WheelPosition.FRONT_RIGHT)));
//        useCommand(secondary, 6, new RotateWheel(hardware.getWheel(WheelPosition.BACK_RIGHT)));
    }

    /** Normal (silver/brighter) joystick that supports rotation */
    private static DriveJoystick getJoystick() {
        return new DriveJoystick(DRIVE_JOYSTICK_PORT).setDriveSensitivity(.15, 5)
                                                     .setRotationSensitivity(.1, 1);
    }

    /** Flysky Drone Controller */
    private static DriveJoystick getFlysky() {
        return new DriveJoystick(FLYSKY_PORT, 4).setDriveSensitivity(.3, 1)
                                                .setRotationSensitivity(.05, 2.5);
    }

    private static DriveXboxController getXbox(int port){
        return new DriveXboxController(port).setDriveSensitivity(.3,1)
                                                       .setRotationSensitivity(.3,1.5);
    }

    /** Currently the darker joystick that doesn't support rotation */
    private static Joystick getTechnicalJoystick() {
        return new Joystick(TECHNICAL_JOYSTICK_PORT);
    }

    private static Joystick getNumpad() {
        return new Joystick(NUMPAD_PORT);
    }

    /** bind command to the given joystick button */
    public static void useCommand(Joystick joystick, int button, BindableCommand command) {
        command.bind(new JoystickButton(joystick, button));
    }
}
