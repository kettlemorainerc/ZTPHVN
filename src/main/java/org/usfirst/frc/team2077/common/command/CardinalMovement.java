/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.common.command;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.common.*;
import org.usfirst.frc.team2077.common.control.DriveStick;
import org.usfirst.frc.team2077.common.drivetrain.*;

public class CardinalMovement extends CommandBase {
    public static final double ACCELERATION_G_LIMIT = .4;
    public static final double DECELERATION_G_LIMIT = ACCELERATION_G_LIMIT; //1e10 //.35 is the value used for the 03-05-21 version

    protected DriveStick stick;
    protected DriveChassisIF chassis;

    public CardinalMovement(HardwareRequirements<?, ?> hardware, DriveStick stick) {
        addRequirements(hardware.getPosition());

        this.stick = stick;
        this.chassis = hardware.getChassis();
        this.chassis.setGLimits(ACCELERATION_G_LIMIT, DECELERATION_G_LIMIT);
    }

    @Override public void execute() {
        double north = -stick.getNorth();
        double east = stick.getEast();



        // Tank drive
//		north = Math.abs(north) >= Math.abs(east) ? north : 0;
//		east = Math.abs(east) > Math.abs(north) ? east : 0;

        if(DriverStation.isTeleop()) chassis.setVelocityPercent(north, east);
    }

    @Override public void end(boolean interrupted) {
    }

    @Override public boolean isFinished() {
        return false;
    }
}
