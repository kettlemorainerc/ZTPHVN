/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.common.command;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.common.*;
import org.usfirst.frc.team2077.common.control.DriveXboxController;
import org.usfirst.frc.team2077.common.drivetrain.*;
import org.usfirst.frc.team2077.util.SmartDashNumber;

public class CardinalMovement extends CommandBase {
    public static final SmartDashNumber ACCELERATION_G_LIMIT = new SmartDashNumber("Cardinal Acceleration Gs", 1D, true);
    public static final SmartDashNumber DECELERATION_G_LIMIT = new SmartDashNumber("Carindal Deceleration Gs", 9999D, true);

    protected DriveXboxController stick;
    protected DriveChassisIF chassis;

    public CardinalMovement(HardwareRequirements<?, ?> hardware, DriveXboxController stick) {
        addRequirements(hardware.getPosition());

        this.stick = stick;
        this.chassis = hardware.getChassis();
        updateAccelerationLimits();
        ACCELERATION_G_LIMIT.onChange(this::updateAccelerationLimits);
        DECELERATION_G_LIMIT.onChange(this::updateAccelerationLimits);
    }

    private void updateAccelerationLimits() {
        this.chassis.setGLimits(ACCELERATION_G_LIMIT.get(), DECELERATION_G_LIMIT.get());
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
