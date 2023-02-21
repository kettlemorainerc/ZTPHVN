package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.subsystem.SwerveMotor;
import org.usfirst.frc.team2077.util.SmartDashNumber;

public class RotateWheel extends RepeatedCommand {
    private static SmartDashNumber percentSpin = new SmartDashNumber("rotate wheel percent", 0.05, true);

    private final SwerveMotor wheel;

    public RotateWheel(SwerveMotor wheel) {
        addRequirements(wheel);
        this.wheel = wheel;
    }

    @Override public void initialize() {
    }

    @Override public void execute() {
//        wheel.setMagnitude(percentSpin.get().doubleValue());
        wheel.setDirectionPercent(percentSpin.get().doubleValue());
    }

    @Override
    public void end(boolean interrupted) {
        wheel.setDirectionPercent(0);
//        wheel.setMagnitude(0);
    }
}
