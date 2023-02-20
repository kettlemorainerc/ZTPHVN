package org.usfirst.frc.team2077.command;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.common.command.SelfDefinedCommand;

public class OpenClaw extends SelfDefinedCommand {

    private final TalonSRX claw;
    private final double speed;
    private boolean finished;
    public OpenClaw(RobotHardware hardware, double speed){
        claw = hardware.claw;
        this.speed = speed;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        claw.set(TalonSRXControlMode.PercentOutput, speed);
    }

    @Override
    public void end(boolean interrupted) {
        claw.set(TalonSRXControlMode.PercentOutput,0);
    }

    @Override
    public void bind(JoystickButton button){
        button.onFalse(this);
    }
}
