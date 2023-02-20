package org.usfirst.frc.team2077.command;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;

public class RaiseArm extends RepeatedCommand {

    public enum PistonDirection{
        UP(1),
        DOWN(-1);
        private final int direction;
        PistonDirection(int direction) {
            this.direction = direction;
        }
        public int getDirection(){
            return direction;
        }
    }

    private final TalonSRX screw;
    private final double motorSpeed;
    private PistonDirection direction;

    public RaiseArm(RobotHardware hardware, PistonDirection direction, double motorSpeed){
        screw=hardware.piston;
        this.direction = direction;
        this.motorSpeed = motorSpeed;
    }

    @Override
    public void initialize() {
        screw.set(ControlMode.PercentOutput, motorSpeed * direction.getDirection());
    }


    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted){
        screw.set(ControlMode.PercentOutput, 0);
    }

}
