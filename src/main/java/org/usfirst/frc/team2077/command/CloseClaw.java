package org.usfirst.frc.team2077.command;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;

public class CloseClaw extends RepeatedCommand {

    private ClawDirection direction;

    private final TalonSRX claw;
    private final double speed;

    public enum ClawDirection{
        OPEN(1), CLOSE(-1);
        private final int direction;
        ClawDirection(int direction){
            this.direction = direction;
        }
        public int getDirection(){
            return direction;
        }
    }

    public CloseClaw(RobotHardware hardware, ClawDirection direction, double speed ){
        this.direction = direction;
        claw = hardware.claw;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        claw.set(TalonSRXControlMode.PercentOutput,speed * direction.getDirection());
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        claw.set(TalonSRXControlMode.PercentOutput,0);
    }
}