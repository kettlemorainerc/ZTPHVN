package org.usfirst.frc.team2077.common.command.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.VelocityDirection;
import org.usfirst.frc.team2077.common.drivetrain.AbstractChassis;
import org.usfirst.frc.team2077.common.math.Position;

import java.util.Map;

public class AutonomousPIDedMove extends CommandBase {

    private static SmartDashNumber ppid = new SmartDashNumber("P in Auto", 0.01, true);
    private static SmartDashNumber ipid = new SmartDashNumber("I in Auto", 0.001, true);

//    24.75 TODO: Tell Cayden this number

    private Position position;
    private AbstractChassis<?> chassis;

    private PIDController pid = new PIDController(ppid.get(), ipid.get(), 0.0); // TODO: Find these values

    private double remainingDistance;

    private final double forwardMultiplier;
    private final double strafeMultiplier;
    private final double maxSpeed;

    public AutonomousPIDedMove(RobotHardware hardware, double direction, double distance){
        chassis = hardware.getChassis();
        position = chassis.getPosition();
        maxSpeed = chassis.getMaximumVelocity().get(VelocityDirection.FORWARD);

        pid.setSetpoint(0.0);

        remainingDistance = distance;

        direction = Math.toRadians(-direction + 90);
        forwardMultiplier = Math.sin(direction);
        strafeMultiplier = Math.cos(direction);
    }

    double lastTime;

    private double getDeltaTime(){
        double t = Clock.getSeconds();
        double dt = t - lastTime;
        lastTime = t;
        return dt;
    }

    @Override public void initialize() {
        lastTime = Clock.getSeconds();

    }

    @Override public void execute() {
        Map<VelocityDirection, Double> currentVelocity = chassis.getVelocityMeasured();

        double dt = getDeltaTime();

        double forwardVelocity = currentVelocity.get(VelocityDirection.FORWARD);
        double strafeVelocity = currentVelocity.get(VelocityDirection.STRAFE);

        remainingDistance -= forwardVelocity * dt * forwardMultiplier;
        remainingDistance -= strafeVelocity * dt * strafeMultiplier;
        double targetForward = Math.abs(pid.calculate( remainingDistance ));

        double percent = Math.min(1, Math.max(0, targetForward / maxSpeed));
        double forward = percent * forwardMultiplier;
        double strafe = percent * strafeMultiplier;

        chassis.setVelocityPercent(
                forward,
                strafe,
                0
        );
    }

    @Override public void end(boolean interrupted) {
        chassis.halt();
    }

    @Override public boolean isFinished(){
        return(remainingDistance <= 0);
    }

}