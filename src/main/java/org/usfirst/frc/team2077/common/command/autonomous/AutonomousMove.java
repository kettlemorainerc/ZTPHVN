package org.usfirst.frc.team2077.common.command.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.VelocityDirection;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.subsystem.SwerveMotor;

import java.util.Map;

public class AutonomousMove extends CommandBase {

    private static final double DEACCELERATION = 50; // in/s/s

    private double north = 0;
    private double east = 0;

    private double speed = 0;
    private double direction = 0;

    private double northMovement = 0;
    private double eastMovement = 0;

    private double lastTime = 0;

    private RobotHardware hardware;

    private final SwerveChassis chassis;

    public AutonomousMove(RobotHardware hardware, double north, double east) {
        this(hardware);

        this.north = north;
        this.east = east;
    }

    private AutonomousMove(RobotHardware hardware) {

        this.hardware = hardware;

        chassis = hardware.getChassis();

        SmartDashboard.putNumber("MoveData", 0);

        SwerveMotor.rotateFirst = true;

    }

    @Override
    public void initialize() {

        lastTime = Clock.getSeconds();

        speed = (double) chassis.getMaximumVelocity().get(VelocityDirection.FORWARD);

        direction = Math.atan2(north, east);

    }

    private double getDeltaTime(){
        double currentTime = Clock.getSeconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        return dt;
    }

    @Override
    public void execute() {

//        System.out.println(chassis.driveModules);
        System.out.println("Execute For Autonomous is being called");

        Map<VelocityDirection, Double> currentVelocity = chassis.getVelocityMeasured();

        double dt = getDeltaTime();

        double northVelocity = currentVelocity.get(VelocityDirection.FORWARD);
        double eastVelocity = currentVelocity.get(VelocityDirection.STRAFE);

        northMovement += northVelocity * dt;
        eastMovement += eastVelocity * dt;

//        System.out.printf("[dt=%s][drive module=%s]", dt, Map.of(
//                WheelPosition.FRONT_RIGHT, chassis.driveModules.get(WheelPosition.FRONT_RIGHT).getVelocity() * dt,
//                WheelPosition.FRONT_LEFT, chassis.driveModules.get(WheelPosition.FRONT_LEFT).getVelocity() * dt,
//                WheelPosition.BACK_RIGHT, chassis.driveModules.get(WheelPosition.BACK_RIGHT).getVelocity() * dt,
//                WheelPosition.BACK_LEFT, chassis.driveModules.get(WheelPosition.BACK_LEFT).getVelocity() * dt
//        ));
//
//        System.out.println("nroth movmement: " + northVelocity * dt);

        SwerveMotor.checkDirection();
//
        double distance = pythag(
            north - northMovement,
            east - eastMovement
        );

        double currentSpeed = pythag(
            northVelocity,
            eastVelocity
        );

        double stoppingDistance = Math.pow(currentSpeed, 2) / (2 * DEACCELERATION);

        if(stoppingDistance > distance) {

            double deccel = Math.pow(currentSpeed, 2) / (2 * distance);

            speed = Math.max(speed - deccel, 45);

        }

        SmartDashboard.putNumber("MoveSpeed", speed);


        double targetForward = speed * Math.sin(direction);
        double targetStrafe = speed * Math.cos(direction);
//        System.out.printf("[forward set=%s][strafe set=%s]%n", targetForward, targetStrafe);

        SmartDashboard.putNumber("targetForward", targetForward);
        SmartDashboard.putNumber("targetStrafe", targetStrafe);

        chassis.setVelocity(
            targetForward,
            targetStrafe,
            0
        );

    }


    @Override
    public boolean isFinished() {

        boolean finished = true;

        if(north != 0){
            finished = Math.signum(north - northMovement) != Math.signum(north);
        }

        if(finished && east != 0){
            finished = Math.signum(east - eastMovement) != Math.signum(east);
        }

        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.halt();
        SwerveMotor.rotateFirst = false;

        SmartDashboard.putNumber("MoveData", 1);
    }

    private double pythag(double a, double b){
        return Math.sqrt(a * a + b * b);
    }

}
