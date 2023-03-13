package org.usfirst.frc.team2077.common.command.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.VelocityDirection;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.subsystem.SwerveMotor;

import java.util.Map;

public class AutonomousRotate extends CommandBase {

    private static final double DEACCELERATION = 175.0; // degrees/s/s
    private static final double TICK = 1.0 / 50.0;

    private double rotation = 0;
    private double rotationalMovement = 0;

    private double angularSpeed = 0;
    private boolean clockwise = true;

    private double lastTime = 0;

    private RobotHardware hardware;

    private final SwerveChassis chassis;

    public AutonomousRotate(RobotHardware hardware, double angle) {
        this(hardware);

        rotation = angle;
    }

    private AutonomousRotate(RobotHardware hardware) {

        this.hardware = hardware;

        chassis = hardware.getChassis();

        SmartDashboard.putNumber("MoveSpeed", 0);
        SmartDashboard.putNumber("MoveData", 0);

        SwerveMotor.rotateFirst = true;

    }

    @Override
    public void initialize() {

        lastTime = Clock.getSeconds();

        angularSpeed = (double) chassis.getMaximumVelocity().get(VelocityDirection.ROTATION);

//        angularSpeed /= 50.0; //Rough estimate of tick rate for the
//        angularSpeed = Math.min(angularSpeed, Math.abs(rotation));
    }

    private double getDeltaTime(){
        double currentTime = Clock.getSeconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        return dt;
    }

    @Override
    public void execute() {
        // region degenerate

        double angleDiff = (rotationalMovement - rotation);

        SmartDashboard.putNumber("MoveData", angleDiff);

        if(Math.abs(angleDiff) > 180) angleDiff -= Math.signum(angleDiff) * 360;

        boolean hasToMoveClockwise = angleDiff <= 0;

        boolean finished = hasToMoveClockwise != clockwise;

//        System.out.println(angularSpeed + ": " + (finished? "finished" : "waiting"));

        if(finished){

            chassis.halt();
            return;

        }

        //endregion degenerate


//        System.out.println(chassis.driveModules);
        Map<VelocityDirection, Double> currentVelocity = chassis.getVelocityMeasured();
        Map<VelocityDirection, Double> targetVelocity = chassis.getVelocitySet();


        double dt = getDeltaTime();

        double rotationalVelocity = currentVelocity.get(VelocityDirection.ROTATION);//double check this (ask david)

        rotationalMovement += rotationalVelocity;

        SwerveMotor.checkDirection();
//
        angleDiff = (rotation - rotationalMovement);
//        if(Math.abs(angleDiff) > 180) angleDiff -= Math.signum(angleDiff) * 360;

//        SmartDashboard.putNumber("MoveData", angleDiff);

        double deccel = Math.pow(angularSpeed, 2) / (2 * Math.abs(angleDiff));

        angularSpeed = Math.max(angularSpeed - deccel, 10);

//        System.out.println(angularSpeed);

        double targetRotate = angularSpeed;

//        printStuff("measured velocity", rotationalVelocity, "set as", targetRotate, "target", targetVelocity.get(VelocityDirection.ROTATION));

//        System.out.printf("[forward set=%s][strafe set=%s]%n", targetForward, targetStrafe);

        chassis.setVelocity(
                0, 0,
                targetRotate
        );

    }


    @Override
    public boolean isFinished() {

        double angleDiff = (rotationalMovement - rotation);

        if(Math.abs(angleDiff) > 180) angleDiff -= Math.signum(angleDiff) * 360;

        boolean hasToMoveClockwise = angleDiff <= 0;

        boolean finished = hasToMoveClockwise != clockwise;

//        finished = Math.signum(north - northMovement) != Math.signum(north);

//        return finished;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.halt();
        SwerveMotor.rotateFirst = false;
    }

    private double pythag(double a, double b){
        return Math.sqrt(a * a + b * b);
    }

    private static void printStuff(Object... stuff) {
        for(int i = 0 ; i < stuff.length; i += 2) {
            System.out.printf("[%s=%s]", stuff[i], stuff[i + 1]);
        }
        System.out.println();
    }
}
