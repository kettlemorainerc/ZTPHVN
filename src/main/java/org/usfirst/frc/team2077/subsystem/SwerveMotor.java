package org.usfirst.frc.team2077.subsystem;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.DriveModuleIF;
import org.usfirst.frc.team2077.drivetrain.SwerveModule;
import org.usfirst.frc.team2077.util.SmartDashNumber;

public class SwerveMotor implements Subsystem, SwerveModule, DriveModuleIF {

    public enum MotorPosition {
        // MAX_RPM: 5800
        // max rpm 2: 5817
        // MIN PERCENT .0105
        FRONT_RIGHT(WheelPosition.FRONT_RIGHT, 7, 8, 4754, 11182, 0.9030,0.0105),
        // Max: 5600
        // Max RPM 2: 5817
        // MIN PERCENT: 0.02
        FRONT_LEFT(WheelPosition.FRONT_LEFT, 1, 2, 5217, 11160, 0.1638, 0.02),
        // Max: 5700
        // Max RPM 2: 5862
        // MIN PERCENT: 0.02
        BACK_RIGHT(WheelPosition.BACK_RIGHT, 5, 6, 4817, 11091, 0.2581, 0.02),
        // Max 5700,
        // Max rpm 2: 5737
        // MIN_PERCENT: 0.0305
        BACK_LEFT(WheelPosition.BACK_LEFT, 3, 4, 4142, 10931, 0.8882, 0.0305);

        private final WheelPosition wheelPosition;
        private final int directionId;
        private final int magnitudeId;
        private final double maxRPM;
        private final double encoderOffset;
        private final double maxRotationRPM;
        private final double minPercent;

        MotorPosition(
                WheelPosition wheelPosition,
                int directionId,
                int magnitudeId,
                double maxRPM,
                double maxRotationRPM,
                double encoderOffset,
                double minPercent
        ) {
            this.wheelPosition = wheelPosition;
            this.directionId = directionId;
            this.magnitudeId = magnitudeId;
            this.maxRPM = maxRPM;
            this.maxRotationRPM = maxRotationRPM;
            this.encoderOffset = encoderOffset;
            this.minPercent = minPercent;
        }

        public static MotorPosition of(WheelPosition pos) {
            for(MotorPosition drivePos : values()) if(drivePos.wheelPosition == pos) return drivePos;

            throw new IllegalArgumentException("No DrivePosition found for wheel position: " + pos);
        }

    }

    private static final double DEAD_ANGLE = 1;

    private static final double MAX_DRIVE_PERCENT = 0.2;

    private static final double MIN_ROTATE_PERCENT = 0.0305;
//    private static final double MAX_ROTATE_PERCENT = 1.0;

    private static final double WHEEL_RADIUS = 2.0;
    private static final double WHEEL_CIRCUMFERENCE = (2 * Math.PI * WHEEL_RADIUS);

    private static final double ANGLE_DIFF_DIVISION = -135;
    private static final double MAX_DIRECTION_PERCENT = 0.8;

    private static final double DRIVE_GEAR_RATIO = 5.08;

    private static final double MAX_ROTATION_RPM;

    public static boolean rotateFirst = false;

    static{
        double max = Integer.MAX_VALUE;

        for(MotorPosition position : MotorPosition.values()){
            max = Math.min(position.maxRotationRPM, max);
        }

        MAX_ROTATION_RPM = max;
    }

    private final CANSparkMax directionMotor;
    private final CANSparkMax magnitudeMotor;

    private final AbsoluteEncoder absoluteEncoder;

    private double targetAngle = 0;
    private double targetMagnitude = 0;

    private double targetVelocity = 0;

    private boolean flipMagnitude;

    private PIDController pid;

    private MotorPosition position;
    private String angleKey;

    private static SmartDashNumber p = new SmartDashNumber("P in PID", 0.000008, true);
    private static SmartDashNumber i = new SmartDashNumber("l in PID", 0.000001, true);

    public SwerveMotor(
            int directionId,
            int magnitudeId
    ) {
        angleKey = "angle_key";

        directionMotor = new CANSparkMax(directionId, CANSparkMaxLowLevel.MotorType.kBrushless);
        magnitudeMotor = new CANSparkMax(magnitudeId, CANSparkMaxLowLevel.MotorType.kBrushless);

        absoluteEncoder = directionMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
//K: 0.000001
        pid = new PIDController(0.000008, 0.000001, 0);//p.get(), i.get(), 0);

        p.onChange(this::updatePID);
        i.onChange(this::updatePID);

        this.register();

    }

    public SwerveMotor(MotorPosition motorPosition) {
        this(
                motorPosition.directionId,
                motorPosition.magnitudeId
        );
        position = motorPosition;

        angleKey = motorPosition.name() + "_Angle";
    }

    private void updatePID(){
//        pid.setPID(p.get(), i.get(), 0);
    }

    public void setMagnitude(double magnitude) {
        setTargetMagnitude(magnitude);
    }

    @Override public void setTargetDegrees(double degrees) {
        setTargetAngle(degrees);
    }

    @Override public void setTargetMagnitude(double magnitude) {
        if(flipMagnitude) magnitude = -magnitude;
        this.targetMagnitude = magnitude;
    }

    public void setTargetAngle(double angle) {

        targetAngle = angle;

        double currentWheelAngle = getWheelAngle();
        double angleDifference = getAngleDifference(currentWheelAngle, targetAngle);

        flipMagnitude = false;
        if(Math.abs(angleDifference) > 90) {
            targetAngle -= 180;
            flipMagnitude = true;
        }

        targetAngle %= 360;
        if(targetAngle < 0) targetAngle += 360;

    }

    public void setDirectionPercent(double percent) {
        setDirectionMotor(percent);
    }

    public double getWheelAngle() {
        double percentTurned = getWheelRotation();
        double angle = percentTurned * 360;

        angle %= 360.0;
        if(angle < 0) angle += 360.0;

//        SmartDashboard.putNumber(angleKey, angle);

        return angle;
    }

    public boolean isAtTarget(){
        return Math.abs(getAngleDifference(getWheelAngle(), targetAngle)) < DEAD_ANGLE;
    }

    @Override public void periodic() {
        updateMagnitude();

        updateRotation();

        System.out.printf("e45drgyHUGIawsedrghjkBY*PMOIDVR^D;R*jokl[p = %s][i = %s]", pid.getP(), pid.getI());

        if(targetVelocity != 0) {
            SmartDashboard.putNumber("targetVelocity: ", targetVelocity);
            SmartDashboard.putNumber(position.wheelPosition + " velocity" ,
//                    magnitudeMotor.get()
                magnitudeMotor.getEncoder().getVelocity()
            );
        }

//        getVelocity();

//        SmartDashboard.putNumber("Zero Offset " + position + ": ", absoluteEncoder.getZeroOffset());
//        SmartDashboard.putNumber("position " + position + ": ", absoluteEncoder.getPosition());

//        SmartDashboard.putNumber(angleKey, getVelocity());
    }

    private void setMagnitudePercent(double velocity) {
        double deltaPercent = pid.calculate( magnitudeMotor.getEncoder().getVelocity() );

        double newPercent = magnitudeMotor.get() + deltaPercent;

        if(Math.abs(newPercent) < 0.005 ) newPercent = 0;

        System.out.printf(
                "[actual rpm=%s][set rpm=%s][pid target %%=%s]%n",
                magnitudeMotor.getEncoder().getVelocity(),
                velocity,
                newPercent
        );

        if(newPercent != 0) {
            SmartDashboard.putNumber(position.wheelPosition + " Target", newPercent);
        }
        magnitudeMotor.set(newPercent);
    }

    private void updateMagnitude() {

        if(rotateFirst){
            setMagnitudePercent(0);
            return;
        }

        pid.setSetpoint(targetVelocity);
        magnitudeMotor.setInverted(!flipMagnitude);

        setMagnitudePercent(targetVelocity);

    }

//    private static final SmartDashNumber SLOW_DIVISION = new SmartDashNumber("Slow Division in PID", -20, true);
//    private static final SmartDashNumber MAX_PERCENT = new SmartDashNumber("Maximum Percent in PID", .3, true);

    private void updateRotation() {
//        if (/*this.targetMagnitude == 0 && */!rotateFirst) {
//            setDirectionMotor(0);
//            return;
//        }

//        if(this.targetMagnitude == 0){
//            setDirectionMotor(0);
//            return;
//        }

        double currentAngle = getWheelAngle();
        double angleDifference = getAngleDifference(currentAngle, targetAngle);

//        SmartDashboard.putNumber("angleDiff" + position.wheelPosition, angleDifference);

        double angleDifferenceDividedByAConstant = angleDifference / ANGLE_DIFF_DIVISION;

        double speed = 2.0 / ( 1.0 + Math.pow(Math.E, angleDifferenceDividedByAConstant)) - 1.0;

        speed *= MAX_DIRECTION_PERCENT;

        speed = Math.max(Math.abs(speed), position.minPercent) * Math.signum(speed);

        if(Math.abs(angleDifference) < DEAD_ANGLE) speed = 0.0;

        setDirectionMotor(speed);
    }

    private void setDirectionMotor(double percent) {
//        SmartDashboard.putNumber("percentPower" + position.wheelPosition, percent);

        percent *= (MAX_ROTATION_RPM / position.maxRotationRPM);

        directionMotor.set(percent);
    }

    public MotorPosition getPosition() {
        return position;
    }

    private double getAngleDifference(double from, double to) {
        double angleDifference = from - to;

        if(Math.abs(angleDifference) > 180) {
            angleDifference -= 360 * Math.signum(angleDifference);
        }

        return angleDifference;
    }

    public double getMaximumSpeed() {
        double rawRPM = position.maxRPM;
        double rawWheelRPM = rawRPM / DRIVE_GEAR_RATIO;
        double rawWheelRPS = rawWheelRPM / 60;
        double rawVelocity = rawWheelRPS * WHEEL_CIRCUMFERENCE;

        return rawVelocity;
    }

    @Override public void setVelocity(double velocity) {
         targetVelocity = MAX_DRIVE_PERCENT * velocity * DRIVE_GEAR_RATIO * 60 / WHEEL_CIRCUMFERENCE;
//       setMagnitude(velocity / getMaximumSpeed());
    }

    @Override public WheelPosition getWheelPosition() {
        return null;
    }

    private double maxObservedRPM = 0;

    @Override public double getVelocity() {

        double rawRPM = magnitudeMotor.getEncoder().getVelocity();

        if(rawRPM > maxObservedRPM){
            maxObservedRPM = rawRPM;
            SmartDashboard.putNumber("ObservedRPM" + position.wheelPosition, rawRPM);
        }

        double rawWheelRPM = rawRPM / DRIVE_GEAR_RATIO;
        double rawWheelRPS = rawWheelRPM / 60;
        double rawDistance = rawWheelRPS * WHEEL_CIRCUMFERENCE;

        return rawDistance;

    }

    @Override public double getDistance() {
        return 0;
    }

    @Override public void resetDistance() {}

    //Checks if motors are all facing towards target, if so removes the "rotate" flag, allowing the magnitude motors to run
    public static void checkDirection(){

        if(!rotateFirst) return;

        boolean atDirection = true;
        RobotHardware hardware = RobotHardware.getInstance();

        for(MotorPosition position : MotorPosition.values()){
            SwerveMotor module = hardware.getWheel(position.wheelPosition);
            atDirection &= module.isAtTarget();
        }

        if(atDirection) rotateFirst = false;

    }

    @Override
    public String toString() {
        return "" + getVelocity();
    }

    private double getWheelRotation(){
        return absoluteEncoder.getPosition() - position.encoderOffset;
    }
}
