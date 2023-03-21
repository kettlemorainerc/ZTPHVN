package org.usfirst.frc.team2077.subsystem;

import com.revrobotics.*;
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
        FRONT_RIGHT(WheelPosition.FRONT_RIGHT, 7, 8, 4754, 11182, 0.068,0.0105),
        // Max: 5600
        // Max RPM 2: 5817
        // MIN PERCENT: 0.02
        FRONT_LEFT(WheelPosition.FRONT_LEFT, 1, 2, 5217, 11160, 0.659, 0.02),
        // Max: 5700
        // Max RPM 2: 5862
        // MIN PERCENT: 0.02
        BACK_RIGHT(WheelPosition.BACK_RIGHT, 5, 6, 4817, 11091, 0.100, 0.02),
        // Max 5700,
        // Max rpm 2: 5737
        // MIN_PERCENT: 0.0305
        BACK_LEFT(WheelPosition.BACK_LEFT, 3, 4, 4142, 10931, 0.397, 0.0305);

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

//    private static final double SLOW_DOWN_RANGE = 0;
    private static final double MINIMUM_DIRECTION_PERCENT = 0.075;

    private static final double MAX_DRIVE_PERCENT = 0.5;

    private static final double WHEEL_RADIUS = 2.0;
    private static final double WHEEL_CIRCUMFERENCE = (2 * Math.PI * WHEEL_RADIUS);

    private static final double DRIVE_GEAR_RATIO = 5.08;

    private static final double MAX_DIRECTION_RPM;

    public static boolean rotateFirst = false;

    static{
        double max = Integer.MAX_VALUE;

        for(MotorPosition position : MotorPosition.values()){
            max = Math.min(position.maxRotationRPM, max);
        }

        MAX_DIRECTION_RPM = max;
    }

    private final BetterCanSparkMax directionMotor;
    private final BetterCanSparkMax magnitudeMotor;

    private final AbsoluteEncoder absoluteEncoder;

    private double targetAngle = 0;
    private double targetVelocity = 0;

    private boolean flipMagnitude;

    private PIDController pid = new PIDController(0.024, 0.009, 0);

    private MotorPosition position;
    private String angleKey;

    private static SmartDashNumber p = new SmartDashNumber("P in PID", -135.0, true);
    private static SmartDashNumber i = new SmartDashNumber("l in PID", 1.0, true);

    public SwerveMotor(
            int directionId,
            int magnitudeId
    ) {
        angleKey = "angle_key";

        directionMotor = new BetterCanSparkMax(directionId, CANSparkMaxLowLevel.MotorType.kBrushless);
        magnitudeMotor = new BetterCanSparkMax(magnitudeId, CANSparkMaxLowLevel.MotorType.kBrushless);

        absoluteEncoder = directionMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        p.onChange(this::updatePID);
        i.onChange(this::updatePID);

        pid.setSetpoint(0.0);

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
        pid.setPID(p.get(), i.get(), 0);
    }

    @Override public void setTargetDegrees(double degrees) {
        setTargetAngle(degrees);
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

    private double getWheelRotation(){
        return absoluteEncoder.getPosition() - position.encoderOffset;
    }

    public double getWheelAngle() {
        double percentTurned = getWheelRotation();
        double angle = percentTurned * 360;

        angle %= 360.0;
        if(angle < 0) angle += 360.0;

        return angle;
    }

    @Override public void periodic() {

        updateMagnitude();

        updateRotation();

    }

    @Override
    public void setTargetMagnitude(double magnitude) {}

    private void setMagnitudePercent(double velocity) {

//        if(!SwerveMotor.checkDirection()) return;

        if(flipMagnitude) velocity = -velocity;
        magnitudeMotor.setTargetVelocity(-velocity);
    }

    private void updateMagnitude() {
        setMagnitudePercent(targetVelocity);
    }




    private void updateRotation() {

        if(this.targetVelocity == 0){
            setDirectionMotor(0);
            return;
        }

        double currentAngle = getWheelAngle();
        double angleDifference = getAngleDifference(currentAngle, targetAngle);

       SmartDashboard.putNumber(position.wheelPosition + " Target", angleDifference);

        double c = pid.calculate( angleDifference );

        if(Math.abs(c) < 0.0001) c = 0;

        directionMotor.set(-c);
//
//        //Equation now decreases acceleration as the angle approaches 0, NEEDS TESTING.
//        double a = p.get();
////        double a = SLOW_DOWN_RANGE;
//
//        double b = (Math.abs(angleDifference) - 2 * a);
//
//        double v = angleDifference * angleDifference * b * b;
//
//        v /= Math.pow(a, 4);
//
//        v = Math.min(Math.max(v, i.get()), 1.0) * Math.signum(angleDifference);
////        v = Math.min(Math.max(v, MINIMUM_DIRECTION_PERCENT), 1.0) * Math.signum(angleDifference);
//
//        v *= MAX_DIRECTION_RPM;
//
//        //Old way of doing it.
////        double speed = 2.0 / ( 1.0 + Math.pow(Math.E, angleDifferenceDividedByAConstant)) - 1.0;
////        speed = Math.max(Math.abs(speed), position.minPercent) * Math.signum(speed);
//
//        if(Math.abs(angleDifference) < DEAD_ANGLE) v = 0.0;

//        setDirectionMotor(v);

    }

    private void setDirectionMotor(double percent) {
//        SmartDashboard.putNumber("percentPower" + position.wheelPosition, percent);

//        percent *= (MAX_ROTATION_RPM / position.maxRotationRPM);

        directionMotor.setTargetVelocity(percent);
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

    public boolean isAtTarget(){
        return Math.abs(getAngleDifference(getWheelAngle(), targetAngle)) < DEAD_ANGLE;
    }

    //Checks if motors are all facing towards target, if so removes the "rotate" flag, allowing the magnitude motors to run
    public static boolean checkDirection(){

//        if(!rotateFirst) return;

        boolean atDirection = true;
        RobotHardware hardware = RobotHardware.getInstance();

        for(MotorPosition position : MotorPosition.values()){
            SwerveMotor module = hardware.getWheel(position.wheelPosition);
            atDirection &= module.isAtTarget();
        }

        return atDirection;
//        if(atDirection) rotateFirst = false;

    }

    @Override
    public String toString() {
        return "" + getVelocity();
    }
}
