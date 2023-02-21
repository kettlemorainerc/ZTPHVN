package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.DriveModuleIF;
import org.usfirst.frc.team2077.common.subsystem.CANLineSubsystem;
import org.usfirst.frc.team2077.drivetrain.SwerveModule;
import org.usfirst.frc.team2077.util.SmartDashNumber;

public class SwerveMotor implements Subsystem, SwerveModule, DriveModuleIF {

    /* This is, essentially, what the public enum below does
    public static class MotorPosition {
      public static MotorPosition FRONT_RIGHT = new MotorPosition();
      public static MotorPosition FRONT_LEFT = new MotorPosition();
      public static MotorPosition BACK_RIGHT = new MotorPosition();
      public static MotorPosition BACK_LEFT = new MotorPosition();
    }
    */

    public enum MotorPosition {
        // MAX_RPM: 5800
        // MIN PERCENT .0105
        FRONT_RIGHT(WheelPosition.FRONT_RIGHT, 7, 8, 7, 8, 13, 6.67, 2, 5800, 0.7355),
        // Max: 5600
        // MIN PERCENT: 0.02
        FRONT_LEFT(WheelPosition.FRONT_LEFT, 1, 2, 1, 2, 10, 6.67, 2, 5800, 0.1638),
        // Max: 5700
        // MIN PERCENT: 0.02
        BACK_RIGHT(WheelPosition.BACK_RIGHT, 5, 6, 5, 6, 11, 6.67, 2, 5800, 0.7581),
        // Max 5700,
        // MIN_PERCENT: 0.0305
        BACK_LEFT(WheelPosition.BACK_LEFT, 3, 4, 3, 4, 12, 6.67, 2, 5700, 0.3882);

        private final WheelPosition wheelPosition;
        private final int directionId;
        private final int encoderChannelA;
        private final int encoderChannelB;
        private final int magnitudeId;
        private final int hallEffectChannel;
        private final double gearRatio;
        private final double radius;
        private final double maxRPM;
        private final double wheelCircumference;
        private final double encoderOffset;

        private MotorPosition(
              WheelPosition wheelPosition,
              int directionId,
              int magnitudeId,
              int encoderChannelA,
              int encoderChannelB,
              int hallEffectChannel,
              double gearRatio,
              double radius,
              double maxRPM,
              double encoderOffset
        ) {
            this.wheelPosition = wheelPosition;
            this.directionId = directionId;
            this.magnitudeId = magnitudeId;
            this.encoderChannelA = encoderChannelA;
            this.encoderChannelB = encoderChannelB;
            this.hallEffectChannel = hallEffectChannel;
            this.gearRatio = gearRatio;
            this.radius = radius;
            this.maxRPM = maxRPM;
            this.wheelCircumference = (2 * Math.PI * radius);
            this.encoderOffset = encoderOffset;

        }

        public static MotorPosition of(WheelPosition pos) {
            for(MotorPosition drivePos : values()) if(drivePos.wheelPosition == pos) return drivePos;

            throw new IllegalArgumentException("No DrivePosition found for wheel position: " + pos);
        }

    }

    private static final double DEAD_ANGLE = 3;
    private static final double SPEED_REDUCTION = 0.6;
    private static final double MIN_ROTATE_PERCENT = 0.0305;
    private static final double MAX_ROTATE_PERCENT = 1;

    private static final double MAX_DIRECTIONMOTOR_VELOCITY = 1000.0;

    public static boolean rotateFirst = false;

    public final CANSparkMax directionMotor;
    public final Encoder encoder;

    private final CANSparkMax magnitudeMotor;
    private final AbsoluteEncoder absoluteEncoder;

    private double targetAngle = 135, targetMagnitude = 0;

    private double previousDirectionMotorPercent = 0;

    private boolean flipMagnitude;

    private double lastError = 0.0;
    private double errorAccum = 0.0;

    private Timer time = new Timer();
    private double lastTime = 0.0;
    private MotorPosition position;
    private DigitalInput hallEffectSensor;

    private String angleKey;

    public SwerveMotor(
          int directionId,
          int magnitudeId,
          int encoderChannelA,
          int encoderChannelB,
          int hallEffectChannel
    ) {
        angleKey = "angle_key";

        directionMotor = new CANSparkMax(directionId, CANSparkMaxLowLevel.MotorType.kBrushless);

        encoder = new Encoder(encoderChannelA, encoderChannelB);

        magnitudeMotor = new CANSparkMax(magnitudeId, CANSparkMaxLowLevel.MotorType.kBrushless);
        absoluteEncoder = directionMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        this.register();

        hallEffectSensor = new DigitalInput(hallEffectChannel);

//        SmartDashboard.putNumber("Maximum Percent in PID", 0.1);
//        SmartDashboard.putNumber("Slow Division in PID", -17);

    }

    public SwerveMotor(MotorPosition motorPosition) {
        this(
              motorPosition.directionId,
              motorPosition.magnitudeId,
              motorPosition.encoderChannelA,
              motorPosition.encoderChannelB,
              motorPosition.hallEffectChannel
        );
        position = motorPosition;
        angleKey = motorPosition.name() + "_Angle";
        SmartDashboard.putNumber("Zero Offset " + motorPosition + ": ", absoluteEncoder.getZeroOffset());
        SmartDashboard.putNumber("position " + motorPosition + ": ", absoluteEncoder.getPosition());
    }

    public void setMagnitude(double magnitude) {
        setTargetMagnitude(magnitude);
    }

    @Override public void setTargetDegrees(double degrees) {
        setTargetAngle(degrees);
    }

    @Override public void setTargetMagnitude(double magnitude) {
//        if(this.position == MotorPosition.BACK_RIGHT && magnitude > 0) System.out.printf("[target magnitude=%s]%n", magnitude);
        if(magnitude == 0) setTargetAngle(0);
        this.targetMagnitude = magnitude;

    }

    public void setTargetAngle(double angle) {

        targetAngle = angle;
        double currentWheelAngle = getWheelAngle();
        double angleDifference = getAngleDifference(currentWheelAngle, targetAngle);

        flipMagnitude = false;
        if(Math.abs(angleDifference) > 90
           // We're not sure if this actually makes a difference
           // Is supposed prevent turning around if already heading in one direction
//           (
//                 previousDirectionMotorPercent == 0 || Math.signum(previousDirectionMotorPercent) != Math.signum(angleDifference)
//           )
        ) {
            targetAngle -= 180;
            flipMagnitude = true;
        }

        if(targetAngle > 360 || targetAngle < 0) {
            targetAngle -= 360 * Math.signum(targetAngle);
        }

    }

    public void setDirectionPercent(double percent) {
        setDirectionMotor(percent);
    }

    public double getWheelAngle() {
        //double percentTurned = -encoder.get() / ENCODER_COUNTS_PER_REVOLUTION;
        double percentTurned = getWheelRotation();
        double angle = percentTurned * 360;

        angle %= 360.0;
        if(angle < 0) angle += 360.0;

        SmartDashboard.putNumber(angleKey, angle);

        return angle;
    }

    public boolean isAtTarget(){
        return Math.abs(getAngleDifference(getWheelAngle(), targetAngle)) < DEAD_ANGLE;
    }

    public static MotorPosition LOGGED_POSITION = MotorPosition.FRONT_RIGHT;

    boolean clockwiseToTarget;

    @Override public void periodic() {
        updateMagnitude();

        updateRotation();

        SmartDashboard.putNumber("Zero Offset " + position + ": ", absoluteEncoder.getZeroOffset());
        SmartDashboard.putNumber("position " + position + ": ", absoluteEncoder.getPosition());

//        SmartDashboard.putNumber(angleKey, getVelocity());
    }

    private void setMagnitudePercent(double pct) {
        this.magnitudeMotor.set(pct);
    }

    private void updateMagnitude() {

        if(rotateFirst){
//             if(Math.abs(getAngleDifference(targetAngle, getWheelAngle())) > DEAD_ANGLE) {
            setMagnitudePercent(0);
            System.out.println("waiting for angle");
//            }
            return;
        }

        double magnitude = targetMagnitude * SPEED_REDUCTION;

        if(flipMagnitude) magnitude *= -1;


        setMagnitudePercent(magnitude);

    }

    private static final SmartDashNumber SLOW_DIVISION = new SmartDashNumber("Slow Division in PID", -20, true);
    private static final SmartDashNumber MAX_PERCENT = new SmartDashNumber("Maximum Percent in PID", .3, true);

    private void updateRotation() {
//        if (/*this.targetMagnitude == 0 && */!rotateFirst) {
//            setDirectionMotor(0);
//            return;
//        }

        double currentAngle = getWheelAngle();
        double angleDifference = getAngleDifference(currentAngle, targetAngle);

        SmartDashboard.putNumber("angleDiff" + position.wheelPosition, angleDifference);

//        double speed = Math.signum(angleDifference);

        double angleDifferenceDividedByAConstant = angleDifference / SLOW_DIVISION.get().doubleValue();

        double speed = 2.0 / ( 1.0 + Math.pow(Math.E, angleDifferenceDividedByAConstant)) - 1.0;

        speed *= MAX_PERCENT.get().doubleValue();

        speed = Math.max(Math.min(Math.abs(speed), MAX_ROTATE_PERCENT), MIN_ROTATE_PERCENT) * Math.signum(speed);

//        System.out.println(SmartDashboard.getNumber("Maximum Percent in PID", -0.1));



//        speed = 0;

//        if(Math.abs(angleDifference) < 30) {
//            speed = Math.signum(angleDifference) * 0.1;
////            speed = angleDifference / 300;//Math.pow(2, Math.floor(Math.abs(angleDifference)));
////
////            speed = Math.max(Math.abs(speed), 0.02) * Math.signum(speed);
//        }

//        if(Math.abs(angleDifference) < 15) {
//            speed = angleDifference / 30; // Math.pow(2, Math.abs(angleDifference));
//
//
//        }
        if(Math.abs(angleDifference) < DEAD_ANGLE) speed = 0.0;

        setDirectionMotor(speed);
    }

    private void setDirectionMotor(double percent) {
//        previousDirectionMotorPercent = percent;
        SmartDashboard.putNumber("percentPower" + position.wheelPosition, percent);

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

    public boolean getHallValue() {
        return hallEffectSensor.get();
    }

    public double getMaximumSpeed() {
        double rawRPM = position.maxRPM;
        double rawWheelRPM = rawRPM / position.gearRatio;
        double rawWheelRPS = rawWheelRPM / 60;
        double rawVelocity = rawWheelRPS * position.wheelCircumference;

        return rawVelocity;
    }

    @Override public void setVelocity(double velocity) {
        setMagnitude(velocity / getMaximumSpeed());
    }

    @Override public WheelPosition getWheelPosition() {
        return null;
    }

    @Override public double getVelocity() {

        double rawRPM = magnitudeMotor.getEncoder().getVelocity();

        double rawWheelRPM = rawRPM / position.gearRatio;
        double rawWheelRPS = rawWheelRPM / 60;
        double rawDistance = rawWheelRPS * position.wheelCircumference;

        return rawDistance;

    }

    @Override public double getDistance() {
        return 0;
    }

    @Override public void resetDistance() {

    }

    //Checks if motors are all facing towards target, if so removes the rotate before move tag.
    public static void checkDirection(){

        if(!rotateFirst) return;

        boolean atDirection = true;
        RobotHardware hardware = RobotHardware.getInstance();

        for(MotorPosition position : MotorPosition.values()){
            SwerveMotor module = hardware.getWheel(position.wheelPosition);
            atDirection &= module.isAtTarget();
        }

        if(atDirection){
            rotateFirst = false;
        }
    }

    @Override
    public String toString() {
        return "" + getVelocity();
    }

    private double getWheelRotation(){
        return absoluteEncoder.getPosition() - position.encoderOffset + 0.5;
    }
}
