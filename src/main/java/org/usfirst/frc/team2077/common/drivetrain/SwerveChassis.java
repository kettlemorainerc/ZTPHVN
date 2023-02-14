package org.usfirst.frc.team2077.common.drivetrain;


import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.HardwareRequirements;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.math.AccelerationLimits;
import org.usfirst.frc.team2077.common.sensor.AngleSensor;
import org.usfirst.frc.team2077.math.SwerveMath;
import org.usfirst.frc.team2077.math.SwerveTargetValues;
import org.usfirst.frc.team2077.subsystem.SwerveMotor;

import java.util.Comparator;
import java.util.EnumMap;
import java.util.function.Supplier;

import static org.usfirst.frc.team2077.common.VelocityDirection.*;

public class SwerveChassis extends AbstractChassis<SwerveMotor> {
    private static final double WHEELBASE = 20.375; // inches
    private static final double TRACK_WIDTH = 25.5; // inches
    private static final double WHEEL_RADIUS = 4.0; // inches

//    private static final RotationMotor directionMotor = new RotationMotor();

    private final SwerveMath math;
    private final AngleSensor angleSensor;

    private static EnumMap<WheelPosition, SwerveMotor> buildDriveTrain(HardwareRequirements<SwerveMotor, SwerveChassis> hardware) {
        EnumMap<WheelPosition, SwerveMotor> map = new EnumMap<>(WheelPosition.class);

        for(WheelPosition p : WheelPosition.values()) {
            map.put(p, hardware.getWheel(p));
        }

        return map;
    }

    public SwerveChassis(HardwareRequirements<SwerveMotor, SwerveChassis> hardware, Supplier<Double> getSeconds) {
        super(buildDriveTrain(hardware), WHEELBASE, TRACK_WIDTH, WHEEL_RADIUS, getSeconds);
//        super(buildDriveModule(pos -> hardware.getWheel(pos).motor), WHEELBASE, TRACK_WIDTH, WHEEL_RADIUS, getSeconds);
//        rotationModules = buildDriveModule(hardware::getRotationModule);
        this.angleSensor = hardware.getAngleSensor();
        math = new SwerveMath(WHEELBASE, TRACK_WIDTH);

        this.maximumRotation = 1;
        this.maximumSpeed =  this.driveModules.values()
                .stream()
                .map(DriveModuleIF::getMaximumSpeed)
                .min(Comparator.naturalOrder())
                .orElseThrow();;
        this.minimumRotation = 0;
        this.minimumSpeed =  this.maximumSpeed * 0.1;
    }

    public SwerveChassis(HardwareRequirements<SwerveMotor, SwerveChassis> hardware) {
        this(hardware, Clock::getSeconds);
    }


    @Override
    protected void updatePosition() {
        velocitySet = getVelocityCalculated();
        velocityMeasured = math.velocitiesForTargets(driveModules);

        positionSet.moveRelative(
                velocitySet.get(FORWARD) * timeSinceLastUpdate,
                velocitySet.get(STRAFE) * timeSinceLastUpdate,
                velocitySet.get(ROTATION) * timeSinceLastUpdate
        );
        positionMeasured.moveRelative(
                velocityMeasured.get(FORWARD) * timeSinceLastUpdate,
                velocityMeasured.get(STRAFE) * timeSinceLastUpdate,
                velocityMeasured.get(ROTATION) * timeSinceLastUpdate
        );
    }

    public static WheelPosition LOGGED_POSITION = WheelPosition.FRONT_RIGHT;
    @Override
    protected void updateDriveModules() {
        var wheelTargets = math.targetsForVelocities(targetVelocity);

        wheelTargets.forEach( (key, value) -> {
            SwerveMotor motor = this.driveModules.get(key);

//            if(key == LOGGED_POSITION && (sentinel = (sentinel + 1) % 25) == 0) {
////                System.out.println("[position=" + LOGGED_POSITION + "][targets=" + value + ']');
//            }

            motor.setTargetAngle(value.getAngle());
            motor.setMagnitude(value.getMagnitude());
        });
    }

    @Override
    public void setVelocity(double north, double east, double clockwise, AccelerationLimits accelerationLimits) {
        setVelocity(north, east, accelerationLimits);
        setRotation(clockwise, accelerationLimits);
    }

    @Override
    public void setVelocity(double north, double east, AccelerationLimits accelerationLimits) {
        targetVelocity.put(FORWARD, north);
        this.accelerationLimits.set(FORWARD, accelerationLimits.get(FORWARD));

        targetVelocity.put(STRAFE, east);
        this.accelerationLimits.set(STRAFE, accelerationLimits.get(STRAFE));
    }

    @Override
    public void setRotation(double clockwise, AccelerationLimits accelerationLimits) {
        targetVelocity.put(ROTATION, clockwise);
        this.accelerationLimits.set(ROTATION, accelerationLimits.get(ROTATION));
    }
}