package org.usfirst.frc.team2077.drivetrain;

import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.HardwareRequirements;
import org.usfirst.frc.team2077.common.VelocityDirection;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.AbstractChassis;
import org.usfirst.frc.team2077.common.math.AccelerationLimits;
import org.usfirst.frc.team2077.common.sensor.AngleSensor;
import org.usfirst.frc.team2077.math.SwerveMath;
import org.usfirst.frc.team2077.math.SwerveTargetValues;
import org.usfirst.frc.team2077.subsystem.SwerveMotor;

import java.util.Comparator;
import java.util.EnumMap;
import java.util.Map;
import java.util.function.Supplier;

import static org.usfirst.frc.team2077.common.VelocityDirection.*;

public class SwerveChassis extends AbstractChassis<SwerveMotor> {
    private static final double WHEELBASE = 19.25; // inches
    private static final double TRACK_WIDTH = 22.5; // inches
    private static final double WHEEL_RADIUS = 2.0; // inches

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

    public SwerveChassis(
          HardwareRequirements<SwerveMotor, SwerveChassis> hardware,
          Supplier<Double> getSeconds
    ) {
        super(buildDriveTrain(hardware), WHEELBASE, TRACK_WIDTH, WHEEL_RADIUS, getSeconds);

        this.angleSensor = hardware.getAngleSensor();
        math = new SwerveMath(WHEELBASE, TRACK_WIDTH);

        this.maximumSpeed = this.driveModules.values()
              .stream()
              .map(e -> {
                  Double max = e.getMaximumSpeed();
                  System.out.printf("[%s=%s]", e.getPosition(), max);
                  return max;
              })
              .min(Comparator.naturalOrder())
              .orElseThrow();
        ;
        System.out.println();

        Map<WheelPosition, SwerveTargetValues> wheelTargets = math.targetsForVelocities(Map.of(
              FORWARD,
              0.0,
              STRAFE,
              0.0,
              ROTATION,
              1.0
        ));

        Map<WheelPosition, TestSwerve> map = new EnumMap<>(WheelPosition.class);
        wheelTargets.forEach((k, v) -> {
            map.put(k, new TestSwerve(v.getAngle(), v.getMagnitude() * maximumSpeed));
        });

        maximumRotation = math.velocitiesForTargets(map)
              .get(ROTATION);

        this.minimumSpeed = this.maximumSpeed * 0.1;
    }

    public SwerveChassis(HardwareRequirements<SwerveMotor, SwerveChassis> hardware) {
        this(hardware, Clock::getSeconds);
    }

    @Override protected void updatePosition() {
        velocitySet = getVelocityCalculated();
        velocityMeasured = math.velocitiesForTargets(driveModules);

        positionSet.moveRelative(velocitySet.get(FORWARD) * timeSinceLastUpdate,
              velocitySet.get(STRAFE) * timeSinceLastUpdate,
              velocitySet.get(ROTATION) * timeSinceLastUpdate
        );
        positionMeasured.moveRelative(velocityMeasured.get(FORWARD) * timeSinceLastUpdate,
              velocityMeasured.get(STRAFE) * timeSinceLastUpdate,
              velocityMeasured.get(ROTATION) * timeSinceLastUpdate
        );
    }

    public static WheelPosition LOGGED_POSITION = WheelPosition.FRONT_RIGHT;

    @Override protected void updateDriveModules() {
        Map<WheelPosition, SwerveTargetValues> wheelTargets = math.targetsForVelocities(
              targetVelocity,
              maximumSpeed,
              maximumRotation
        );

        System.out.printf("[target rotate=%s]", targetVelocity.get(ROTATION));
        wheelTargets.forEach((key, value) -> {

            SwerveMotor motor = this.driveModules.get(key);

            motor.setTargetAngle(value.getAngle());

            double targetVelocity = value.getMagnitude() * maximumSpeed;
            if(Math.abs(value.getMagnitude()) > 0.0001) {
                targetVelocity = Math.max(targetVelocity, minimumSpeed);
            }
            motor.setVelocity(targetVelocity);
            System.out.printf("[%s mag=%s]", key, value.getMagnitude());
        });
        System.out.println();
    }

    @Override public void setVelocity(
          double north,
          double east,
          double clockwise,
          AccelerationLimits accelerationLimits
    ) {
        setVelocity(north, east, accelerationLimits);
        setRotation(clockwise, accelerationLimits);
    }

    @Override public void setVelocity(
          double north,
          double east,
          AccelerationLimits accelerationLimits
    ) {
        targetVelocity.put(FORWARD, north);
        this.accelerationLimits.set(FORWARD, accelerationLimits.get(FORWARD));

        targetVelocity.put(STRAFE, east);
        this.accelerationLimits.set(STRAFE, accelerationLimits.get(STRAFE));
    }

    @Override public void setRotation(
          double clockwise,
          AccelerationLimits accelerationLimits
    ) {
        targetVelocity.put(ROTATION, clockwise);
        this.accelerationLimits.set(ROTATION, accelerationLimits.get(ROTATION));
    }

    private record TestSwerve(double wheelAngle, double velocity) implements SwerveModule {
        @Override public double getVelocity() {
            return this.velocity;
        }

        @Override public double getWheelAngle() {
            return this.wheelAngle;
        }

        // region Ignored Things
        @Override public double getMaximumSpeed() {
            return 0;
        }

        @Override public void setVelocity(double velocity) {

        }

        @Override public WheelPosition getWheelPosition() {
            return null;
        }

        @Override public double getDistance() {
            return 0;
        }

        @Override public void resetDistance() {

        }

        @Override public void setTargetDegrees(double degrees) {

        }

        @Override public void setTargetMagnitude(double magnitude) {

        }
        // endregion
    }
}