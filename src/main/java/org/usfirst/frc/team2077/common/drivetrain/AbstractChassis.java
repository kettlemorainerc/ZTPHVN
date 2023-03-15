/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.common.drivetrain;

import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.VelocityDirection;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.math.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc.team2077.common.math.AccelerationLimits.*;

import java.util.*;
import java.util.function.*;

import static org.usfirst.frc.team2077.common.VelocityDirection.*;

public abstract class AbstractChassis<DriveModule> extends SubsystemBase implements DriveChassisIF {

    private static <T> EnumMap<VelocityDirection, T> defaultedDirectionMap(T defaultValue) {
        EnumMap<VelocityDirection, T> newMap = new EnumMap<>(VelocityDirection.class);
        for (VelocityDirection d : VelocityDirection.values()) newMap.put(d, defaultValue);
        return newMap;
    }

    /**
     * DriveModules (wheel container classes, typically) by their position.
     * SparkNewDriveModules for Mecanums, SwerveMotor for Swerves, etc.
     * */
    public final Map<WheelPosition, DriveModule> driveModules;
    protected final double wheelbase;
    protected final double trackWidth;
    protected final double wheelRadius;
    protected final Supplier<Double> getSeconds;

    /** The maximum allowed N/E velocity */
    protected double maximumSpeed;
    /** The maximum allowed Rotation velocity */
    protected double maximumRotation;
    /** The minimum required velocity to actually have motion for our N/E motors */
    protected double minimumSpeed;
    /** The minimum required wheel velocity to actually turn our wheels for rotation */
    protected double minimumRotation;

    // Ideally accel/decel values are set just below wheelspin or skidding to a stop.
    // Optimal values are highly dependent on wheel/surface traction and somewhat on
    // weight distribution.
    // For safety err on the low side for acceleration, high for deceleration.
    protected AccelerationLimits accelerationLimits = new AccelerationLimits(false, .5, .5, this);

    /** The time previously returned by {@link #getSeconds} */
    protected double lastUpdateTime = 0;
    /** The difference between lastUpdateTime and the previous update time */
    protected double timeSinceLastUpdate = 0;

    /** The position as measured by {@link #velocitySet} */
    protected final Position positionSet = new Position(); // Continuously updated by integrating velocity setpoints (velocitySet_).
    /** The position as measured by {@link #velocityMeasured} */
    protected final Position positionMeasured = new Position(); // Continuously updated by integrating measured velocities (velocityMeasured_).

    /** target velocity for next period */
    protected Map<VelocityDirection, Double> velocity = defaultedDirectionMap(0d);
    /** Actual velocity target */
    protected Map<VelocityDirection, Double> targetVelocity = defaultedDirectionMap(0d);
    /** The previous period's set */
    protected Map<VelocityDirection, Double> velocitySet = defaultedDirectionMap(0d);
    /** The next period's target velocities from kinematics */
    protected Map<VelocityDirection, Double> velocityMeasured = defaultedDirectionMap(0d);

    public AbstractChassis(Map<WheelPosition, DriveModule> driveModules, double wheelbase, double trackWidth, double wheelRadius, Supplier<Double> getSeconds) {
        this.driveModules = driveModules;
        this.wheelbase = wheelbase;
        this.trackWidth = trackWidth;
        this.wheelRadius = wheelRadius;
        this.getSeconds = getSeconds;
    }

    public AbstractChassis(Map<WheelPosition, DriveModule> driveModules, double wheelbase, double trackWidth, double wheelRadius) {
        this(driveModules, wheelbase, trackWidth, wheelRadius, Clock::getSeconds);
    }

    @Override public void periodic() {
        double now = getSeconds.get();
        timeSinceLastUpdate = now - lastUpdateTime;
        lastUpdateTime = now;

        updatePosition();
        limitVelocity(FORWARD, maximumSpeed);
        limitVelocity(STRAFE, maximumSpeed);
        limitVelocity(ROTATION, maximumRotation);
        updateDriveModules();
    }

    protected void limitVelocity(VelocityDirection direction, double max) {
        double currentVelocity = this.velocity.get(direction);
        double targetVelocity = this.targetVelocity.get(direction);
        double deltaRequested = targetVelocity - currentVelocity;

        boolean accelerating = Math.signum(deltaRequested) == Math.signum(targetVelocity);
        Type type = accelerating ? Type.ACCELERATION : Type.DECELERATION;

        double deltaLimit = accelerationLimits.get(direction, type) * timeSinceLastUpdate; // always positive
        double delta = Math.min(deltaLimit, Math.abs(deltaRequested)) * Math.signum(deltaRequested);
        double v = currentVelocity + delta;
        double newDirectionVelocity = Math.max(-max, Math.min(max, v));

        velocity.put(direction, newDirectionVelocity);
    }

    /**
     * Perform any position calculations necessary to account for
     * movement since last update.
     * Called by {@link #periodic()}.
     */
    protected abstract void updatePosition();

    /**
     * Update drive module setpoints.
     * Called by {@link #periodic()}.
     */
    protected abstract void updateDriveModules();

    @Override public Map<VelocityDirection, Double> getVelocitySet() {
        return new EnumMap<>(targetVelocity);
    }

    @Override public Map<VelocityDirection, Double> getVelocityCalculated() {
        return velocity;
    }

    @Override public Map<VelocityDirection, Double> getVelocityMeasured() {
        return velocityMeasured;
    }

    @Override public Map<VelocityDirection, Double> getMaximumVelocity() {
        Map<VelocityDirection, Double> stuff = new EnumMap<>(VelocityDirection.class);

        stuff.put(FORWARD, maximumSpeed);
        stuff.put(STRAFE, maximumSpeed);
        stuff.put(ROTATION, maximumRotation);

        return stuff;
    }

    @Override public Map<VelocityDirection, Double> getMinimumVelocity() {
        Map<VelocityDirection, Double> stuff = new EnumMap<>(VelocityDirection.class);

        stuff.put(FORWARD, minimumSpeed);
        stuff.put(STRAFE, minimumSpeed);
        stuff.put(ROTATION, minimumRotation);

        return stuff;
    }

    @Override public Position getPosition() {
        return positionSet.copy();
    }

    @Override public void setVelocity(double north, double east, double clockwise) {
        setVelocity(north, east, clockwise, accelerationLimits);
    }

    @Override public void setVelocity(double north, double east) {
        setVelocity(north, east, accelerationLimits);
    }

    @Override public void setRotation(double clockwise) {
        setRotation(clockwise, accelerationLimits);
    }

    @Override public final void setVelocityPercent(double north, double east, double clockwise) {
        Map<VelocityDirection, Double> max = getMaximumVelocity();
        setVelocity(north * max.get(FORWARD), east * max.get(STRAFE), clockwise * max.get(ROTATION));
    }

    @Override public final void setVelocityPercent(double north, double east) {
        Map<VelocityDirection, Double> max = getMaximumVelocity();
        setVelocity(north * max.get(FORWARD), east * max.get(STRAFE));
    }

    @Override public final void setRotationPercent(double clockwise) {
        setRotation(clockwise * getMaximumVelocity().get(ROTATION));
    }

    @Override public void halt() {
        double max = 10000;
        setVelocity(0, 0, 0, new AccelerationLimits(new double[][]{{max, max}, {max, max}, {max, max}}, this));
    }

    @Override public void setGLimits(double accelerationG, double decelerationG) {
        double eastAccelerationRatio = .7;
        accelerationLimits = new AccelerationLimits(false, accelerationG, decelerationG, this, new double[]{1, eastAccelerationRatio, 1});
    }

    @Override public AccelerationLimits getAccelerationLimits() {
        return accelerationLimits.getAdjustedAdjustments();
    }
}
