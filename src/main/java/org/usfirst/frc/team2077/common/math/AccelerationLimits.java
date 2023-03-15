/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.common.math;

import org.usfirst.frc.team2077.common.drivetrain.DriveChassisIF;
import org.usfirst.frc.team2077.common.VelocityDirection;

import java.util.*;

import static org.usfirst.frc.team2077.common.VelocityDirection.*;
import static org.usfirst.frc.team2077.common.math.AccelerationLimits.Type.*;

/**
 * Acceleration limits by {@link VelocityDirection} and {@link AccelerationLimits.Type}
 * <p>
 * Ideally accel/decel values are set just below wheelspin or skidding to a stop.
 * Optimal values are highly dependent on wheel/surface traction and somewhat on weight distribution.
 * For safety err on the low side for acceleration, high for deceleration.
 * <p>
 * North/East limits are in inches/second/second
 * Rotation limits are in degrees/second/second
 */
public class AccelerationLimits {

    public enum Type {
        ACCELERATION,
        DECELERATION
    }

    protected final EnumMatrix<Type, VelocityDirection> LIMITS = new EnumMatrix<>(Type.class, VelocityDirection.class);
    ;
    protected final DriveChassisIF defaultChassis;
    public static final double G = 386.09; // Acceleration of gravity in inches/second/second.

    public AccelerationLimits(double accelerationG, double decelerationG, DriveChassisIF chassis) {
        this(accelerationG, decelerationG, chassis, new double[]{1, 1, 0});
    }

    public AccelerationLimits(double accelerationG, double decelerationG, DriveChassisIF chassis, double[] scale) {
        this(true, accelerationG, decelerationG, chassis, scale);
    }

    public AccelerationLimits(boolean calculateRotation, double accelerationG, double decelerationG, DriveChassisIF chassis) {
        this(calculateRotation, accelerationG, decelerationG, chassis, new double[]{1, 1, 1});
    }

    public AccelerationLimits(boolean calculateRotation, double accelerationG, double decelerationG, DriveChassisIF chassis, double[] scale) {
        this.defaultChassis = chassis;

        put(FORWARD, accelerationG, decelerationG, scale[FORWARD.ordinal()]);
        put(STRAFE, accelerationG, decelerationG, scale[STRAFE.ordinal()]);

        if (calculateRotation) {
            Map<VelocityDirection, Double> max = chassis.getMaximumVelocity();
            double inchesToDegrees = max.get(ROTATION) / max.get(FORWARD);

            put(ROTATION, inchesToDegrees * accelerationG, inchesToDegrees * decelerationG, scale[ROTATION.ordinal()]);
        } else {
            put(ROTATION, 0, 0, 0);
        }
    }

    public AccelerationLimits(double[][] doubles, DriveChassisIF chassis) {
        set(FORWARD, doubles[FORWARD.ordinal()]);
        set(STRAFE, doubles[STRAFE.ordinal()]);
        set(ROTATION, doubles[ROTATION.ordinal()]);

        defaultChassis = chassis;
    }

    private void put(VelocityDirection d, double accelerationInGs, double decelerationInGs, double scale) {
        LIMITS.set(ACCELERATION, d, G * accelerationInGs * scale);
        LIMITS.set(DECELERATION, d, G * decelerationInGs * scale);
    }

    public void set(VelocityDirection d, double[] limits) {
        LIMITS.set(ACCELERATION, d, limits[ACCELERATION.ordinal()]);
        LIMITS.set(DECELERATION, d, limits[DECELERATION.ordinal()]);
    }

    public void set(VelocityDirection d, double acceleration, double deceleration) {
        LIMITS.set(ACCELERATION, d, acceleration);
        LIMITS.set(DECELERATION, d, deceleration);
    }

    public double[] get(VelocityDirection d, DriveChassisIF chassis) {
        if (d == ROTATION) {
            double[] limits = LIMITS.getMatrix()[d.ordinal()];
            Map<VelocityDirection, Double> max = chassis.getMaximumVelocity();
            double[] adjustedLimits = new double[2];
            int accel = ACCELERATION.ordinal();
            int decel = DECELERATION.ordinal();

            adjustedLimits[accel] = limits[accel] > 0 ? limits[accel] : LIMITS.get(ACCELERATION, FORWARD) * max.get(ROTATION) / max.get(FORWARD);
            adjustedLimits[decel] = limits[decel] > 0 ? limits[decel] : LIMITS.get(DECELERATION, FORWARD) * max.get(ROTATION) / max.get(FORWARD);

            return adjustedLimits;
        }

        return LIMITS.getMatrix()[d.ordinal()];
    }

    public double[] get(VelocityDirection d) {
        return LIMITS.getMatrix()[d.ordinal()];
    }

    public double[][] getAdjusted(DriveChassisIF chassis) {
        return new double[][]{
                get(FORWARD, chassis),
                get(STRAFE, chassis),
                get(ROTATION, chassis)
        };
    }

    public AccelerationLimits getAdjustedAdjustments() {
        return new AccelerationLimits(getAdjusted(defaultChassis), defaultChassis);
    }

    /**
     * gets the raw, unadjusted value of the limit
     *
     * @param d Direction
     * @param t Type
     * @return the limit for {@link VelocityDirection} {@code d} and {@link Type} {@code t}
     */
    public double get(VelocityDirection d, Type t) {
        return LIMITS.get(t, d);
    }

    /**
     * @return Acceleration/deceleration limits in inches/second/second and degrees/second/second.
     */
    public double[][] get() {
        return LIMITS.getMatrix();
    }
}
