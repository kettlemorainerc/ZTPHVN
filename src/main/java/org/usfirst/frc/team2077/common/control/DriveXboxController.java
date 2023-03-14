package org.usfirst.frc.team2077.common.control;

import edu.wpi.first.wpilibj.*;

/**
 * An xbox controller that can be used for driving
 */
public class DriveXboxController extends XboxController implements DriveStick {

    protected double driveDeadBand, driveExponent;
    protected double rotationDeadBand, rotationExponent;

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public DriveXboxController(int port) {
        super(port);
    }


    public DriveXboxController setDriveSensitivity(double deadBand, double exponent) {
        this.driveDeadBand = deadBand;
        this.driveExponent = exponent;
        return this;
    }

    public DriveXboxController setRotationSensitivity(double deadBand, double exponent) {
        this.rotationDeadBand = deadBand;
        this.rotationExponent = exponent;
        return this;
    }

    public DriveXboxController setSensitivity(double deadBand, double exponent) {
        return setDriveSensitivity(deadBand, exponent).setRotationSensitivity(deadBand, exponent);
    }


    @Override
    public double getNorth() {

        int pov = super.getPOV();
        if(pov == -1) return DriveStick.adjustInputSensitivity(super.getLeftY(), driveDeadBand, driveExponent);

        if(pov == 0 || pov == 45 || pov == 315 ){
            return 1;
        }

        if(pov == 180 || pov == 135 || pov == 225){
            return -1;
        }

        return 0;
    }

    @Override
    public double getEast() {

        int pov = super.getPOV();
        if(pov == -1) return DriveStick.adjustInputSensitivity(super.getLeftX(), driveDeadBand, driveExponent);

        if(pov == 45 || pov == 90 || pov == 135 ){
            return 1;
        }

        if(pov == 225 || pov == 270 || pov == 315){
            return -1;
        }

        return 0;

    }

    @Override
    public double getRotation() {
        return DriveStick.adjustInputSensitivity(super.getRightX(), driveDeadBand, driveExponent);
    }

}
