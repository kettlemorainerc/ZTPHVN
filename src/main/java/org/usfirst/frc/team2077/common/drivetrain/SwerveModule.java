package org.usfirst.frc.team2077.common.drivetrain;

public interface SwerveModule extends DriveModuleIF {

    public void setTargetDegrees(double degrees);

    public void setTargetMagnitude(double magnitude);

    public double getWheelAngle();

}
