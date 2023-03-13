package org.usfirst.frc.team2077;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import org.usfirst.frc.team2077.common.HardwareRequirements;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.subsystem.CANLineSubsystem;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.subsystem.Claw;
import org.usfirst.frc.team2077.subsystem.ScissorArm;
import org.usfirst.frc.team2077.subsystem.SwerveMotor;

public class RobotHardware extends HardwareRequirements<SwerveMotor, SwerveChassis> {
    private final SwerveChassis chassis;

    private static RobotHardware instance;

    public static RobotHardware getInstance() {
        if(instance == null) instance = new RobotHardware();
        return instance;
    }

//    public final TalonSRX piston = new TalonSRX(10);

//    public final CANSparkMax arm = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
    public final ScissorArm scissorArm = new ScissorArm();

    public final Claw claw = new Claw();

    public final SwerveMotor northEast = new SwerveMotor(SwerveMotor.MotorPosition.FRONT_RIGHT);
    public final SwerveMotor northWest = new SwerveMotor(SwerveMotor.MotorPosition.FRONT_LEFT);
    public final SwerveMotor southEast = new SwerveMotor(SwerveMotor.MotorPosition.BACK_RIGHT);
    public final SwerveMotor southWest = new SwerveMotor(SwerveMotor.MotorPosition.BACK_LEFT);

    public RobotHardware() {

        instance = this;

        chassis = new SwerveChassis(this); // new MecanumChassis(this);

    }

    @Override public SwerveChassis getChassis() {
        return chassis;
    }

    @Override public SwerveMotor getWheel(WheelPosition position) {
        switch(position) {
            case FRONT_RIGHT:
                return northEast;
            case BACK_RIGHT:
                return southEast;
            case BACK_LEFT:
                return southWest;
            case FRONT_LEFT:
                return northWest;
        }

        return null;
    }
}
