package org.usfirst.frc.team2077;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.*;
import org.usfirst.frc.team2077.common.*;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.subsystem.*;

public class RobotHardware extends HardwareRequirements<SwerveMotor, SwerveChassis> {
    private final SwerveChassis chassis;

    private static RobotHardware instance = null;

    public static RobotHardware getInstance() {
        if(instance == null) instance = new RobotHardware();
        return instance;
    }

    public final Cannon cannon;
    public final AirCompressor airCompressor;
    public final TalonSRX piston = new TalonSRX(9);
    public final Horn horn = new Horn(10);
    public final SwerveMotor northEast = new SwerveMotor(SwerveMotor.MotorPosition.FRONT_RIGHT);
    public final SwerveMotor northWest = new SwerveMotor(SwerveMotor.MotorPosition.FRONT_LEFT);
    public final SwerveMotor southEast = new SwerveMotor(SwerveMotor.MotorPosition.BACK_RIGHT);
    public final SwerveMotor southWest = new SwerveMotor(SwerveMotor.MotorPosition.BACK_LEFT);

    public RobotHardware() {

        instance = this;

        chassis = new SwerveChassis(this); // new MecanumChassis(this);

        airCompressor = new AirCompressor(1);

        cannon = new Cannon();


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
