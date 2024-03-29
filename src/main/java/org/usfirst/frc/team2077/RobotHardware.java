package org.usfirst.frc.team2077;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.*;
import org.usfirst.frc.team2077.common.sensor.AngleSensor;
import org.usfirst.frc.team2077.common.subsystem.CANLineSubsystem;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.subsystem.*;

public class RobotHardware implements org.usfirst.frc.team2077.common.RobotHardware<SwerveMotor, SwerveChassis> {
    private final Subsystem HEADING = new Subsystem() {};
    private final Subsystem POSITION = new Subsystem() {};
    private final AbstractChassis CHASSIS;

    public final Cannon CANNON;
    public final AirCompressor AIRCOMPRESSOR;
    public final TalonSRX PISTON = new TalonSRX(0);
    public static final double WHEELBASE = 20.375; // inches
    public static final double TRACK_WIDTH = 25.5; // inches
    public static final double WHEEL_RADIUS = 4.0; // inches
    public static final double EAST_ADJUSTMENT = .65;

    public final SwerveMotor northEast = new SwerveMotor(SwerveMotor.MotorPosition.FRONT_RIGHT);
    public final SwerveMotor northWest = new SwerveMotor(SwerveMotor.MotorPosition.FRONT_LEFT);
    public final SwerveMotor southEast = new SwerveMotor(SwerveMotor.MotorPosition.BACK_RIGHT);
    public final SwerveMotor southWest = new SwerveMotor(SwerveMotor.MotorPosition.BACK_LEFT);


    public final CANLineSubsystem.SparkNeo FRONT_LEFT_WHEEL = new CANLineSubsystem.SparkNeo(SparkNeoDriveModule.DrivePosition.FRONT_LEFT);


    public RobotHardware() {
        CHASSIS = new SwerveChassis(this); // new MecanumChassis(this);


        AIRCOMPRESSOR = new AirCompressor(1);
//        CHASSIS = new MecanumChassis(this);
//        CHASSIS = new SwerveChassis(this);

        // TODO: determine channels
        Solenoid loadValve = new Solenoid(42, PneumaticsModuleType.CTREPCM, 1);
        Relay launchValve = new Relay(0, Relay.Direction.kForward);
        PressureSensor pressure = new PressureSensor(2);
        CANNON = new Cannon(loadValve, launchValve, pressure);
    }

    @Override
    public Subsystem getHeading() {
        return HEADING;
    }

    @Override
    public Subsystem getPosition() {
        return POSITION;
    }

    @Override
    public SwerveChassis getChassis() {
        return null;
    }

    @Override
    public AngleSensor getAngleSensor() {
        return null;
    }

    @Override
    public AHRS getNavX() {
        return null;
    }

    @Override
    public SwerveMotor getWheel(WheelPosition position) {
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
