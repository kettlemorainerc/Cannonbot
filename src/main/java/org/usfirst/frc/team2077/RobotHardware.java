package org.usfirst.frc.team2077;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.common.drivetrain.*;
import org.usfirst.frc.team2077.common.subsystems.*;

public class RobotHardware implements org.usfirst.frc.team2077.common.RobotHardware<SparkNeoDriveModule> {
    private final Subsystem HEADING = new Subsystem() {};
    private final Subsystem POSITION = new Subsystem() {};
    private final AbstractChassis CHASSIS;


    public final RotationMotor northEast = new RotationMotor(RotationMotor.MotorPosition.FRONT_RIGHT);
    public final RotationMotor northWest = new RotationMotor(RotationMotor.MotorPosition.FRONT_LEFT);
    public final RotationMotor southEast = new RotationMotor(RotationMotor.MotorPosition.BACK_RIGHT);
    public final RotationMotor southWest = new RotationMotor(RotationMotor.MotorPosition.BACK_LEFT);


    public final CANLineSubsystem.SparkNeo FRONT_LEFT_WHEEL = new CANLineSubsystem.SparkNeo(SparkNeoDriveModule.DrivePosition.FRONT_LEFT);
//    public final CANLineSubsystem.SwerveModule NORTH_EAST_MODULE = new CANLineSubsystem.SwerveModule(SparkNeoDriveModule.DrivePosition.NORTH_EAST_SWERVE_MODULE);
//    public final  = new SwerveModule(RobotMap.magnitudeMotor1, RobotMap.directionMotor1, RobotMap.encoder1ChannelA, RobotMap.encoder1ChannelB, length/2, width/2);
//        northEast.setPID(deadAngle, Pvalue, Ivalue, Dvalue);
//
//
//    southEast = new SwerveModule(RobotMap.magnitudeMotor2, RobotMap.directionMotor2, RobotMap.encoder2ChannelA, RobotMap.encoder2ChannelB, -length/2, width/2);
//        southEast.setPID(deadAngle, Pvalue, Ivalue, Dvalue);
//
//    southWest = new SwerveModule(RobotMap.magnitudeMotor3, RobotMap.directionMotor3, RobotMap.encoder3ChannelA, RobotMap.encoder3ChannelB, -length/2, -width/2);
//        southWest.setPID(deadAngle, Pvalue, Ivalue, Dvalue);
//
//    northWest = new SwerveModule(RobotMap.magnitudeMotor4, RobotMap.directionMotor4, RobotMap.encoder4ChannelA, RobotMap.encoder4ChannelB, length/2, -width/2);
//        northWest.setPID(deadAngle, Pvalue, Ivalue, Dvalue);


//    public final CANLineSubsystem.SparkNeo FRONT_LEFT_WHEEL = new CANLineSubsystem.SparkNeo(SparkNeoDriveModule.DrivePosition.FRONT_LEFT);

    public RotationModule getRotationModule(MecanumMath.WheelPosition position) {
        //TODO: Place stuff here
        return null;
    }


    public RobotHardware() {
//        CHASSIS = new MecanumChassis(this);
        CHASSIS = new SwerveChassis(this);
    }

    @Override public Subsystem getHeading() {
        return HEADING;
    }

    @Override public Subsystem getPosition() {
        return POSITION;
    }

    @Override public AbstractChassis getChassis() {
        return CHASSIS;
    }

    @Override public AHRS getNavX() {
        return null;
    }

    @Override public CANLineSubsystem<SparkNeoDriveModule> getWheel(MecanumMath.WheelPosition position) {
        return null;
    }
}
