package org.usfirst.frc.team2077;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.common.drivetrain.*;
import org.usfirst.frc.team2077.common.subsystems.*;

public class RobotHardware implements org.usfirst.frc.team2077.common.RobotHardware<SparkNeoDriveModule> {
    private final Subsystem HEADING = new Subsystem() {};
    private final Subsystem POSITION = new Subsystem() {};
    private final AbstractChassis CHASSIS;


    public final CANLineSubsystem.SparkNeo FRONT_LEFT_WHEEL = new CANLineSubsystem.SparkNeo(SparkNeoDriveModule.DrivePosition.FRONT_LEFT);
    public final CANLineSubsystem.SwerveModule NORTH_EAST_MODULE = new CANLineSubsystem.SwerveModule(SparkNeoDriveModule.DrivePosition.NORTH_EAST_SWERVE_MODULE);

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
