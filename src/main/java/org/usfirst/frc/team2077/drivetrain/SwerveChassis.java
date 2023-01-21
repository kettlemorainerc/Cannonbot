package org.usfirst.frc.team2077.drivetrain;


import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.RobotHardware;
import org.usfirst.frc.team2077.common.drivetrain.AbstractChassis;
import org.usfirst.frc.team2077.common.drivetrain.DriveModuleIF;
import org.usfirst.frc.team2077.common.drivetrain.MecanumMath;
import org.usfirst.frc.team2077.common.math.AccelerationLimits;
import org.usfirst.frc.team2077.common.sensors.AngleSensor;
import org.usfirst.frc.team2077.common.subsystems.CANLineSubsystem;
import org.usfirst.frc.team2077.math.SwerveMath;
import org.usfirst.frc.team2077.math.SwerveTargetValues;
import org.usfirst.frc.team2077.subsystem.SwerveMotor;

import java.util.Comparator;
import java.util.EnumMap;
import java.util.Map;
import java.util.function.Supplier;

import static org.usfirst.frc.team2077.common.drivetrain.MecanumMath.VelocityDirection.*;
import static org.usfirst.frc.team2077.common.drivetrain.MecanumMath.VelocityDirection.EAST;

public class SwerveChassis extends AbstractChassis<SwerveMotor> {
    private static final double WHEELBASE = 20.375; // inches
    private static final double TRACK_WIDTH = 25.5; // inches
    private static final double WHEEL_RADIUS = 4.0; // inches
    private static final double EAST_ADJUSTMENT = .65;

//    private static final RotationMotor directionMotor = new RotationMotor();

    private final SwerveMath math;
    private final AngleSensor angleSensor;

    private static EnumMap<MecanumMath.WheelPosition, SwerveMotor> buildDriveTrain(RobotHardware<SwerveMotor> hardware) {
        EnumMap<MecanumMath.WheelPosition, SwerveMotor> map = new EnumMap<>(MecanumMath.WheelPosition.class);

        for(MecanumMath.WheelPosition p : MecanumMath.WheelPosition.values()) {
            map.put(p, hardware.getWheel(p));
        }

        return map;
    }

    public SwerveChassis(RobotHardware<SwerveMotor> hardware, Supplier<Double> getSeconds) {
        super(buildDriveTrain(hardware), WHEELBASE, TRACK_WIDTH, WHEEL_RADIUS, getSeconds);
//        super(buildDriveModule(pos -> hardware.getWheel(pos).motor), WHEELBASE, TRACK_WIDTH, WHEEL_RADIUS, getSeconds);
//        rotationModules = buildDriveModule(hardware::getRotationModule);
        this.angleSensor = hardware.getAngleSensor();
        math = new SwerveMath(WHEELBASE, TRACK_WIDTH);

        this.maximumRotation = 1;
        this.maximumSpeed =  this.driveModule.values()
                .stream()
                .map(DriveModuleIF::getMaximumSpeed)
                .min(Comparator.naturalOrder())
                .orElseThrow();;
        this.minimumRotation = 0;
        this.minimumSpeed =  this.maximumSpeed * 0.1;
    }

    public SwerveChassis(RobotHardware<SwerveMotor> hardware) {
        this(hardware, Clock::getSeconds);
    }


    @Override
    protected void updatePosition() {
        velocitySet = getVelocityCalculated();
        // TODO: update velocityMeasured by reading rotation motors and driveModule motors and calculating the current velocity
        velocityMeasured = math.velocitiesForTargets(driveModule);
        // Technically whichever following todo isn't returned by AbstractChassis#getPosition is optional
        // TODO: update positionSet using velocity set
        // TODO: update positionMeasured using velocity measured
        positionSet.moveRelative(
                velocitySet.get(NORTH) * timeSinceLastUpdate,
                velocitySet.get(EAST) * timeSinceLastUpdate,
                velocitySet.get(ROTATION) * timeSinceLastUpdate
        );
        positionMeasured.moveRelative(
                velocityMeasured.get(NORTH) * timeSinceLastUpdate,
                velocityMeasured.get(EAST) * timeSinceLastUpdate,
                velocityMeasured.get(ROTATION) * timeSinceLastUpdate
        );
    }

    int sentinel;
    public static MecanumMath.WheelPosition LOGGED_POSITION = MecanumMath.WheelPosition.NORTH_EAST;
    @Override
    protected void updateDriveModules() {
        // The following is the BIG one
        // TODO: Convert targetVelocity to target magnitude and wheel angle

        Map<MecanumMath.WheelPosition, SwerveTargetValues> wheelTargets = math.targetsForVelocities(targetVelocity);

        wheelTargets.forEach( (key, value) -> {
            SwerveMotor motor = this.driveModule.get(key);

            if(key == LOGGED_POSITION && (sentinel = (sentinel + 1) % 25) == 0) {
//                System.out.println("[position=" + LOGGED_POSITION + "][targets=" + value + ']');
            }
            motor.setTargetAngle(value.getAngle());
            motor.setMagnitude(value.getMagnitude());
        });

        // TODO: update each rotation motor to the target wheel angle (setTargetAngle)
        // TODO: update each driveModule motor to the target magnitude (setPercent)
    }

    @Override
    public void setVelocity(double north, double east, double clockwise, AccelerationLimits accelerationLimits) {
        setVelocity(north, east, accelerationLimits);
        setRotation(clockwise, accelerationLimits);
    }

    @Override
    public void setVelocity(double north, double east, AccelerationLimits accelerationLimits) {
        targetVelocity.put(NORTH, north);
        this.accelerationLimits.set(NORTH, accelerationLimits.get(NORTH));

        targetVelocity.put(EAST, east);
        this.accelerationLimits.set(EAST, accelerationLimits.get(EAST));
    }

    @Override
    public void setRotation(double clockwise, AccelerationLimits accelerationLimits) {
        targetVelocity.put(ROTATION, clockwise);
        this.accelerationLimits.set(ROTATION, accelerationLimits.get(ROTATION));
    }
}