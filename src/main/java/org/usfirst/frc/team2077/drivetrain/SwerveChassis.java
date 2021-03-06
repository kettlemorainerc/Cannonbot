package org.usfirst.frc.team2077.drivetrain;


import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.drivetrain.AbstractChassis;
import org.usfirst.frc.team2077.common.drivetrain.MecanumMath;
import org.usfirst.frc.team2077.common.math.AccelerationLimits;
import org.usfirst.frc.team2077.common.sensors.AngleSensor;

import java.util.EnumMap;
import java.util.function.Supplier;

import static org.usfirst.frc.team2077.common.drivetrain.MecanumMath.VelocityDirection.*;
import static org.usfirst.frc.team2077.common.drivetrain.MecanumMath.VelocityDirection.EAST;

public class SwerveChassis extends AbstractChassis {
    private static final double WHEELBASE = 20.375; // inches
    private static final double TRACK_WIDTH = 25.5; // inches
    private static final double WHEEL_RADIUS = 4.0; // inches
    private static final double EAST_ADJUSTMENT = .65;

//    private static final RotationMotor directionMotor = new RotationMotor();

    private final AngleSensor angleSensor;
    public final EnumMap<MecanumMath.WheelPosition, SwerveModule> rotationModules;

    public SwerveChassis(RobotHardware hardware, Supplier<Double> getSeconds) {
        super(null, WHEELBASE, TRACK_WIDTH, WHEEL_RADIUS, getSeconds);
//        super(buildDriveModule(pos -> hardware.getWheel(pos).motor), WHEELBASE, TRACK_WIDTH, WHEEL_RADIUS, getSeconds);
//        rotationModules = buildDriveModule(hardware::getRotationModule);
        rotationModules = null;
        this.angleSensor = hardware.getAngleSensor();
    }


    @Override
    protected void updatePosition() {
        velocitySet = getVelocityCalculated();
        // TODO: update velocityMeasured by reading rotation motors and driveModule motors and calculating the current velocity

        // Technically whichever following todo isn't returned by AbstractChassis#getPosition is optional
        // TODO: update positionSet using velocity set
        // TODO: update positionMeasured using velocity measured
    }

    @Override
    protected void updateDriveModules() {
        // The following is the BIG one
        // TODO: Convert targetVelocity to target magnitude and wheel angle

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

//    @Override
//    public void setVelocity(double north, double east, double clockwise, AccelerationLimits accelerationLimits) {
//
//        directionMotor.set(ControlMode.PercentOutput, percentage);
//    }

//    @Override
//    public void setVelocity(double north, double east, AccelerationLimits accelerationLimits) {
//
//    }

//    @Override
//    public void setRotation(double clockwise, AccelerationLimits accelerationLimits) {
//
//        // angle error
//        double deltaDir = getWheelDir() - dir;
//
//        // Map the delta onto the same -180 to 180 polar coordinate system
//        // used by the FIRST joystick class.
//        deltaDir = deltaDir % 360;
//        if (deltaDir > 180.0) {
//            deltaDir -= 360.0;
//        } else if (deltaDir < -180.0) {
//            deltaDir += 360.0;
//        }
//
//        // Start PID
//        double error = deltaDir / 180.0;
//        errorAccum += error * dt;
//
//        // Set the direction motor to value specified by PID
//        dirCtrl.set(Pvalue * error + Ivalue * errorAccum + Dvalue * ((error - lastError / dt)));
//
//        // If the remaining error is small enough (small delta angle), then
//        // engage the magnitude motor
//        if (Math.abs(deltaDir) > deadAngle) {
//            magCtrl.set(0.0); // Outside of deadzone
//        } else {
//            // Inside of deadzone
//            magCtrl.set(mag);
//        }
//
//        lastError = error;
//
//        //SmartDashboard.putNumber("Delta Dir", deltaDir);
//        //SmartDashboard.putNumber("Delta Time", dt);
//    }
}