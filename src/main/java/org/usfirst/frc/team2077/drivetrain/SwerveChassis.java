package org.usfirst.frc.team2077.drivetrain;


import com.ctre.phoenix.led.TwinkleAnimation;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.*;
import org.usfirst.frc.team2077.common.drivetrain.*;
import org.usfirst.frc.team2077.common.math.AccelerationLimits;
import org.usfirst.frc.team2077.common.sensor.AngleSensor;
import org.usfirst.frc.team2077.math.*;
import org.usfirst.frc.team2077.subsystem.SwerveMotor;

import java.util.*;
import java.util.function.Supplier;

import static javax.swing.SwingConstants.NORTH;
import static org.usfirst.frc.team2077.common.VelocityDirection.*;

public class SwerveChassis extends AbstractChassis<SwerveMotor> {
    private static final double WHEELBASE = 20.375; // inches
    private static final double TRACK_WIDTH = 25.5; // inches
    private static final double WHEEL_RADIUS = 4.0; // inches

//    private static final RotationMotor directionMotor = new RotationMotor();

    private final SwerveMath math;
    private final AngleSensor angleSensor;

    private static EnumMap<WheelPosition, SwerveMotor> buildDriveTrain(HardwareRequirements<SwerveMotor, SwerveChassis> hardware) {
        EnumMap<WheelPosition, SwerveMotor> map = new EnumMap<>(WheelPosition.class);

        for(WheelPosition p : WheelPosition.values()) {
            map.put(p, hardware.getWheel(p));
        }

        return map;
    }

    public SwerveChassis(HardwareRequirements<SwerveMotor, SwerveChassis> hardware, Supplier<Double> getSeconds) {
        super(buildDriveTrain(hardware), WHEELBASE, TRACK_WIDTH, WHEEL_RADIUS, getSeconds);
//        super(buildDriveModule(pos -> hardware.getWheel(pos).motor), WHEELBASE, TRACK_WIDTH, WHEEL_RADIUS, getSeconds);
//        rotationModules = buildDriveModule(hardware::getRotationModule);
        this.angleSensor = hardware.getAngleSensor();
        math = new SwerveMath(WHEELBASE, TRACK_WIDTH);

        this.maximumRotation = 1;
        this.maximumSpeed =  this.driveModules.values()
                .stream()
                .map( e -> {
                    Double max = e.getMaximumSpeed();
                    System.out.printf("[%s=%s]", e.getPosition(), max);
                    return max;
                })
                .min(Comparator.naturalOrder())
                .orElseThrow();;
        System.out.println();

//        maximumSpeed

//        TestSwerve()
        Map<WheelPosition, SwerveTargetValues> wheelTargets = math.targetsForVelocities(
            Map.of(
                FORWARD, 0.0, STRAFE, 0.0, ROTATION, 1.0
            )
        );

        Map<WheelPosition, TestSwerve> map = new EnumMap<>(WheelPosition.class);
        wheelTargets.forEach(
            (k, v) -> {
                map.put(k, new TestSwerve(v.getAngle(), v.getMagnitude() * maximumSpeed));
            }
        );

        maximumRotation = math.velocitiesForTargets(map).get(ROTATION);

        this.minimumSpeed =  this.maximumSpeed * 0.1;
    }

    public SwerveChassis(HardwareRequirements<SwerveMotor, SwerveChassis> hardware) {
        this(hardware, Clock::getSeconds);
    }

    @Override
    protected void updatePosition() {
        velocitySet = getVelocityCalculated();
        velocityMeasured = math.velocitiesForTargets(driveModules);

        positionSet.moveRelative(
                velocitySet.get(FORWARD) * timeSinceLastUpdate,
                velocitySet.get(STRAFE) * timeSinceLastUpdate,
                velocitySet.get(ROTATION) * timeSinceLastUpdate
        );
        positionMeasured.moveRelative(
                velocityMeasured.get(FORWARD) * timeSinceLastUpdate,
                velocityMeasured.get(STRAFE) * timeSinceLastUpdate,
                velocityMeasured.get(ROTATION) * timeSinceLastUpdate
        );
    }

    public static WheelPosition LOGGED_POSITION = WheelPosition.FRONT_RIGHT;
    @Override
    protected void updateDriveModules() {
        Map<WheelPosition, SwerveTargetValues> wheelTargets = math.targetsForVelocities(targetVelocity);

        wheelTargets.forEach( (key, value) -> {
            SwerveMotor motor = this.driveModules.get(key);

            motor.setTargetAngle(value.getAngle());
            motor.setVelocity(value.getMagnitude() * maximumSpeed);
        });
    }

    @Override
    public void setVelocity(double north, double east, double clockwise, AccelerationLimits accelerationLimits) {
        setVelocity(north, east, accelerationLimits);
        setRotation(clockwise, accelerationLimits);
    }

    @Override
    public void setVelocity(double north, double east, AccelerationLimits accelerationLimits) {
        targetVelocity.put(FORWARD, north);
        this.accelerationLimits.set(FORWARD, accelerationLimits.get(FORWARD));

        targetVelocity.put(STRAFE, east);
        this.accelerationLimits.set(STRAFE, accelerationLimits.get(STRAFE));
    }

    @Override
    public void setRotation(double clockwise, AccelerationLimits accelerationLimits) {
        targetVelocity.put(ROTATION, clockwise);
        this.accelerationLimits.set(ROTATION, accelerationLimits.get(ROTATION));
    }

    private static class TestSwerve implements SwerveModule{

        private final double wheelAngle;
        private final double velocity;

        TestSwerve(double wheelAngle, double velocity) {
            this.wheelAngle = wheelAngle;
            this.velocity = velocity;
        }

        @Override
        public double getWheelAngle() {
            return wheelAngle;
        }

        @Override
        public double getVelocity() {
            return velocity;
        }

        // region Ignored Things
        @Override
        public double getMaximumSpeed() {
            return 0;
        }

        @Override
        public void setVelocity(double velocity) {

        }

        @Override
        public WheelPosition getWheelPosition() {
            return null;
        }

        @Override
        public double getDistance() {
            return 0;
        }

        @Override
        public void resetDistance() {

        }

        @Override
        public void setTargetDegrees(double degrees) {

        }

        @Override
        public void setTargetMagnitude(double magnitude) {

        }
        // endregion
    }
}