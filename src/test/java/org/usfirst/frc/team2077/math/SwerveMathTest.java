package org.usfirst.frc.team2077.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.*;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.*;
import org.usfirst.frc.team2077.drivetrain.SwerveModule;

import java.util.*;

import static java.lang.Math.toRadians;
import static org.junit.jupiter.api.Assertions.*;

public class SwerveMathTest {
    static double wheelBase = 25.5;
    static double trackWidth = 21;
    static SwerveMath math = new SwerveMath(wheelBase, trackWidth);

    @Test void simple_targets_return_simple_targets() {
        assertMagnitudes(1, 0, 0, 1, 0);
        assertMagnitudes(0, 1, 0, 1, 90);
        assertMagnitudes(-1, 0, 0, 1, 180);
        assertMagnitudes(0, -1, 0, 1, 270);

        assertMagnitudesAngle(0.5, 0.5, 0, 45);
        assertMagnitudesAngle(-0.5, 0.5, 0, 135);
        assertMagnitudesAngle(-0.5, -0.5, 0, 225);
        assertMagnitudesAngle(0.5, -0.5, 0, 315);

        // Use a perfectly square robot for angular velocity, normalizes output
        var halfY = 2.5;
        var halfX = 2.5;
        var garbage = new SwerveDriveKinematics(
                new Translation2d(-halfX, halfY), new Translation2d(halfX, halfY),
                new Translation2d(-halfX, -halfY), new Translation2d(halfX, -halfY)
        );
        for(var thing : garbage.toSwerveModuleStates(new ChassisSpeeds(0, 0, toRadians(1)))) {
            System.out.println(thing);
        }
        System.out.println(garbage.toChassisSpeeds(
                state(20, 45), state(20, 135),
                state(20, 315), state(20, 225)
        ));
        math = new SwerveMath(5, 5);
        var test = math.targetsForVelocities(Map.of(
                MecanumMath.VelocityDirection.NORTH, 0.,
                MecanumMath.VelocityDirection.EAST, 0.,
                MecanumMath.VelocityDirection.ROTATION, 1.
        ));
        var test2 = math.velocitiesForTargets(mapOf(
           1, 45, 1, 135,
           1, 315, 1, 225
        ));
        System.out.println(test);
        System.out.println(test2);
        assertMagnitudeResults(
            0, 0, 1,
            1, 45,
            1, 315,
                1, 135,
                1, 225
        );

        assertMagnitudeResults(
            0, 0, -1,
            1, 225,
            1, 135,
                1, 315,
                1, 45
        );
    }

    static SwerveModuleState state(double speed, double angle) {
        return new SwerveModuleState(speed, new Rotation2d(toRadians(angle)));
    }

    @Test void foo() {
        assertVelocities(
                new TestModule(5, 0), new TestModule(5, 0),
                new TestModule(5, 0), new TestModule(5, 0),
                5, 0, 0
        );

        assertVelocities(
                new TestModule(5, 90), new TestModule(5, 90),
                new TestModule(5, 90), new TestModule(5, 90),
                0, 5, 0
        );

        var halfX = wheelBase / 2;
        var halfY = trackWidth / 2;
        var theirs = new SwerveDriveKinematics(
                new Translation2d(-halfX, halfY), new Translation2d(halfX, halfY),
                new Translation2d(-halfX, -halfY), new Translation2d(halfX, -halfY)
        );
        var thing = theirs.toSwerveModuleStates(new ChassisSpeeds(0, 0, toRadians(5)));
        for(var t : thing) {
            System.out.println(t);
        }
        var theirs2 = theirs.toChassisSpeeds(
                state(5, 50.53), state(5, 129.47),
                state(5, 309.47), state(5, 230.53)
        );
        System.out.println(theirs2);

        assertVelocities(
                new TestModule(5, 45), new TestModule(5, 135),
                new TestModule(5, 315), new TestModule(5, 225),
                0, 0, 40.15
        );
    }

    private static Map<WheelPosition, TestModule> mapOf(
            TestModule frontLeft, TestModule frontRight,
            TestModule backLeft, TestModule backRight
    ) {
        return Map.of(
                WheelPosition.FRONT_LEFT, frontLeft,
                WheelPosition.FRONT_RIGHT, frontRight,
                WheelPosition.BACK_LEFT, backLeft,
                WheelPosition.BACK_RIGHT, backRight
        );
    }

    private static Map<WheelPosition, TestModule> mapOf(
        double frontLeftMag, double frontLeftAngle, double frontRightMag, double frontRightAng,
        double backLeftMag, double backLeftAng, double backRightMag, double backRightAng
    ) {
        return mapOf(
                new TestModule(frontLeftMag, frontLeftAngle), new TestModule(frontRightMag, frontRightAng),
                new TestModule(backLeftMag, backLeftAng), new TestModule(backRightMag, backRightAng)
        );
    }

    private static void assertVelocities(
            TestModule frontLeft, TestModule frontRight,
            TestModule backLeft, TestModule backRight,
            double forward, double strafe, double angular
    ) {
        var velocities = math.velocitiesForTargets(mapOf(
                frontLeft, frontRight,
                backRight, backLeft
        ));

        assertEquals(forward, velocities.get(MecanumMath.VelocityDirection.NORTH), 0.001, "Forward mismatch");
        assertEquals(strafe, velocities.get(MecanumMath.VelocityDirection.EAST), 0.001, "East mismatch");
        assertEquals(angular, velocities.get(MecanumMath.VelocityDirection.ROTATION), 0.001, "Rotation mismatch");
    }

    private static void assertNotVelocities(
            TestModule frontLeft, TestModule frontRight,
            TestModule backLeft, TestModule backRight,
            double forward, double strafe, double angular
    ) {
        var velocities = math.velocitiesForTargets(Map.of(
                WheelPosition.FRONT_LEFT, frontLeft,
                WheelPosition.FRONT_RIGHT, frontRight,
                WheelPosition.BACK_RIGHT, backRight,
                WheelPosition.BACK_LEFT, backLeft
        ));

        assertNotEquals(forward, velocities.get(MecanumMath.VelocityDirection.NORTH), 0.001, "Forward mismatch");
        assertNotEquals(strafe, velocities.get(MecanumMath.VelocityDirection.EAST), 0.001, "East mismatch");
        assertNotEquals(angular, velocities.get(MecanumMath.VelocityDirection.ROTATION), 0.001, "Rotation mismatch");
    }

    private static void assertMagnitudes(double north, double east, double rotation, double expectedMag, double expectedAng) {
        Map<WheelPosition, SwerveTargetValues> values = math.targetsForVelocities(magnitudeMap(north, east, rotation));

        values.forEach((k, val) -> assertTargets(k, val, expectedMag, expectedAng));
    }

    private static void assertMagnitudesAngle(double north, double east, double rotation, double expectedAng) {
        Map<WheelPosition, SwerveTargetValues> values = math.targetsForVelocities(magnitudeMap(north, east, rotation));

        values.forEach((k, val) -> assertEquals(expectedAng, val.getAngle(), .1, k + " angle unexpected"));
    }
    
    private static void assertMagnitudeResults(
        double north, double east, double rotation,
        double frontLeftMag, double frontLeftAng,
        double backLeftMag, double backLeftAng,
        double frontRightMag, double frontRightAng,
        double backRightMag, double backRightAng
    ) {
        var targets = math.targetsForVelocities(magnitudeMap(north, east, rotation));
        assertMapValues(
            targets,
            frontLeftMag, frontLeftAng,
            backLeftMag, backLeftAng,
            frontRightMag, frontRightAng,
            backRightMag, backRightAng
        );
    }
    
    private static void assertMapValues(
        Map<WheelPosition, SwerveTargetValues> values,
        double frontLeftMag, double frontLeftAng,
        double backLeftMag, double backLeftAng,
        double frontRightMag, double frontRightAng,
        double backRightMag, double backRightAng
    ) {
        assertTargets(WheelPosition.FRONT_LEFT, values.get(WheelPosition.FRONT_LEFT), frontLeftMag, frontLeftAng);
        assertTargets(WheelPosition.BACK_LEFT, values.get(WheelPosition.BACK_LEFT), backLeftMag, backLeftAng);
        assertTargets(WheelPosition.FRONT_RIGHT, values.get(WheelPosition.FRONT_RIGHT), frontRightMag, frontRightAng);
        assertTargets(WheelPosition.BACK_RIGHT, values.get(WheelPosition.BACK_RIGHT), backRightMag, backRightAng);
    }

    private static void assertTargets(
        WheelPosition k,
        SwerveTargetValues targets,
        double magnitude,
        double angle
    ) {
        assertEquals(magnitude, targets.getMagnitude(), .1, k + " magnitude mismatch");
        assertEquals(angle, targets.getAngle(), .1, k + " angle mismatch");
    }

    private static Map<MecanumMath.VelocityDirection, Double> magnitudeMap(double north, double east, double rotation) {
        EnumMap<MecanumMath.VelocityDirection, Double> map = new EnumMap<>(MecanumMath.VelocityDirection.class);

        map.put(MecanumMath.VelocityDirection.NORTH, north);
        map.put(MecanumMath.VelocityDirection.EAST, east);
        map.put(MecanumMath.VelocityDirection.ROTATION, rotation);

        return map;
    }

    private static class TestModule implements SwerveModule {
        private final double angle, velocity;

        TestModule(double velocity, double angle) {
            this.angle = angle;
            this.velocity = velocity;
        }

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
        public double getVelocity() {
            return velocity;
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

        @Override
        public double getWheelAngle() {
            return angle;
        }
    }
}
