package org.usfirst.frc.team2077.math;

import org.junit.jupiter.api.*;
import org.usfirst.frc.team2077.common.drivetrain.*;
import org.usfirst.frc.team2077.drivetrain.SwerveModule;

import java.util.*;

import static java.lang.Math.toDegrees;
import static org.junit.jupiter.api.Assertions.*;

public class SwerveMathTest {
    static SwerveMath math = new SwerveMath(25.5, 21);

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
        math = new SwerveMath(5, 5);
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

        assertVelocities(
                new TestModule(5, 45), new TestModule(5, 135),
                new TestModule(5, 315), new TestModule(5, 225),
                0, 0, 40.514
        );
    }

    private static void assertVelocities(
            TestModule frontLeft, TestModule frontRight,
            TestModule backLeft, TestModule backRight,
            double forward, double strafe, double angular
    ) {
        var velocities = math.velocitiesForTargets(Map.of(
                MecanumMath.WheelPosition.NORTH_WEST, frontLeft,
                MecanumMath.WheelPosition.NORTH_EAST, frontRight,
                MecanumMath.WheelPosition.SOUTH_EAST, backRight,
                MecanumMath.WheelPosition.SOUTH_WEST, backLeft
        ));

        assertEquals(forward, velocities.get(MecanumMath.VelocityDirection.NORTH), 0.001, "Forward mismatch");
        assertEquals(strafe, velocities.get(MecanumMath.VelocityDirection.EAST), 0.001, "East mismatch");
        assertEquals(angular, velocities.get(MecanumMath.VelocityDirection.ROTATION), 0.001, "Rotation mismatch");
    }

    private static void assertMagnitudes(double north, double east, double rotation, double expectedMag, double expectedAng) {
        Map<MecanumMath.WheelPosition, SwerveTargetValues> values = math.targetsForVelocities(magnitudeMap(north, east, rotation));

        values.forEach((k, val) -> assertTargets(k, val, expectedMag, expectedAng));
    }

    private static void assertMagnitudesAngle(double north, double east, double rotation, double expectedAng) {
        Map<MecanumMath.WheelPosition, SwerveTargetValues> values = math.targetsForVelocities(magnitudeMap(north, east, rotation));

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
        Map<MecanumMath.WheelPosition, SwerveTargetValues> values,
        double frontLeftMag, double frontLeftAng,
        double backLeftMag, double backLeftAng,
        double frontRightMag, double frontRightAng,
        double backRightMag, double backRightAng
    ) {
        assertTargets(MecanumMath.WheelPosition.NORTH_WEST, values.get(MecanumMath.WheelPosition.NORTH_WEST), frontLeftMag, frontLeftAng);
        assertTargets(MecanumMath.WheelPosition.SOUTH_WEST, values.get(MecanumMath.WheelPosition.SOUTH_WEST), backLeftMag, backLeftAng);
        assertTargets(MecanumMath.WheelPosition.NORTH_EAST, values.get(MecanumMath.WheelPosition.NORTH_EAST), frontRightMag, frontRightAng);
        assertTargets(MecanumMath.WheelPosition.SOUTH_EAST, values.get(MecanumMath.WheelPosition.SOUTH_EAST), backRightMag, backRightAng);
    }

    private static void assertTargets(
        MecanumMath.WheelPosition k,
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
        public MecanumMath.WheelPosition getWheelPosition() {
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
