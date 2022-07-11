package org.usfirst.frc.team2077.math;

import org.junit.jupiter.api.*;
import org.usfirst.frc.team2077.common.drivetrain.*;

import java.util.*;

import static org.junit.jupiter.api.Assertions.*;

public class SwerveMathTests {
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

        assertMagnitudeResults(
            0, 0, 1,
            1, 90,
            1, 270,
            1, 90,
            1, 270
        );

        assertMagnitudeResults(
            0, 0, -1,
            1, 270,
            1, 90,
            1, 270,
            1, 90
        );
    }

    private static void assertMagnitudes(double north, double east, double rotation, double expectedMag, double expectedAng) {
        Map<MecanumMath.WheelPosition, SwerveTargetValues> values = math.targetsFor(magnitudeMap(north, east, rotation));

        values.forEach((k, val) -> assertTargets(k, val, expectedMag, expectedAng));
    }

    private static void assertMagnitudesAngle(double north, double east, double rotation, double expectedAng) {
        Map<MecanumMath.WheelPosition, SwerveTargetValues> values = math.targetsFor(magnitudeMap(north, east, rotation));

        values.forEach((k, val) -> assertEquals(expectedAng, val.getAngle(), .1, k + " angle unexpected"));
    }
    
    private static void assertMagnitudeResults(
        double north, double east, double rotation,
        double frontLeftMag, double frontLeftAng,
        double backLeftMag, double backLeftAng,
        double frontRightMag, double frontRightAng,
        double backRightMag, double backRightAng
    ) {
        var targets = math.targetsFor(magnitudeMap(north, east, rotation));
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
}
