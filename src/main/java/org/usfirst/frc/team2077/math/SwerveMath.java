package org.usfirst.frc.team2077.math;

import org.usfirst.frc.team2077.common.*;
import org.usfirst.frc.team2077.common.drivetrain.*;

import java.util.*;

import static java.lang.Math.*;

/**
 * Handle calculating the necessary magnitude and angle for a set of swerve wheels.
 * We may want to find a better equation or come up with our own we like better.
 *
 * Based on pdf found <a href="https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf">here</a>.
 *
 * <dl>
 *     <dt>Y</dt>
 *     <dd>Equates to the value of the {@link DriveStick}'s north value. May need to be inverted</dd>
 *     <dt>X</dt>
 *     <dd>Equates to the value of the {@link DriveStick}'s east value. Shouldn't need to be inverted</dd>
 *     <dt>Z</dt>
 *     <dd>Equations the value of the {@link DriveStick}'s rotation value.</dd>
 *     <dt>L - (wheelbase)</dt>
 *     <dt>Length between the center of a front wheel and the center of the back wheel on the same side</dt>
 *     <dt>W - (trackWidth)</dt>
 *     <dt>Width between the left and right wheels in the front/back</dt>
 * </dl>
 */
public class SwerveMath {
    private static final EnumMap<MecanumMath.WheelPosition, JointKey> WHEEL_TO_SIDES = new EnumMap<>(MecanumMath.WheelPosition.class);
    static {
        WHEEL_TO_SIDES.put(MecanumMath.WheelPosition.NORTH_WEST, new JointKey(RobotSide.FRONT, RobotSide.LEFT));
        WHEEL_TO_SIDES.put(MecanumMath.WheelPosition.SOUTH_WEST, new JointKey(RobotSide.BACK, RobotSide.LEFT));

        WHEEL_TO_SIDES.put(MecanumMath.WheelPosition.NORTH_EAST, new JointKey(RobotSide.FRONT, RobotSide.RIGHT));
        WHEEL_TO_SIDES.put(MecanumMath.WheelPosition.SOUTH_EAST, new JointKey(RobotSide.BACK, RobotSide.RIGHT));
    }
    private double wheelbase, trackWidth, R;

    public SwerveMath(double wheelbase, double trackWidth) {
        setWheelbase( wheelbase);
        setTrackWidth(trackWidth);
    }

    public void setWheelbase(double wheelbase) {
        this.wheelbase = wheelbase;
        updateR();
    }

    public void setTrackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
        updateR();
    }

    private void updateR() {
        this.R = Math.sqrt(pow(wheelbase, 2) + pow(trackWidth, 2));
    }

    private EnumMap<RobotSide, Double> createRobotSideValueMap(
        double forward, double strafe, double rotation
    ) {
        EnumMap<RobotSide, Double> multipliers = new EnumMap<>(RobotSide.class);

        // A
        multipliers.put(RobotSide.BACK, strafe - rotation * wheelbase / R);
        // B
        multipliers.put(RobotSide.FRONT, strafe + rotation * wheelbase / R);
        // C
        multipliers.put(RobotSide.RIGHT, forward - rotation * trackWidth / R);
        // D
        multipliers.put(RobotSide.LEFT, forward + rotation * trackWidth / R);

        return multipliers;
    }

    private SwerveTargetValues wheelTargets(
        Map<RobotSide, Double> values,
        RobotSide east,
        RobotSide north
    ) {
        double mag = Math.sqrt(pow(values.get(east), 2) + pow(values.get(north), 2));
        double ang = toDegrees(atan2(values.get(north), values.get(east)));

        if(Double.isNaN(mag)) mag = 0;
        if(Double.isNaN(ang)) ang = 0;
        else if(ang < 0) {
            ang += 360;
        }

        return new SwerveTargetValues(mag, ang);
    }

    public Map<MecanumMath.WheelPosition, SwerveTargetValues> targetsForVelocities(
        Map<MecanumMath.VelocityDirection, Double> targetMagnitudes
    ) {
        double north = targetMagnitudes.get(MecanumMath.VelocityDirection.NORTH);
        double strafe = targetMagnitudes.get(MecanumMath.VelocityDirection.EAST);
        double rotation = targetMagnitudes.get(MecanumMath.VelocityDirection.ROTATION);

        if(rotation == 0 && north == 0 && strafe == 0) {
            return Map.of(
                    MecanumMath.WheelPosition.NORTH_WEST, new SwerveTargetValues(0, 0),
                    MecanumMath.WheelPosition.NORTH_EAST, new SwerveTargetValues(0, 0),
                    MecanumMath.WheelPosition.SOUTH_WEST, new SwerveTargetValues(0, 0),
                    MecanumMath.WheelPosition.SOUTH_EAST, new SwerveTargetValues(0, 0)
            );
        }

        // Some mix of north/strafe/rotation
        Map<RobotSide, Double> valueMap = createRobotSideValueMap(north, strafe, rotation);

        Map<MecanumMath.WheelPosition, SwerveTargetValues> targetValues = new EnumMap<>(MecanumMath.WheelPosition.class);

        WHEEL_TO_SIDES.forEach((position, sides) -> targetValues.put(position, wheelTargets(valueMap, sides.east, sides.north)));

        double max = targetValues.values().stream().mapToDouble(SwerveTargetValues::getMagnitude).max().orElse(0d);
        if(max > 1) targetValues.values().forEach(val -> val.setMagnitude(val.getMagnitude() / max));

        return targetValues;
    }

    enum RobotSide {
        LEFT(true), RIGHT(true), FRONT(false), BACK(false);
        public final boolean east;

        RobotSide(boolean east) {
            this.east = east;
        }
    }

    private static class JointKey {
        final RobotSide east;
        final RobotSide north;

        JointKey(RobotSide north, RobotSide east) {
            this.east = east;
            this.north = north;
        }

        @Override public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            JointKey jointKey = (JointKey) o;
            return east == jointKey.east && north == jointKey.north;
        }

        @Override public int hashCode() {
            return Objects.hash(east, north);
        }

        @Override public String toString() {
            return "JointKey{" +
                   "east=" + east +
                   ", north=" + north +
                   '}';
        }
    }
}
