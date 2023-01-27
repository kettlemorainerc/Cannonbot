package org.usfirst.frc.team2077.math;

import org.apache.commons.math3.fitting.leastsquares.*;
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.optim.ConvergenceChecker;
import org.apache.commons.math3.util.Pair;
import org.usfirst.frc.team2077.common.*;
import org.usfirst.frc.team2077.common.drivetrain.MecanumMath;
import org.usfirst.frc.team2077.drivetrain.SwerveModule;

import java.util.*;

import static java.lang.Math.*;

/**
 * Handle calculating the necessary magnitude and angle for a set of swerve wheels.
 * We may want to find a better equation or come up with our own we like better.
 *
 * Based on pdf found <a href="https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf">here</a>.
 * Forward kinematics from <a href="https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/joe.2014.0241">this paper</a>
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
    private static final EnumMap<WheelPosition, Multiplier> WHEEL_MULTIPLIERS = new EnumMap<>(WheelPosition.class);
    private static final EnumMap<WheelPosition, JointKey> WHEEL_TO_SIDES = new EnumMap<>(WheelPosition.class);
    static {
        WHEEL_TO_SIDES.put(WheelPosition.FRONT_LEFT, new JointKey(RobotSide.FRONT, RobotSide.LEFT));
        WHEEL_TO_SIDES.put(WheelPosition.BACK_LEFT, new JointKey(RobotSide.BACK, RobotSide.LEFT));
        WHEEL_MULTIPLIERS.put(WheelPosition.FRONT_LEFT, new Multiplier(-1, 1));
        WHEEL_MULTIPLIERS.put(WheelPosition.BACK_LEFT, new Multiplier(-1, -1));

        WHEEL_TO_SIDES.put(WheelPosition.FRONT_RIGHT, new JointKey(RobotSide.FRONT, RobotSide.RIGHT));
        WHEEL_TO_SIDES.put(WheelPosition.BACK_RIGHT, new JointKey(RobotSide.BACK, RobotSide.RIGHT));
        WHEEL_MULTIPLIERS.put(WheelPosition.FRONT_RIGHT, new Multiplier(1, 1));
        WHEEL_MULTIPLIERS.put(WheelPosition.BACK_RIGHT, new Multiplier(1, -1));
    }

    private static double pythag(double a, double b) {
        return sqrt(pow(a, 2) + pow(b, 2));
    }

    private final RealMatrix forward;
    private final RealMatrix inverse;
    private double wheelbase, trackWidth, radius;

    public SwerveMath(double wheelbase, double trackWidth) {
        setWheelbase(wheelbase);
        setTrackWidth(trackWidth);

        double halfY = wheelbase / 2;
        double halfX = trackWidth / 2;
        var positions = WheelPosition.values().length;

        this.inverse = new BlockRealMatrix(new double[positions * 2][3]);
        int i = 0;
        for(WheelPosition position : WheelPosition.values()) {
            var mults = WHEEL_MULTIPLIERS.get(position);

            inverse.setRow(i++, new double[]{1, 0, mults.y * halfY});
            inverse.setRow(i++, new double[]{0, 1, mults.x * halfX});
        }

        SingularValueDecomposition dec = new SingularValueDecomposition(inverse);
        this.forward = dec.getSolver().getInverse();
    }

    public void setWheelbase(double wheelbase) {
        this.wheelbase = wheelbase;
        updateRadius();
    }

    public void setTrackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
        updateRadius();
    }

    private void updateRadius() {
        this.radius = pythag(wheelbase, trackWidth);
    }

    private EnumMap<RobotSide, Double> createRobotSideValueMap(
            double forward, double strafe, double rotation
    ) {
        EnumMap<RobotSide, Double> multipliers = new EnumMap<>(RobotSide.class);

        // A
        multipliers.put(RobotSide.BACK, strafe - rotation * wheelbase / radius);
        // B
        multipliers.put(RobotSide.FRONT, strafe + rotation * wheelbase / radius);
        // C
        multipliers.put(RobotSide.RIGHT, forward - rotation * trackWidth / radius);
        // D
        multipliers.put(RobotSide.LEFT, forward + rotation * trackWidth / radius);

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

    public Map<WheelPosition, SwerveTargetValues> targetsForVelocities(
            Map<MecanumMath.VelocityDirection, Double> targetMagnitudes
    ) {
        double north = targetMagnitudes.get(MecanumMath.VelocityDirection.NORTH);
        double strafe = targetMagnitudes.get(MecanumMath.VelocityDirection.EAST);
        double rotation = targetMagnitudes.get(MecanumMath.VelocityDirection.ROTATION);

        if(rotation == 0 && north == 0 && strafe == 0) {
            return Map.of(
                    WheelPosition.FRONT_LEFT, new SwerveTargetValues(0, 0),
                    WheelPosition.FRONT_RIGHT, new SwerveTargetValues(0, 0),
                    WheelPosition.BACK_LEFT, new SwerveTargetValues(0, 0),
                    WheelPosition.BACK_RIGHT, new SwerveTargetValues(0, 0)
            );
        }

        // Some mix of north/strafe/rotation
        Map<RobotSide, Double> valueMap = createRobotSideValueMap(north, strafe, rotation);

        Map<WheelPosition, SwerveTargetValues> targetValues = new EnumMap<>(WheelPosition.class);

        WHEEL_TO_SIDES.forEach((position, sides) -> targetValues.put(position, wheelTargets(valueMap, sides.east, sides.north)));

        double max = targetValues.values().stream().mapToDouble(SwerveTargetValues::getMagnitude).max().orElse(0d);
        if(max > 1) targetValues.values().forEach(val -> val.setMagnitude(val.getMagnitude() / max));

        return targetValues;
    }

    public Map<MecanumMath.VelocityDirection, Double> velocitiesForTargets(
        Map<WheelPosition, ? extends SwerveModule> targets
    ) {
        WheelPosition[] values = WheelPosition.values();

        var stateMatrix = new BlockRealMatrix(values.length * 2, 1);
        int idx = 0;
        for(WheelPosition position : values) {
            SwerveModule motor = targets.get(position);
            double velocity = motor.getVelocity();
            double angle = toRadians(motor.getWheelAngle());

            stateMatrix.setEntry(idx++, 0, velocity * cos(angle));
            stateMatrix.setEntry(idx++, 0, velocity * sin(angle));
        }

        var product = forward.multiply(stateMatrix);

        return Map.of(
                MecanumMath.VelocityDirection.NORTH, product.getEntry(0, 0),
                MecanumMath.VelocityDirection.EAST, product.getEntry(1, 0),
                MecanumMath.VelocityDirection.ROTATION, toDegrees(product.getEntry(2, 0))
        );
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

    private static class Multiplier {
        int x, y;
        Multiplier(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }
}
