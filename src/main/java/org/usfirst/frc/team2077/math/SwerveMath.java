package org.usfirst.frc.team2077.math;

import org.apache.commons.math3.fitting.leastsquares.*;
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.stat.regression.GLSMultipleLinearRegression;
import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.Pair;
import org.usfirst.frc.team2077.common.*;
import org.usfirst.frc.team2077.common.drivetrain.*;
import org.usfirst.frc.team2077.common.math.Matrix;
import org.usfirst.frc.team2077.drivetrain.SwerveModule;
import org.usfirst.frc.team2077.subsystem.SwerveMotor;

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
    private static final EnumMap<MecanumMath.WheelPosition, Multiplier> WHEEL_MULTIPLIERS = new EnumMap<>(MecanumMath.WheelPosition.class);
    private static final EnumMap<MecanumMath.WheelPosition, JointKey> WHEEL_TO_SIDES = new EnumMap<>(MecanumMath.WheelPosition.class);
    static {
        WHEEL_TO_SIDES.put(MecanumMath.WheelPosition.NORTH_WEST, new JointKey(RobotSide.FRONT, RobotSide.LEFT));
        WHEEL_TO_SIDES.put(MecanumMath.WheelPosition.SOUTH_WEST, new JointKey(RobotSide.BACK, RobotSide.LEFT));
        WHEEL_MULTIPLIERS.put(MecanumMath.WheelPosition.NORTH_WEST, new Multiplier(-1, 1));
        WHEEL_MULTIPLIERS.put(MecanumMath.WheelPosition.SOUTH_WEST, new Multiplier(-1, -1));

        WHEEL_TO_SIDES.put(MecanumMath.WheelPosition.NORTH_EAST, new JointKey(RobotSide.FRONT, RobotSide.RIGHT));
        WHEEL_TO_SIDES.put(MecanumMath.WheelPosition.SOUTH_EAST, new JointKey(RobotSide.BACK, RobotSide.RIGHT));
        WHEEL_MULTIPLIERS.put(MecanumMath.WheelPosition.NORTH_EAST, new Multiplier(1, 1));
        WHEEL_MULTIPLIERS.put(MecanumMath.WheelPosition.SOUTH_EAST, new Multiplier(1, -1));
    }

    private final RealMatrix forward;
    private final RealMatrix inverse;

    public SwerveMath(double wheelbase, double trackWidth) {
        double halfY = wheelbase / 2;
        double halfX = trackWidth / 2;
        var positions = MecanumMath.WheelPosition.values().length;

        inverse = new BlockRealMatrix(new double[positions * 2][3]);
        int i = 0;
        for(MecanumMath.WheelPosition position : MecanumMath.WheelPosition.values()) {
            var mults = WHEEL_MULTIPLIERS.get(position);

            inverse.setRow(i++, new double[]{1, 0, mults.y * halfY});
            inverse.setRow(i++, new double[]{0, 1, mults.x * halfX});
        }

        SingularValueDecomposition dec = new SingularValueDecomposition(inverse);
        forward = dec.getSolver().getInverse();
    }

    public Map<MecanumMath.WheelPosition, SwerveTargetValues> targetsForVelocities(
            Map<MecanumMath.VelocityDirection, Double> targetMagnitudes
    ) {
        var speeds = new Array2DRowRealMatrix(new double[][] {
            {targetMagnitudes.get(MecanumMath.VelocityDirection.NORTH)},
            {targetMagnitudes.get(MecanumMath.VelocityDirection.EAST)},
            {targetMagnitudes.get(MecanumMath.VelocityDirection.ROTATION)},
        }, false);

        var moduleMatrix = inverse.multiply(speeds);

        var ret = new EnumMap<MecanumMath.WheelPosition, SwerveTargetValues>(MecanumMath.WheelPosition.class);
        int i = 0;
        System.out.println(moduleMatrix);
        for(MecanumMath.WheelPosition p : MecanumMath.WheelPosition.values()) {
            double x = moduleMatrix.getEntry(i++, 0);
            double y = moduleMatrix.getEntry(i++, 0);

            double speed = Math.hypot(x, y);
            double angle = toDegrees(Math.atan2(y / speed, x / speed));
            if(angle < 0) angle += 360;

            ret.put(p, new SwerveTargetValues(speed, angle));
        }

        return ret;
    }

    public Map<MecanumMath.VelocityDirection, Double> velocitiesForTargets(
        Map<MecanumMath.WheelPosition, SwerveModule> targets
    ) {
        MecanumMath.WheelPosition[] values = MecanumMath.WheelPosition.values();

        var stateMatrix = new BlockRealMatrix(values.length * 2, 1);
        int idx = 0;
        for(MecanumMath.WheelPosition position : values) {
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
