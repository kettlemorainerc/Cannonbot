/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.common.math;

import org.usfirst.frc.team2077.common.VelocityDirection;

import java.util.*;

import static org.usfirst.frc.team2077.common.VelocityDirection.*;

public class Position extends EnumMap<VelocityDirection, Double> {

    public Position() {
        this(new double[]{0, 0, 0});
    }

    public Position(Position from) {
        this(new double[]{from.get(FORWARD), from.get(STRAFE), from.get(ROTATION)});
    }

    public Position(double[] position) {
        super(VelocityDirection.class);
        put(FORWARD, position[FORWARD.ordinal()]);
        put(STRAFE, position[STRAFE.ordinal()]);
        put(ROTATION, position[ROTATION.ordinal()]);
    }

    public void moveAbsolute(double north, double east, double rotation) {
        compute(FORWARD, (k, val) -> val + north);
        compute(STRAFE, (k, v) -> v + east);
        compute(ROTATION, (k, v) -> v + rotation);
    }

    public void moveRelative(double north, double east, double rotation) {
        double distance = Math.sqrt(north * north + east * east); // along curve
        double directionStraight = Math.atan2(east, north);
        double directionCurve = 0;

        if (distance != 0 && rotation != 0) {
            double radians = Math.toRadians(rotation);
            double radius = distance / radians;
            double ahead = radius * Math.sin(radians);
            double side = radius - radius * Math.cos(radians);
            directionCurve = Math.atan2(side, ahead);
            distance = Math.sqrt(side * side + ahead * ahead);
        }

        double direction = Math.toRadians(get(ROTATION)) + directionStraight + directionCurve;
        moveAbsolute(distance * Math.cos(direction), distance * Math.sin(direction), rotation);
    }

    public void moveRelative(Map<VelocityDirection, Double> velocities, double changeInTime) {
        moveRelative(
              velocities.get(FORWARD) * changeInTime,
              velocities.get(STRAFE) * changeInTime,
              velocities.get(ROTATION) * changeInTime
        );
    }

    public Map<VelocityDirection, Double> distanceAbsolute(Position origin) {
        Map<VelocityDirection, Double> distanceTo = this.clone();

        distanceTo.compute(FORWARD, (k, v) -> v - origin.getOrDefault(FORWARD, 0d));
        distanceTo.compute(STRAFE, (k, v) -> v - origin.getOrDefault(STRAFE, 0d));
        distanceTo.compute(ROTATION, (k, v) -> v - origin.getOrDefault(ROTATION, 0d));

        return distanceTo;
    }


    public Map<VelocityDirection, Double> distanceRelative(Position origin) {
        Map<VelocityDirection, Double> absolute = distanceAbsolute(origin);

        double rotation = absolute.get(ROTATION);
        double forward = absolute.get(FORWARD);
        double strafe = absolute.get(STRAFE);

        double distance = Math.sqrt(forward * forward + strafe * strafe);

        double direction = Math.atan2(strafe, forward) - Math.toRadians(origin.get(ROTATION));
        double directionCurve = 0;

        if (distance != 0 && rotation != 0) {
            double radians = Math.toRadians(rotation);
            double radius = distance / 2 / Math.sin(radians / 2);
            double ahead = radius * Math.sin(radians);
            double side = radius - radius * Math.cos(radians);
            directionCurve = Math.atan2(side, ahead);
            distance = radius * radians;
        }
        double directionStraight = direction - directionCurve;

        Map<VelocityDirection, Double> distanceTo = new EnumMap<>(VelocityDirection.class);

        // cos as 0 degrees is forward
        distanceTo.put(FORWARD, distance * Math.cos(directionStraight));
        distanceTo.put(STRAFE, distance * Math.sin(directionStraight));
        distanceTo.put(ROTATION, rotation);

        return distanceTo;
    }

    public void set(double north, double east, double heading) {
        put(FORWARD, north);
        put(STRAFE, east);
        setHeading(heading);
    }

    public void setHeading(double heading) {
        put(ROTATION, heading);
    }

    public double[] get() {
        return new double[]{get(FORWARD), get(STRAFE), get(ROTATION)};
    }

    public Position copy() {
        return new Position(this);
    }

    @Override
    public String toString() {
        return "N:" + Math.round(get(FORWARD) * 10.) / 10. + "in E:" + Math.round(get(STRAFE) * 10.) / 10. + "in A:" + Math.round(get(ROTATION) * 10.) / 10. + "deg";
    }

    private static String toString(double[] doubleArray) {
        StringBuffer sb = new StringBuffer();
        for (Double d : doubleArray) {
            sb.append(Math.round(d * 10.) / 10.);
            sb.append(" ");
        }
        return sb.toString();
    }
}
