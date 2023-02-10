/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.common.command;

import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.common.*;
import org.usfirst.frc.team2077.common.drivetrain.*;
import org.usfirst.frc.team2077.common.math.*;
import org.usfirst.frc.team2077.common.math.AccelerationLimits.*;

import java.util.*;

public class Move extends CommandBase {
	public static final double ACCELERATION_G_LIMIT = .1;
	public static final double DECELERATION_G_LIMIT = .3;
	private final AbstractChassis<?> chassis;
	private final double[] distanceTotal; // {north, east, rotation} (signed)
	private final int method; // 1 2 or 3 (#args to setVelocity/setRotation)

	private double[] fast = new double[3]; // {north, east, rotation} (signed)
	public double[] slow = new double[3]; // {north, east, rotation} (signed)
	private AccelerationLimits acceleration; // like getAccelerationLimits, but scaled

	private double[] distanceRemaining = new double[3]; // {north, east, rotation} (signed)
	private boolean[] finished = new boolean[3]; // {north, east, rotation}

	private Position origin;

	public Move(HardwareRequirements<?, ?> hardware, double north, double east, double rotation) {
		this(hardware, north, east, rotation, 3, hardware.getPosition(), hardware.getHeading());
	}

	public Move(HardwareRequirements<?, ?> hardware, double north, double east) {
		this(hardware, north, east, 0, 2, hardware.getPosition());
	}

	public Move(HardwareRequirements<?, ?> hardware, double rotation) {
		this(hardware, 0, 0, rotation, 1, hardware.getHeading());
	}

	private Move(HardwareRequirements<?, ?> hardware, double north, double east, double rotation, int method, Subsystem... requirements) {
		addRequirements(requirements);
		this.chassis = hardware.getChassis();

//		distanceTotal_ = new double[]{north, east * .68, rotation * 7 / 8}; //fudged values for the multipliers
		distanceTotal = new double[]{north, east, rotation}; //fudged values for the multipliers
		this.method = method;
	}


	@Override
	public void initialize() {

		Map<VelocityDirection, Double> max = chassis.getMaximumVelocity();
		Map<VelocityDirection, Double> min = chassis.getMinimumVelocity();

		// scale factors for north/east/rotation by fraction of maximum velocity
		double[] scale = {
			Math.abs(distanceTotal[0]) / max.get(VelocityDirection.FORWARD),
			Math.abs(distanceTotal[1]) / max.get(VelocityDirection.STRAFE),
			Math.abs(distanceTotal[2]) / max.get(VelocityDirection.ROTATION)
		};
		double maxScale = Math.max(scale[0], Math.max(scale[1], scale[2]));

		double[] sign = new double[3];
		for(int i = 0; i < 3; i++){

			VelocityDirection axis = VelocityDirection.values()[i];

			scale[i] /= maxScale; // 0 - 1

			sign[i] = Math.signum(distanceTotal[i]);

			// scale speeds and acceleration/deceleration
			fast[i] = Math.max(
				min.get(axis), max.get(axis) * scale[i]
			) * sign[i];// don't let maximum scale below minimum

			slow[i] = min.get(axis) * sign[i]; // don't scale below minimum

			distanceRemaining[i] = distanceTotal[i];
			finished[i] = Math.abs(distanceRemaining[0]) == 0.;

		}

		acceleration = new AccelerationLimits(ACCELERATION_G_LIMIT, DECELERATION_G_LIMIT, chassis, scale);

		origin = new Position(chassis.getPosition());

	}

	@Override
	public void execute() {

		Map<VelocityDirection, Double> currentVelocity = chassis.getVelocityCalculated();

		Map<VelocityDirection, Double> distanceTraveled = (new Position(chassis.getPosition())).distanceRelative(
			origin);

		double[] newVelocity = new double[3];

		for(int i = 0; i < 3; i++) {
			VelocityDirection axis = VelocityDirection.values()[i];

			distanceRemaining[i] = distanceTotal[i] - distanceTraveled.get(axis);

			double distanceToStop = Math.abs(currentVelocity.get(axis)) /
					(2 * acceleration.get(axis, Type.DECELERATION));// exact absolute value per physics (should be Vo/2a I think)

			distanceToStop += Math.max(
				distanceToStop * .05,
				Math.abs(currentVelocity.get(axis)) * .04
			); // pad just a bit to avoid overshoot

			boolean useSlow = Math.abs(distanceRemaining[i]) <= distanceToStop; // slow down within padded stopping distance

			if(finished[i]) continue;

			newVelocity[i] = useSlow ? slow[i] : fast[i];

		}

//		boolean s = Math.abs(distanceTotal[2]) > 0 ? useSlow[2] : (useSlow[0] && useSlow[1]);

//		for(int i = 0; i < 3; i++) {
//			newVelocity[i] = finished[i] ? 0. : s ? this.slow[i] : fast[i];
//		}

		chassis.setVelocity(newVelocity[0], newVelocity[1], newVelocity[2], acceleration);

	}

	@Override
	public boolean isFinished() {
		for(int i = 0; i < 3; i++) {
			finished[i] = finished[i] || (Math.signum(distanceRemaining[i]) != Math.signum(distanceTotal[i]));
		}
		return finished[0] && finished[1] && finished[2];
//		return Math.abs(distanceTotal[2]) > 0 ? finished[2] : (finished[0] && finished[1]);
	}
}
