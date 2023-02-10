/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.common.drivetrain;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.common.VelocityDirection;
import org.usfirst.frc.team2077.common.math.*;

import java.util.*;

/*
Notes on units:

    DriveChassis motion units are defined to be inches or inches/second for chassis movement,
    and degrees or degrees/second for chassis rotation.

    Exceptions are setVelocityPercent() and setRotationPercent() methods, which operate on values ranging
    from -1.0 to +1.0, which are internally mapped to the maximum negative and positive
    inches/second or degrees/second values possible for the chassis within WheelModule limits.

    DriveModule wheel travel units are defined to be inches or inches/second.
*/

public interface DriveChassisIF extends Subsystem {

    Position getPosition();


    /**
     * Maximum velocities as determined by {@link DriveModuleIF} capabilities.
     * {@link #setVelocity} and {@link #setVelocity} should limit input values
     * to these values (positive or negative).
     *
     * @return {north, east, rotation} Units are inches and degrees per second.
     */
    Map<VelocityDirection, Double> getMaximumVelocity();

    /**
     * Minimum velocities necessary to ensure motion, as determined by
     * {@link DriveModuleIF} capabilities.
     * {@link #setVelocity} and {@link #setVelocity} should limit input values
     * to these values (positive or negative).
     *
     * @return {north, east, rotation} Units are inches and degrees per second.
     */
    Map<VelocityDirection, Double> getMinimumVelocity();

    /**
     * Set maximum allowable linear (N/S or E/W) acceleration and deceleration.
     * Rotational values are set in proportion to the ratios in {@link #getMaximumVelocity}.
     * Used to control wheelspin or lockup, depending on surface traction.
     *
     * @param linearAcceleration In units of G (acceleration of gravity).
     * @param linearDeceleration In units of G.
     */
    public void setGLimits(double linearAcceleration, double linearDeceleration);

    /**
     * Acceleration and deceleration limits in units inches/second/second and degrees/second/second.
     *
     * @return {linearAcceleration, linearDeceleration, rotationalAcceleration, rotationalDeceleration}
     */
    AccelerationLimits getAccelerationLimits();

    /**
     * @param north              In inches per second.
     * @param east               In inches per second.
     * @param clockwise          In degrees per second.
     * @param accelerationLimits As returned by {@link #getAccelerationLimits()}.
     */
    void setVelocity(double north, double east, double clockwise, AccelerationLimits accelerationLimits);

    /**
     * @param north     In inches per second.
     * @param east      In inches per second.
     * @param clockwise In degrees per second.
     */
    void setVelocity(double north, double east, double clockwise);

    /**
     * @param north              In inches per second.
     * @param east               In inches per second.
     * @param accelerationLimits As returned by {@link #getAccelerationLimits()}.
     */
    void setVelocity(double north, double east, AccelerationLimits accelerationLimits);

    /**
     * @param north In inches per second.
     * @param east  In inches per second.
     */
    void setVelocity(double north, double east);

    /**
     * @param clockwise          In degrees per second.
     * @param accelerationLimits As returned by {@link #getAccelerationLimits()}.
     */
    void setRotation(double clockwise, AccelerationLimits accelerationLimits);

    /**
     * @param clockwise In degrees per second.
     */
    void setRotation(double clockwise);

    /**
     * @param north     Fraction of nominal maximum in range -1.0 to 1.0.
     * @param east      Fraction of nominal maximum in range -1.0 to 1.0.
     * @param clockwise Fraction of nominal maximum in range -1.0 to 1.0.
     */
    void setVelocityPercent(double north, double east, double clockwise);

    /**
     * @param north Fraction of nominal maximum in range -1.0 to 1.0.
     * @param east  Fraction of nominal maximum in range -1.0 to 1.0.
     */
    void setVelocityPercent(double north, double east);

    /**
     * @param clockwise Fraction of nominal maximum in range -1.0 to 1.0.
     */
    void setRotationPercent(double clockwise);

    /**
     * Set velocity to zero immediately, ignoring any deceleration limits.
     */
    void halt();

    /**
     * Velocity set point.
     *
     * @return {north, east, rotation} Units are inches and degrees per second.
     */
    Map<VelocityDirection, Double> getVelocitySet();

    /**
     * Internal velocity set point, after adjustment for acceleration and velocity limits.
     *
     * @return {north, east, rotation} Units are inches and degrees per second.
     */
    Map<VelocityDirection, Double> getVelocityCalculated();

    /**
     * Measured velocity based on motor or wheel encoders if present.
     * May be affected by acceleration limits or calculated from relative settings.
     *
     * @return {north, east, rotation} Units are inches and degrees per second.
     */
    Map<VelocityDirection, Double> getVelocityMeasured();
}