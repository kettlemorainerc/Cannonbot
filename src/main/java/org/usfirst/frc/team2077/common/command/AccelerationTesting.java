package org.usfirst.frc.team2077.common.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.VelocityDirection;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.AbstractChassis;
import org.usfirst.frc.team2077.common.math.Position;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.drivetrain.SwerveModule;
import org.usfirst.frc.team2077.subsystem.SwerveMotor;

import java.util.EnumMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

public class AccelerationTesting extends CommandBase {

    private static final double DEACCELERATION = 0.3; // in/s/s

    private double north = 0;
    private double east = 0;

    private double speed = 0;
    private double direction = 0;

    private boolean isFinished = false;
    private double startDeccel = 0;
    private double maxSpeed = 0;

    private double northMovement = 0;
    private double eastMovement = 0;

    private double lastTime = 0;

    private final SwerveChassis chassis;

    public AccelerationTesting(RobotHardware hardware, double north, double east) {
        this(hardware);

        this.north = north;
        this.east = east;
    }

    private AccelerationTesting(RobotHardware hardware) {
        chassis = hardware.getChassis();

        SmartDashboard.putNumber("MoveData", 0);

    }

    @Override
    public void initialize() {
        lastTime = Clock.getSeconds();

        speed = (double) chassis.getMaximumVelocity().get(VelocityDirection.FORWARD);

        direction = Math.atan2(north, east);
    }

    private double getDeltaTime(){
        double currentTime = Clock.getSeconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        return dt;
    }

    @Override
    public void execute() {

        Map<VelocityDirection, Double> currentVelocity = chassis.getVelocityMeasured();

        double dt = getDeltaTime();

        double northVelocity = currentVelocity.get(VelocityDirection.FORWARD);
        double eastVelocity = currentVelocity.get(VelocityDirection.STRAFE);

        northMovement += northVelocity * dt;
        eastMovement += eastVelocity * dt;

////        System.out.printf("[dt=%s][drive module=%s]", dt, Map.of(
////                WheelPosition.FRONT_RIGHT, chassis.driveModules.get(WheelPosition.FRONT_RIGHT).getVelocity() * dt,
////                WheelPosition.FRONT_LEFT, chassis.driveModules.get(WheelPosition.FRONT_LEFT).getVelocity() * dt,
////                WheelPosition.BACK_RIGHT, chassis.driveModules.get(WheelPosition.BACK_RIGHT).getVelocity() * dt,
////                WheelPosition.BACK_LEFT, chassis.driveModules.get(WheelPosition.BACK_LEFT).getVelocity() * dt
////        ));
////
////        System.out.println("nroth movmement: " + northVelocity * dt);
////
//        double distance = pythag(
//                north - northMovement,
//                east - eastMovement
//        );
//
//        double currentSpeed = pythag(
//                northVelocity,
//                eastVelocity
//        );
//
//        double stoppingDistance = Math.pow(currentSpeed, 2) / (2 * DEACCELERATION);
//
//        if(stoppingDistance > distance) {
//
//            double deccel = Math.pow(currentSpeed, 2) / (2 * distance);
//
//            speed = Math.max(speed - deccel, 0.1);
//
//        }
//
//
//
//        double targetForward = speed * Math.sin(direction);
//        double targetStrafe = speed * Math.cos(direction);
//        System.out.printf("[forward set=%s][strafe set=%s]%n", targetForward, targetStrafe);

        maxSpeed = Math.max(maxSpeed, currentVelocity.get(VelocityDirection.FORWARD));

        if(speed == 0 && Math.abs(currentVelocity.get(VelocityDirection.FORWARD)) < 0.1){
            double deltaTime = Clock.getSeconds() - startDeccel;

//            System.out.println("Accel: " + maxSpeed);
//            System.out.println("Deltatime: " + deltaTime);
//            System.out.println("Accel: " + maxSpeed * 2 / deltaTime);

            double deccel = maxSpeed * 2 * deltaTime / Math.pow(deltaTime, 2);
            SmartDashboard.putNumber("MoveSpeed", deccel);

            isFinished = true;
        }

        if(northMovement >= 20){

            speed = 0;
            startDeccel = Clock.getSeconds();

        }

        chassis.setVelocity(
            speed,
            0,
            0
        );

    }

    @Override
    public boolean isFinished() {
        System.out.println(isFinished);
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.halt();
    }

    private double pythag(double a, double b){
        return Math.sqrt(a * a + b * b);
    }

}
