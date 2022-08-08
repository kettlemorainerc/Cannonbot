package org.usfirst.frc.team2077.subsystem;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.common.drivetrain.MecanumMath;
import org.usfirst.frc.team2077.drivetrain.SwerveModule;

import static org.usfirst.frc.team2077.common.drivetrain.MecanumMath.WheelPosition.*;

public class SwerveMotor implements Subsystem, SwerveModule {

    /* This is, essentially, what the public enum below does
    public static class MotorPosition {
      public static MotorPosition FRONT_RIGHT = new MotorPosition();
      public static MotorPosition FRONT_LEFT = new MotorPosition();
      public static MotorPosition BACK_RIGHT = new MotorPosition();
      public static MotorPosition BACK_LEFT = new MotorPosition();
    }
    */

    public enum MotorPosition{
        FRONT_RIGHT(NORTH_EAST,2, 1,2),
        FRONT_LEFT(NORTH_WEST,8,7,8),
        BACK_RIGHT(SOUTH_EAST,4,3,4),
        BACK_LEFT(SOUTH_WEST,5,5,6);

        private final MecanumMath.WheelPosition wheelPosition;
        private final int talonID;
        private final int encoderChannelA;
        private final int encoderChannelB;

        private MotorPosition(MecanumMath.WheelPosition wheelPosition, int talonId, int encoderChannelA, int encoderChannelB){
            this.wheelPosition =  wheelPosition;
            this.talonID = talonId;
            this.encoderChannelA = encoderChannelA;
            this.encoderChannelB = encoderChannelB;
        }

        public static MotorPosition of(MecanumMath.WheelPosition pos) {
            for(MotorPosition drivePos : values()) if (drivePos.wheelPosition == pos) return drivePos;

            throw new IllegalArgumentException("No DrivePosition found for wheel position: " + pos);
        }

    }

    private static final double Pvalue = 1.0;
    private static final double Ivalue = 0.0;
    private static final double Dvalue = 0.0;

    private static final double DEAD_ANGLE = 1.0;

    private static final double revsPerTick =  1D / 414D;


    private final Talon talonMotor;
    private final Encoder encoder;


    private double targetAngle = 0, targetMagnitude = 0;

    private double lastError = 0.0;
    private double errorAccum = 0.0;

    private Timer time = new Timer();
    private double lastTime = 0.0;

    public SwerveMotor(int talonID, int encoderChannelA, int encoderChannelB){
        talonMotor = new Talon/*SRX*/(talonID);
        encoder = new Encoder(encoderChannelA, encoderChannelB);
        encoder.reset();
    }

    public SwerveMotor(MotorPosition motorPosition){
        this(motorPosition.talonID, motorPosition.encoderChannelA, motorPosition.encoderChannelB);
    }

    @Override
    public void setTargetDegrees(double degrees) {
        setTargetAngle(degrees);
    }

    @Override
    public void setTargetMagnitude(double magnitude) {
        this.targetMagnitude = magnitude;
    }

    public void setTargetAngle(double angle){
        targetAngle = angle;
        double currentWheelAngle = getWheelAngle();
        double changeWheelAngle = currentWheelAngle - this.targetAngle;
        if(Math.abs(changeWheelAngle) > SwerveMotor.DEAD_ANGLE) {
            if(Math.abs(changeWheelAngle) > 90){ // Rotate Counter-Clockwise

            }
        }
    }

    public double getWheelAngle() {
        double angle = encoder.get() * revsPerTick;
        angle = angle % 360.0;

//        if (angle > 180.0) {
//            angle -= 360.0;
//        } else if (angle < -180.0) {
//            angle += 360.0;
//        }

        return angle;
    }


    // TODO: add comments
    public void goSwerve(double dir, long dt) {

        // angle error
        double deltaDir = getWheelAngle() - dir;

        // Map the delta onto the same -180 to 180 polar coordinate system
        // used by the FIRST joystick class.
        deltaDir = deltaDir % 360;
        if (deltaDir > 180.0) {
            deltaDir -= 360.0;
        } else if (deltaDir < -180.0) {
            deltaDir += 360.0;
        }

        // Start PID
        double error = deltaDir / 180.0;
        errorAccum += error * dt;

        // Set the direction motor to value specified by PID
        talonMotor.set(Pvalue * error + Ivalue * errorAccum + Dvalue * ((error - lastError / dt)));

        lastError = error;
    }

    @Override public void periodic() {
        long curTime = (long) time.get();
        long dt = (long) (curTime - lastTime);
        lastTime = curTime;
        // TODO: Don't use go swerve
//        goSwerve(targetAngle, dt);

        // TODO: Determine if we need to rotate to reach our targetAngle and which direction we want to rotate, if so
        double currentWheelAngle = getWheelAngle();
        double changeWheelAngle = currentWheelAngle - this.targetAngle;

        // TODO: update set the talon's request value
    }

}
