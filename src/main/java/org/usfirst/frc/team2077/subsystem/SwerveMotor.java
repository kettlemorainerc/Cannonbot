package org.usfirst.frc.team2077.subsystem;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        FRONT_RIGHT(NORTH_EAST,1, 1,2),
        FRONT_LEFT(NORTH_WEST,7,7,8),
        BACK_RIGHT(SOUTH_EAST,3,3,4),
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

    private static final double revsPerTick =  360.0 / 414.0;

    public final Talon talonMotor;
    public final Encoder encoder;

    private double targetAngle = 135, targetMagnitude = 0;
    private boolean flipMagnitude, clockwise = true;
    private double lastError = 0.0;
    private double errorAccum = 0.0;

    private Timer time = new Timer();
    private double lastTime = 0.0;
    private MotorPosition position;

    public SwerveMotor(int talonID, int encoderChannelA, int encoderChannelB){
        talonMotor = new Talon/*SRX*/(talonID);
        encoder = new Encoder(encoderChannelA, encoderChannelB);
        encoder.reset();

        this.register();
    }

    public SwerveMotor(MotorPosition motorPosition){
        this(motorPosition.talonID, motorPosition.encoderChannelA, motorPosition.encoderChannelB);
        position = motorPosition;
    }

    public void setMagnitude(double magnitude) {
        setTargetMagnitude(magnitude);
    }

    @Override
    public void setTargetDegrees(double degrees) {
        setTargetAngle(degrees);
    }

    @Override
    public void setTargetMagnitude(double magnitude) {
        this.targetMagnitude = magnitude;
    }

    public void setTargetAngle(double angle) {
//        if(
//            Math.abs(angle - targetAngle) < DEAD_ANGLE ||
//            Math.abs(angle - targetAngle + 180) < DEAD_ANGLE
//        ) return;

        targetAngle = angle;
        double currentWheelAngle = getWheelAngle();

        double diff = angleDiff(currentWheelAngle, targetAngle);

        if(Math.abs(diff) > 90){
            targetAngle -= 180;
//            diff -= 180 * Math.signum(diff);

        }

        if(targetAngle > 360 || targetAngle < 0){
            targetAngle -= 360 * Math.signum(targetAngle);
        }


//        if(sentinel == 0) {
//            System.out.println("[given=" + angle + "][target=" + targetAngle + "]");
//        }
    }

    public double getWheelAngle() {
        double angle = -encoder.get() * revsPerTick;

        angle = angle % 360.0;

        if(angle < 0){
            angle += 360.0;
        }

//        while(angle > 360) angle -= 360;
//        while(angle < 0) angle += 360;

        return angle;
    }

    public static MotorPosition LOGGED_POSITION = MotorPosition.FRONT_RIGHT;
    int localId = 0;
    int sentinel = 0;
    double prev;
//    @Override public void periodic() {
////        if(Math.abs(talonMotor.get() - prev) > .02) {
////            System.out.println("[recorded=" + talonMotor.get() + "][prev=" + prev + "]");
////        }
////        long curTime = (long) time.get();
////        long dt = (long) (curTime - lastTime);
////        lastTime = curTime;
//
//
//        double currentAngle = getWheelAngle();
//        double diff = angleDiff(currentAngle, targetAngle);
////        double targetAngle = this.targetAngle;
//
//        if(position == LOGGED_POSITION && (sentinel = (sentinel + 1) % 25) == 0) {
//            System.out.println("[target=" + targetAngle + "][current=" + currentAngle + "]");
//        }
//
//        if(targetMagnitude != 0) {
//            if(Math.abs(diff) < DEAD_ANGLE){
//                if(talonMotor.get() != 0) {
//                    if(position == LOGGED_POSITION) System.out.println("Should stop");
//                    this.talonMotor.set(prev = 0.0);
//                }
////                if((sentinel = (sentinel + 1) % 25) == 0) {
////                    System.out.println("[position=" + position + "][diff=" + diff + "] should be stopped");
////                }
//            }else{
//                double speed;
//                if(Math.abs(diff) < 40) speed = .15;
//                else speed = .75;
//                boolean invert = diff < 0 || diff > 90;
//                prev = speed;
//                if(Math.abs(talonMotor.get() - speed) > .02 || (speed == .15 && talonMotor.get() != .15)) {
//                    this.talonMotor.set(invert ? -speed : speed);
//                }
////                if((sentinel = (sentinel + 1) % 25) == 0) {
////                    System.out.println("[position=" + position + "][diff=" + diff + "][current=" + currentAngle + "][speed=" + speed + "]");
////                }
//            }
//        }else{
//            if(talonMotor.get() != 0) {
//                this.talonMotor.set(0.0);
//            }
//        }
//
//
////        else if(clockwise){
////            this.talonMotor.set(Math.abs(diff * 10) /  360);
////        }else{
////            this.talonMotor.set(-Math.abs(diff * 10) / 360);
////        }
////
////
////        // TODO: Don't use go swerve
////
//////        if(getWheelAngle() > DEAD_ANGLE){
//////            talonMotor.set(0.5);
//////        }
////
//////        goSwerve(targetAngle, dt);
////
////        // TODO: Determine if we need to rotate to reach our targetAngle and which direction we want to rotate, if so
////        double currentWheelAngle = getWheelAngle();
////        double changeWheelAngle = currentWheelAngle - this.targetAngle;
////
////        // TODO: update set the talon's request value
//    }

    boolean clockwiseToTarget;
    @Override public void periodic(){
        updateRotation();
    }

    private void updateRotation(){
        if(this.targetMagnitude == 0){
            setTalonMotor(0);
            return;
        }

        double currentAngle = getWheelAngle();
        double diff = angleDiff(currentAngle, targetAngle);

        boolean clockwiseToTarget = diff > 0;

        double speed = 1;
        if(Math.abs(diff) < 30) {
            speed = diff / 75;
            if(talonMotor.get() == 0 && speed < 0.15) speed = .15;
        }

        if(Math.abs(diff) < DEAD_ANGLE) speed = 0.0;

        if(clockwiseToTarget != this.clockwiseToTarget){
            speed *= -1;
        }

        this.clockwiseToTarget = clockwiseToTarget;

        setTalonMotor(speed);
    }

    private void setTalonMotor(double percent){
        if(talonMotor.get() != percent){
            talonMotor.set(percent);
        }
    }

    public MotorPosition getPosition(){
        return position;
    }

    private double angleDiff(double from, double to){
        double diff = from - to;

        if(Math.abs(diff) > 180){
            diff -= 360 * Math.signum(diff);
        }

        return diff;
    }

}
