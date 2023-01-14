package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        FRONT_RIGHT(NORTH_EAST,1, 2, 1,2),
        FRONT_LEFT(NORTH_WEST,7, 8, 7,8),
        BACK_RIGHT(SOUTH_EAST,3, 4, 3,4),
        BACK_LEFT(SOUTH_WEST,5, 6, 5,6);

        private final MecanumMath.WheelPosition wheelPosition;
        private final int rotationId;
        private final int encoderChannelA;
        private final int encoderChannelB;
        private final int magnitudeId;

        private MotorPosition(MecanumMath.WheelPosition wheelPosition, int rotationId, int magnitudeId, int encoderChannelA, int encoderChannelB){
            this.wheelPosition =  wheelPosition;
            this.rotationId = rotationId;
            this.magnitudeId = magnitudeId;
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

    private static final double DEAD_ANGLE = 0.5;
    private static final double SPEED_REDUCTION = 0.3;

    private static final double ENCODER_COUNTS_PER_REVOLUTION = 497.0 * (5.0 / 6.0); //encoder counts multiplied by the gear ratio

    public final TalonSRX directionMotor;
    public final Encoder encoder;

    private final CANSparkMax magnitudeMotor;

    private double targetAngle = 135, targetMagnitude = 0;
    private boolean flipMagnitude;

    private double lastError = 0.0;
    private double errorAccum = 0.0;

    private Timer time = new Timer();
    private double lastTime = 0.0;
    private MotorPosition position;

    private String angleKey;

    public SwerveMotor(int rotationId, int magnitudeId, int encoderChannelA, int encoderChannelB){
        angleKey = "angle_key";

        directionMotor = new TalonSRX(rotationId);

        encoder = new Encoder(encoderChannelA, encoderChannelB);

        magnitudeMotor = new CANSparkMax(magnitudeId, CANSparkMaxLowLevel.MotorType.kBrushless);

        this.register();
    }

    public SwerveMotor(MotorPosition motorPosition){
        this(motorPosition.rotationId, motorPosition.magnitudeId, motorPosition.encoderChannelA, motorPosition.encoderChannelB);
        position = motorPosition;
        angleKey = motorPosition.name() + "_Angle";
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
        if(this.position == MotorPosition.BACK_RIGHT && magnitude > 0) System.out.printf("[target magnitude=%s]%n", magnitude);
        this.targetMagnitude = magnitude;
    }

    public void setTargetAngle(double angle) {

        targetAngle = angle;
        double currentWheelAngle = getWheelAngle();
        double angleDifference = getAngleDifference(currentWheelAngle, targetAngle);

        flipMagnitude = false;
        if(Math.abs(angleDifference) > 90){
            targetAngle -= 180;
            flipMagnitude = true;
        }

        if(targetAngle > 360 || targetAngle < 0){
            targetAngle -= 360 * Math.signum(targetAngle);
        }

    }

    public double getWheelAngle() {
        double angle = -encoder.get() * 360 / ENCODER_COUNTS_PER_REVOLUTION;

        angle %= 360.0;
        if(angle < 0) angle += 360.0;

        SmartDashboard.putNumber(angleKey, angle);

        return angle;
    }

    public static MotorPosition LOGGED_POSITION = MotorPosition.FRONT_RIGHT;


    boolean clockwiseToTarget;
    @Override public void periodic(){

        updateMagnitude();

        updateRotation();

    }

    private void updateMagnitude(){

        double magnitude = targetMagnitude * SPEED_REDUCTION;

        if(flipMagnitude) magnitude *= -1;

        magnitudeMotor.set(magnitude);

    }

    private void updateRotation(){
        if(this.targetMagnitude == 0){
            setDirectionMotor(0);
            return;
        }

        double currentAngle = getWheelAngle();
        double angleDifference = getAngleDifference(currentAngle, targetAngle);

        double speed = Math.signum(angleDifference);
        if(Math.abs(angleDifference) < 15) {
            speed = angleDifference / 30; //Math.pow(2, Math.abs(angleDifference));

            speed = Math.min(Math.abs(speed), 0.05) * Math.signum(speed);
        }
//
        if(Math.abs(angleDifference) < DEAD_ANGLE) speed = 0.0;

        setDirectionMotor(speed);
    }

    private void setDirectionMotor(double percent){
//        if(rotationMotor.getMotorOutputPercent() != percent){

        directionMotor.set(ControlMode.PercentOutput,percent);
    }

    public MotorPosition getPosition(){
        return position;
    }

    private double getAngleDifference(double from, double to) {
        double angleDifference = from - to;

        if (Math.abs(angleDifference) > 180) {
            angleDifference -= 360 * Math.signum(angleDifference);
        }

        return angleDifference;
    }

}
