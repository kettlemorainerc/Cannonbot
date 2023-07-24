package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.DriveModuleIF;
import org.usfirst.frc.team2077.drivetrain.SwerveModule;
import org.usfirst.frc.team2077.util.SmartDashNumber;

public class SwerveMotor implements Subsystem, SwerveModule, DriveModuleIF {

    public enum MotorPosition{
        FRONT_RIGHT(WheelPosition.FRONT_RIGHT,1, 2, 1,2, 0, 5800),// MAX_RPM: 5800
        FRONT_LEFT(WheelPosition.FRONT_LEFT,7, 8, 7,8, 11, 5600),// Max: 5600
        BACK_RIGHT(WheelPosition.BACK_RIGHT,3, 4, 3,4, 12, 5700),// Max: 5700
        BACK_LEFT(WheelPosition.BACK_LEFT,5, 6, 5,6, 13, 5700);// Max 5700,

        private final WheelPosition wheelPosition;
        private final int directionId, magnitudeId;
        private final int encoderChannelA, encoderChannelB;
        private final int hallEffectChannel;
        private final int maxRPM;

        MotorPosition(WheelPosition wheelPosition, int directionId, int magnitudeId, int encoderChannelA, int encoderChannelB, int hallEffectChannel, int maxRPM){
            this.wheelPosition =  wheelPosition;
            this.directionId = directionId;
            this.magnitudeId = magnitudeId;
            this.encoderChannelA = encoderChannelA;
            this.encoderChannelB = encoderChannelB;
            this.hallEffectChannel = hallEffectChannel;
            this.maxRPM = maxRPM;
        }

        public static MotorPosition of(WheelPosition pos) {
            for(MotorPosition drivePos : values()) if (drivePos.wheelPosition == pos) return drivePos;

            throw new IllegalArgumentException("No DrivePosition found for wheel position: " + pos);
        }
    }

    public static final double WHEEL_RADIUS = 2.0;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * Math.PI * 2;
    public static final double GEAR_RATIO = 20.0 / 3.0;
    private static final double ENCODER_COUNTS_PER_REVOLUTION = 497.0 * (5.0 / 6.0); //encoder counts multiplied by the gear ratio
    private static final double SPEED_MULTIPLIER = 0.3;

    public static final int MAX_RPM = 5600;

    private static final double Pvalue = 0.02;
    private static final double Ivalue = 0.002600;
    private static final double Dvalue = 0.0;

    private final CANSparkMax magnitudeMotor;
    public final TalonSRX directionMotor;
    public final Encoder encoder;
    private final PIDController directionPID;
    private DigitalInput hallEffectSensor;

    private MotorPosition position;
    private String angleKey;

    private double targetAngle = 135, targetMagnitude = 0;
    private boolean flipMagnitude;

    private Timer time = new Timer();
    private double lastTime = 0.0;

    private static SmartDashNumber smartdashP = new SmartDashNumber("P in PID", 0.0, true);
    private static SmartDashNumber smartdashI = new SmartDashNumber("I in PID", 0.0, true);

    public SwerveMotor(int directionId, int magnitudeId, int encoderChannelA, int encoderChannelB, int hallEffectChannel){
        angleKey = "angle_key";

        directionMotor = new TalonSRX(directionId);
        encoder = new Encoder(encoderChannelA, encoderChannelB);
        directionPID = new PIDController(Pvalue, Ivalue, Dvalue);
        directionPID.setSetpoint(0.0);

        smartdashP.onChange(this::updatePID);
        smartdashI.onChange(this::updatePID);

        magnitudeMotor = new CANSparkMax(magnitudeId, CANSparkMaxLowLevel.MotorType.kBrushless);

        hallEffectSensor = new DigitalInput(hallEffectChannel);

        this.register();
    }

    public SwerveMotor(MotorPosition motorPosition){
        this(motorPosition.directionId, motorPosition.magnitudeId, motorPosition.encoderChannelA, motorPosition.encoderChannelB, motorPosition.hallEffectChannel);
        position = motorPosition;
        angleKey = motorPosition.name() + "_Angle";
    }

    private void updatePID(){
        directionPID.setPID(smartdashP.get(), smartdashI.get(), 0);
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
        magnitude *= MAX_RPM / (double) position.maxRPM;
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

    public void setDirectionPercent(double percent){
        directionMotor.set(TalonSRXControlMode.PercentOutput, percent);
    }

    public double getWheelAngle() {
        double angle = -encoder.get() * 360 / ENCODER_COUNTS_PER_REVOLUTION;

        angle %= 360.0;
        if(angle < 0) angle += 360.0;

        SmartDashboard.putNumber(angleKey, angle);

        return angle;
    }

    public static MotorPosition LOGGED_POSITION = MotorPosition.FRONT_RIGHT;


    @Override public void periodic(){
        updateMagnitude();

        updateRotation();
    }

    private void updateMagnitude(){
        double magnitude = targetMagnitude * SPEED_MULTIPLIER;

        if(flipMagnitude) magnitude = -magnitude;

        magnitudeMotor.set(magnitude);
    }

    private void updateRotation(){
        if(this.targetMagnitude == 0){
            setDirectionMotor(0);
            return;
        }

        double currentAngle = getWheelAngle();
        double angleDifference = getAngleDifference(currentAngle, targetAngle);

        double speed = -directionPID.calculate(angleDifference);

        if(Math.abs(speed) > 1.0){
            speed = Math.signum(speed);
        }

        setDirectionMotor(speed);
    }

    private void setDirectionMotor(double percent){
        directionMotor.set(ControlMode.PercentOutput, percent);
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

    public boolean getHallValue(){
        return hallEffectSensor.get();
    }

    public double getMaximumSpeed() {
        return (MAX_RPM / GEAR_RATIO) / (60 / WHEEL_CIRCUMFERENCE);
    }

    @Override
    public void setVelocity(double velocity) {

    }

    @Override
    public WheelPosition getWheelPosition() {
        return null;
    }

    @Override
    public double getVelocity() {
        double rawRPM = magnitudeMotor.getEncoder().getVelocity();
        return (rawRPM / GEAR_RATIO) / (60 / WHEEL_CIRCUMFERENCE);
    }

    @Override
    public double getDistance() {
        return 0;
    }

    @Override
    public void resetDistance() {}

}
