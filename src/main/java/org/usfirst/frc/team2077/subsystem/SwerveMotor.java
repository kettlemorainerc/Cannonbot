package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.BetterCanSparkMax;
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

    public enum MotorPosition {
        //MAX 5800
        FRONT_RIGHT(WheelPosition.FRONT_RIGHT, 1, 2, 1, 2, 0), // Max: 5600
        FRONT_LEFT(WheelPosition.FRONT_LEFT, 7, 8, 7, 8, 10), // Max: 5700
        BACK_RIGHT(WheelPosition.BACK_RIGHT, 3, 4, 3, 4, 11), // Max 5700,
        BACK_LEFT(WheelPosition.BACK_LEFT, 5, 6, 5, 6, 12);

        private final WheelPosition wheelPosition;
        private final int directionId;
        private final int encoderChannelA;
        private final int encoderChannelB;
        private final int magnitudeId;
        private final int hallEffectChannel;

        MotorPosition(
              WheelPosition wheelPosition,
              int directionId,
              int magnitudeId,
              int encoderChannelA,
              int encoderChannelB,
              int hallEffectChannel
        ) {
            this.wheelPosition = wheelPosition;
            this.directionId = directionId;
            this.magnitudeId = magnitudeId;
            this.encoderChannelA = encoderChannelA;
            this.encoderChannelB = encoderChannelB;
            this.hallEffectChannel = hallEffectChannel;
        }

        public static MotorPosition of(WheelPosition pos) {
            for(MotorPosition drivePos : values()) if(drivePos.wheelPosition == pos) return drivePos;

            throw new IllegalArgumentException("No DrivePosition found for wheel position: " + pos);
        }

    }

    public static final double MAX_RPM = 5600;

    private static final double Pvalue = 1.0;
    private static final double Ivalue = 0.0;
    private static final double Dvalue = 0.0;

    private static final double PERCENT_OF_MAX_SPEED = 0.3;

    private static final double GEAR_RATIO = 6.67;

    private static final double WHEEL_RADIUS = 2; //in
    private static final double WHEEL_CIRCUMFERENCE = (2 * Math.PI * WHEEL_RADIUS);

    private static final double ENCODER_COUNTS_PER_REVOLUTION = 497.0 * (5.0 / 6.0); // encoder counts multiplied by the gear ratio

    public final TalonSRX directionMotor;
    public final Encoder encoder;

    private final BetterCanSparkMax magnitudeMotor;

    private double targetAngle = 135, targetMagnitude = 0;

    private boolean flipMagnitude;

    private Timer time = new Timer();
    private double lastTime = 0.0;
    private MotorPosition position;
    private DigitalInput hallEffectSensor;

    private PIDController pid = new PIDController(0, 0, 0);

    private static SmartDashNumber p = new SmartDashNumber("P in PID", 0.0, true);
    private static SmartDashNumber i = new SmartDashNumber("l in PID", 0.0, true);

    private String angleKey;

    public SwerveMotor(
          int directionId,
          int magnitudeId,
          int encoderChannelA,
          int encoderChannelB,
          int hallEffectChannel
    ) {
        angleKey = "angle_key";

        directionMotor = new TalonSRX(directionId);

        encoder = new Encoder(encoderChannelA, encoderChannelB);

        magnitudeMotor = new BetterCanSparkMax(magnitudeId, CANSparkMaxLowLevel.MotorType.kBrushless);

        p.onChange(this::updatePID);

        this.register();

        hallEffectSensor = new DigitalInput(hallEffectChannel);

    }

    private void updatePID(){
        pid.setPID(p.get(), i.get(), 0.0);
    }

    public SwerveMotor(MotorPosition motorPosition) {
        this(
              motorPosition.directionId,
              motorPosition.magnitudeId,
              motorPosition.encoderChannelA,
              motorPosition.encoderChannelB,
              motorPosition.hallEffectChannel
        );
        position = motorPosition;
        angleKey = motorPosition.name() + "_Angle";
    }

    @Override public void setTargetDegrees(double degrees) {
        setTargetAngle(degrees);
    }

    @Override public void setTargetMagnitude(double magnitude) {
        this.targetMagnitude = magnitude;
    }

    public void setTargetAngle(double angle) {

        targetAngle = angle;
        double currentWheelAngle = getWheelAngle();
        double angleDifference = getAngleDifference(currentWheelAngle, targetAngle);

        flipMagnitude = false;
        if(Math.abs(angleDifference) > 90) {
            targetAngle -= 180;
            flipMagnitude = true;
        }

        if(targetAngle > 360 || targetAngle < 0) {
            targetAngle -= 360 * Math.signum(targetAngle);
        }

    }

    public void setDirectionPercent(double percent) {
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

    @Override public void periodic() {
        updateMagnitude();

        updateRotation();
    }

    private void updateMagnitude() {

        double magnitude = targetMagnitude * PERCENT_OF_MAX_SPEED;

        if(flipMagnitude) magnitude = -magnitude;
        magnitudeMotor.setTargetVelocity(magnitude);

    }

    private void updateRotation() {

        double currentAngle = getWheelAngle();
        double angleDifference = getAngleDifference(currentAngle, targetAngle);

        double percent = -pid.calculate(angleDifference);

        setDirectionMotor(percent);

//        if(this.targetMagnitude == 0) {
//            setDirectionMotor(0);
//            return;
//        }
//
//        double currentAngle = getWheelAngle();
//        double angleDifference = getAngleDifference(currentAngle, targetAngle);
//
//        double speed = Math.signum(angleDifference);
//        if(Math.abs(angleDifference) < 15) {
//            speed = angleDifference / 30; // Math.pow(2, Math.abs(angleDifference));
//
//            speed = Math.min(Math.abs(speed), 0.05) * Math.signum(speed);
//        }
//        //
//        if(Math.abs(angleDifference) < DEAD_ANGLE) speed = 0.0;
//
//        setDirectionMotor(speed);
    }

    private void setDirectionMotor(double percent) {
        directionMotor.set(ControlMode.PercentOutput, percent);
    }

    public MotorPosition getPosition() {
        return position;
    }

    private double getAngleDifference(
          double from,
          double to
    ) {
        double angleDifference = from - to;

        if(Math.abs(angleDifference) > 180) {
            angleDifference -= 360 * Math.signum(angleDifference);
        }

        return angleDifference;
    }

    public boolean getHallValue() {
        return hallEffectSensor.get();
    }

    public double getMaximumSpeed() {
        return (MAX_RPM / GEAR_RATIO) / (60 / WHEEL_CIRCUMFERENCE);
    }

    @Override public void setVelocity(double velocity) {}

    @Override public WheelPosition getWheelPosition() {
        return null;
    }

    @Override public double getVelocity() {

        double rawRPM = magnitudeMotor.getEncoder()
                                      .getVelocity();

        return (rawRPM / GEAR_RATIO) / (60 / WHEEL_CIRCUMFERENCE);

    }

    @Override public double getDistance() {
        return 0;
    }

    @Override public void resetDistance() {

    }

}
