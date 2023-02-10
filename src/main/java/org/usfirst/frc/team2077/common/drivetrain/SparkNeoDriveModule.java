
package org.usfirst.frc.team2077.common.drivetrain;

import com.revrobotics.*;
import org.usfirst.frc.team2077.common.WheelPosition;

public class SparkNeoDriveModule extends CANSparkMax implements DriveModuleIF {
    private static final double WHEEL_GEAR_RATIO = 10.714, WHEEL_RADIUS = 4;
    private static final double LAUNCHER_GEAR_RATIO = 1, LAUNCHER_WHEEL_RADIUS = 2;
    private static final int MAX_SHOOTER_RPM = 5400, MAX_WHEEL_RPM = 4000;//4500 for MAX_WHEEL_RPM, 5400 for shooter
    private static final boolean USE_SOFTWARE_PID = true;

    public enum DrivePosition {
        FRONT_RIGHT(WheelPosition.FRONT_RIGHT, 2, true, WHEEL_GEAR_RATIO, WHEEL_RADIUS, MAX_WHEEL_RPM, 1e-4, 1e-6, 0),
        BACK_RIGHT(WheelPosition.BACK_RIGHT, 1, true, WHEEL_GEAR_RATIO, WHEEL_RADIUS, MAX_WHEEL_RPM, 1.1e-4, 1e-6, 0),
        BACK_LEFT(WheelPosition.BACK_LEFT, 4, false, WHEEL_GEAR_RATIO, WHEEL_RADIUS, MAX_WHEEL_RPM, 1.4e-4, 1e-6, 0),
        FRONT_LEFT(WheelPosition.FRONT_LEFT, 3, false, WHEEL_GEAR_RATIO, WHEEL_RADIUS, MAX_WHEEL_RPM, 1.4e-4, 1e-6, 0),

        SHOOTER(null, 5, false, LAUNCHER_GEAR_RATIO, LAUNCHER_WHEEL_RADIUS, MAX_SHOOTER_RPM);;

        public final double gearRatio;
        public final double radius;
        public final double maxRPM;
        public final int ID;
        public final boolean INVERSE;
        public final double P, I, D;
        public final WheelPosition WHEEL_POSITION;

        DrivePosition(WheelPosition position, int id, boolean inverse, double gearRatio, double radius, double maxRPM) {
            this(position, id, inverse, gearRatio, radius, maxRPM, 1.4E-4, 1e-6, 0);
        }

        DrivePosition(WheelPosition position, int id, boolean inverse, double gearRatio, double radius, double maxRPM, double p, double i, double d) {
            WHEEL_POSITION = position;
            ID = id;
            INVERSE = inverse;
            this.gearRatio = gearRatio;
            this.radius = radius;
            this.maxRPM = maxRPM;
            this.P = p;
            this.I = i;
            this.D = d;
        }

        public static DrivePosition forWheelPosition(WheelPosition pos) {
            for (DrivePosition drivePos : values()) if (drivePos.WHEEL_POSITION == pos) return drivePos;

            throw new IllegalArgumentException("No DrivePosition found for wheel position: " + pos);
        }
    }

    //6 inch wheels on rnd bot
    private final SparkMaxPIDController pidController;
    private final RelativeEncoder encoder;
    private double setPoint;
    public final double circumference;
    public final double maxRPM;
    private final DrivePosition position;

    public SparkNeoDriveModule(final DrivePosition pos) {
        super(pos.ID, MotorType.kBrushless);
        this.position = pos;
        maxRPM = pos.maxRPM;
        circumference = pos.radius * 2 * Math.PI;
        pidController = this.getPIDController();
        encoder = this.getEncoder();

        if (USE_SOFTWARE_PID || position == DrivePosition.SHOOTER) {
            pidController.setP(position.P);
            pidController.setI(position.I);
            pidController.setD(position.D);
            pidController.setIZone(0);
            pidController.setFF(0);
            pidController.setOutputRange(-1, 1);
        }

    }

    public SparkNeoDriveModule(WheelPosition position) {
        this(DrivePosition.forWheelPosition(position));
    }

    public DrivePosition getPosition() {
        return position;
    }

    @Override
    public double getMaximumSpeed() {
        return (maxRPM / position.gearRatio) / (60 / (2 * Math.PI * position.radius));
    }

    public void setVelocity(final double velocity) {
        //convert from inches/second to rpm
        setPoint = velocity * position.gearRatio * 60 / circumference;
//        if (setPoint > maxRPM) {
//            setPoint = maxRPM;
//        }
        setRPM(setPoint > maxRPM ? maxRPM : setPoint);
    }

    @Override
    public WheelPosition getWheelPosition() {
        return position.WHEEL_POSITION;
    }

    public void setRPM(double rpm) {
        setPoint = Math.min(rpm, maxRPM);
        if (position.INVERSE) {
            pidController.setReference(-setPoint, ControlType.kVelocity);
        } else {
            pidController.setReference(setPoint, ControlType.kVelocity);
        }
//        if (position == DrivePosition.SHOOTER) System.out.println("SET RPM: " + setPoint);//PRINTS TOO OFTEN WHEN IN goIfShooterSpeedReady
    }

    public double getRPM() {
        final double velocity = encoder.getVelocity();
        if (position.INVERSE) {
            return -velocity;
        } else {
            return velocity;
        }
    }

    public double getSetPoint() {
        if (position.INVERSE) {
            return -setPoint;
        }
        return setPoint;
    }

    private static final double SECONDS_IN_MINUTE = 60;
    public double getVelocity() {
        double motorRPM = encoder.getVelocity();
        double motorRPS = motorRPM / SECONDS_IN_MINUTE;
        double wheelRPS = motorRPS / position.gearRatio;
        double wheelInPS = wheelRPS * circumference;
        final double velocity = wheelInPS;

        if (position.INVERSE) {
            return -velocity;
        } else {
            return velocity;
        }
    }

    public double getDistance() {
        return encoder.getPosition() / position.gearRatio * circumference;
    }

    public void resetDistance() {
        encoder.setPosition(0);
    }
}