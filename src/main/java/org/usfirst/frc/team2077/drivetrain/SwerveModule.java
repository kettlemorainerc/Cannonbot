package org.usfirst.frc.team2077.drivetrain;

import org.usfirst.frc.team2077.common.drivetrain.DriveModuleIF;

public interface SwerveModule extends DriveModuleIF {

    public void setTargetDegrees(double degrees);

    public void setTargetMagnitude(double magnitude);

    public double getWheelAngle();
}
