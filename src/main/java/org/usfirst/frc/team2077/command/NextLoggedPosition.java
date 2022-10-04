package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.common.commands.SelfDefinedCommand;
import org.usfirst.frc.team2077.common.drivetrain.MecanumMath;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.subsystem.SwerveMotor;

public class NextLoggedPosition extends SelfDefinedCommand {
    private boolean finished;

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        switch (SwerveMotor.LOGGED_POSITION) {
            case BACK_RIGHT:
                SwerveMotor.LOGGED_POSITION = SwerveMotor.MotorPosition.BACK_LEFT;
                SwerveChassis.LOGGED_POSITION = MecanumMath.WheelPosition.SOUTH_WEST;
                return;
            case BACK_LEFT:
                SwerveMotor.LOGGED_POSITION = SwerveMotor.MotorPosition.FRONT_LEFT;
                SwerveChassis.LOGGED_POSITION = MecanumMath.WheelPosition.NORTH_WEST;
                return;
            case FRONT_RIGHT:
                SwerveMotor.LOGGED_POSITION = SwerveMotor.MotorPosition.BACK_RIGHT;
                SwerveChassis.LOGGED_POSITION = MecanumMath.WheelPosition.SOUTH_EAST;
                return;
            case FRONT_LEFT:
                SwerveMotor.LOGGED_POSITION = SwerveMotor.MotorPosition.FRONT_RIGHT;
                SwerveChassis.LOGGED_POSITION = MecanumMath.WheelPosition.NORTH_EAST;
                return;
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}
