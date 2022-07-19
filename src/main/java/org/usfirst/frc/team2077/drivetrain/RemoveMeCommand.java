package org.usfirst.frc.team2077.drivetrain;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.commands.RepeatedCommand;
import org.usfirst.frc.team2077.common.drivetrain.MecanumMath;


public class RemoveMeCommand extends RepeatedCommand {

    private SwerveModule motor1;

    public RemoveMeCommand(RobotHardware hardware) {
//    public RemoveMeCommand() {
//        motor1 = (hardware.getRotationModule(MecanumMath.WheelPosition.NORTH_EAST));
    }

    @Override
    public void execute() {
        motor1.setTargetMagnitude(.5);
        motor1.setTargetDegrees(45);
    }


    @Override
    public void initialize() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public enum Speed{
        NONE, LOW, MID, HIGH;
    }
}
