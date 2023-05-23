package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.subsystem.SwerveMotor;

import java.util.Arrays;
import java.util.Map;

public class ZeroSwerveMotors extends RepeatedCommand {

    private RobotHardware hardware;

    public ZeroSwerveMotors(RobotHardware hardware){
        this.hardware = hardware;
    }

    @Override
    public void execute() {

        SwerveMotor.rotateFirst = true;

        for(WheelPosition position : WheelPosition.values()){

            SwerveMotor module = hardware.getWheel(position);
            module.setTargetAngle(0);
            //This is set to give the wheels a non-zero magnitude, but the wheels do not spin because of rotateFirst
            module.setTargetMagnitude(0.1);

        }

    }

    @Override
    public void end(boolean interrupted) {
        SwerveMotor.rotateFirst = false;
        hardware.getChassis().halt();
    }
}
