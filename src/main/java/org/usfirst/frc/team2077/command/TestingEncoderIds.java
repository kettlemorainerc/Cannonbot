package org.usfirst.frc.team2077.command;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.subsystem.SwerveMotor;

public class TestingEncoderIds extends RepeatedCommand {

    private final SwerveMotor swerveMotor;
    public TestingEncoderIds(SwerveMotor swerveMotor){
        this.swerveMotor = swerveMotor;
    }

    public void execute() {
        ;
    }

    public void end(boolean interrupted) {
        System.out.println(
                swerveMotor.getWheelAngle() + "\nAt position: " + swerveMotor.getPosition() + "\nEncoder value: " + swerveMotor.encoder.get()
        );
    }
}
