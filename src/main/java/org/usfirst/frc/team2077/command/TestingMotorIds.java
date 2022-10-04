package org.usfirst.frc.team2077.command;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.commands.RepeatedCommand;

public class TestingMotorIds extends RepeatedCommand {

    private final Talon talon;
    public TestingMotorIds(Talon talon){
        this.talon = talon;
    }

    public void execute() {
        talon.set(0.2);
    }

    public void end(boolean interrupted) {
        talon.set(0);
    }
}
