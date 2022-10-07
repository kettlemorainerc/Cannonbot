package org.usfirst.frc.team2077.command;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.commands.RepeatedCommand;

public class TestingMotorIds extends RepeatedCommand {

    private final Victor talon;
    public TestingMotorIds(Victor talon){
        this.talon = talon;
    }

    @Override
    public void initialize() {
        talon.set(0.5);
    }

    public void execute() {
    }

    public void end(boolean interrupted) {
        talon.set(0);
    }
}
