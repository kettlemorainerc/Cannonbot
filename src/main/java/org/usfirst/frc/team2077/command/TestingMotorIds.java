package org.usfirst.frc.team2077.command;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.commands.RepeatedCommand;

public class TestingMotorIds extends RepeatedCommand {

    private final BaseMotorController talon;
    public TestingMotorIds(BaseMotorController talon){
        this.talon = talon;
    }

    @Override
    public void initialize() {
    }

    public void execute() {
        talon.set(ControlMode.PercentOutput,0.5);
    }

    public void end(boolean interrupted) {
        talon.set(ControlMode.PercentOutput,0);
    }
}
