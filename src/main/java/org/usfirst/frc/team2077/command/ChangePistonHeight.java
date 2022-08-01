package org.usfirst.frc.team2077.command;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.commands.RepeatedCommand;

public class ChangePistonHeight extends RepeatedCommand {

    public enum PistonDirection{
        UP(1),
        DOWN(-1);
        private final int direction;
        PistonDirection(int direction) {
            this.direction = direction;
        }
        public int getDirection(){
            return direction;
        }
    }

    private final TalonSRX screw;
    private final double motorSpeed = 0.05;
    private PistonDirection direction;

    //TODO: make sure this works;
    public ChangePistonHeight(RobotHardware hardware, PistonDirection direction){
        screw = new TalonSRX(/*TODO: find port*/99);
        screw.configFactoryDefault();
        this.direction = direction;
    }

    public void execute() {
        screw.set(
            TalonSRXControlMode.PercentOutput, motorSpeed * direction.getDirection()
        );
    }

    public void end(boolean interrupted) {
        screw.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
