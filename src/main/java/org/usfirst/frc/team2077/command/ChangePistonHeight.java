package org.usfirst.frc.team2077.command;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
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

    private final Talon screw;
    private final double motorSpeed = 0.75;
    private PistonDirection direction;

    //TODO: make sure this works;
    public ChangePistonHeight(RobotHardware hardware, PistonDirection direction){
        screw = hardware.PISTON;;
        this.direction = direction;
//        addRequirements(screw);
    }

    @Override
    public void initialize() {
        screw.set(motorSpeed * direction.getDirection());
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        screw.set(0);
    }
}
