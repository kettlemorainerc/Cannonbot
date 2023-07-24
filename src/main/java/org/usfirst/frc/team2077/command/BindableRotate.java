package org.usfirst.frc.team2077.command;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.usfirst.frc.team2077.*;
import org.usfirst.frc.team2077.common.command.*;
import org.usfirst.frc.team2077.subsystem.*;

public class BindableRotate extends RepeatedCommand {
    private boolean finished;
    private final Cannon cannon;
    private int dir;

    public BindableRotate(RobotHardware hardware, int dir) {
        this.cannon = hardware.cannon;
//        this.addRequirements(cannon);
        this.dir = dir;
    }


    @Override public void initialize() {

    }

    @Override public void execute() {

//        System.out.println("TEST+ "+cannon.getCurrentPressure());
        RobotHardware.getInstance().getChassis().setRotationPercent(dir * 0.5);


    }
    @Override public void end(boolean interrupted) {
        RobotHardware.getInstance().getChassis().halt();
    }
}
