package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.*;
import org.usfirst.frc.team2077.common.command.*;
import org.usfirst.frc.team2077.subsystem.*;

public class LoadLauncher extends SelfDefinedCommand {
    private boolean executed;
    private final Cannon cannon;

    public LoadLauncher(RobotHardware hardware) {
        this.cannon = hardware.CANNON;
        this.addRequirements(cannon);
    }

    @Override public boolean isFinished() {
        return executed;
    }

    @Override public void initialize() {
        executed = false;
    }

    @Override public void execute() {
        cannon.load();
//        System.out.println("TEST+ "+cannon.getCurrentPressure());
        executed = true;
    }
    @Override public void end(boolean interrupted) {}
}
