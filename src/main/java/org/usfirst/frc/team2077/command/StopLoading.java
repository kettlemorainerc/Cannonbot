package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.*;
import org.usfirst.frc.team2077.common.command.*;
import org.usfirst.frc.team2077.subsystem.*;

public class StopLoading extends SelfDefinedCommand {
    private boolean executed;
    private final Cannon cannon;

    public StopLoading(RobotHardware hardware) {
        this.cannon = hardware.CANNON;
    }

    @Override public boolean isFinished() {
        return executed;
    }

    @Override public void initialize() {
        executed = false;
    }

    @Override public void execute() {
        executed = true;
        cannon.closeLoad();
    }
    @Override public void end(boolean interrupted) {}
}
