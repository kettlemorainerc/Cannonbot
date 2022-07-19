package org.usfirst.frc.team2077.command;

import edu.wpi.first.wpilibj2.command.button.*;
import org.usfirst.frc.team2077.*;
import org.usfirst.frc.team2077.common.commands.*;
import org.usfirst.frc.team2077.subsystem.*;

public class LaunchCannon extends SelfDefinedCommand {
    private boolean finished;
    private final Cannon cannon;
    private final JoystickButton button;

    public LaunchCannon(RobotHardware hardware, JoystickButton boundTo) {
        this.cannon = hardware.CANNON;
        this.button = boundTo;
    }

    @Override public boolean isFinished() {
        return finished;
    }

    @Override public void initialize() {
        finished = false;
        System.out.println("Launching: " + System.currentTimeMillis());
        cannon.launch();
        System.out.println("Launched: " + System.currentTimeMillis());
    }

    @Override public void execute() {
        finished = !button.get();
    }

    @Override public void end(boolean interrupted) {
        cannon.closeLaunch();
    }
}
