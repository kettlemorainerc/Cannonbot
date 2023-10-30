package org.usfirst.frc.team2077.command;

import edu.wpi.first.wpilibj2.command.button.*;
import org.usfirst.frc.team2077.*;
import org.usfirst.frc.team2077.common.command.*;
import org.usfirst.frc.team2077.subsystem.*;

public class LaunchCannon extends SelfDefinedCommand {
    private boolean finished;
    private final Cannon cannon;
    private final JoystickButton button;

    public LaunchCannon(RobotHardware hardware, JoystickButton boundTo) {
        this.cannon = hardware.cannon;
        this.button = boundTo;
    }

    @Override public boolean isFinished() {
        return true;
    }

    @Override public void initialize() {
        finished = false;
    }

    @Override public void execute() {
        Horn horn = RobotHardware.getInstance().horn;

        if(horn.hasBeenRunningLongEnough()) {
            cannon.launch();
            horn.setHornTime(0);
        }
    }

    @Override public void end(boolean interrupted) {
//        cannon.load();
//        System.out.println("released");
    }
}
