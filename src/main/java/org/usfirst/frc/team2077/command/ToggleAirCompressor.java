package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.SelfDefinedCommand;
import org.usfirst.frc.team2077.subsystem.AirCompressor;

public class ToggleAirCompressor extends SelfDefinedCommand {
    private boolean finished = true;

    private final AirCompressor airCompressor;

    public ToggleAirCompressor(RobotHardware hardware) {
        this.airCompressor = hardware.airCompressor;
        this.addRequirements(airCompressor);
    }

    @Override public boolean isFinished() {
        return finished;
    }

    @Override public void initialize() { finished = false; }

    @Override public void execute() {

        System.out.println("Button is pressed");

        airCompressor.toggleDesired();

        finished = true;

    }
    @Override public void end(boolean interrupted) {}
}
