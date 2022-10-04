package org.usfirst.frc.team2077.command;

import edu.wpi.first.wpilibj.Relay;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.commands.SelfDefinedCommand;
import org.usfirst.frc.team2077.subsystem.AirCompressor;
import org.usfirst.frc.team2077.subsystem.Cannon;

public class ToggleAirCompressor extends SelfDefinedCommand {
    private boolean finished = true;

    private final AirCompressor airCompressor;

    public ToggleAirCompressor(RobotHardware hardware) {
        this.airCompressor = hardware.AIRCOMPRESSOR;
        this.addRequirements(airCompressor);
    }

    @Override public boolean isFinished() {
        return finished;
    }

    @Override public void initialize() { finished = false; }

    @Override public void execute() {

        System.out.println("Button is pressed");

        if(airCompressor.isOn()){
            airCompressor.turnOff();
        }else{
            airCompressor.turnOn();
        }

        finished = true;

    }
    @Override public void end(boolean interrupted) {}
}
