package org.usfirst.frc.team2077.subsystem;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AirCompressor extends SubsystemBase {

    private final Relay relay;

    public AirCompressor(int channel) {

        relay = new Relay(channel, Relay.Direction.kForward);

    }

    public void turnOn(){relay.set(Relay.Value.kOn);}
    public void turnOff(){relay.set(Relay.Value.kOff);}

    public boolean isOn(){return relay.get() == Relay.Value.kOn;}

    @Override public void periodic() {

//        System.out.println(relay.get());

        boolean pressureIsAboveThreshold = false;
        if(isOn() && pressureIsAboveThreshold){
            turnOff();
        }
    }
}
