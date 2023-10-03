package org.usfirst.frc.team2077.subsystem;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.util.SmartDashNumber;

public class AirCompressor extends SubsystemBase {

    private final Relay relay;

    private static SmartDashNumber RunningCompressor = new SmartDashNumber("RunningCompressor", 0.0, true);

    public AirCompressor(int channel) {

        relay = new Relay(channel, Relay.Direction.kForward);

    }

    public void turnOn(){relay.set(Relay.Value.kOn);}
    public void turnOff(){relay.set(Relay.Value.kOff);}

    public boolean isOn(){return relay.get() == Relay.Value.kOn;}

    @Override public void periodic() {

//        boolean pressureIsAboveThreshold = false;
//        if(isOn() && pressureIsAboveThreshold){

        double cumulativeVelocity = RobotHardware.getInstance().getChassis().getVelocitySet().values().stream().reduce(0.0, (a, b) -> a + b);

        if (cumulativeVelocity > 0.1) {
            turnOff();
        }

        RunningCompressor.set( isOn()? 1.0 : 0.0 );

    }
}
