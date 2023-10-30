package org.usfirst.frc.team2077.subsystem;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashString;

public class AirCompressor extends SubsystemBase {

    private final Relay relay;
    private boolean desiredState = false;

    private static SmartDashString compressorState = new SmartDashString("compressorState", "", true);

    public AirCompressor(int channel) {
        relay = new Relay(channel, Relay.Direction.kForward);
    }

    public boolean getDesiredState(){
        return desiredState;
    }

    public void toggleDesired(){
        desiredState = !desiredState;
    }

    private void turnOn(){relay.set(Relay.Value.kOn);}
    private void turnOff(){relay.set(Relay.Value.kOff);}

    public boolean isRunning(){return relay.get() == Relay.Value.kOn;}

    @Override public void periodic() {

//        boolean pressureIsAboveThreshold = false;
//        if(isOn() && pressureIsAboveThreshold){

        if(!desiredState){
            if(isRunning()) turnOff();
            return;
        }

        turnOn();

        double cumulativeVelocity = RobotHardware.getInstance().getChassis().getVelocitySet().values().stream().reduce(0.0, (a, b) -> a + b);

        if (cumulativeVelocity > 0.1) {
            turnOff();
        }

        compressorState.set(
            "Desired:\t" + (desiredState? "on" : "off") +
            "\nState:\t" + (isRunning()? "on" : "off")
        );

    }
}
