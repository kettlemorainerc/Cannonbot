package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team2077.util.SmartDashNumber;


public class Horn {
    private final TalonSRX horn;
    private double maxPercent;

    private int m = 1;

    private static SmartDashNumber SmartDashMaxHornPercent = new SmartDashNumber("Max Horn Percent", 0.4, true);

    public Horn(int deviceNumber){
        horn = new TalonSRX(deviceNumber);

        SmartDashMaxHornPercent.onChange(this::updateMaxPercent);
    }

    public void updateMaxPercent(){
        setMaxPercent(SmartDashMaxHornPercent.get());
    }

    public void setMaxPercent(double percent){
        maxPercent = percent;
    }

    public void run(){
        horn.set(TalonSRXControlMode.PercentOutput, maxPercent);
    }

    public void stop(){
        horn.set(TalonSRXControlMode.PercentOutput, 0);
    }


}
