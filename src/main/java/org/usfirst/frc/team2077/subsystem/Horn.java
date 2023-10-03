package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team2077.util.SmartDashNumber;


public class Horn {
    private final TalonSRX horn;

    private double percent = 0.6;

    private static SmartDashNumber SmartDashMaxHornPercent = new SmartDashNumber("Max Horn Percent", 0.6, true);

    public Horn(int deviceNumber){
        horn = new TalonSRX(deviceNumber);

        SmartDashMaxHornPercent.onChange(this::setRunPercent);
    }

//    public void updateMaxPercent(){
//        setRunPercent();
//    }

    public void setRunPercent(){
        this.percent = SmartDashMaxHornPercent.get();
    }

    public void run(){
        horn.set(TalonSRXControlMode.PercentOutput, percent);
    }

    public void stop(){
        horn.set(TalonSRXControlMode.PercentOutput, 0);
    }


}
