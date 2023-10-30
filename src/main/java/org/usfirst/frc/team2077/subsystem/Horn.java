package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.util.SmartDashNumber;


public class Horn {
    private final TalonSRX horn;

    private double percent = 0.6;

    private static SmartDashNumber SmartDashMaxHornPercent = new SmartDashNumber("Max Horn Percent", 0.6, true);
    private static SmartDashNumber SmartDashHornCountdown = new SmartDashNumber("Horn Countdown", 3.0, true);
    private double hornStartTime;
    private double hornTime;
    private boolean hornRunning;

    private int COUNTDOWN_TIME = 3;

    public Horn(int deviceNumber){
        horn = new TalonSRX(deviceNumber);

        SmartDashMaxHornPercent.onChange(this::setRunPercent);
    }

    public void setRunPercent(){
        this.percent = SmartDashMaxHornPercent.get();

    }

    public void run(){
        horn.set(TalonSRXControlMode.PercentOutput, percent);
        hornRunning = true;

        SmartDashHornCountdown.set(Math.max(COUNTDOWN_TIME - getHornTime(),0));

        //hornStartTime = Clock.getSeconds();
    }

    public void stop(){
        horn.set(TalonSRXControlMode.PercentOutput, 0);
        hornRunning = false;
        SmartDashHornCountdown.set((double)COUNTDOWN_TIME);
    }

    public boolean hasBeenRunningLongEnough(){
        return isRunning() && (getHornTime() >= COUNTDOWN_TIME);
    }

    public double getHornTime() {
        return hornTime = Clock.getSeconds() - hornStartTime;
    }

    public void setHornTime(double hornTime) {
        this.hornTime = hornTime;
    }

    public void setHornStartTime(double hornStartTime) {
        this.hornStartTime = hornStartTime;
    }
    public double getHornStartTime(){
        return hornStartTime;
    }

    public boolean isRunning(){
        return hornRunning;
    }
}
