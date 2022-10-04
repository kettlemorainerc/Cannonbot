package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.led.FireAnimation;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;

import java.awt.*;

public class Cannon extends SubsystemBase {
    private final Relay launchValve;
    private final Solenoid loadValve;
    private final PressureSensor pressure;

    private boolean allowedToFire = false;

//    private Solenoid targetedSolenoid;

    private int delay;
    private Runnable task;

    enum FiringTankPressures {//MUST BE IN ORDER BY VOLTAGE
        EMPTY(2.0, "empty, it's safe to put away", Color.GREEN),
        PSI_10(2, "ready to fire short range", Color.ORANGE),
        PSI_11(2, "ready to fire short range", Color.ORANGE),
        PSI_15(2, "pressing up for normal shot", Color.ORANGE),
        PSI_20(2, "ready for a normal shot", Color.ORANGE),
        PSI_30(2, "tank is over-pressure, leak before firing", Color.RED),
        PSI_40(2, "tank is over-pressure, do not fire, system should lock button", Color.RED);

        public final double pressure_voltage;
        public final String pressure_description;
        public final Color pressure_warning_color;

        FiringTankPressures(double voltage, String message, Color tankSafetyColor){
            pressure_voltage = voltage;
            pressure_description = message;
            pressure_warning_color = tankSafetyColor;
        }

    }

    public Cannon(Solenoid loadValve, Relay launchValve, PressureSensor pressure) {
        this.launchValve = launchValve;
        this.loadValve = loadValve;
//        launchValve.setPulseDuration(.03);
        this.pressure = pressure;
    }

    public double getCurrentPressure() {return pressure.getCurrentPressure();}
    public void updateActionsOnPressureByVoltage() {
        //Determine Enum
//        this.allowedToFire
        double currentPressure = pressure.getCurrentPressure();
        for(int i=0; i < FiringTankPressures.values().length; i++) {
            if(FiringTankPressures.values()[i].pressure_voltage <= currentPressure){
                if(FiringTankPressures.values()[i].pressure_warning_color == Color.GREEN || FiringTankPressures.values()[i].pressure_warning_color == Color.ORANGE){
                    allowedToFire = true;
                }else{
                    allowedToFire = false;
                }
                System.out.println("pressure_description = "+FiringTankPressures.values()[i].pressure_description);
            }
        }
    }
    public boolean isLoadOpen() {return loadValve.get();}
    public boolean isLaunchOpen() {return launchValve.get() == Relay.Value.kOn;}

    public void closeLaunch() {if(isLaunchOpen()) launchValve.set(Relay.Value.kOff);}
    public void closeLoad() {if(isLoadOpen()) loadValve.set(false);}

    private void openLaunch() {
        if(this.delay > 0) return;
        
        if(isLoadOpen()) {
            closeLoad();
            schedule(() -> launchValve.set(Relay.Value.kOn), 2);
        } else if(!isLaunchOpen()) {
//            launchValve.startPulse();
            launchValve.set(Relay.Value.kOn);
        }
    }

    private void openLoad() {
        if(this.delay > 0) return;

        if(isLaunchOpen()) {
            closeLaunch();
            schedule(() -> loadValve.set(true), 2);
        } else if(!isLoadOpen()) {
            System.out.println("Opening load valve");
            loadValve.set(true);
        }
    }

    public void load() {
        openLoad();
    }

    public void stopLoading() {
        closeLoad();
    }

    public void launch() {
        updateActionsOnPressureByVoltage();
//        if(allowedToFire){
            openLaunch();
//        }
    }

    @Override public void periodic() {
        if(delay > 0) {
            delay--;
            if(delay == 0 && task != null) {
                System.out.println("Running task: " + System.currentTimeMillis());
                task.run();
                task = null;
                System.out.println("Ran and cleared task: " + System.currentTimeMillis());
            }
        }

        // TODO: Determine if we want this running and what the target pressure/voltage is
        // don't test pressure if we're actively launching
//        if(launchValve.get() == Relay.Value.kOn) return;

//        double pressure = getCurrentPressure();
//        if(isLoadOpen()) {
//            if(pressure >= 40) closeLoad();
//        } else {
//            if(pressure <= 35) openLoad();
//        }
    }

    protected void schedule(
        Runnable task,
        int ticks
    ) {
        delay = Math.max(1, ticks);
        this.task = task;
    }
}
