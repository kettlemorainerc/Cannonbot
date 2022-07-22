package org.usfirst.frc.team2077.subsystem;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;

import java.util.concurrent.*;

public class Cannon extends SubsystemBase {
    private final Solenoid launchValve;
    private final Solenoid loadValve;
    private final PressureSensor pressure;

//    private Solenoid targetedSolenoid;

    private int delay;
    private Runnable task;

    public Cannon(Solenoid loadValve, Solenoid launchValve, PressureSensor pressure) {
        this.launchValve = launchValve;
        this.loadValve = loadValve;
//        launchValve.setPulseDuration(.03);
        this.pressure = pressure;
    }

    public double getCurrentPressure() {return pressure.getCurrentPressure();}
    public boolean isLoadOpen() {return loadValve.get();}
    public boolean isLaunchOpen() {return launchValve.get();}

    public void closeLaunch() {if(isLaunchOpen()) launchValve.set(false);}
    public void closeLoad() {if(isLoadOpen()) loadValve.set(false);}

    private void openLaunch() {
        if(this.delay > 0) return;
        
        if(isLoadOpen()) {
            closeLoad();
            schedule(() -> launchValve.set(true), 2);
        } else if(!isLaunchOpen()) {
//            launchValve.startPulse();
            launchValve.set(true);
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
        openLaunch();
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
