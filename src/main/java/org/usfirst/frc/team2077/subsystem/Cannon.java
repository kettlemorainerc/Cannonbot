package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.led.FireAnimation;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;

import java.awt.*;

public class Cannon extends SubsystemBase {

    public final Relay launchValve;
    public final Solenoid loadValve;

    private final PressureSensor pressure;

    private static final Relay.Value OPEN = Relay.Value.kOn;
    private static final Relay.Value CLOSE = Relay.Value.kOff;


    private int delay;
    private Runnable task;

    public Cannon(Solenoid loadValve, Relay launchValve, PressureSensor pressure) {
        this.launchValve = launchValve;
        this.loadValve = loadValve;

        this.pressure = pressure;
        launchValve.set(CLOSE);
    }

    public double getCurrentPressure() {return pressure.getCurrentPressure();}

    public boolean isLoadOpen() {return loadValve.get();}
    public boolean isLaunchOpen() {return launchValve.get() == OPEN;}

    public void closeLaunch() { launchValve.set(CLOSE);}
    public void closeLoad() { loadValve.set(true);}

    private void openLaunch() {
        launchValve.set(OPEN);
    }

    private void openLoad() {
        loadValve.set(false);
    }

    public void load() {
        openLoad();
        closeLaunch();
    }

    public void launch() {
        openLaunch();
        closeLoad();

        schedule( () -> load(), 25);
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
