package org.usfirst.frc.team2077.subsystem;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;

import java.util.concurrent.*;

public class Cannon extends SubsystemBase {
    private final Relay launchValve;
    private final DigitalOutput loadValve;
    private final PressureSensor pressure;

    private final ScheduledThreadPoolExecutor scheduler = new ScheduledThreadPoolExecutor(1);
    private ScheduledFuture<?> scheduledTask;

    public Cannon(DigitalOutput loadValve, Relay launchValve, PressureSensor pressure) {
        this.launchValve = launchValve;
        this.loadValve = loadValve;
        this.pressure = pressure;
        scheduler.setRemoveOnCancelPolicy(true);
    }

    public double getCurrentPressure() {return pressure.getCurrentPressure();}
    public boolean isLoadOpen() {return loadValve.get();}
    public boolean isLaunchOpen() {return launchValve.get() == Relay.Value.kOn;}

    public void closeLaunch() {launchValve.set(Relay.Value.kOff);}
    public void closeLoad() {loadValve.set(false);}

    private void openLaunch() {
        if(launchValve.get() == Relay.Value.kOn && scheduledTask != null) return;

        if(isLoadOpen()) {
            loadValve.set(false);
            schedule(() -> launchValve.set(Relay.Value.kOn), 5);
        } else {
            launchValve.set(Relay.Value.kOn);
        }
    }

    private void openLoad() {
        if(launchValve.get() == Relay.Value.kOn) {
            launchValve.set(Relay.Value.kOff);
            schedule(() -> loadValve.set(true), 50);
        } else {
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
        long millis
    ) {
        if(scheduledTask != null && !scheduledTask.isDone()) {
            scheduledTask.cancel(true);
        }

        scheduledTask = scheduler.schedule(
            () -> {
                scheduledTask = null;
                task.run();
            },
            millis,
            TimeUnit.MILLISECONDS
        );
    }
}
