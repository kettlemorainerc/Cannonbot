package org.usfirst.frc.team2077.subsystem;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cannon extends SubsystemBase {

    enum ValveState{
        OPEN,
        CLOSE;
    }

    private final Relay launchValve;
    private final Solenoid loadValve;
    private final PressureSensor pressure;

    private Runnable task = null;
    private int delay;

    public Cannon(){
        loadValve = new Solenoid(42, PneumaticsModuleType.CTREPCM, 1);
        launchValve = new Relay(0, Relay.Direction.kForward);

        pressure = new PressureSensor(0);
    }

    private void setLaunchValve(ValveState state){
        launchValve.set(state == ValveState.OPEN? Relay.Value.kOn : Relay.Value.kOff);
    }

    private void setLoadValve(ValveState state){
        loadValve.set(!(state == ValveState.OPEN));
    }

    private void load(){
        setLoadValve(ValveState.OPEN);
        setLaunchValve(ValveState.CLOSE);
    }

    private void release(){
        setLoadValve(ValveState.CLOSE);
        setLaunchValve(ValveState.OPEN);
    }

    public void launch(){
        release();
        schedule(this::load, 12);
    }

    public void schedule(Runnable task, int ticks){
        this.task = task;
        this.delay = Math.max(1, ticks);
    }

    @Override
    public void periodic() {

        if(task != null){
            delay--;
            if(delay <= 0){
                task.run();
                task = null;
            }
        }
    }
}
