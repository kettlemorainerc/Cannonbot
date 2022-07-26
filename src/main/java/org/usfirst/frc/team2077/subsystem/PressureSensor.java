package org.usfirst.frc.team2077.subsystem;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;

public class PressureSensor extends SubsystemBase {
    private final AnalogInput sensor;
    private double pressure;

    public PressureSensor(int analogSlot) {
        this.sensor = new AnalogInput(analogSlot);
    }

    public double getCurrentPressure() {
        updatePressure();
        return pressure;
    }

    protected void updatePressure() {
        double voltage = sensor.getVoltage();

        // TODO: figure out voltage -> pressure ratio
        pressure = voltage;
    }

    @Override public void periodic() {
        updatePressure();
    }
}
