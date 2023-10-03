package org.usfirst.frc.team2077.subsystem;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import edu.wpi.first.wpilibj.AnalogTriggerOutput.*;

public class PressureSensor extends SubsystemBase {
    private final AnalogInput sensor;
    private double pressure;

    private static SmartDashNumber PressureOutPutVoltage = new SmartDashNumber("PressureOutputVoltage", 0.0, true);
    private static SmartDashNumber ReadPressure = new SmartDashNumber("ReadPressure", 0.0, true);

    public PressureSensor(int analogSlot) {
        this.sensor = new AnalogInput(analogSlot);
    }

    public double getCurrentPressure() {
        updatePressure();
        return pressure;
    }

    protected void updatePressure() {
       double voltage = sensor.getVoltage();

        pressure = voltageToPressureFunction(voltage);

        ReadPressure.set(pressure);
    }

    private double voltageToPressureFunction(double x){
//        PressureOutPutVoltage.set(x);
        return 50 * (x - 0.5);
    }

    public double getPressure(){
        return pressure;
    }

    @Override public void periodic() {
        updatePressure();
    }
}
