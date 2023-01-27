package org.usfirst.frc.team2077.subsystem;


import org.usfirst.frc.team2077.common.subsystem.CANLineSubsystem;

public final class Victor extends CANLineSubsystem<edu.wpi.first.wpilibj.motorcontrol.Victor> {

    public Victor(int canId){
        super(new edu.wpi.first.wpilibj.motorcontrol.Victor(canId), canId);
    }

    @Override
    public void setRPM(double RPM) {
        throw new UnsupportedOperationException("Talon motors do not support set RPM. Either determine their maximum RPM and update Talon#setRPM, or use Talon#setPercent");
    }

    @Override
    public void setPercent(double percent) {
        motor.set(percent);
    }
}
