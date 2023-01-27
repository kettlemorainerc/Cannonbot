package org.usfirst.frc.team2077.command;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.command.SelfDefinedCommand;
import org.usfirst.frc.team2077.common.drivetrain.MecanumMath;
import org.usfirst.frc.team2077.subsystem.SwerveMotor;

public class CalibrateRotation extends SelfDefinedCommand {

    private boolean finished;
    private SwerveMotor[] hardware;
    private int wheelNumber = 0;
    public CalibrateRotation(RobotHardware hardware) {
        WheelPosition[] positionValues = WheelPosition.values();
        this.hardware = new SwerveMotor[positionValues.length];
        for(int i = 0; i < positionValues.length; i++){
            this.hardware[i] = hardware.getWheel(positionValues[i]);
        }
        this.addRequirements(this.hardware);

    }


    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
//        SwerveMotor current = hardware[wheelNumber];
//        if(current.isAligned()){
//            current.setRotationPercent(0);
//            wheelNumber++;
//            finished = wheelNumber == hardware.length;
//        }else{
//            current.setRotationPercent(0.05);
//        }
        SmartDashboard.putBoolean("hallValueRaw", hardware[0].getHallValue());
        finished = true;

        //System.out.println(current + String.valueOf(current.isAligned()));
        System.out.println(hardware[0].getHallValue());


    }

    @Override
    public void end(boolean interrupted) {

    }
    //
}
