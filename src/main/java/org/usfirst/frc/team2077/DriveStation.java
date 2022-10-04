/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.button.*;
import org.usfirst.frc.team2077.command.*;
import org.usfirst.frc.team2077.common.*;
import org.usfirst.frc.team2077.common.commands.*;
import org.usfirst.frc.team2077.common.drivetrain.MecanumMath;

public class DriveStation {
    private static final int DRIVE_JOYSTICK_PORT = 0;
    private static final int DRIVE_XBOX_PORT = 1;
    private static final int TECHNICAL_JOYSTICK_PORT = 1;
    private static final int FLYSKY_PORT = 2;
    private static final int NUMPAD_PORT = 5;

    private final DriveStick driveStick;
    private final Joystick technicalStick;

    public DriveStation(RobotHardware hardware) {
//        driveStick = getFlysky();
//        driveStick = getJoystick();
        driveStick = getXbox();

//        technicalStick = getTechnicalJoystick();
        technicalStick = getNumpad();

        bind(hardware);
    }

    public void bind(RobotHardware hardware) {
        hardware.getPosition().setDefaultCommand(new CardinalMovement(hardware, driveStick));
        hardware.getHeading().setDefaultCommand(new RotationMovement(hardware, driveStick));

//        useCommand(technicalStick, 18, new TestingMotorIds(hardware.getWheel(MecanumMath.WheelPosition.NORTH_EAST).talonMotor));
//        useCommand(technicalStick, 17, new TestingMotorIds(hardware.getWheel(MecanumMath.WheelPosition.NORTH_WEST).talonMotor));
//        useCommand(technicalStick, 21, new TestingMotorIds(hardware.getWheel(MecanumMath.WheelPosition.SOUTH_EAST).talonMotor));
//        useCommand(technicalStick, 22, new TestingMotorIds(hardware.getWheel(MecanumMath.WheelPosition.SOUTH_WEST).talonMotor));

        useCommand(technicalStick, 18, new TestingEncoderIds(hardware.getWheel(MecanumMath.WheelPosition.NORTH_WEST)));
        useCommand(technicalStick, 17, new TestingEncoderIds(hardware.getWheel(MecanumMath.WheelPosition.NORTH_EAST)));
        useCommand(technicalStick, 21, new TestingEncoderIds(hardware.getWheel(MecanumMath.WheelPosition.SOUTH_WEST)));
        useCommand(technicalStick, 22, new TestingEncoderIds(hardware.getWheel(MecanumMath.WheelPosition.SOUTH_EAST)));

        bindDriverControl(hardware, driveStick);
//        bindTechnicalControl(hardware, technicalStick);
    }

    /**
     * Bind related commands/controls here
     */
    private static void bindDriverControl(RobotHardware hardware, DriveStick primary) {
    }

    /**
     * Bind secondary/techincal driver commands/controls here
     */
    private void bindTechnicalControl(RobotHardware hardware, Joystick secondary) {
        JoystickButton load = new JoystickButton(secondary, 1);
        JoystickButton stopLoad = new JoystickButton(secondary, 5);
        JoystickButton launch = new JoystickButton(secondary, 2);

        JoystickButton pistonUp = new JoystickButton(secondary, 12);
        JoystickButton pistonDown = new JoystickButton(secondary, 16);
        JoystickButton toggleAirCompressor = new JoystickButton(secondary, 6);


        new LoadLauncher(hardware).bind(load);
        new StopLoading(hardware).bind(stopLoad);
        new LaunchCannon(hardware, launch).bind(launch);
        new ToggleAirCompressor(hardware).bind(toggleAirCompressor);

        new ChangePistonHeight(hardware, ChangePistonHeight.PistonDirection.UP).bind(pistonUp);
        new ChangePistonHeight(hardware, ChangePistonHeight.PistonDirection.DOWN).bind(pistonDown);
    }

    /** Normal (brighter/silver) joystick that supports rotation */
    private static DriveJoystick getJoystick() {
        return new DriveJoystick(DRIVE_JOYSTICK_PORT).setDriveSensitivity(.3, 5).setRotationSensitivity(.4, 1);
    }

    /** Flysky Drone Controller */
    private static DriveJoystick getFlysky() {
        return new DriveJoystick(FLYSKY_PORT, 4).setDriveSensitivity(.3, 1)
                                   .setRotationSensitivity(.05, 2.5);
    }

    private static DriveXboxController getXbox(){
        return new DriveXboxController(DRIVE_XBOX_PORT).setDriveSensitivity(.15,1).setRotationSensitivity(.05,1.5);
    }

    /** Currently the darker joystick that doesn't support rotation */
    private static Joystick getTechnicalJoystick() {
        return new Joystick(TECHNICAL_JOYSTICK_PORT);
    }

    private static Joystick getNumpad() {
        return new Joystick(NUMPAD_PORT);
    }

    /** bind command to the given joystick button */
    public static void useCommand(Joystick joystick, int button, BindableCommand command) {
        command.bind(new JoystickButton(joystick, button));
    }
}
