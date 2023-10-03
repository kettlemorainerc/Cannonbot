/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.button.*;
import org.usfirst.frc.team2077.command.*;
import org.usfirst.frc.team2077.common.command.*;
import org.usfirst.frc.team2077.common.control.*;

/**
 * This class is intended to be the center point of defining actions that can be utilized during teleop segments of
 * control. This is where we should define what USB port joysticks should be registered as in `FRC Driver Station`'s usb
 * menu. As well as define what buttons on primary/technical driver's controllers should do what.
 * */
public class DriveStation {
    // Common controller port numbers
    // Joysticks that support rotation
    private static final int DRIVE_JOYSTICK_PORT = 0;
    private static final int DRIVE_XBOX_PORT = 1;
    private static final int FLYSKY_PORT = 2;

    // Joysticks that do not support rotation
    private static final int TECHNICAL_JOYSTICK_PORT = 4;
    private static final int NUMPAD_PORT = 5;

    private final DriveStick driveStick;
    private final Joystick technicalStick;

    public DriveStation(RobotHardware hardware) {
        /** Set the driver's control method this MUST be a {@link DriveStick} implementation */
//        driveStick = getFlysky();
//        driveStick = getJoystick();
        driveStick = getXbox();

        /** Set the technical control method. This can be any {@link Joystick} implementation */
//        technicalStick = getTechnicalJoystick();
        technicalStick = getNumpad();

        bind(hardware);
    }

    /**
     * This method binds any subsystem's default command and bind commands to a user's chosen
     * control method.
     */
    public void bind(RobotHardware hardware) {
        hardware.getPosition().setDefaultCommand(new CardinalMovement(hardware, driveStick));
        hardware.getHeading().setDefaultCommand(new RotationMovement(hardware, driveStick));

//        useCommand(technicalStick, 4, new ZeroSwerveMotors(hardware));


//        useCommand(technicalStick, 18, new TestingMotorIds(hardware.getWheel(WheelPosition.NORTH_EAST).rotationMotor));

//        useCommand(technicalStick, 18, new TestingEncoderIds(hardware.getWheel(WheelPosition.NORTH_EAST)));

        bindDriverControl(hardware, driveStick);
        bindTechnicalControl(hardware, technicalStick);
    }

    /** Bind primary driver's button commands here */
    private static void bindDriverControl(RobotHardware hardware, DriveStick primary) {
    }

    /** Bind technical driver button commands here */
    private void bindTechnicalControl(RobotHardware hardware, Joystick secondary) {
        JoystickButton launch = new JoystickButton(secondary, 1);

        new RunHorn(hardware).bind(new JoystickButton(secondary, 3));
        JoystickButton toggleAirCompressor = new JoystickButton(secondary, 4);

        JoystickButton pistonUp = new JoystickButton(secondary, 12);
        JoystickButton pistonDown = new JoystickButton(secondary, 16);

        new BindableRotate(hardware, -1).bind(new JoystickButton(secondary, 9));
        new BindableRotate(hardware, 1).bind(new JoystickButton(secondary, 10));

        new LaunchCannon(hardware, launch).bind(launch);
        new ToggleAirCompressor(hardware).bind(toggleAirCompressor);

        new ChangePistonHeight(hardware, ChangePistonHeight.PistonDirection.UP).bind(pistonUp);
        new ChangePistonHeight(hardware, ChangePistonHeight.PistonDirection.DOWN).bind(pistonDown);

//        new OpenRelay(hardware.cannon.launchValve, true).bind( new JoystickButton(secondary, 17) );


//        JoystickButton calibrateRotation = new JoystickButton(secondary, 3);



//        new LoadLauncher(hardware).bind(loa
//        d);
//        new StopLoading(hardware).bind(stopLoad);

//        JoystickButton runHornLow = new JoystickButton(secondary, 22);


//        new CalibrateRotation(hardware).bind(calibrateRotation);
    }

    /** Normal (silver/brighter) joystick that supports rotation */
    private static DriveJoystick getJoystick() {
        return new DriveJoystick(DRIVE_JOYSTICK_PORT).setDriveSensitivity(.3, 5)
                                                     .setRotationSensitivity(.4, 1);
    }

    /** Flysky Drone Controller */
    private static DriveJoystick getFlysky() {
        return new DriveJoystick(FLYSKY_PORT, 4).setDriveSensitivity(.3, 1)
                                                .setRotationSensitivity(.05, 2.5);
    }

    private static DriveXboxController getXbox(){
        return new DriveXboxController(DRIVE_XBOX_PORT).setDriveSensitivity(.15,1)
                                                       .setRotationSensitivity(.05,1.5);
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
