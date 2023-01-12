// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //**JOYSTICKS */
  Joystick controller = new Joystick(Constants.ControllerConstants.USB_CONTROLLER);
  Joystick leftStick = new Joystick(Constants.ControllerConstants.USB_LEFT_STICK);
  Joystick rightStick = new Joystick(Constants.ControllerConstants.USB_RIGHT_STICK);

  // SUBSYSTEMS****.
  private final Drive drive = new Drive();

  //COMMANDS****
  //AUTO

  //DRIVE
 private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drive, leftStick, rightStick);
      

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drive.setDefaultCommand(driveWithJoysticks);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
      // CREATE BUTTONS
    // *CONTROLLER
    JoystickButton x = new JoystickButton(controller, ControllerConstants.LogitechF310.X);
    JoystickButton a = new JoystickButton(controller, ControllerConstants.LogitechF310.A);
    JoystickButton b = new JoystickButton(controller, ControllerConstants.LogitechF310.B);
    JoystickButton y = new JoystickButton(controller, ControllerConstants.LogitechF310.Y);
    JoystickButton lb = new JoystickButton(controller, ControllerConstants.LogitechF310.LB);
    JoystickButton rb = new JoystickButton(controller, ControllerConstants.LogitechF310.RB);
    JoystickButton back = new JoystickButton(controller, ControllerConstants.LogitechF310.BACK);
    JoystickButton start = new JoystickButton(controller, ControllerConstants.LogitechF310.START);
    JoystickButton leftPress = new JoystickButton(controller, ControllerConstants.LogitechF310.LEFT_PRESS);
    JoystickButton rightPress = new JoystickButton(controller, ControllerConstants.LogitechF310.RIGHT_PRESS);

    // *LEFT STICK
    JoystickButton leftTrigger = new JoystickButton(leftStick,ControllerConstants.Thrustmaster.TRIGGER);
    JoystickButton leftMiddle = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.BUTTON_MIDDLE);
    JoystickButton leftStickLeft = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.BUTTON_LEFT);
    JoystickButton leftStickRight = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.BUTTON_RIGHT);
    JoystickButton extraL1 = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.LEFT_BASE_1);
    JoystickButton extraL2 = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.LEFT_BASE_2);
    JoystickButton extraL3 = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.LEFT_BASE_3);
    JoystickButton extraL4 = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.LEFT_BASE_4);
    JoystickButton extraL5 = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.RIGHT_BASE_5);
    JoystickButton extraL6 = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.RIGHT_BASE_6);
    JoystickButton extraL7 = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.RIGHT_BASE_7);
    JoystickButton extraL8 = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.RIGHT_BASE_8);
    
    // *RIGHT STICK
    JoystickButton rightTrigger = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.TRIGGER); 
    JoystickButton rightMiddle = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.BUTTON_MIDDLE);
    JoystickButton rightStickLeft = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.BUTTON_LEFT);
    JoystickButton rightStickRight = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.BUTTON_RIGHT);
    JoystickButton extraR1 = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.LEFT_BASE_1);
    JoystickButton extraR2 = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.LEFT_BASE_2);
    JoystickButton extraR3 = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.LEFT_BASE_3);
    JoystickButton extraR4 = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.LEFT_BASE_4);
    JoystickButton extraR5 = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.RIGHT_BASE_5);
    JoystickButton extraR6 = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.RIGHT_BASE_6);
    JoystickButton extraR7 = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.RIGHT_BASE_7);
    JoystickButton extraR8 = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.RIGHT_BASE_8);

    // ASSIGN BUTTONS TO COMMANDS
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public boolean getAutonomousCommand() {
    // An example command will be run in autonomous
    return(false);
  }
}
