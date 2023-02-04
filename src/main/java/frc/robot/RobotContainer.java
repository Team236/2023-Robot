// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Arm.ArmExtend;
import frc.robot.commands.Arm.ArmPID;
import frc.robot.commands.Arm.ArmRetract;
import frc.robot.commands.Arm.ArmWithAxis;
import frc.robot.commands.Autos.AutoPIDDrive;
import frc.robot.commands.Drive.DoubleArcadeDrive;
import frc.robot.commands.Drive.DriveWithJoysticks;
import frc.robot.commands.Drive.TankDriveWithGyro;
//import frc.robot.commands.Drive.DriveWithJoysticks;
import frc.robot.commands.Gripper.Grab;
import frc.robot.commands.Gripper.GrabReleaseToggle;
import frc.robot.commands.Gripper.ReleasePiece;
import frc.robot.commands.Pivot.PivotToggle;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.PPivot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
  XboxController xboxController = new XboxController(Constants.ControllerConstants.USB_CONTROLLER);
  // SUBSYSTEMS****.
  private final Drive drive = new Drive();
  private final Arm arm = new Arm();
  private final Gripper gripper = new Gripper();
  private final PPivot pivot = new PPivot();
  

  //COMMANDS****
  //AUTO

  //DRIVE
 private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drive, gripper, leftStick, rightStick);
 private final DoubleArcadeDrive doubleArcadeDrive = new DoubleArcadeDrive(drive, gripper);

 //ARM
 private final ArmWithAxis armWithAxis = new ArmWithAxis(arm, controller); 
 private final ArmExtend armExtend = new ArmExtend(arm, controller);
 private final ArmRetract armRetract = new ArmRetract(arm, controller);

 //GRIPPER
private final Grab grab = new Grab(gripper);
private final ReleasePiece releasePiece = new ReleasePiece(gripper);
private final GrabReleaseToggle grabReleaseToggle = new GrabReleaseToggle(gripper);
private final PivotToggle pivotToggle = new PivotToggle(pivot);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

  drive.setDefaultCommand(doubleArcadeDrive);
  //drive.setDefaultCommand(driveWithJoysticks);

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
    // *XBOXCONTROLLER
    JoystickButton x = new JoystickButton(xboxController, ControllerConstants.XboxController.X);
    JoystickButton a = new JoystickButton(xboxController, ControllerConstants.XboxController.A);
    JoystickButton b = new JoystickButton(xboxController, ControllerConstants.XboxController.B);
    JoystickButton y = new JoystickButton(xboxController, ControllerConstants.XboxController.Y);
    JoystickButton lb = new JoystickButton(xboxController, ControllerConstants.XboxController.LB);
    JoystickButton rb = new JoystickButton(xboxController, ControllerConstants.XboxController.RB);
    JoystickButton lm = new JoystickButton(xboxController, ControllerConstants.XboxController.LM);
    JoystickButton rm = new JoystickButton(xboxController, ControllerConstants.XboxController.RM);
    JoystickButton view = new JoystickButton(xboxController, ControllerConstants.XboxController.VIEW);
    JoystickButton menu = new JoystickButton(xboxController, ControllerConstants.XboxController.MENU);
    POVButton upPov = new POVButton(xboxController, Constants.ControllerConstants.XboxController.POVXbox.UP_ANGLE);
    POVButton downPov = new POVButton(xboxController, Constants.ControllerConstants.XboxController.POVXbox.DOWN_ANGLE); 



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
   //LEFTSTICK*****
   leftStickLeft.whileTrue(new AutoPIDDrive(drive, -Constants.DriveConstants.GRID_TO_CENTER));
   leftStickRight.whileTrue(new TankDriveWithGyro(drive, 0.1, 100,0.3));
   //RIGHTSTICK*****
   //CONTROLLER******
  upPov.whileTrue(armExtend);
  downPov.whileTrue(armRetract);
  a.whileTrue(new ArmPID(arm, Constants.ArmConstants.ARM_OUT));
  x.whileTrue(grabReleaseToggle);  
  b.whileTrue(pivotToggle);
  menu.whileTrue(armWithAxis);
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
