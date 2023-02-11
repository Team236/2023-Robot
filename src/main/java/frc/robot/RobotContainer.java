// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.Arm.ArmExtend;
import frc.robot.commands.Arm.ArmPID;
import frc.robot.commands.Arm.ArmRetract;
import frc.robot.commands.Arm.ArmWithAxis;
import frc.robot.commands.Autos.AutoPIDDrive;
import frc.robot.commands.Autos.AutoTrapezoidalPID;
import frc.robot.commands.Autos.GrabScoreFlatGround;
import frc.robot.commands.Autos.TurnPID;
import frc.robot.commands.Drive.DoubleArcadeDrive;
import frc.robot.commands.Drive.DriveWithJoysticks;
import frc.robot.commands.Drive.TankDriveWithGyro;
import frc.robot.commands.Drive.ToggleTransmission;
//import frc.robot.commands.Drive.DriveWithJoysticks;
import frc.robot.commands.Gripper.Grab;
import frc.robot.commands.Gripper.GrabReleaseToggle;
import frc.robot.commands.Gripper.ReleasePiece;
import frc.robot.commands.Pivot.PPivotToggle;
import frc.robot.commands.Turret.TurretCCW;
import frc.robot.commands.Turret.TurretCW;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.PPivot;
import frc.robot.subsystems.Turret;
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
  XboxController controller = new XboxController(Constants.ControllerConstants.USB_AUXCONTROLLER);
  XboxController driveController = new XboxController(Constants.ControllerConstants.USB_DRIVECONTROLLER);
  // SUBSYSTEMS****.
  private final Drive drive = new Drive();
  private final Arm arm = new Arm();
  private final Gripper gripper = new Gripper();
  private final Turret turret = new Turret();
  private final PPivot pivot = new PPivot();

  //COMMANDS****
  //AUTO

  //DRIVE
 private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drive, gripper, driveController);
 private final DoubleArcadeDrive doubleArcadeDrive = new DoubleArcadeDrive(drive, gripper, driveController);
 private final ToggleTransmission toggleTransmission = new ToggleTransmission(drive);

 //ARM
 //private final ArmWithAxis armWithAxis = new ArmWithAxis(arm, controller); //OBSOLETE WITH POV
 private final ArmExtend armExtend = new ArmExtend(arm, ArmConstants.ARM_EX_SPEED, driveController);
 private final ArmRetract armRetract = new ArmRetract(arm, ArmConstants.ARM_RE_SPEED, driveController);

 //GRIPPER
private final Grab grab = new Grab(gripper);
private final ReleasePiece releasePiece = new ReleasePiece(gripper);
private final GrabReleaseToggle grabReleaseToggle = new GrabReleaseToggle(gripper);
private final PPivotToggle pivotToggle = new PPivotToggle(pivot);

//TURRET
private final TurretCW turretCW = new TurretCW(turret, TurretConstants.TURRET_SPEED, driveController);
private final TurretCCW turretCCW = new TurretCCW(turret, -TurretConstants.TURRET_SPEED, driveController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  drive.setDefaultCommand(doubleArcadeDrive);
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
    // *XBOXCONTROLLER - DRIVER
    JoystickButton x = new JoystickButton(driveController, ControllerConstants.XboxController.X);
    JoystickButton a = new JoystickButton(driveController, ControllerConstants.XboxController.A);
    JoystickButton b = new JoystickButton(driveController, ControllerConstants.XboxController.B);
    JoystickButton y = new JoystickButton(driveController, ControllerConstants.XboxController.Y);
    JoystickButton lb = new JoystickButton(driveController, ControllerConstants.XboxController.LB);
    JoystickButton rb = new JoystickButton(driveController, ControllerConstants.XboxController.RB);
    JoystickButton lm = new JoystickButton(driveController, ControllerConstants.XboxController.LM);
    JoystickButton rm = new JoystickButton(driveController, ControllerConstants.XboxController.RM);
    JoystickButton view = new JoystickButton(driveController, ControllerConstants.XboxController.VIEW);
    JoystickButton menu = new JoystickButton(driveController, ControllerConstants.XboxController.MENU);
    POVButton upPov = new POVButton(driveController, Constants.ControllerConstants.XboxController.POVXbox.UP_ANGLE);
    POVButton downPov = new POVButton(driveController, Constants.ControllerConstants.XboxController.POVXbox.DOWN_ANGLE); 
    POVButton leftPov = new POVButton(driveController, Constants.ControllerConstants.XboxController.POVXbox.LEFT_ANGLE);
    POVButton rightPov = new POVButton(driveController, Constants.ControllerConstants.XboxController.POVXbox.RIGHT_ANGLE);
// XBOX CONTROLLER - CONTROLLER
    JoystickButton x1 = new JoystickButton(controller, ControllerConstants.XboxController.X);
    JoystickButton a1 = new JoystickButton(controller, ControllerConstants.XboxController.A);
    JoystickButton b1 = new JoystickButton(controller, ControllerConstants.XboxController.B);
    JoystickButton y1 = new JoystickButton(controller, ControllerConstants.XboxController.Y);
    JoystickButton lb1 = new JoystickButton(controller, ControllerConstants.XboxController.LB);
    JoystickButton rb1 = new JoystickButton(controller, ControllerConstants.XboxController.RB);
    JoystickButton lm1 = new JoystickButton(controller, ControllerConstants.XboxController.LM);
    JoystickButton rm1 = new JoystickButton(controller, ControllerConstants.XboxController.RM);
    JoystickButton view1 = new JoystickButton(controller, ControllerConstants.XboxController.VIEW);
    JoystickButton menu1 = new JoystickButton(controller, ControllerConstants.XboxController.MENU);
    POVButton upPov1 = new POVButton(controller, Constants.ControllerConstants.XboxController.POVXbox.UP_ANGLE);
    POVButton downPov1 = new POVButton(controller, Constants.ControllerConstants.XboxController.POVXbox.DOWN_ANGLE);
    // ASSIGN BUTTONS TO COMMANDS
    //AUXController
   a1.whileTrue(new AutoPIDDrive(drive, -Constants.DriveConstants.GRID_TO_CENTER));
   b1.whileTrue(new TankDriveWithGyro(drive, 0.001, 60, 0.3));
   //DRIVECONTROLLER******
  upPov.whileTrue(armExtend);
  downPov.whileTrue(armRetract);
  leftPov.whileTrue(turretCCW);
  rightPov.whileTrue(turretCW);
  a.whileTrue(new ArmPID(arm, Constants.ArmConstants.ARM_OUT));
  x.whileTrue(grabReleaseToggle);  
  b.whileTrue(pivotToggle);
  y.whileTrue(new AutoTrapezoidalPID(drive, -105, 0.005, 0, 0));
  rb.whileTrue(toggleTransmission);
  lb.whileTrue(new TurnPID(drive, 90));

  menu.whileTrue(new GrabScoreFlatGround(drive, gripper, arm, turret));
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
