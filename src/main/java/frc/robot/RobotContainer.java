// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.LinkedList;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.Arm.ArmExtend;
import frc.robot.commands.Arm.ArmPID;
import frc.robot.commands.Arm.ArmRetract;
import frc.robot.commands.Autos.DriveAtSetSpeed;
import frc.robot.commands.Autos.HighBalanceAuto;
import frc.robot.commands.Autos.MidBalanceAuto;
import frc.robot.commands.Autos.ScoreCCRed;
import frc.robot.commands.Autos.ScoreCCBlue;
import frc.robot.commands.Autos.ScoreDrive;
import frc.robot.commands.Drive.DoubleArcadeDrive;
import frc.robot.commands.Drive.HighGear;
import frc.robot.commands.Drive.LowGear;
import frc.robot.commands.Drive.ToggleTransmission;
import frc.robot.commands.Gripper.GrabReleaseToggle;
import frc.robot.commands.Pivot.PivotDown;
import frc.robot.commands.Pivot.PivotPID;
import frc.robot.commands.Pivot.PivotUp;
import frc.robot.commands.ScoringPositions.LoadStation;
import frc.robot.commands.ScoringPositions.Pickup;
import frc.robot.commands.ScoringPositions.PickupToStow;
import frc.robot.commands.ScoringPositions.ScoreHigh;
import frc.robot.commands.ScoringPositions.ScoreHigh90;
import frc.robot.commands.ScoringPositions.ScoreHighN90;
import frc.robot.commands.ScoringPositions.ScoreLow;
import frc.robot.commands.ScoringPositions.ScoreLow90;
import frc.robot.commands.ScoringPositions.ScoreMid;
import frc.robot.commands.ScoringPositions.ScoreMid90;
import frc.robot.commands.ScoringPositions.ScoreMidN90;
import frc.robot.commands.ScoringPositions.Stowe;
import frc.robot.commands.ScoringPositions.StoweFromUP;
import frc.robot.commands.Targeting.AprilFollow;
import frc.robot.commands.Targeting.LLAngle;
import frc.robot.commands.Targeting.LLDistance;
import frc.robot.commands.Targeting.LLSideDistance;
import frc.robot.commands.Targeting.LLTarget;
import frc.robot.commands.Turret.TurretBrake;
import frc.robot.commands.Turret.TurretCCW;
import frc.robot.commands.Turret.TurretCW;
import frc.robot.commands.Turret.TurretPID;
import frc.robot.commands.Turret.TurretRelease;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Turret;
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
  private final Pivot pivot = new Pivot();
  private static DigitalInput autoSwitch1 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_1);
  private static DigitalInput autoSwitch2 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_2);
  private static DigitalInput autoSwitch3 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_3);
  private static DigitalInput autoSwitch4 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_4);
  //DRIVE
 private final DoubleArcadeDrive doubleArcadeDrive = new DoubleArcadeDrive(drive, gripper, driveController);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  drive.setDefaultCommand(doubleArcadeDrive);
  //arm.setDefaultCommand(new ArmWithAxis(arm, controller));
  //pivot.setDefaultCommand();
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
    POVButton leftPov1 = new POVButton(controller, Constants.ControllerConstants.XboxController.POVXbox.LEFT_ANGLE);
    POVButton rightPov1 = new POVButton(controller, Constants.ControllerConstants.XboxController.POVXbox.RIGHT_ANGLE);
    // ASSIGN BUTTONS TO COMMANDS
    //AUXController
    menu1.whileTrue(new HighGear(drive));
    view1.whileTrue(new LowGear(drive));
    y1.whileTrue(new LoadStation(arm, pivot, gripper));
    x1.onTrue(new Pickup(arm, pivot, gripper));
    b1.onTrue(new PickupToStow(pivot, arm));
    a1.onTrue(new GrabReleaseToggle(gripper));
    rb1.onTrue(new ScoreHigh90(arm, pivot, gripper, turret));
    rm1.onTrue(new ScoreMid90(arm, pivot, gripper, turret));
    lb1.onTrue(new ScoreHighN90(arm, pivot, gripper, turret));
    lm1.onTrue(new ScoreMidN90(arm, pivot, gripper, turret));

   //upPov1.whileTrue(new TurretBrake(turret));
 // upPov1.whileTrue(new LLAngle(drive, 0));
   upPov1.onTrue(new StoweFromUP(arm, pivot, turret));
   downPov1.whileTrue(new LLSideDistance(drive, turret, 0));
  //downPov1.whileTrue(new LLDistance(drive, 0, 40, 18));
  //downPov1.whileTrue(new Stowe(arm, pivot, turret));
   //downPov1.whileTrue(new TurretRelease(turret));
   leftPov1.whileTrue(new TurretCCW(turret, TurretConstants.TURRET_CCW_SPEED));
   rightPov1.whileTrue(new TurretCW(turret, TurretConstants.TURRET_CW_SPEED));
 



    
   //DRIVECONTROLLER******
  a.whileTrue(new ScoreCCBlue(arm, pivot, turret, gripper, drive));
  b.onTrue(new GrabReleaseToggle(gripper));
  //x.whileTrue;
  y.onTrue(new Pickup(arm, pivot, gripper));
  rb.onTrue(new LoadStation(arm, pivot, gripper));
 lb.onTrue(new ScoreMid(arm, pivot, gripper));
 rm.onTrue(new ScoreHigh(arm, pivot, gripper));
 lm.onTrue(new PickupToStow(pivot, arm));
 
menu.onTrue(new TurretPID(turret, 1));
view.onTrue(new TurretBrake(turret));

 upPov.whileTrue(new PivotUp(pivot, 0.8));
 downPov.whileTrue(new PivotDown(pivot, 0.7));
 rightPov.whileTrue(new ArmExtend(arm, 0.75));
  leftPov.whileTrue(new ArmRetract(arm, 0.55));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  public Command getAutonomousCommand() {
    if (!autoSwitch1.get() && autoSwitch2.get() && autoSwitch3.get() && autoSwitch4.get()) {
      return (new MidBalanceAuto(arm, gripper, drive, pivot));
    } else if (autoSwitch1.get() && !autoSwitch2.get() && autoSwitch3.get() && autoSwitch4.get())
  {
    return (new HighBalanceAuto(arm, gripper, drive, pivot));
  } else if (!autoSwitch3.get()) {
      return (new ScoreCCBlue(arm, pivot, turret, gripper, drive));} 
      else if (!autoSwitch4.get()) {
        return (new ScoreCCRed(arm, pivot, turret, gripper, drive));
      } else {
        return null;
      }
     /*else if (!autoSwitch1.get() && !autoSwitch3.get()) {
      return quadruplePosition1;
    } else if (!autoSwitch1.get() && !autoSwitch4.get()) {
      return extendedTriple1;
    } else if (!autoSwitch1.get()) {
      return doubleTarmac1;
    } else if (!autoSwitch2.get()) {
      return doubleTarmac2;
    } else if (!autoSwitch3.get()) {
      return triplePosition1;
    } else if (!autoSwitch4.get()) {
      return triplePosition2;
    } else {
      return doubleTarmac1;
    }*/
    //return doubleArcadeDrive;
  }
}