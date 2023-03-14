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
import frc.robot.commands.Autos.AutoScoreHigh;
import frc.robot.commands.Autos.AutoScoreMid;
import frc.robot.commands.Autos.AutoTrapezoidalPID;
import frc.robot.commands.Autos.BackwardCenter;
import frc.robot.commands.Autos.BalanceOnChargeAuto;
import frc.robot.commands.Autos.DriveAtSetSpeed;
import frc.robot.commands.Autos.ScoreDrive;
import frc.robot.commands.Autos.ScoreToCenter;
import frc.robot.commands.Autos.TurnPID;
import frc.robot.commands.Drive.AutoBalanceGyro;
import frc.robot.commands.Drive.DoubleArcadeDrive;
import frc.robot.commands.Drive.LLTagDistance;
//import frc.robot.commands.Drive.DriveWithJoysticks;
import frc.robot.commands.Drive.TDWG_No;
import frc.robot.commands.Drive.ToggleTransmission;
//import frc.robot.commands.Drive.DriveWithJoysticks;
import frc.robot.commands.Gripper.Grab;
import frc.robot.commands.Gripper.GrabReleaseToggle;
import frc.robot.commands.Gripper.ReleasePiece;
import frc.robot.commands.Pivot.PivotDown;
import frc.robot.commands.Pivot.PivotPID;
import frc.robot.commands.Pivot.PivotUp;
import frc.robot.commands.ScoringPositions.LoadStationPosition;
import frc.robot.commands.ScoringPositions.PickupPosition;
import frc.robot.commands.ScoringPositions.ScoreHighPosition;
import frc.robot.commands.ScoringPositions.ScoreLow;
import frc.robot.commands.ScoringPositions.ScoreMiddlePosition;
import frc.robot.commands.ScoringPositions.StowPosition;
import frc.robot.commands.Targeting.AprilFollow;
import frc.robot.commands.Targeting.LLAngle;
import frc.robot.commands.Targeting.LLDistance;
import frc.robot.commands.Targeting.LLTarget;
import frc.robot.commands.Turret.TurretCCW;
import frc.robot.commands.Turret.TurretCW;
import frc.robot.commands.Turret.TurretPID;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Turret;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final Pivot pivot = new Pivot();
   private final Limelight camera = new Limelight();
  private static DigitalInput autoSwitch1 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_1);
  private static DigitalInput autoSwitch2 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_2);
  private static DigitalInput autoSwitch3 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_3);
  private static DigitalInput autoSwitch4 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_4);
  // *DRIVE


  //COMMANDS****
  //AUTO
  private final ScoreMiddlePosition scoreMiddleLevel = new ScoreMiddlePosition(arm, pivot, gripper);
  //Command scoreMid = Commands.sequence(new PivotPID(arm, 10065).andThen(new ArmPID(arm, 7.25)).andThen(new ReleasePiece(gripper)));
  private final StowPosition stowPosition = new StowPosition(arm, pivot);
  //DRIVE
 private final DoubleArcadeDrive doubleArcadeDrive = new DoubleArcadeDrive(drive, gripper, driveController);
 private final ToggleTransmission toggleTransmission = new ToggleTransmission(drive);
  /** Creates a new DriveToCS. */
 private final  DriveAtSetSpeed driveAtSetSpeed = new DriveAtSetSpeed(drive, 130, 0.5);

 //ARM
 private final PivotUp pivotUp = new PivotUp(pivot, 0.7);
 private final PivotDown pivotDown = new PivotDown(pivot, 0.6);
 private final PivotPID pivotMidPID = new PivotPID(pivot, 10065);
 private final PivotPID pivotHighPID = new PivotPID(pivot, 11188);
 private final PivotPID pivotLowPID = new PivotPID(pivot, 4862);

 //GRIPPER
private final Grab grab = new Grab(gripper);
private final ReleasePiece releasePiece = new ReleasePiece(gripper);
private final GrabReleaseToggle grabReleaseToggle = new GrabReleaseToggle(gripper);

//TURRET
private final TurretCW turretCW = new TurretCW(turret, TurretConstants.TURRET_CW_SPEED);
private final TurretCCW turretCCW = new TurretCCW(turret, TurretConstants.TURRET_CCW_SPEED);

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
   x1.whileTrue(new LLAngle(drive, 0)); //aprilTags
   //y1.whileTrue(new LLDistance(drive, 0, 37, 27.5)); //apriltags //27.5
  y1.whileTrue(new LLTagDistance(drive, 0, 30, camera)); 
   b1.whileTrue(new DriveAtSetSpeed(drive, 50, 0.3)); //must pass in the pipeline
    a1.whileTrue(new ToggleTransmission(drive));
    upPov1.whileTrue(new BackwardCenter(arm, gripper, drive, pivot));
    downPov1.whileTrue(new AutoScoreHigh(arm, pivot, gripper));
    lm1.whileTrue(new TurnPID(drive, 0));
    rm1.whileTrue(new TurnPID(drive, 90));
    lb1.whileTrue(new TurnPID(drive, 270));
   // rb1.whileTrue(new TurnPID(drive, 90));



    
   //DRIVECONTROLLER******
  a.whileTrue(new ToggleTransmission(drive));
  b.whileTrue(grabReleaseToggle);
  x.whileTrue(new ScoreLow(arm, pivot, gripper));
  y.whileTrue(new PickupPosition(arm, pivot, gripper));
  rb.whileTrue(new LoadStationPosition(arm, pivot, gripper));
 lb.whileTrue(scoreMiddleLevel);
 rm.whileTrue(new ScoreHighPosition(arm, pivot, gripper));
 lm.whileTrue(stowPosition);


 upPov.whileTrue(pivotUp);
 downPov.whileTrue(pivotDown);
 rightPov.whileTrue(new ArmExtend(arm, 0.5));
  leftPov.whileTrue(new ArmRetract(arm, 0.5));


  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  public Command getAutonomousCommand() {
    if (!autoSwitch1.get() && autoSwitch2.get() && autoSwitch3.get() && autoSwitch4.get()) {
      return (new BackwardCenter(arm, gripper, drive, pivot));
    } else {
      return (new ScoreDrive(arm, pivot, drive, gripper));
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