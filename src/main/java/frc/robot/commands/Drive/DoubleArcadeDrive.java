// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gripper;
import frc.robot.Constants;

public class DoubleArcadeDrive extends CommandBase {
  private Drive drive;
  private Gripper gripper;
  private Boolean isDeadzone = Constants.DriveConstants.IS_DEADZONE;
  private double max, L, R, kPgyro, error;
  private XboxController driveController;
  private AHRS navX;
  /** Creates a new DoubleArcadeDrive. */
  public DoubleArcadeDrive(Drive drive1, Gripper gripper1, XboxController driveController1) {
    this.gripper = gripper1;
    this.drive = drive1;
    this.driveController = driveController1;
    addRequirements(gripper);
    addRequirements(drive);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isDeadzone = true;
    navX = new AHRS();
    drive.resetLeftEncoder();
    drive.resetRightEncoder();
    drive.openRampRate();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Adjust the Y speed so the robot drives straight (no angle change) when X
    // (turn) input is low
    // change all signs before getLeftX for 2022 robot (and also before gyro error correction factor?)

   // SmartDashboard.putNumber("Controller Left Stick X Value:", driveController.getLeftX());
    //SmartDashboard.putNumber( "Controller Right Stick Y Value:", driveController.getRightY());
   // getRightY is negative when driving forward.  getLeftX and navX.getRate are positive Clockwise.
    //change 0.05 /  0.05 to refer to Constants - Deadzone
    double max, L, R, kPgyro, error;
    kPgyro = 0.00; //0.09
    error = 0;//navX.getRate(); // until we verify that this doesn't break drive
    if ((Math.abs(driveController.getLeftX()) > 0.05) && (Math.abs(driveController.getRightY()) <= 0.05)) {
      L = driveController.getLeftX();
      R = -driveController.getLeftX();
    } else if ((Math.abs(driveController.getLeftX()) <= 0.05) && (Math.abs(driveController.getRightY()) <= 0.05)) {
      L = (0);//-driveController.getRightY() //+ (kPgyro*error));
      R = (0);//-driveController.getRightY() //- (kPgyro*error));
    } else if ((Math.abs(driveController.getLeftX()) > 0.05) && (Math.abs(driveController.getRightY()) > 0.05)) {
      L = -driveController.getRightY() + driveController.getLeftX();
      R = -driveController.getRightY() - driveController.getLeftX();
    } else {
      L = (-driveController.getRightY());//- (kPgyro*error));
      R = (-driveController.getRightY()); //+ (kPgyro*error));
    }
  
    
    max = Math.abs(L);
    if (max < Math.abs(R)) {
      max = Math.abs(R);
    }

    if (max > 1) {
      L /= max;
      R /= max;
    }
    // SmartDashboard.putNumber("left arcade speed", L);
    // SmartDashboard.putNumber("right Arcade speed", R);

    drive.setLeftSpeed(L);
    drive.setRightSpeed(R);
   // SmartDashboard.putNumber("Arcade Drive Left Encoder", drive.getLeftDistance());
    //SmartDashboard.putNumber("Roll in Arcade Drive", drive.getRoll());
   gripper.autoGrab();
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}