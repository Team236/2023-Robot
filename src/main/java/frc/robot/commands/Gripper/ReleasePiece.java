// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gripper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;
public class ReleasePiece extends CommandBase {
  private Gripper gripper;
  /** Creates a new ReleasePiece. */
  public ReleasePiece(Gripper relGripper) {
    this.gripper = relGripper;
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gripper.release();
    gripper.resetGripperEyeCount();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (!gripper.isGripping) {
    //   SmartDashboard.putBoolean("Release-isFinished", true);
    //   return true;
    // } else {
    //   SmartDashboard.putBoolean("Release-isNotFinished", true);
    //   return false;
    // }
  
      return false;
  }
  
}
