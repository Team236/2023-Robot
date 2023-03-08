// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gripper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class GrabReleaseToggle extends CommandBase{ 
  private Gripper gripper2;
  private boolean toggle;
  /** Creates a new GrabReleaseToggle. */
  public GrabReleaseToggle(Gripper toggleGripper) {
    this.gripper2=toggleGripper;
    addRequirements(gripper2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    toggle = false;
    
   if (gripper2.isGripping()) {
    gripper2.release();
    gripper2.resetGripperEyeCount();
    toggle = true;
    } else if (!gripper2.isGripping()) {
    gripper2.grab();
    toggle = true;
   }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //SmartDashboard.putBoolean("finish toggle", toggle);
    return toggle;
  }
}
