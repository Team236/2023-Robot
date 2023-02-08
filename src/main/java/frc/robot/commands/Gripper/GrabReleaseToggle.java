// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class GrabReleaseToggle extends CommandBase{ 

  private Gripper gripper;
  private boolean toggle;

  /** Creates a new GrabReleaseToggle. */
  public GrabReleaseToggle(Gripper passed_gripper) {

    this.gripper = passed_gripper;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(passed_gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    toggle = false;

   if (gripper.isGripping()) {
    gripper.release();
    gripper.resetGripperEyeCount();
    toggle = true;

    } else if (!gripper.isGripping()) {
    gripper.grab();
    toggle = true;

   }

   SmartDashboard.putNumber("Eye Count", gripper.getGripperEyeCount());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return toggle;


  }
}
