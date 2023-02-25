// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.XboxController;
public class ArmRetract extends CommandBase {
  /** Creates a new ArmRetract. */
  private Arm arm;
  private XboxController controller;
  private double speed;
  public ArmRetract(Arm arm, double speed, XboxController controller) {
  this.arm = arm;
  addRequirements(arm);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setArmSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((speed > 0.008) && arm.isAExtRetLimit()) {
      // if mast is going up and top limit is triggered
      // the 0.008 is because when the axis is at rest, it reads 0.0078125 so doing speed > 0.008 acts as a deadzone
      return true;
    } else if ((speed < 0) && arm.isAExtRetLimit()) {
      arm.resetArmEncoder();
      return true;
      // } else if (arm.getTotalArmLength() > 
      // (Constants.ArmConstants.MAST_HEIGHT - Constants.ArmConstants.ARM_FLOOR_STANDOFF) / Math.cos(arm.getPivotAngle)){
       //  return true; 
    } else {
      return false;
    }
  }
}
