// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class ArmExtend extends CommandBase {
  /** Creates a new ArmExtend. */
  private Arm arm1;
  private double speed1;
  public ArmExtend(Arm armExt, double speedExt) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm1 = armExt;
    this.speed1 = speedExt;
    addRequirements(arm1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm1.setArmSpeed(speed1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm1.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((speed1 > 0.008) && arm1.isAExtLimit()) {
      return true;
    } else if ((speed1 < 0) && arm1.isARetLimit()) {
      arm1.resetArmEncoder();
      return true;
    } else {
      return false;
    }
    //return false;
  }
}