// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PPivot;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.CommandBase;
public class PivotDown extends CommandBase {
  private Arm arm;
  private XboxController xboxController;
  private double speed;
  /** Creates a new PivotUp. */
  public PivotDown(Arm arm, XboxController xboxController, double speed) {
    this.arm = arm;
    this.xboxController = xboxController;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //arm.setPivotSpeed(-Constants.PivotConstants.pvtSPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // arm.pivotStop();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   /* if ((speed > 0.008) && arm.isPHighLimit()) {
      // if mast is going up and top limit is triggered
      // the 0.008 is because when the axis is at rest, it reads 0.0078125 so doing speed > 0.008 acts as a deadzone
      return true;
    } else if ((speed < 0) && arm.isPLowLimit()) {
      arm.resetPivotEncoder();
      return true;
    } else {
      return false;
    }*/
    return false;

  }
}
