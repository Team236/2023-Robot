// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;


public class BrakeReleaseToggle extends CommandBase {
  
private Turret turret;
private boolean toggle;

  /** Creates a new TurretBrakeRelease. */
  public BrakeReleaseToggle(Turret _toggleTurret){
      this.turret = _toggleTurret;
      addRequirements(turret);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    toggle = false;
    
   if (turret.isBraking()) {
    turret.turretBrake();
    toggle = true;
    } else if (!turret.isBraking()) {
    turret.turretRelease();
    toggle = true;
   }
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
