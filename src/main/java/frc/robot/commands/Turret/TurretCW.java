// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class TurretCW extends CommandBase {
  private Turret turret2;
  private double speed;
  /** Creates a new TurretClockwise. */
  public TurretCW(Turret turretcw, double speed7) {
    this.turret2 = turretcw;
    this.speed = speed7;
    addRequirements(turret2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret2.setTurretSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret2.turretStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    /*if ((speed > 0.008) && turre2.isTLimit()) {
      // if mast is going up and top limit is triggered
      // the 0.008 is because when the axis is at rest, it reads 0.0078125 so doing speed > 0.008 acts as a deadzone
      return true;
    } else if ((speed < 0) && turret2.isTLimit()) {
      turret2.resetTurretEncoder();
      return true;
    } else {
      return false;
    }*/
  }
  }
