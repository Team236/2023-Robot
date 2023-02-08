// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gripper;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gripper;

public class ColorSensorGripper extends CommandBase {
  ColorSensorV3 colorSensor;
  Color foundColor;
  Drive drive;

  /** Creates a new ColorSensorGripper. */
  public ColorSensorGripper(Drive passed_drive) {
    this.drive = passed_drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(passed_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    I2C.Port colorSensorPort = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(colorSensorPort);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    foundColor = colorSensor.getColor();
    double redAmount = foundColor.red;
    double redTarget = 0.5;
  
    // If it is a red object
    if (redAmount > redTarget) {
       // do something
    } else {
      // do something else
    }
    
    SmartDashboard.putNumber("R", foundColor.red);
    SmartDashboard.putNumber("G", foundColor.green);
    SmartDashboard.putNumber("B", foundColor.blue);
    SmartDashboard.putNumber("IR", colorSensor.getIR());
    SmartDashboard.putNumber("proximity", colorSensor.getProximity());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
