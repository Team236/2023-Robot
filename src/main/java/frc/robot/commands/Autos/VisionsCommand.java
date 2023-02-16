// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VisionsCommand extends CommandBase {
   //*Creates a new VisionsCommand.
  PhotonPipelineResult result;
   PhotonCamera lCamera;
  public VisionsCommand(PhotonCamera camera) {
  //  lCamera = camera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*lCamera.setPipelineIndex(1);
    lCamera.setLED(VisionLEDMode.kOn);
    result = lCamera.getLatestResult();

    double xPos = result.getBestTarget().getBestCameraToTarget().getX();
    double yPos = result.getBestTarget().getBestCameraToTarget().getY();
    double zPos = result.getBestTarget().getBestCameraToTarget().getZ();
    double skew =  result.getBestTarget().getSkew();

    SmartDashboard.putBoolean("hasTarget", result.hasTargets());

    if (result.hasTargets()){
      SmartDashboard.putNumber("xPos", xPos);
      SmartDashboard.putNumber("yPos", yPos);
      SmartDashboard.putNumber("zPos", zPos);
      SmartDashboard.putNumber("Skew", skew);
    }
    else{
      SmartDashboard.putNumber("xPos", 0);
      SmartDashboard.putNumber("yPos", 0);
      SmartDashboard.putNumber("zPos", 0);
      SmartDashboard.putNumber("Skew", 0);
      
  } */
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
