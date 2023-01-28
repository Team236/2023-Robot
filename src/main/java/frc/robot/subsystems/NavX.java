// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {
  /** Creates a new NavX. */
  private final AHRS navx = new AHRS(SPI.Port.kMXP);
  public NavX() {}

  public double navxgetz(){
    return navx.getRawGyroZ();
  }
  public double navxgetza(){
    return navx.getRawAccelZ();
  }
  public double navxgetyaw(){
    return navx.getYaw();
  }
  public double navxgety(){
    return navx.getRawGyroY();
  }
  public double navxgetya(){
    return navx.getRawAccelY();
  }
  public double navxgetroll(){
    return navx.getRoll();
  }
  public double navxgetx(){
    return navx.getRawGyroX();
  }
  public double navxgetxa(){
    return navx.getRawAccelX();
  }
  public double navxgetpitch(){
    return navx.getPitch();
  }
  public double getangle(){
    return navx.getAngle();
  }

  public void navxreset(){
  navx.reset();
  }

  public boolean isconnected(){
   return navx.isConnected();
  }

  public double getrate(){
    return navx.getRate();
  }

  public double magx(){
    return navx.getRawMagX();
  }

  
  public double magy(){
    return navx.getRawMagY();
  }

  
  public double magz(){
    return navx.getRawMagZ();
  }

  
  public double heading(){
    return navx.getCompassHeading();
  }

  public boolean iscalibrating(){
    return navx.isCalibrating();
  }

  public void calibrate(){
    navx.calibrate();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Yaw1", navxgetyaw());
    SmartDashboard.putNumber("Roll1", navxgetroll());
    SmartDashboard.putNumber("Pitch1", navxgetpitch());
    SmartDashboard.putNumber("magx", magx());
    SmartDashboard.putNumber("magy", magy());
    SmartDashboard.putNumber("magz", magz());
    SmartDashboard.putNumber("Angle1", getangle());
    SmartDashboard.putBoolean("is Connected", isconnected());
    SmartDashboard.putNumber("heading", heading());
    SmartDashboard.putBoolean("isCalibrating", iscalibrating());
    // This method will be called once per scheduler run
  }
}
