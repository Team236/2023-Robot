// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class LimelightPosition extends SubsystemBase {
  private Turret turret;
  private double xValue, yValue, zValue , radius, startAngle; 
  private double pivotZoffset; 
  
  /** Creates a new Limelight. */
  public LimelightPosition(Turret _turret) {
    this.turret = _turret;

    // xpos-robot right, ypos-down, zpos-out front , Rx,Ry,Rz  
    xValue = Constants.DEFAULT_CAMER_POSITION[0];
    yValue = Constants.DEFAULT_CAMER_POSITION[1];
    zValue = Constants.DEFAULT_CAMER_POSITION[2];
    pivotZoffset = 5;  // this is the distance the rotation of the turret center is from the robot center

    radius = Math.sqrt( xValue*xValue + zValue*zValue );  // 3-4-5 right triangle has 36.87 angle
    startAngle = Math.asin(xValue/radius);   // sin(angle) = opp/hyp
    SmartDashboard.putNumber("starting Angle of turret", startAngle);
    
  // maintain updates to Network table camera to robot position based on turret rotation 
  double xValue = Math.sqrt((Math.pow(radius,2))-Math.pow((radius*Math.sin(turret.getTurretAngle())),2) );  // pivot is located on x axis no offset required
    
   // (SQRT( (5^2)-(5*cos(turret.turretEncoder.get() )^2) )-pivotZoffset 
   double yValue = (Math.sqrt((radius*radius)-Math.pow((radius*Math.cos(turret.getTurretAngle())),2)))-pivotZoffset;
   double zValue = -32.625;
  }

  public void periodic() {
    // read a double value from Y, and set Out to that value multiplied by 2
    // set the pipiline value to change

    double currentAngle= turret.getTurretAngle();

    if (currentAngle == 0 ) {
      //  calcualate the x y offset position of the camera as it rotates with turre movement
      LimelightHelpers.setCameraPose_RobotSpace("", -xValue, yValue, zValue , 0.0, 0.0, currentAngle  );

    } else if (currentAngle > 0 && currentAngle <= 90 ) {
      LimelightHelpers.setCameraPose_RobotSpace("", xValue, yValue, zValue , 0.0, 0.0, currentAngle  );

    } else if (currentAngle > 90 && currentAngle <= 180 )  { 
      LimelightHelpers.setCameraPose_RobotSpace("", xValue, -yValue, zValue , 0.0, 0.0, currentAngle  );

    } else if (currentAngle > 180 && currentAngle <= 270 )  { 
      LimelightHelpers.setCameraPose_RobotSpace("", -xValue, -yValue, zValue , 0.0, 0.0,currentAngle );

    } else if (currentAngle < 0 )  { 
      LimelightHelpers.setCameraPose_RobotSpace("", -xValue, yValue, zValue , 0.0, 0.0,currentAngle  );
     } // end-if_currentAngle
    
  } // end-periodic

  public void ResetDefaultPosition () {
    LimelightHelpers.setCameraPose_RobotSpace("",                      // empty string defaults to "limelight"
    Constants.DEFAULT_CAMER_POSITION[0], Constants.DEFAULT_CAMER_POSITION[1],Constants.DEFAULT_CAMER_POSITION[2],   // fwd, side, up 
    Constants.DEFAULT_CAMER_POSITION[3], Constants.DEFAULT_CAMER_POSITION[4],Constants.DEFAULT_CAMER_POSITION[5]);  // roll pitch , yaw
  }

  // often not required in robot code, unless this class doesn't exist for
  // the lifetime of the entire robot program, in which case close() needs to be
  // called to stop subscribing
  public void close() {   }
  
}