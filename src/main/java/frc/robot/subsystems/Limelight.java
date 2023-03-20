// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {

  private double[] defaultCameraPose;
  private double xValue, yValue, zValue,radius,startAngle, pivotZoffset;
  
  /** Creates a new Limelight. */
  public Limelight() {
    double defaultCameraPose[] = { 4, -32.625, 3, 0.0, 0.0, 0.0 };  // xpos-robot right, ypos-down, zpos-out front , Rx,Ry,Rz
     xValue = defaultCameraPose[0];
     yValue = defaultCameraPose[1];
     zValue = defaultCameraPose[3];
    
     radius = Math.sqrt( xValue*xValue + zValue*zValue );  // 3-4-5 right triangle has 36.87 angle
     startAngle = Math.asin(xValue/radius);   // sin(angle) = opp/hyp
    SmartDashboard.putNumber("starting Angle of turret", startAngle);
     pivotZoffset = 8;  // this is the distance the rotation of the turret center is from the robot center

  // // maintain updates to Network table camera to robot position based on turret rotation 
  // double xValue = SQRT((radius*radius)-(radius*Math.sin(turret.getTurretAngle() ))^2);  // pivot is located on x axis no offset required
    
  //  // (SQRT( (5^2)-(5*cos(turret.turretEncoder.get() )^2) )-pivotZoffset 
  //  double yValue = (SQRT((radius*radius)-(radius*Math.cos(turret.getTurretAngle()))^2))-pivotZoffset;
  //  double zValue = -32.625;
  }

  public void periodic() {
    // read a double value from Y, and set Out to that value multiplied by 2
    // set the pipiline value to change

    // double currentAngle= turret.getTurretAngle();

    // if (currentAngle == 0 ) {
    //   //  calcualate the x y offset position of the camera as it rotates with turre movement
    //   LimelightHelpers.setCameraPose_RobotSpace("", {-xValue, yValue, zValue ,0.0,0.0,currentAngle } );

    // } else if (currentAngle > 0 && currentAngle <= 90 ) {
    //   LimelightHelpers.setCameraPose_RobotSpace("", {xValue, yValue, zValue ,0.0,0.0,currentAngle } );

    // } else if (currentAngle > 90 && currentAngle <= 180 )  { 
    //   LimelightHelpers.setCameraPose_RobotSpace("", {xValue, -yValue, zValue ,0.0,0.0,currentAngle } );

    // } else if (currentAngle > 180 && currentAngle <= 270 )  { 
    //   LimelightHelpers.setCameraPose_RobotSpace("", {-xValue, -yValue, zValue ,0.0,0.0,currentAngle} );

    // } else if (currentAngle < 0 )  { 
    //   LimelightHelpers.setCameraPose_RobotSpace("", {-xValue, yValue, zValue ,0.0,0.0,currentAngle } );
    //  } // end-if_currentAngle
    
  } // end-periodic

  public void ResetDefaultPosition () {
    LimelightHelpers.setCameraPose_RobotSpace("",                      // empty string defaults to "limelight"
    defaultCameraPose[0], defaultCameraPose[1],defaultCameraPose[2],   // fwd, side, up 
    defaultCameraPose[3], defaultCameraPose[4],defaultCameraPose[5]);  // roll pitch , yaw
  }

  // often not required in robot code, unless this class doesn't exist for
  // the lifetime of the entire robot program, in which case close() needs to be
  // called to stop subscribing
  public void close() {   }
  
}