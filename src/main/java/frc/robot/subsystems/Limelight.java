// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  NetworkTable table;
  DoubleSubscriber xSub, ySub, areaSub;
  IntegerPublisher outPipeline, outLedMode;
  DoubleArraySubscriber robotPose, cameraPose;


  /** Creates a new Limelight. */
  public Limelight() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // get the subtable called "datatable"
    NetworkTable datatable = inst.getTable("limelight");

    // subscribe to the Topics in "datatable"    
    // set default value in subscribe at zero
    tvSub = datatable.getBooleanTopic("tv")subscribe(0.0);
    txSub = datatable.getDoubleTopic("tx").subscribe(0.0);
    tySub = datatable.getDoubleTopic("ty").subscribe(0.0);
    areaSub = datatable.getDoubleTopic("ta").subscribe(0.0);
    // robotPose = datatable.getDoubleArrayTopic("camtran").subscribe(new double[] {});

    // xxxxxSub = datatable.getDoubleTopic("xxxxx").subscribe(0.0);

    // publish to the topic in "datatable" called "Out"
    outPipeline = datatable.getIntegerTopic("getPipe").publish();
    outLedMode = datatable.getIntegerTopic("ledmode").publish();

    
    // Robot transform in field-space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
    double[7] fieldRobotPose = datatable.getDoubleArrayTopic("botpose").subscribe(new double[] {});

    // Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
    double[7] blueRobotPose	 = datatable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});

    // Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
    double[7] redRobotPose = datatable.getDoubleArrayTopic("botpose_wpired").subscribe(new double[] {});

    //	3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6))
    double[6] targetCameraPose = datatable.getDoubleArrayTopic("camerapose_targetspace").subscribe(new double[] {});

    //	3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6))
    double[6] cameraTargetPose = datatable.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {});

    // 3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6))
    double[6] robotSpacePose = datatable.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[] {});

    // 3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6))
    double[6] targetRobotPose = datatable.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[] {});

    // 3D transform of the camera in the coordinate system of the robot (array (6))
    double[6] robotCameraPose = datatable.getDoubleArrayTopic("camerapose_robotspace").subscribe(new double[] {});             

    // ID of the primary in-view AprilTag
    Integer tid = datatable.getIntegerTopic("tid").subscribe(0);
    }


public void periodic() {
  // read a double value from Y, and set Out to that value multiplied by 2
  // set the pipiline value to change 
  
}

  // often not required in robot code, unless this class doesn't exist for
  // the lifetime of the entire robot program, in which case close() needs to be
  // called to stop subscribing
  public void close() {
  // ySub.close();
  // outPub.close();
  }

  public void setLedMode(Integer i) { outLedMode.set(i);  }
  public void setPipeline(Integer i) { outPipeline.set(i);  }
  public boolean getTv() { return tvSub.get();  }
  public double getX() { return xSub.get();  }
  public double getY() { return ySub.get();  }
  public Integer getTagData() { return tid.get();   }

  // Robot transform in field-space
public double getBotposeX() { return fieldRobotPose[0].get(); }
public double getBotposeY() { return fieldRobotPose[1].get(); }
public double getBotposeZ() { return fieldRobotPose[2].get(); }

// Robot transform in field-space (blue driverstation
public double getBotpose_wpiblueX() { return blueRobotPose[0]; } 
public double getBotpose_wpiblueY() { return blueRobotPose[1]; } 
public double getBotpose_wpiblueZ() { return blueRobotPose[2]; } 

// Robot transform in field-space (red driverstation
public double getBotpose_wpiredX(){ return redRobotPose[0]; } 
public double getBotpose_wpiredY(){ return redRobotPose[1]; } 
public double getBotpose_wpiredZ(){ return redRobotPose[2]; } 	

// 3D transform of the camera in the coordinate system of the primary in-view AprilTag
public double getCamerapose_targetspaceX(){return targetCameraPose[0];}
public double getCamerapose_targetspaceY(){return targetCameraPose[1];}
public double getCamerapose_targetspaceZ(){return targetCameraPose[2];}
							
// 3D transform of the primary in-view AprilTag in the coordinate system of the Camera
public double getTargetpose_cameraspaceX(){return cameraTargetPose[0];}
public double getTargetpose_cameraspaceY(){return cameraTargetPose[1];}
public double getTargetpose_cameraspaceZ(){return cameraTargetPose[2];}

// 3D transform of the primary in-view AprilTag in the coordinate system of the Robot
public double getTargetpose_robotspaceX() {return robotSpacePose[0];}
public double getTargetpose_robotspaceY() {return robotSpacePose[1];}
public double getTargetpose_robotspaceZ() {return robotSpacePose[2];}
						    
// 3D transform of the robot in the coordinate system of the primary in-view AprilTag
public double getBotpose_targetspaceX() {return targetRobotPose[0];}
public double getBotpose_targetspaceY() {return targetRobotPose[1];}
public double getBotpose_targetspaceZ() {return targetRobotPose[2];}
						   
// 3D transform of the camera in the coordinate system of the robot
public double getCamerapose_robotspaceX() { return robotCameraPose[0]; }
public double getCamerapose_robotspaceY() { return robotCameraPose[1]; }
public double getCamerapose_robotspaceZ() { return robotCameraPose[2]; }

}