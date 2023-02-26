// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  
  NetworkTable table;

  BooleanSubscriber tvSub;
  IntegerSubscriber tid;
  int tag;
  DoubleSubscriber txSub, tySub, areaSub;
  IntegerPublisher pipeline, outLedMode;

  DoubleArraySubscriber blueStationToRobotPose, redStationToRobotPose, targetToCameraPose, cameraToTargetPose; 
  DoubleArraySubscriber robotSpaceToTargetPose, targetToRobotPose, robotCameraPosition;

  /** Creates a new Limelight. */
  public Limelight() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // get the subtable called "datatable"
    var datatable = inst.getTable("limelight");

    // subscribe to the Topics in "datatable"    
    // set default value in subscribe at zero
    tvSub = datatable.getBooleanTopic("tv").subscribe(false);
    tid = datatable.getIntegerTopic("tid").subscribe(0);
    txSub = datatable.getDoubleTopic("tx").subscribe(0.0);
    tySub = datatable.getDoubleTopic("ty").subscribe(0.0);
    areaSub = datatable.getDoubleTopic("ta").subscribe(0.0);

    // publish to the topic in "datatable" called "Out"
    pipeline = datatable.getIntegerTopic("/Limelight/getPipe").publish();
    outLedMode = datatable.getIntegerTopic("/Limelight/ledmode").publish();

    // Robot transform in field-space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
    // DoubleArraySubscriber fieldRobotPose = datatable.getDoubleArrayTopic("botpose").subscribe(new double[] {});

   
    // Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     blueStationToRobotPose = datatable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});

    // Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     redStationToRobotPose = datatable.getDoubleArrayTopic("botpose_wpired").subscribe(new double[] {});

    //	3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6))
     targetToCameraPose = datatable.getDoubleArrayTopic("camerapose_targetspace").subscribe(new double[] {});

    //	3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6))
     cameraToTargetPose = datatable.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {});

    // 3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6))
     robotSpaceToTargetPose = datatable.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[] {});

    // 3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6))
     targetToRobotPose = datatable.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[] {});

    // 3D transform of the camera in the coordinate system of the robot (array (6))
    //  robotCameraPosition = datatable.getDoubleArrayTopic("camerapose_robotspace").subscribe(new double[] {});             

    // ID of the primary in-view AprilTag
     tid = datatable.getIntegerTopic("tid").subscribe(0);
  
    }


public void periodic() {
  // read a double value from Y, and set Out to that value multiplied by 2
  // set the pipiline value to change 
  tag = (int) tid.get();
  SmartDashboard.putBoolean("HasTarget", tvSub.get());
  SmartDashboard.putNumber("TargetNumber", tag);
}

  // often not required in robot code, unless this class doesn't exist for
  // the lifetime of the entire robot program, in which case close() needs to be
  // called to stop subscribing
  public void close() {
  // pipeline.close();
  // outLedMode.close();
  }

  // public void setLedMode(Integer i) { outLedMode.set(i);  
  // }

  public void setPipeline(Integer i) { 
    pipeline.set((long)i);  
  }

  public boolean getTv() { 
     return tvSub.get() ;  
    }

  public double getX() { 
    return txSub.get();  
  }

  public double getY() { 
    return tySub.get();  
  }

  public Long getTagData() { 
    return tid.get();   
  }

// Robot transform in field-space (blue driverstation)
public double[] blueStationToRobotPose() {
  return = blueStationToRobotPose.get(); 
} 

// Robot transform in field-space (blue driverstation)
public double blueStationToRobotPoseX() {
  double[] blue = blueStationToRobotPose.get(); 
  return blue[0];
} 

public double getBlueBotPoseY() { 
  double[] blue = blueStationToRobotPose.get();
  return blue[1];
} 
public double getBlueBotPoseZ() { 
  double[] blue  = blueStationToRobotPose.get();
  return blue[2];
 } 

 public double[] getRedBotPose(){ 
  double[] red = redStationToRobotPose.get();
 }
// Robot transform in field-space (red driverstation)
public double getRedBotPoseX(){ 
  double[] red = redStationToRobotPose.get();
  return red[0];
 } 

public double getRedBotPoseY(){ 
  double[]  red= redStationToRobotPose.get();
  return red[1]; 
} 

public double getRedBotPoseZ(){ 
  double[]  red= redStationToRobotPose.get();
  return red[2]; 
} 	

//	3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6))
public double getTargetToCameraPoseX() {
  var  tcamera = targetToCameraPose.get();
  return tcamera[0];
}

//	3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6))
public double getTargetToCameraPoseY() {
  var  tcamera = targetToCameraPose.get();
  return tcamera[1];
}

//	3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6))
public double getTargetToCameraPoseZ() {
  var  tcamera = targetToCameraPose.get();
  return tcamera[2];
}

public double getCameraToTargetPoseX() {
	double[] ctarget = cameraToTargetPose.get();
	return ctarget[0];
	}
	
	public double getCameraToTargetPoseY() {
	double[] ctarget = cameraToTargetPose.get();
	return ctarget[2];
	}
	
	public double getCameraToTargetPoseZ() {
	double[] ctarget = cameraToTargetPose.get();
	return ctarget[3];
	}

// @TODO need to build the Y and Z for these three
  public double getRobotSpaceToTargetPoseX(){
    double[] rSpace = robotSpaceToTargetPose.get();
    return rSpace[0];
    }
    
      public double getTargetToRobotPoseX()     {
        double[] tRobot = targetToRobotPose.get();
    return tRobot[0];
    }
    
    public double getRobotCameraPositionX()   {
      double[] rcamera = robotCameraPosition.get();
    return rcamera[0];
    }
}