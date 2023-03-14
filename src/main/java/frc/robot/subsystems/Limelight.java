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

  IntegerSubscriber tid;
  int tag;
  DoubleSubscriber tvSub,txSub, tySub, areaSub;
  IntegerPublisher pipeline, outLedMode,driverMode;

  DoubleArraySubscriber blueStationToRobotPose, redStationToRobotPose, targetToCameraPose, cameraToTargetPose; 
  DoubleArraySubscriber robotToTargetPose, targetToRobotPose, robotCameraPosition;

  /** Creates a new Limelight. */
  public Limelight() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // get the subtable called "datatable"
    var datatable = inst.getTable("limelight");

    // subscribe to the Topics in "datatable"    
    // set default value in subscribe at zero
    tvSub = datatable.getDoubleTopic("tv").subscribe(0);
    tid = datatable.getIntegerTopic("tid").subscribe(0);

    txSub = datatable.getDoubleTopic("tx").subscribe(0.0);
    tySub = datatable.getDoubleTopic("ty").subscribe(0.0);
    areaSub = datatable.getDoubleTopic("ta").subscribe(0.0);

    // publish to the topic in "datatable" called "Out"
    pipeline = datatable.getIntegerTopic("getPipe").publish();
    outLedMode = datatable.getIntegerTopic("ledmode").publish();
    driverMode = datatable.getIntegerTopic("drivermode").publish();

 
    //	3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6))
     targetToCameraPose = datatable.getDoubleArrayTopic("camerapose_targetspace").subscribe(new double[] {});

    //	3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6))
     cameraToTargetPose = datatable.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {});
 
    // 3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6))
     robotToTargetPose = datatable.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[] {});
    
    // 3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6))
     targetToRobotPose = datatable.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[] {});
    
     // this will be publish or preset in limelight pipelines
     // 3D transform of the camera in the coordinate system of the robot (array (6))
    //  robotCameraPosition = datatable.getDoubleArrayTopic("camerapose_robotspace").subscribe(new double[] {});       
    
    // Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     blueStationToRobotPose = datatable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
 
    // Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     redStationToRobotPose = datatable.getDoubleArrayTopic("botpose_wpired").subscribe(new double[] {});
    
  
    }


public void periodic() {
  // read a double value from Y, and set Out to that value multiplied by 2
  // set the pipiline value to change 
  tag = (int) tid.get();
  SmartDashboard.putNumber("HasTarget", tvSub.get());
  SmartDashboard.putNumber("TargetNumber", tag);
  }

  // often not required in robot code, unless this class doesn't exist for
  // the lifetime of the entire robot program, in which case close() needs to be
  // called to stop subscribing
  public void close() {
  // pipeline.close();
  // outLedMode.close();
  }

  /**
   * @param pipeline1
   */
  public void setPipeline(double pipeline1) { 
    pipeline.set((long)pipeline1);  
  }

  /**
   * @return
   */
  public boolean HasTarget() { 
    if ((tvSub.get() > 0 ))
    {
      return true;}
    else {
      return false;
    }
      
    }

  /**
   * @return
   */
  public double getX() { 
    return txSub.get();  
  }

  public double getY() { 
    return tySub.get();  
  }

  public Long getTagData() { 
    return tid.get();   
  }

  public void setLedModeOn() {
    outLedMode.set(1);
  }
  
  public void setLedModeOff() {
    outLedMode.set(0);
  }
  
  public void setDriverModeOn() {
    driverMode.set(1);
  }
  
  public void setDriverModeOff() {
    driverMode.set(0);
  }
// 3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6))
public double[] getTargetToCameraPose() {
  return targetToCameraPose.get();
}

    //	Y distance of the camera in the coordinate system of the primary in-view AprilTag 
    public double getTargetToCameraPoseX() {
      var  tcamera = targetToCameraPose.get();
      return tcamera[0];
    }

    //	Y distance of the camera in the coordinate system of the primary in-view AprilTag 
    public double getTargetToCameraPoseY() {
      var  tcamera = targetToCameraPose.get();
      return tcamera[1];
    }

    //	Z distance of the camera in the coordinate system of the primary in-view AprilTag 
    public double getTargetToCameraPoseZ() {
      var  tcamera = targetToCameraPose.get();
      return tcamera[2];
}

// 3D transform of the primary in-view AprilTag in the coordinate system of the Camera 
public double[] getCameraToTargetPose() {
	return cameraToTargetPose.get();
	}

      //	X distance of the AprilTag in the coordinate system of the Camera 
      public double getCameraToTargetPoseX() {
      double[] ctarget = cameraToTargetPose.get();
      return ctarget[0];
      }

      //	Y distance of the AprilTag in the coordinate system of the Camera 
      public double getCameraToTargetPoseY() {
      double[] ctarget = cameraToTargetPose.get();
      return ctarget[1];
      }
      
      //	Z distance of the AprilTag in the coordinate system of the Camera 
      public double getCameraToTargetPoseZ() {
      double[] ctarget = cameraToTargetPose.get();
      return ctarget[2];
      }

  // PUBLISH not SUBSCRIBE might not be needed
  // public double getRobotToCameraPoseX()   {
  //   double[] rcamera = robotCameraPosition.get();
  // return rcamera[0];
  // }

// 3D transform of the primary in-view AprilTag in the coordinate system of the Robot 
public double[] getTargetToRobotPose()     {
  return targetToRobotPose.get();
}
    //	X distance of the AprilTag in the coordinate system of the Robot
    public double getTargetToRobotPoseX()     {
      double[] tRobot = targetToRobotPose.get();
    return tRobot[0];
    }
    //	Y distance of the AprilTag in the coordinate system of the Robot
    public double getTargetToRobotPoseY()     {
      double[] tRobot = targetToRobotPose.get();
    return tRobot[1];
    }
    //	Z distance of the AprilTag in the coordinate system of the Robot
    public double getTargetToRobotPoseZ()     {
      double[] tRobot = targetToRobotPose.get();
    return tRobot[2];
    }
  // 3D transform of the robot in the coordinate system of the primary in-view AprilTag 
  public double[] getRobotToTargetPose(){
    return robotToTargetPose.get();
    }
      //	X distance of the robot in the coordinate system of the primary in-view AprilTag 
      public double getRobotToTargetPoseX(){
        double[] rSpace = robotToTargetPose.get();
        return rSpace[0];
        }
      //	Y distance of the robot in the coordinate system of the primary in-view AprilTag 
      public double getRobotToTargetPoseY(){
        double[] rSpace = robotToTargetPose.get();
        return rSpace[1];
        }
      //	Z distance of the robot in the coordinate system of the primary in-view AprilTag 
      public double getRobotToTargetPoseZ(){
        double[] rSpace = robotToTargetPose.get();
        return rSpace[2];
        }



    // Robot transform in field-space (blue driverstation)
public double[] getBlueStationToRobotPose() {
  return blueStationToRobotPose.get(); 	 
} 

    // Robot transform in field-space (blue driverstation)
    public double getBlueStationToRobotPoseX() {
      double[] blue = blueStationToRobotPose.get(); 
      return blue[0];
    } 

    public double getBlueStationToRobotPoseY() { 
      double[] blue = blueStationToRobotPose.get();
      return blue[1];
    } 
    public double getBlueStationToRobotPoseZ() { 
      double[] blue  = blueStationToRobotPose.get();
      return blue[2];
    } 

 public double[] getRedStationToRobotPose(){ 
  return redStationToRobotPose.get();
 }

    // Robot transform in field-space (red driverstation)
    public double getRedStationToRobotPoseX(){ 
      double[] red = redStationToRobotPose.get();
      return red[0];
    } 

    public double getRedStationToRobotPoseY(){ 
      double[]  red= redStationToRobotPose.get();
      return red[1]; 
    } 

    public double getRedStationToRobotPoseZ(){ 
      double[]  red= redStationToRobotPose.get();
      return red[2]; 
    } 	

}