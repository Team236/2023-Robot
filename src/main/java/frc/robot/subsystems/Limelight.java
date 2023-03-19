// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.xml.xpath.XPathEvaluationResult;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.*;

public class Limelight extends SubsystemBase {
  
  NetworkTable datatable;
  NetworkTableInstance inst;
  private double[] defaultCareraPose = {4.0,32.675,3.0,0.0,0.0,0.0};
  private double tag;
  private DoubleSubscriber tvSub,txSub, tid, tySub, areaSub;
  private DoublePublisher pipeline, outLedMode, driverMode;
  private DoubleArrayPublisher robotCameraPosition;  
      
  private DoubleArraySubscriber blueStationToRobotPose, redStationToRobotPose, targetToCameraPose, cameraToTargetPose; 
  private DoubleArraySubscriber robotToTargetPose, targetToRobotPose;

  /** Creates a new Limelight. */
  public Limelight() {
    
    try (var inst = NetworkTableInstance.getDefault()) {
      // get the subtable called "datatable"
      var datatable = inst.getTable("limelight");

      // subscribe to the Topics in "datatable"    
      // set default value in subscribe at zero
      tvSub = datatable.getDoubleTopic("tv").subscribe(0);
      tid = datatable.getDoubleTopic("tid").subscribe(0);

      txSub = datatable.getDoubleTopic("tx").subscribe(0.0);
      tySub = datatable.getDoubleTopic("ty").subscribe(0.0);
      areaSub = datatable.getDoubleTopic("ta").subscribe(0.0);

      // publish to the topic in "datatable" called "Out"
      pipeline = datatable.getDoubleTopic("getPipe").publish();
      pipeline.set(0);
      outLedMode = datatable.getDoubleTopic("ledmode").publish();
      outLedMode.setDefault(0);;
      driverMode = datatable.getDoubleTopic("drivermode").publish();
      driverMode.setDefault(0);

      robotCameraPosition = datatable.getDoubleArrayTopic("camerapose_robotspace").publish();
      robotCameraPosition.setDefault(defaultCareraPose);
      
      //	3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6))
       targetToCameraPose = datatable.getDoubleArrayTopic("camerapose_targetspace").subscribe(new double[] {});

      //	3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6))
       cameraToTargetPose = datatable.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {});
 
      // 3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6))
       robotToTargetPose = datatable.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[] {});
      
      // 3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6))
       targetToRobotPose = datatable.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[] {});  
      
      // Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
       blueStationToRobotPose = datatable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
 
      // Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
       redStationToRobotPose = datatable.getDoubleArrayTopic("botpose_wpired").subscribe(new double[] {});
       
      } // end-try
    
    }


public void periodic() {
  // read a double value from Y, and set Out to that value multiplied by 2
  // set the pipiline value to change 
  tag = tid.get();
  SmartDashboard.putNumber("HasTarget", tvSub.get());
  SmartDashboard.putNumber("TargetNumber", tag);

  //   private double x = 3;
  //   private double y = 4;
  //   private double radius = 5;
  //   private double pivotZoffset = 8;

  //   double xValue = SQRT((radius*radius)-(radius*sin(turret.turretEncoder.get()))^2);
  //   // (SQRT( (5^2)-(5*cos(turret.turretEncoder.get() )^2) )-pivotZoffset 
  //   double yValue = (SQRT((radius*radius)-(radius*cos(turret.turretEncoder.get()))^2))-pivotZoffset;
  //   double zValue = -32.625;

  // if (turret.turretEncoder.get()) == 0 ) {
  //   //  calcualate the x y offset position of the camera as it rotates with turre movement
  //   cameraToTargetPose.getTopic().publish( {-xValue, yValue, zValue ,0.0,0.0,turret.turretEncoder.get()} );

  // } else if (turret.turretEncoder.get() > 0 && turret.turretEncoder.get() <= 90) {
  //   cameraToTargetPose.getTopic().publish( {xValue, yValue, zValue ,0.0,0.0,turret.turretEncoder.get()} );

  // } else if (turret.turretEncoder.get() > 90 && turret.turretEncoder.get() <= 180)  { 
  //   cameraToTargetPose.getTopic().publish( {xValue, -yValue, zValue ,0.0,0.0,turret.turretEncoder.get()} );

  // } else if (turret.turretEncoder.get() > 180 && turret.turretEncoder.get() <= 270)  { 
  //   cameraToTargetPose.getTopic().publish( {-xValue, -yValue, zValue ,0.0,0.0,turret.turretEncoder.get()} );

  // } else if (turret.turretEncoder.get() < 0 )  { 
  //   cameraToTargetPose.getTopic().publish( {-xValue, yValue, zValue ,0.0,0.0,turret.turretEncoder.get()} );

  //  } // end-if

  }

  // often not required in robot code, unless this class doesn't exist for
  // the lifetime of the entire robot program, in which case close() needs to be
  // called to stop subscribing
  public void close() {
  // pipeline.close();
  // outLedMode.close();

  }

  /**
   * @param pipeline
   */
  public void setPipeline(double _pipeline) { 
        pipeline.set((long) _pipeline);  
  }

    // PUBLISH not SUBSCRIBE might not be needed
    public void setCameraRobotPose(double[] _camerposition)   {
      robotCameraPosition.setDefault(_camerposition);;
    }


  public boolean HasTarget() { 
     if ((tvSub.get()== 0.0 ))
      {return false;}
     else { return true; }
    }

  public double getX() { 
    return txSub.get();  
  }

  public double getY() { 
    return tySub.get();  
  }

  public double getTagData() { 
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