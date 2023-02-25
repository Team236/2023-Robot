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
  DoubleSubscriber tvSub,thSub, tXSub, txSub,tySub, areaSub;
  IntegerPublisher outPipeline, outLedMode;
  DoubleArraySubscriber pose;


  /** Creates a new Limelight. */
  public Limelight() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // get the subtable called "datatable"
    NetworkTable datatable = inst.getTable("limelight");

    // subscribe to the topic in "datatable" called "Y"
    // default value is 0

    //tV = 1 if there are any targets found, =0 if not
    //tx = horizontal offset from crosshair to target -27 to +27 degrees
    //ty = vertical offset from crosshair to target -20.5 to +20.5 degrees
    //ta = targeting area (0 to 100), indicates %a of target in total image area
    //ts = rotation/skew of target
    // LED mode for RR tape, Fudicial Markers using CLassic 16h5 standard for AprilTags
    txSub = datatable.getDoubleTopic("tx").subscribe(0.0);
    tySub = datatable.getDoubleTopic("ty").subscribe(0.0);

    tvSub = datatable.getDoubleTopic("tv").subscribe(0.0);
    thSub = datatable.getDoubleTopic("th").subscribe(0.0);

    areaSub = datatable.getDoubleTopic("ta").subscribe(0.0);
    pose = datatable.getDoubleArrayTopic("camtran").subscribe(new double[] {});

    // publish to the topic in "datatable" called "Out"
    outPipeline = datatable.getIntegerTopic("getPipe").publish();
    outLedMode = datatable.getIntegerTopic("ledmode").publish();
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

  public void setLedMode(Integer i) {
    outLedMode.set(i);
  }

  public void setPipeline(Integer i) {
    outPipeline.set(i);
  }

  public double getTx() {
    return txSub.get();
  }

  public double getTy() {
    return tySub.get();
  }
  
  public double getTv() {
    return tvSub.get();
  }

  public double getTh() {
    return thSub.get();
  }

  public double[] getTagData() {
    return pose.get();
   }
}
