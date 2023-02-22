// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  NetworkTable table;
  DoubleSubscriber xSub,ySub,areaSub;
  IntegerPublisher outPipeline,outLedMode;
  DoubleArraySubscriber pose;

  /** Creates a new Limelight. */
  public Limelight() {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  // get the subtable called "datatable"
  NetworkTable datatable = inst.getTable("limelight");

  // subscribe to the topic in "datatable" called "Y"
  // default value is 0
  xSub = datatable.getDoubleTopic("tx").subscribe(0.0);
  ySub = datatable.getDoubleTopic("ty").subscribe(0.0);
  areaSub = datatable.getDoubleTopic("ta").subscribe(0.0);
  areaSub = datatable.getDoubleTopic("ta").subscribe(0.0);
  areaSub = datatable.getDoubleTopic("ta").subscribe(0.0);
// pose = datatable.getDoubleArrayTopic("camtran").sub(0.0);

  // xxxxxSub = datatable.getDoubleTopic("xxxxx").subscribe(0.0);
  
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
// public void close() {
//   ySub.close();
//   outPub.close();
// }

public void setLedMode(limelight.ledmode mode){
outLedMode.set(mode);
}



  public void setPipeline( ){
    outPipeline.set(0);
  }   

  public double getX() {
    return xSub.get();
  }

  public double getY(){
    return ySub.get();
  }

  // public double[6] getTagData() {  }
}
