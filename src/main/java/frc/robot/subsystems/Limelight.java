// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  NetworkTable table;

  /** Creates a new Limelight. */
  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    // NetworkTableInstance table = NetworkTableInstance.getDefault();
  }

  @Override
  public void periodic() {
    double number = table.getValue());

    // getDoubleTopic().getDefault().getTable("limelight").getNumber("tx", 0);
    // This method will be called once per scheduler run
  }

  public void setPipeline(int){
    table.putValue(getName("pipeline"), int);
    return;
  } 

  public double getX() {
    var value  = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx"));
    return value.getDouble(getArea());
  }

  public double getY(){
    // var targetOffsetAngle_Vertical = table.getNumber("ty", 0);
    return = table.getEntry("ty");
  }

  public double[6] getTagData() {
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("").getDoubleArray(new double[6]);
  }

  public double getArea(){
    
  }

    public double getAdvanced_RotationToTarget(Advanced_Target raw) {
        NetworkTableEntry txRaw = table.getEntry("tx" + Integer.toString(raw.getValue()));
        double x = txRaw.getDouble(0.0);
        return x;
    }

    // public double getAdvanced_degVerticalToTarget(Advanced_Target raw) {
    //     NetworkTableEntry tyRaw = m_table.getEntry("ty" + Integer.toString(raw.getValue()));
    //     double y = tyRaw.getDouble(0.0);
    //     return y;
    // }

    // public double getAdvanced_TargetArea(Advanced_Target raw) {
    //     NetworkTableEntry taRaw = m_table.getEntry("ta" + Integer.toString(raw.getValue()));
    //     double a = taRaw.getDouble(0.0);
    //     return a;
    // }
    
    // public double getAdvanced_Skew_Rotation(Advanced_Target raw) {
    //     NetworkTableEntry tsRaw = m_table.getEntry("ts" + Integer.toString(raw.getValue()));
    //     double s = tsRaw.getDouble(0.0);
    //     return s;
    // }

    // //Raw Crosshairs:
    // //If you are using raw targeting data, you can still utilize your calibrated crosshairs:
    
    // public double[] getAdvanced_RawCrosshair(Advanced_Crosshair raw){
    //     double[] crosshars = new double[2];
    //     crosshars[0] = getAdvanced_RawCrosshair_X(raw);
    //     crosshars[1] = getAdvanced_RawCrosshair_Y(raw);
    //     return crosshars;
    // }
    // public double getAdvanced_RawCrosshair_X(Advanced_Crosshair raw) {
    //     NetworkTableEntry cxRaw = m_table.getEntry("cx" + Integer.toString(raw.getValue()));
    //     double x = cxRaw.getDouble(0.0);
    //     return x;
    // }

    // public double getAdvanced_RawCrosshair_Y(Advanced_Crosshair raw) {
    //     NetworkTableEntry cyRaw = m_table.getEntry("cy" + Integer.toString(raw.getValue()));
    //     double y = cyRaw.getDouble(0.0);
    //     return y;
    // }


}




