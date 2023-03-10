// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  private Encoder pivotEncoder; //external encoder
  private DigitalInput pvtLowLimit, pvtHighLimit;
  private boolean isPHighUnplugged = false;
  private boolean isPLowUnplugged = false;
  private TalonSRX pivotMotor;
  /** Creates a new Pivot. */
  public Pivot() {
    pivotMotor = new TalonSRX(Constants.MotorControllers.ID_PIVOT); //check ID number, brushed
    pivotMotor.setInverted(false);
    pivotEncoder = new Encoder(PivotConstants.DIO_PVT_ENC_A, PivotConstants.DIO_PVT_ENC_B);  //using external encoder

    try {
      pvtHighLimit = new DigitalInput(PivotConstants.PVT_HIGH);
    } catch (Exception e) {
      isPHighUnplugged = true;
    }

    try {
      pvtLowLimit = new DigitalInput(PivotConstants.PVT_LOW);
    } catch (Exception e) {
      isPLowUnplugged = true;
    }
  }

  public void pivotStop() {
    pivotMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean isPHighLimit() {
    if (isPHighUnplugged) {
      return true;
    } else {
      return !pvtHighLimit.get();
    }
  }
  public boolean isPLowLimit() {
    if (isPLowUnplugged) {
      return true;
    } else {
      return !pvtLowLimit.get();
    }
  } 

  public void resetPivotEncoder() {
    pivotEncoder.reset();  //external encoder
  }
  
  public double getPivotEncoder() {
    //returns value of encoder in Revs
    // return pivotEncoder.get()/128;  //external encoder
    //returns value of encoder in pulses (multiply by 128 to get Revolutions)
    return pivotEncoder.getRaw();
  }

  public void setPivotSpeed(double speed) {
    pivotMotor.set(ControlMode.PercentOutput, speed);
    if (speed > 0) {
      if (isPHighLimit()) {
        // mast going up and top limit is tripped, stop
        pivotStop();
    // ADD AND TEST THE 3 LINES BELOW AFTER INTIAL TESTS OF MANUAL PIVOT / LIMIT SWITCHES!!!!!!!!!!!! (pt 2)
    //} else if ((Math.cos(arm.getPivotAngle) < 
    //   (Constants.ArmConstants.ARM_FLOOR_STANDOFF / arm.getTotalArmLenght))){
    //    pivotStop(); 
      } else {
        // mast going up but top limit is not tripped, go at commanded speed
        pivotMotor.set(ControlMode.PercentOutput, speed);
      }
    } else {
      if (isPLowLimit()) {
        // mast going down and bottom limit is tripped, stop and zero encoder
        pivotStop();
        resetPivotEncoder();
      } else {
        // mast going down but bottom limit is not tripped, go at commanded speed
        pivotMotor.set(ControlMode.PercentOutput, speed);
      }}}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("pvt encoder", getPivotEncoder());
   SmartDashboard.putBoolean("pvt High", isPHighLimit());
   SmartDashboard.putBoolean("pvt low", isPLowLimit());
    // This method will be called once per scheduler run
  }
}
