// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Arm extends SubsystemBase {
  private CANSparkMax armMotor, pivotMotor;
  //private Encoder armEncoder, pivotEncoder; //External
  private RelativeEncoder armEncoder, pivotEncoder; //SparkMax
  private DigitalInput armReturnLimit, armExtendLimit, pvtLowLimit, pvtHighLimit;
  private boolean isAReturnUnplugged = false;
  private boolean isAExtendUnplugged = false;
  private boolean isPHighUnplugged = false;
  private boolean isPLowUnplugged = false;
  /** Creates a new ArmExtend. */
  public Arm() {

    armMotor = new CANSparkMax(Constants.MotorControllers.ID_ARM, MotorType.kBrushless);
    //pivotMotor = new CANSparkMax(Constants.MotorControllers.ID_PIVOT, MotorType.kBrushless); //WILL BE BRUSHED

    armMotor.restoreFactoryDefaults();
    armMotor.setInverted(false);
    //pivotMotor.restoreFactoryDefaults();
    //pivotMotor.setInverted(false);
    armEncoder = armMotor.getEncoder(); //SparkMax
    //pivotEncoder = pivotMotor.getEncoder();
    //armEncoder = new Encoder(ArmConstants.DIO_ARM_ENC_A, ArmConstants.DIO_ARM_ENC_B); //External
    //pivotEncoder = new Encoder(PivotConstants.DIO_PVT_ENC_A, PivotConstants.DIO_PVT_ENC_B);

    
    try {
      armReturnLimit = new DigitalInput(ArmConstants.DIO_ARM_RETURN);
    } catch (Exception e) {
      isAReturnUnplugged = true;
    }

    try {
      armExtendLimit = new DigitalInput(ArmConstants.DIO_ARM_EXTEND);
    } catch (Exception e) {
      isAExtendUnplugged = true;
    }

   /*  try {
      pvtHighLimit = new DigitalInput(PivotConstants.PVT_HIGH);
    } catch (Exception e) {
      isPHighUnplugged = true;
    }

    try {
      pvtLowLimit = new DigitalInput(PivotConstants.PVT_LOW);
    } catch (Exception e) {
      isPLowUnplugged = true;
    }*/

  }
  /*public void pivotStop() {
    pivotMotor.stopMotor();
  }*/
  public void armStop() {
    armMotor.stopMotor();
  }

  public boolean isAReturnLimit() {
    if (isAReturnUnplugged) {
      return true;
    } else {
      return !armReturnLimit.get();
    }
  }
  
  public boolean isAExtendLimit() {
    if (isAExtendUnplugged) {
      return true;
    } else {
      return !armExtendLimit.get();
    }
  }
  /*public boolean isPHighLimit() {
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
  } */

  public void resetArmEncoder() {
    armEncoder.setPosition(0); //SparkMax
    //armEncoder.reset(); //external
  }
  /*public void resetPivotEncoder() {
    pivotEncoder.setPosition(0);
    //pivotEncoder.reset();
  }*/

  //returns encoder position in REVOLUTIONS 
    public double getArmEncoder() {
    return armEncoder.getPosition(); //SparkMax
    //return armEncoder.get() / 128 ; //External 128 ticks per revolution
  }
  /*public double getPivotEncoder() {
    return pivotEncoder.getPosition();
    //return pivotEncoder.get() /128;
  }*/
  public double getArmDistance() {
    return  getArmEncoder() * ArmConstants.armREV_TO_IN;
  } 
  /*public double getPivotAngle() {
    return getArmEncoder() * PivotConstants.pvtREV_TO_DEG;
  }*/
  public void setArmSpeed(double speed) {
    if (speed > 0) {
      if (isAExtendLimit()) {
        // mast going up and top limit is tripped, stop
        armStop();
      } else {
        // mast going up but top limit is not tripped, go at commanded speed
        armMotor.set(speed);
      }
    } else {
      if (isAReturnLimit()) {
        // mast going down and bottom limit is tripped, stop and zero encoder
        armStop();
        resetArmEncoder();
      } else {
        // mast going down but bottom limit is not tripped, go at commanded speed
        armMotor.set(speed);
      }
    }
  }
  /*public void setPivotSpeed(double speed) {
    if (speed > 0) {
      if (isPHighLimit()) {
        // mast going up and top limit is tripped, stop
        pivotStop();
      } else {
        // mast going up but top limit is not tripped, go at commanded speed
        pivotMotor.set(speed);
      }
    } else {
      if (isPLowLimit()) {
        // mast going down and bottom limit is tripped, stop and zero encoder
        pivotStop();
        resetPivotEncoder();
      } else {
        // mast going down but bottom limit is not tripped, go at commanded speed
        pivotMotor.set(speed);
      }
    }*/
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm encoder", getArmEncoder());
    //SmartDashboard.putNumber("pvt encoder", getPivotEncoder());
    //SmartDashboard.putBoolean("pvt High", isPHighLimit());
    //SmartDashboard.putBoolean("pvt low", isPLowLimit());
    SmartDashboard.putBoolean("arm extend limit", isAExtendLimit());
    SmartDashboard.putBoolean("arm return limit", isAReturnLimit());
  }
}
