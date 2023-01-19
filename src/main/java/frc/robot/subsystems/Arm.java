// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  
  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private SparkMaxPIDController armPID;
  private DigitalInput armReturnLimit, armExtendLimit;
  private boolean isAReturnUnplugged = false;
  private boolean isAExtendUnplugged = false;
  /** Creates a new ArmExtend. */
  public Arm() {

    armMotor = new CANSparkMax(Constants.MotorControllers.ID_ARM, MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    armMotor.setInverted(false);

    armPID = armMotor.getPIDController();
    armEncoder = armMotor.getEncoder();

    
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

  }

  public void armOut() {
    armMotor.set(ArmConstants.ARM_EX_SPEED);
  }

  public void armIn() {
    armMotor.set(-ArmConstants.ARM_RE_SPEED);
  }

  public void armStop() {
    armMotor.set(0);
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

  public void resetArmEncoder() {
    armEncoder.setPosition(0);
  }

  //returns encoder position in REVOLUTIONS 
    public double getArmEncoder() {
     return armEncoder.getPosition();
  }

  public void setArmSetPoint(double armDistance) {
    armPID.setReference(armDistance, ControlType.kPosition);
  }

  public void setArmSetPointWLimit (double armDistance) {
    if (armExtendLimit.get() || armReturnLimit.get()) {
      armStop();
    } else {
      armPID.setReference((armDistance * ArmConstants.armIN_TO_REV), ControlType.kPosition);
    }
  }

  public void setArmkP(double kParm){
    armPID.setP(ArmConstants.kParm);
  }

  public void setArmkI (double kIarm) {
    armPID.setI(kIarm);
  }

  public void setArmkD(double kDarm){
    armPID.setD(kDarm);
  }

  public void setArmkF (double kFarm){
    armPID.setFF(kFarm);
  }

  public void setArmOutputRange () {
    armPID.setOutputRange(Constants.ArmConstants.armMIN_OUTPUT, Constants.ArmConstants.armMAX_OUTPUT);
  }

  public double getArmDistance() {
    return  getArmEncoder() * ArmConstants.armREV_TO_IN;
  } 
  
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



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm encoder", getArmEncoder());

    SmartDashboard.putBoolean("arm extend limit", isAExtendLimit());
    SmartDashboard.putBoolean("arm return limit", isAReturnLimit());
    SmartDashboard.putNumber("P value", ArmConstants.kParm);
    SmartDashboard.putNumber("I Value", ArmConstants.kIarm);
    SmartDashboard.putNumber("D Value", ArmConstants.kDarm);
  }
}
