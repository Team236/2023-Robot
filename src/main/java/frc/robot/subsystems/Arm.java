// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private CANSparkMax armMotor;
  //private Encoder armEncoder; //External
  private RelativeEncoder armEncoder; //SparkMax
  private DigitalInput armReturnLimit, armExtendLimit;
  private boolean isAReturnUnplugged = false;
  private boolean isAExtendUnplugged = false;
  /** Creates a new ArmExtend. */
  public Arm() {

    armMotor = new CANSparkMax(Constants.MotorControllers.ID_ARM, MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    armMotor.setInverted(false);

    //armPID = armMotor.getPIDController();
    armEncoder = armMotor.getEncoder(); //SparkMax
    //armEncoder = new Encoder(ArmConstants.DIO_ARM_A, ArmConstants.DIO_ARM_B); //External

    
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
    armEncoder.setPosition(0); //SparkMax
    //armEncoder.reset(); //external
  }

  //returns encoder position in REVOLUTIONS 
    public double getArmEncoder() {
    return armEncoder.getPosition(); //SparkMax
    //return armEncoder.get(); //External 128 ticks per revolution


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
