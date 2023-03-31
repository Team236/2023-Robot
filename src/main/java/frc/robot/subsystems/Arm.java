// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder; //SparkMax encoder
  private DigitalInput armExtendLimit, armReturnLimit;
  private boolean isArmExtLimitUnplugged = false;
  private boolean isArmRetLimitUnplugged = false;
  /** Creates a new ArmExtend. */
  public Arm() {

    armMotor = new CANSparkMax(Constants.MotorControllers.ID_ARM, MotorType.kBrushless); 
    armMotor.setSmartCurrentLimit(40);

    armMotor.restoreFactoryDefaults();
    armMotor.setInverted(false);
    ////pivotMotor.restoreFactoryDefaults(); //I don't think we should use this - n/a for Talon ??
    armEncoder = armMotor.getEncoder(); //will use SparkMax encoder for arm extend/retract
    try {
      armExtendLimit = new DigitalInput(ArmConstants.DIO_ARM_EXTEND);
    } catch (Exception e) {
      isArmExtLimitUnplugged = true;
    }
    try {
      armReturnLimit = new DigitalInput(ArmConstants.DIO_ARM_RETURN);
    } catch (Exception e) {
      isArmRetLimitUnplugged = true;
    }
  }

  public void closedRampRate() {
    armMotor.setClosedLoopRampRate(0.08);
  }

  public void openRampRate() {
    armMotor.setOpenLoopRampRate(0.08);
  }
  public void armStop() {
    armMotor.set(0);;
  }

  public boolean isAExtLimit() {
    if (isArmExtLimitUnplugged) {
      return true;
    } else {
      return !armExtendLimit.get();
    }
  }
  public boolean isARetLimit() {
    if (isArmRetLimitUnplugged) {
      return true;
    } else {
      return !armReturnLimit.get();
    }
  }
  
  

  public void resetArmEncoder() {
    armEncoder.setPosition(0); //SparkMax encoder
  }
 
  //returns encoder position in REVOLUTIONS 
    public double getArmEncoder() {
    return armEncoder.getPosition(); //SparkMax encoder
  }

  public double getArmDistance() {
    return  getArmEncoder() * ArmConstants.armREV_TO_IN;
  } 



 //NO LINEAR RELATION BETWEEN PIVOT ANGLE AND ENCODER _ CANNOT USE THIS METHOD
 //NEED TO RECORD PIVOT ENCODER VALUES AT VARIOUS ANGLES AND USE PID COMMANDS WITH THE DESIRED VALUES PASSED IN
 /*  public double getPivotAngle() {
     //could also use turretEncoder.getDistance() here, since dist per pulse is provided at top of this subystem
    return (getPivotEncoder() + PivotConstants.PIVOT_OFFSET_ANGLE)* PivotConstants.pvtDISTANCE_PER_PULSE;
  } //was told this is sketch, need to fix revtodeg constant (becoming unusable w talon) during bench testing*/

  public double getTotalArmLength(){
    return (getArmEncoder() * ArmConstants.armREV_TO_IN + ArmConstants.RETRACTED_ARM_LENGTH);
  }

  public void setArmSpeed(double speed) {
    armMotor.set(speed);
  if (speed > 0) {
    if (isAExtLimit()) {
        // arm ext/ret limit is tripped going forward, stop 
        // there is 1 magnetic switch and 2 magnets, tripped forward means fully extended
        armStop();
    // ADD AND TEST THE 3 LINES BELOW AFTER INTIAL TESTS OF MANUAL ARM / LIMIT SWITCHES!!!!!!!!!!
    // } else if (arm.getTotalArmLength() > 
    //   (Constants.ArmConstants.MAST_HEIGHT - Constants.ArmConstants.ARM_FLOOR_STANDOFF) / Math.cos(arm.getPivotAngle)){
    //     armStop(); 
     }  else {
        // mast going up but top limit is not tripped, go at commanded speed
        armMotor.set(speed);
      }
    } else {
      if (isARetLimit()) {
        // arm retracting and limit is tripped, stop and zero encoder
        //this means fully retracted
        armStop();
        resetArmEncoder();
      } else {
        // arm retracting but fully retracted limit is not tripped, go at commanded speed
        armMotor.set(speed);
      }
     }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   //SmartDashboard.putNumber("arm encoder", getArmEncoder());
   SmartDashboard.putBoolean("arm extend limit", isAExtLimit());
    SmartDashboard.putBoolean("arm ret lim", isARetLimit());
   SmartDashboard.putNumber("armDist", getArmDistance());
   //SmartDashboard.putNumber("arm Encoder: ", getArmEncoder());
  }
}