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
  private TalonSRX pivotMotor;

  private RelativeEncoder armEncoder; //SparkMax encoder
  private Encoder pivotEncoder; //external encoder
  private DigitalInput armExtendLimit, armReturnLimit, pvtLowLimit, pvtHighLimit;
  private boolean isArmExtLimitUnplugged = false;
  private boolean isArmRetLimitUnplugged = false;
  private boolean isPHighUnplugged = false;
  private boolean isPLowUnplugged = false;
  /** Creates a new ArmExtend. */
  public Arm() {

    armMotor = new CANSparkMax(Constants.MotorControllers.ID_ARM, MotorType.kBrushless);
    pivotMotor = new TalonSRX(Constants.MotorControllers.ID_PIVOT); //check ID number, brushed 

    armMotor.restoreFactoryDefaults();
    armMotor.setInverted(false);
    ////pivotMotor.restoreFactoryDefaults(); //I don't think we should use this - n/a for Talon ??
    pivotMotor.setInverted(false);
    armEncoder = armMotor.getEncoder(); //will use SparkMax encoder for arm extend/retract
    pivotEncoder = new Encoder(PivotConstants.DIO_PVT_ENC_A, PivotConstants.DIO_PVT_ENC_B);  //using external encoder
   

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

  public void resetArmEncoder() {
    armEncoder.setPosition(0); //SparkMax encoder
  }
  
  public void resetPivotEncoder() {
    pivotEncoder.reset();  //external encoder
  }
 
  //returns encoder position in REVOLUTIONS 
    public double getArmEncoder() {
    return armEncoder.getPosition(); //SparkMax encoder
  }

  public double getArmDistance() {
    return  getArmEncoder() * ArmConstants.armREV_TO_IN;
  } 

  public double getPivotEncoder() {
    //returns value of encoder in Revs
    // return pivotEncoder.get()/128;  //external encoder
    //returns value of encoder in pulses (multiply by 128 to get Revolutions)
    return pivotEncoder.getRaw();
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
      }
    }
    }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   //SmartDashboard.putNumber("arm encoder", getArmEncoder());
  SmartDashboard.putNumber("pvt encoder", getPivotEncoder());
   SmartDashboard.putBoolean("pvt High", isPHighLimit());
   SmartDashboard.putBoolean("pvt low", isPLowLimit());
   // SmartDashboard.putBoolean("arm extend limit", isAExtLimit());
    //SmartDashboard.putBoolean("arm ret lim", isARetLimit());
   //SmartDashboard.putNumber("armDist", getArmDistance());
  }
}