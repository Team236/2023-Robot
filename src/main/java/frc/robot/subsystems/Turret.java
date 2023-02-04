// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Turret extends SubsystemBase {
  private CANSparkMax turretMotor;
  private RelativeEncoder turretEncoder; //WILL BE DIFFERENT
  //private Encoder turretEncoder;
  private DigitalInput turretLimit1, turretLimit2;
  private boolean isT1Unplugged = false;
  private boolean isT2Unplugged = false;
  /** Creates a new Turret. */
  public Turret() {
    turretMotor = new CANSparkMax(Constants.MotorControllers.ID_TURRET, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setInverted(false);

    turretEncoder = turretMotor.getEncoder(); 
    //turretEncoder = new Encoder(TurretConstants.DIO_TRRT_ENC_A, TurretConstants.DIO_TRRT_ENC_B);

   /*  try {
      turretLimit1 = new DigitalInput(TurretConstants.DIO_TURRET_CW_LIMIT);
    } catch (Exception e) {
      isT1Unplugged = true;
    }
    try {
      turretLimit2 = new DigitalInput(TurretConstants.DIO_TURRET_CCW_LIMIT);
    } catch (Exception e) {
      isT2Unplugged = true;
    } */

  }

  public boolean isTCWLimit() {
    if (isT1Unplugged) {
      return true;
    } else {
      return !turretLimit1.get();
    }
  }
  
  public boolean isTCCWLimit() {
    if (isT2Unplugged) {
      return true;
    } else {
      return !turretLimit2.get();
    }
  }

  public void resetTurretEncoder() {
    turretEncoder.setPosition(0);
   // turretEncoder.reset();
  }
  public double getTurretEncoder() {
    return turretEncoder.getPosition();
   // return turretEncoder.get()/128;  //128 ticks per rev, returns REVS
  }
  public double getTurretAngle() {
    return  getTurretEncoder() * TurretConstants.turretREV_TO_DEG;
  } 
  
  public void setTurretSpeed(double speed) {
    if (speed > 0) {
      if (isTCWLimit()) {
        // mast going up and top limit is tripped, stop
        turretStop();
      } else {
        // mast going up but top limit is not tripped, go at commanded speed
        turretMotor.set(speed);
      }
    } else {
      if (isTCCWLimit()) {
        // mast going down and bottom limit is tripped, stop and zero encoder
        turretStop();
        resetTurretEncoder();
      } else {
        // mast going down but bottom limit is not tripped, go at commanded speed
        turretMotor.set(speed);
      }
    }
  }
  public void turretStop() {
    turretMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("turret encoder", getTurretEncoder());
    SmartDashboard.putNumber("turret angle", getTurretAngle());
    SmartDashboard.putBoolean("turret CW limit", isTCWLimit());
    SmartDashboard.putBoolean("turret CCW limit", isTCCWLimit());
  }
}
