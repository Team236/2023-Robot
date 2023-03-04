// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Turret extends SubsystemBase {
  private CANSparkMax turretMotor;
  private Encoder turretEncoder;
  private DigitalInput turretLimit;
  private boolean isTUnplugged = false;
  /** Creates a new Turret. */
  public Turret() {
    turretMotor = new CANSparkMax(Constants.MotorControllers.ID_TURRET, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setInverted(true);
    turretEncoder = new Encoder(TurretConstants.DIO_TRRT_ENC_A, TurretConstants.DIO_TRRT_ENC_B); //external encoder
    turretEncoder.setDistancePerPulse(TurretConstants.turretDISTANCE_PER_PULSE);
    try {
      turretLimit = new DigitalInput(TurretConstants.DIO_TURRET_LIMIT);
    } catch (Exception e) {
     isTUnplugged = true;
    }
  }

 public boolean isTLimit() {
    if (isTUnplugged) {
      return true;
    } else {
      return !turretLimit.get();
    }
  }

 

  public void resetTurretEncoder() {
    turretEncoder.reset();
  }
  public double getTurretEncoder() {
    return turretEncoder.getRaw();  //returns encoder reading in pulses, not Rev
   //return turretEncoder.get()/128;  //128 ticks per rev, returns REVS
  }
  public double getTurretAngle() {
    //could also use turretEncoder.getDistance() here, since dist per pulse is provided at top of this subystem
    return (getTurretEncoder() - TurretConstants.turretANGLE_OFFSET)* TurretConstants.turretDISTANCE_PER_PULSE;
  } 
  
   public void setTurretSpeed(double speed) {
    //DO NOT REACH LIMIT GOING CW, SO DON'T CHECK LIMIT HERE:
    if (speed > 0 && isTLimit()) {
     turretStop();
    } else if (speed < 0 && isTLimit()) {
        // mast going down and bottom limit is tripped, stop and zero encoder
        turretStop();
        resetTurretEncoder();
      } else {
        // mast going down but bottom limit is not tripped, go at commanded speed
        turretMotor.set(speed);
      }
    } 
    
  /*   public void setTurretSpeed(double speed) {
      turretMotor.set(speed);
    }*/
  public void turretStop() {
    turretMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   SmartDashboard.putNumber("turret encoder", getTurretEncoder());
    SmartDashboard.putNumber("turret angle", getTurretAngle());
    SmartDashboard.putBoolean("turret limit", isTLimit());
  
  }
}