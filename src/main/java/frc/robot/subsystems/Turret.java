// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Turret extends SubsystemBase {
  private CANSparkMax turretMotor;
  private Encoder turretEncoder;
  private DoubleSolenoid turretBrake;
  private DigitalInput trrtLimit;
  private boolean isTUnplugged = false;
  /** Creates a new Turret. */
  public Turret() {
    turretMotor = new CANSparkMax(Constants.MotorControllers.ID_TURRET, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setInverted(true);
    turretMotor.setSmartCurrentLimit(40);
    turretEncoder = new Encoder(TurretConstants.DIO_TRRT_ENC_A, TurretConstants.DIO_TRRT_ENC_B); //external encoder
    turretEncoder.setDistancePerPulse(TurretConstants.turretDEGREES_PER_PULSE);

    turretBrake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, TurretConstants.TURRET_BRAKE_FOR, TurretConstants.TURRET_BRAKE_REV);

    try {
      trrtLimit = new DigitalInput(TurretConstants.DIO_TCCW_LIMIT);
    } catch (Exception e) {
      isTUnplugged = true;
    }

}

  public boolean isTLimit() {
    if (isTUnplugged) {
      return true;
    } else {
      return !trrtLimit.get();
    }
  }


  public boolean isCWLimit() {


    if (getTurretAngle() >= Constants.TurretConstants.TURRET_CW_STOP_ANGLE) {
      return true;
    } else {
      return false;
    }
    }

   public boolean isCCWLimit() {
        if (getTurretAngle() <= Constants.TurretConstants.TURRET_CCW_STOP_ANGLE) {
          return true;
        } else {
          return false;
        }
      }

  public void turretStop() {
    turretMotor.set(0);
  }

  public void turretBrake() {
    turretBrake.set(Value.kForward);
  }
  public void turretRelease() {
    turretBrake.set(Value.kReverse);
  }

  public boolean isBraking() {
    //return false;
    if (turretBrake.get() == Value.kForward) {
    return true;  
    } else {
      return false;
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
    return (getTurretEncoder() - TurretConstants.turretANGLE_OFFSET)* TurretConstants.turretDEGREES_PER_PULSE;
  } 
  
   public void setTurretSpeed(double speed) {
    //DO NOT REACH LIMIT GOING CW, STOP AT 320 degress CW
    if ((speed < 0) && isTLimit()) {
      resetTurretEncoder();
}
//DO NOT REACH LIMIT GOING CW, STOP AT 320 degress CW
if ((speed > 0) && isCWLimit()) {
  turretStop();
  turretBrake();
 } else if (speed < 0 && isCCWLimit()) {
     // turret going CCW and  limit is tripped, stop and zero encoder
     turretStop();
    turretBrake();
  } else {
     // not a limit going CCW, and not past 300 degrees going CW, go at commanded speed
    //turretRelease(); //??? is this any better than before???
    turretMotor.set(speed);
 }

    } 
    
    /*public void setTurretSpeed(double speed) {
      turretMotor.set(speed);
    }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("turret encoder", getTurretEncoder());
    SmartDashboard.putNumber("turret Angle", getTurretAngle());
    SmartDashboard.putBoolean("turret magnetic limit switch", isTLimit());
    SmartDashboard.putBoolean("turret CW limit (200) ", isCWLimit());
   SmartDashboard.putBoolean("turretCCW limit (-140) ", isCCWLimit());


  }
}