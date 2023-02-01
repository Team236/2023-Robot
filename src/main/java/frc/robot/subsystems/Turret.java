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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Turret extends SubsystemBase {
  private CANSparkMax turretMotor;
  private RelativeEncoder turretEncoder; //WILL BE DIFFERENT
  private DigitalInput turretLimit1, turretLimit2;
  private boolean isT1Unplugged = false;
  private boolean isT2Unplugged = false;
  /** Creates a new Turret. */
  public Turret() {
    turretMotor = new CANSparkMax(Constants.MotorControllers.ID_TURRET, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setInverted(false);

    turretEncoder = turretMotor.getEncoder(); //WILL BE DIFFERENT

    try {
      turretLimit1 = new DigitalInput(TurretConstants.DIO_TURRET_1);
    } catch (Exception e) {
      isT1Unplugged = true;
    }
    try {
      turretLimit2 = new DigitalInput(TurretConstants.DIO_TURRET_2);
    } catch (Exception e) {
      isT2Unplugged = true;
    }

  }

  public void turretClockwise() { //maybe change to left/right? 
    turretMotor.set(TurretConstants.TURRET_SPEED);
  }

  public void turretCounterClockwise() {
    turretMotor.set(-TurretConstants.TURRET_SPEED);
  }

  public void turretStop() {
    turretMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
