// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

  private DoubleSolenoid pivotSolenoid; 

  /** Creates a new Pivot. */
  public Pivot() {

    pivotSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
    Constants.PivotConstants.PIVOT_SOL_FOR, Constants.PivotConstants.PIVOT_SOL_REV);

  }
    public void extend() {
      pivotSolenoid.set(Value.kForward);
      }
    
      //makes the gripper release
      public void retract() {
        pivotSolenoid.set(Value.kReverse);
      }
    
      //Tells if the gripper is gripping or not
    public boolean isextended() {
      if (pivotSolenoid.get() == Value.kForward) {
      return true;  
      } else {
        return false;
      }
    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}