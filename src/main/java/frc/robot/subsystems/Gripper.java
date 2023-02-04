// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
  private DoubleSolenoid gripperSolenoid; 
  private Counter gripperEye;
  private boolean isGripperEyeUnplugged = false;

  /** Creates a new Gripper. */
  public Gripper() {
    gripperSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
    Constants.GripperConstants.GRIPPER_SOL_FOR, Constants.GripperConstants.GRIPPER_SOL_REV);

    try {
      gripperEye = new Counter();
      gripperEye.setUpSource(Constants.GripperConstants.DIO_GRIPPER_EYE);
    } catch (Exception e) {
      isGripperEyeUnplugged = true;
    }
    gripperEye.reset();
  }

  //Makes the gripper grab
  public void grab() {
    gripperSolenoid.set(Value.kForward);
    }
  
    //makes the gripper release
    public void release() {
      gripperSolenoid.set(Value.kReverse);
    }
  
    //Tells if the gripper is gripping or not
  public boolean isGripping() {
    if (gripperSolenoid.get() == Value.kForward) {
    return true;  
    } else {
      return false;
    }
  }
  
  
  //resets the gripper eye counter
  public void resetGripperEyeCount() {
    gripperEye.reset();
  }

  public void autoGrab() {
    if (getGripperEyeCount() >= 1) {
      grab();
      SmartDashboard.putNumber("Eye Count ABOVE ZER0 so should Grab", getGripperEyeCount());
    }
    else {
      SmartDashboard.putNumber("Eye Count not above zero", getGripperEyeCount());
    }
  }
  
  //gets the gripper eye count
  public int getGripperEyeCount() {
   if (isGripperEyeUnplugged) {
    SmartDashboard.putBoolean("Gripper eye unplugged", isGripperEyeUnplugged);
    return 0;
   } else {
    return gripperEye.get();
   }
  
 
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
