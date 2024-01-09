// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    //public static final int USB_LEFT_STICK = 0;
    //public static final int USB_RIGHT_STICK = 1;
    public static final int USB_AUXCONTROLLER = 1; //TEST COMMENT FOR COMMIT
    public static final int USB_DRIVECONTROLLER = 0;
    public static class Thrustmaster {
        public static final int TRIGGER = 1;
        public static final int BUTTON_MIDDLE = 2;
        public static final int BUTTON_LEFT = 3;
        public static final int BUTTON_RIGHT = 4;
        public static final int LEFT_BASE_1 = 11;
        public static final int LEFT_BASE_2 = 16;
        public static final int LEFT_BASE_3 = 13;
        public static final int LEFT_BASE_4 = 14;
        public static final int RIGHT_BASE_5 = 7;
        public static final int RIGHT_BASE_6 = 8;
        public static final int RIGHT_BASE_7 = 5;
        public static final int RIGHT_BASE_8 = 10;

        public static class AxesThrustmaster {
            public static final int X = 0;
            public static final int Y = 1;
            public static final int Z = 2;
            public static final int THROTTLE = 3;
        }       
    }
    public static class XboxController {
      public static final int A = 1;
      public static final int B = 2;
      public static final int X = 3;
      public static final int Y = 4;
      public static final int LB = 5;
      public static final int RB = 6;
      public static final int VIEW = 7;
      public static final int MENU = 8;
      public static final int LM = 9;
      public static final int RM = 10;

      public static class AxesXbox {
        public static final int LX = 0;
        public static final int LY = 1;
        public static final int LTrig = 2;
        public static final int RTrig = 3;
        public static final int RX = 4;
        public static final int RY = 5;
      }
      public class POVXbox {
        public static final int UP_ANGLE = 0;
        public static final int RIGHT_ANGLE = 90;
        public static final int DOWN_ANGLE = 180;
        public static final int LEFT_ANGLE = 270;
      }
    }
    public static class LogitechF310 {
        // ****when controller is in DirectInput mode (use slider on the back of the controller)
        public static final int A = 2;
        public static final int B = 3;
        public static final int X = 1;
        public static final int Y = 4;
        public static final int LB = 5;
        public static final int RB = 6;
        public static final int BACK = 9;
        public static final int START = 10;
        public static final int LEFT_PRESS = 7;
        public static final int RIGHT_PRESS = 8;
        public class AxesController {
            public static final int LEFT_X = 0;
            public static final int LEFT_Y = 1;
            public static final int LT = 2;
            public static final int RT = 3;
            public static final int RIGHT_X = 4;
            public static final int RIGHT_Y = 5;
        }

        public class POVController {
          public static final int UP_ANGLE = 0;
          public static final int RIGHT_ANGLE = 90;
          public static final int DOWN_ANGLE = 180;
          public static final int LEFT_ANGLE = 270;
        }
    }
}
  public static class MotorControllers {

    //placeholder numbers for beginning of season //testbed OG - 2022 old - testbed swapped - 2022 new
    public static final int ID_LEFT_FRONT = 35; //10 - 30 - 11 - 1
    public static final int ID_RIGHT_FRONT = 1; //15 - 43 - 16 - 2 - 31 is fried now
    public static final int ID_LEFT_REAR = 34;// 11 - 44 - 10 - 3
    public static final int ID_RIGHT_REAR = 32; //16 - 45 - 15 - 4

    public static final int ID_ARM = 10;
    public static final int ID_TURRET = 16; //the tested one 
    public static final int ID_PIVOT = 24;
    }


public static class DriveConstants {
  public static final double LEFT_DEADZONE = 0.17; //0.15???
  public static final double RIGHT_DEADZONE = 0.17;
  public static final boolean IS_DEADZONE = true;
  public static final int SOL_HI_GEAR = 2;
  public static final int SOL_LOW_GEAR = 3;

  //robot-specific numbers
  public static final double DIAMETER = 6; 
  public static final double CIRCUMFERENCE = Math.PI * DIAMETER;
  public static final double GEAR_RATIO = 1; //TEMPORARY!!!
  //6.273 - low?? - testbot
  //8.364 - high??? - testbot

  public static final double REV_TO_IN_K = CIRCUMFERENCE / GEAR_RATIO;
  public static final double IN_TO_REV_K = GEAR_RATIO / CIRCUMFERENCE;
  public static final double DISTANCE_PER_PULSE_K =  REV_TO_IN_K/512;
    
  // limelight targeting pipelines and settings
  public static final double PICKUP_STATION_PIPELINE= 0;
  public static final double TAG_STATION_OFFSET_LEFT=0;
  public static final double TAG_CONELEFT_OFFSET_PIPELINE=0;   // used if pipeline contains the left offset
  public static final double TAG_CONERIGHT_OFFSET_PIPELINE=0;  // used if pipeline contains the right offset

  //PID stuff
  public static final double leftkPdrive = 0.022; 
  public static final double leftkIdrive = 0;
  public static final double leftkDdrive = 0;

  public static final double rightkPdrive = 0.022;
  public static final double rightkIdrive = 0;
  public static final double rightkDdrive = 0;

  public static final double kPTurnL = 0.025;
  public static final double kPTurnR = 0.025;
  public static final double kPgyro = 0.02;
  //auto distances
  public static final double AUTO_MARGIN = 0;
  public static final double GRID_TO_CHARGE = 42; //???? Game Manual???
  public static final double GRID_TO_CENTER = 240;
  public static final double MARGIN_GYRO_DRIVE = 3;

  //drive encoder channels
  public static final int DIO_LDRIVE_ENC_A = 18; //was 14
  public static final int DIO_LDRIVE_ENC_B = 19; //was 15
  public static final int DIO_RDRIVE_ENC_A = 13;
  public static final int DIO_RDIRVE_ENC_B = 12;

  //auto selector switches
 public static final int DIO_AUTO_1 = 0;
  public static final int DIO_AUTO_2 = 1;
  public static final int DIO_AUTO_3 = 2; //2
  public static final int DIO_AUTO_4 = 3;

}

public static class ArmConstants { ///FOR TESTBOT: subject to change for final
  public static double armREV_TO_IN = 0.4875; // 1/2 inch per revolution
  public static double armIN_TO_REV = 2.051; //2 revolutions per inch
  public static double MAST_HEIGHT = 41.5; // intsert here the height frpm floor to top of mast, in inches
  public static double RETRACTED_ARM_LENGTH = 26; // insert here length of arm when fully retracted, in inches
  public static double ARM_FLOOR_STANDOFF = 12; // insert here desired minnimum distance from arm to floor, in inches


  public static final int DIO_ARM_RETURN = 22; //7
  public static final int DIO_ARM_EXTEND = 11; //8  was18

  public static final double ARM_MID = 13.25; //inches, arm extend distance for middle level
  public static final double ARM_HIGH = 28.5; //inches, arm extend distance for high level
  public static final double ARM_LOW = 9;  // may need to revise after shortening the arm
  public static final double ARM_90_MID = 5.86; //ADJUST- for when turret at 90 or 270 //11.35
  public static final double ARM_90_HIGH = 24.0; //ADJUST- for when turret at 90 or 270

  public static final double ARM_90_LOW = 9; //ADJUST- for when turret at 90 or 270
  public static double ARM_STOW = 0; //inches, arm extend distance stowed and low level(fully retracted)
  public static double ARM_LOAD_STN = 0; //inches, arm extend distance for getting pieces from loading station
  public static double ARM_PICKUP = 12.5;

  public static final double ARM_EX_SPEED = 0.6;
  public static final double ARM_RE_SPEED = 0.6;

  public static double kParm = 0.23; //0.25
  public static double kIarm = 0;
  public static double kDarm = 0;
  public static double kFarm = 0; //mooooo

  //Use Down constants below when pivot angle less than 90 (gravity is assisting - lower kp)
  public static double kParmDown = 0.12;
  public static double kIarmDown = 0;
  public static double kDarmDown = 0;
}

public static class GripperConstants {
  //Solenoid ports on the PCM (Channels A and B)
  public static final int GRIPPER_SOL_FOR = 1; //0
  public static final int GRIPPER_SOL_REV = 0; //1
  public static final int GRIPPER_SOL2_FOR = 4;
  public static final int GRIPPER_SOL2_REV = 5;
  //Port used on the DIO
  public static final int DIO_GRIPPER_EYE = 10; //0
}

public static class PivotConstants {
//Pivot switches
  public static final int PVT_LOW = 8;
  public static final int PVT_HIGH = 9;
//Pivot Encoder Channels A/B
  public static final int DIO_PVT_ENC_A = 7;
  public static final int DIO_PVT_ENC_B = 6;

//public static final double pvtSPEED = 0.75;

  //NO LINEAR RELATIONSHIP BETWEEN ANGLE AND ENCODER READING FOR PIVOT
  //BELOW ARE VALUES OF PIVOT ENCODER AT VARIOUS ANGLES 
public static final double PVT_ENC_STOW = 0;  //20 degrees?  17 degrees?
public static final double PVT_ENC_PICKUP = 1224;  //29 degrees
public static final double PVT_ENC_45 = 3098; //45 degrees
public static final double PVT_ENC_LOW_SCORE = 4862; //58 degrees
public static final double PVT_ENC_90_LOW_SCORE = 4862; //ADJUST - for when turret at 90 or 270
public static final double PVT_ENC_90 = 8700; //90 degrees - 9289
public static final double PVT_ENC_MID_SCORE = 8900;//95 degrees (7.25" arm extend) //7666
public static final double PVT_ENC_90_MID_SCORE = 9414; //determinied on 3/20/23
public static final double PVT_ENC_99 = 10548;// 99 degrees
public static final double PVT_ENC_HIGH_SCORE = 10650; //104 degrees (24.5" arm extend)
public static final double PVT_ENC_90_HIGH_SCORE = 11188; //ADJUST- for when turret at 90 or 270
public static final double PVT_ENC_LOAD_STN = 9450;// need to double check

public static double kPpvt = 0.0004;//0.0004
public static double kIpvt = 0;
public static double kDpvt = 0;

//Use the constants below for pivot going DOWN (gravity assisting)
public static double kPpvtDown = 0.0001;
public static double kIpvtDown = 0;
public static double kDpvtDown = 0;

}

public static class TurretConstants {
  //DIST PER PULSE BELOW REALLY REPRESENTS DEGREES PER PULSE 
  //DETERMINE DIST PER PULSE BY READING ENCODER VALUES AT VARIOUS ANGLES
  //THE GetTurret ENCODER METHOD READS PULSES, NOT REVOLUTIONS
  //IF WE NEED REV_TO_DEG, IT CAN BE CALCULATED BY MULTIPLYING DIST_PER_PULSE times 128
  public static final double turretRevsToDeg = 10.4136;  //TBD- 128 pulses per Rev
  public static final double turretANGLE_OFFSET = 0;//Encoder pulses reading when arm in front center

  public static final double kPturret = 0.006;
  public static final double kIturret = 0;
  public static final double kDturret = 0;

  public static final int DIO_TCW_LIMIT = 20; //was 16
  public static final int DIO_TCCW_LIMIT = 21; //:)
  public static final int DIO_TRRT_ENC_A = 5;
  public static final int DIO_TRRT_ENC_B = 4;

  public static final double TURRET_CW_STOP_ANGLE = 190;
  public static final double TURRET_CCW_STOP_ANGLE = -190;


 // public static final double TURRET_RANGE = 360;  // -180 to +180 
  public static final double TURRET_FRNTCENT = 0; 
  public static final double TURRET_LFRNT = 315;
  public static final double TURRET_RFRNT = 45;
  public static final double TURRET_RIGHT = 90;
  public static final double TURRET_LEFT = 270;

  public static final double TURRET_CW_SPEED = 0.06;
  public static final double TURRET_CCW_SPEED = 0.07; //This must also be positive.  Negative added in method

//TUrret Brake Solenoid
  public static final int TURRET_BRAKE_FOR = 6;
  public static final int TURRET_BRAKE_REV = 7;

}

}

  