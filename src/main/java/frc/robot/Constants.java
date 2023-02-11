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
    public static final int USB_AUXCONTROLLER = 0;
    public static final int USB_DRIVECONTROLLER = 1;
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

    //placeholder numbers for beginning of season //testbed OG - 2022 bot - testbed swapped 
    public static final int ID_LEFT_FRONT = 11; //10 - 30 - 11
    public static final int ID_RIGHT_FRONT = 16; //15 - 43 - 16
    public static final int ID_LEFT_REAR = 10;// 11 - 44 - 10
    public static final int ID_RIGHT_REAR = 15; //16 - 45 - 15

    public static final int ID_ARM = 38;
    //public static final int ID_TURRET = 32; //test value
    //public static final int ID_PIVOT = 12;// - currently pneumatic, getting redesigned
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
  public static final double GEAR_RATIO = 8.364; //TEMPORARY!!!
  //6.273 - low?? - testbot
  //8.364 - high??? - testbot

  public static final double REV_TO_IN_K = CIRCUMFERENCE / GEAR_RATIO;
  public static final double IN_TO_REV_K = GEAR_RATIO / CIRCUMFERENCE;

  //PID stuff
  public static final double leftkPdrive = 0.01; 
  public static final double leftkIdrive = 0;
  public static final double leftkDdrive = 0;

  public static final double rightkPdrive = 0.01;
  public static final double rightkIdrive = 0;
  public static final double rightkDdrive = 0;

  public static final double kPTurnL = 0.01;
  public static final double kPTurnR = 0.01;

  //auto distances
  public static final double AUTO_MARGIN = 0;
  public static final double GRID_TO_CHARGE = 42; //???? Game Manual???
  public static final double GRID_TO_CENTER = 240;
  public static final double MARGIN_GYRO_DRIVE = 3;

  //drive encoder channels
  public static final int DIO_RDRIVE_ENC_A = 15;
  public static final int DIO_RDRIVE_ENC_B = 16;
  public static final int DIO_LDRIVE_ENC_A = 17;
  public static final int DIO_LDIRVE_ENC_B = 18;

  //auto selector switches
 public static final int DIO_AUTO_1 = 1;
  public static final int DIO_AUTO_2 = 2;
  public static final int DIO_AUTO_3 = 3;
  public static final int DIO_AUTO_4 = 4;

}

public static class ArmConstants { ///FOR TESTBOT: subject to change for final
  public static double armREV_TO_IN = 0.125; // 1/2 inch per revolution
  public static double armIN_TO_REV = 8; //2 revolutions per inch

  public static final int DIO_ARM_RETURN = 3; //7
  public static final int DIO_ARM_EXTEND = 4; //8
  public static final int DIO_ARM_ENC_A = 0;
  public static final int DIO_ARM_ENC_B = 1;

  public static final double ARM_OUT = 4;

  public static final double ARM_EX_SPEED = 0.7;
  public static final double ARM_RE_SPEED = 0.85;

  public static double kParm = 0.08;
  public static double kIarm = 0;
  public static double kDarm = 0;
  public static double kFarm = 0; //mooooo
}
public static class GripperConstants {
  //Solenoid ports on the PCM
  public static final int GRIPPER_SOL_FOR = 6; //0
  public static final int GRIPPER_SOL_REV = 7; //1
  //Port used on the DIO
  public static final int DIO_GRIPPER_EYE = 5; //0

}

public static class TurretConstants {
  public static final double turretREV_TO_DEG = 1;
  public static final double turretDEG_TO_REV = 1;

  public static final double kPturret = 0;
  public static final double kIturret = 0;
  public static final double kDturret = 0;

  public static final int DIO_TURRET_CW_LIMIT = 9;
  public static final int DIO_TURRET_CCW_LIMIT = 10;
  public static final int DIO_TRRT_ENC_A = 11;
  public static final int DIO_TRRT_ENC_B = 12;

  public static final double TURRET_RANGE = 360;
  public static final double TURRET_FRNTCENT = 0; 
  public static final double TURRET_LFRNT = -45;
  public static final double TURRET_RFRNT = 45;
  public static final double TURRET_RIGHT = 90;
  public static final double TURRET_LEFT = -90;

  public static final double TURRET_SPEED = 0.75;


}

public static class PivotConstants {
  public static final int PIVOT_SOL_FOR = 0;
  public static final int PIVOT_SOL_REV = 1;

  public static final int PVT_LOW = 5;
  public static final int PVT_HIGH = 6;

  public static final int DIO_PVT_ENC_A = 13;
  public static final int DIO_PVT_ENC_B = 14;

public static final double pvtREV_TO_DEG = 360; //tester numbers
public static final double pvtDEG_TO_REV = 1/360;

public static final double pvtSPEED = 0.75;
public static double kPpvt = 0;
public static double kIpvt = 0;
public static double kDpvt = 0;
}
}

  

