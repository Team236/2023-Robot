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
    public static final int USB_LEFT_STICK = 0;
    public static final int USB_RIGHT_STICK = 1;
    public static final int USB_CONTROLLER = 2;
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
    }
}
  public static class MotorControllers {

    //placeholder numbers for beginning of season because we haven't built the thing yet, CURRENT #s = 2022
    public static final int ID_LEFT_FRONT = 30; //10 - testbed #s
    public static final int ID_RIGHT_FRONT = 43; //15
    public static final int ID_LEFT_REAR = 44; // 11
    public static final int ID_RIGHT_REAR = 45;//16

    public static final int ID_ARM = 38;

    }


public static class DriveConstants {

  public static final double LEFT_DEADZONE = 0.17; //0.15???
  public static final double RIGHT_DEADZONE = 0.17;
  public static final boolean IS_DEADZONE = true;

  //robot-specific numbers
  public static final double DIAMETER = 6; //THIS IS A GUESS, NOT BUILT YET
  public static final double CIRCUMFERENCE = Math.PI * DIAMETER;
  public static final double GEAR_RATIO = 27; //TEMPORARY!!!! - change once robot is more than a concept

  public static final double REV_TO_IN_K = CIRCUMFERENCE / GEAR_RATIO;
  public static final double IN_TO_REV_K = GEAR_RATIO / CIRCUMFERENCE;

  //PID stuff

  //auto distances
  public static final double AUTO_MARGIN = 0;
  public static final double GRID_TO_CHARGE = 0; //???? Game Manual???

}

public static class ArmConstants { ///FOR TESTBOT: subject to change for final
  public static double armREV_TO_IN = 0.5;
  public static double armIN_TO_REV = 2;

  public static final int DIO_ARM_RETURN = 3;
  public static final int DIO_ARM_EXTEND = 4;

  public static final double ARM_OUT = 6;
  public static final double CLIMB_UP = 6;

  public static final double ARM_EX_SPEED = 0.6;
  public static final double ARM_RE_SPEED = 0.6;

  public static double kParm = 0.005;
  public static double kIarm = 0;
  public static double kDarm = 0;
  public static double kFarm = 0; //mooooo

  public static final double armMARGIN = 2;
  public static final double armMIN_OUTPUT = 10;
  public static final double armMAX_OUTPUT = 3;

}
  }

  

