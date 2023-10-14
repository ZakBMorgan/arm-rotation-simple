// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  public static final int joystick1 = 0;

  public static class OperatorConstants {


    public static final int kDriverControllerPort = 0;

    public static final int armExtension = 11;
    public static final int armRotation = 10;

    //used for driving
    //public static final int leftDriveTalonPort = 2;
    //public static final int rightDriveTalonPort = 3;
  }

  public final class Measurements {


    public static final double threatLength = 0.375;

    public static final double gearRatio = 1.0 / 1.0;

    public static final double baseExtendPower = 1.0;
    public static final double extPowerLimit = 0.4;
    
    public static final double armHoldingVoltage = 1.1;

  }

  public final class ButtonMap {

    public static final int moveArm = 1;
  }
  
  public static class PIDConstants {
    public static final double armPID_P = 0.1;
    public static final double armPID_I = 0.001;
    public static final double armPID_D = 0.0;

  }
  public static int joy1;
}
