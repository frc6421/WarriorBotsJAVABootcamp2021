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
  public final class OIConstants{
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public final class DriveConstants{

    public static final int RIGHT_MAIN_CAN_ID = 10;
    public static final int RIGHT_FOLLOW_CAN_ID = 11;
    public static final int LEFT_MAIN_CAN_ID = 12;
    public static final int LEFT_FOLLOW_CAN_ID = 13;

    public static final double NEUTRAL_DEADBAND = 0.02;

    public static final double RAMP_RATE_DRIVE = 1.0; // TODO adjust as needed for each robot
    
    public static final double NERF_RATE = 0.5; // TODO adjust as needed
    // TODO Add other nerf rates if needed

  }
}
