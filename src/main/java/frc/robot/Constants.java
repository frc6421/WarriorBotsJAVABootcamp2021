// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final class OIConstants{
    public static final int DRIVER_CONTROLLER_PORT = 0;

    public static final double NERF_RATE = 0.5; // TODO adjust as needed
    // TODO Add other nerf rates if needed
  }

  public static final class DriveConstants{

    public static final int RIGHT_MAIN_CAN_ID = 10;
    public static final int RIGHT_FOLLOW_CAN_ID = 11;
    public static final int LEFT_MAIN_CAN_ID = 12;
    public static final int LEFT_FOLLOW_CAN_ID = 13;

    public static final double NEUTRAL_DEADBAND = 0.02;

    public static final double RAMP_RATE_DRIVE = 1.0; // TODO adjust as needed for each robot

    // TODO find out the actual distance between wheels
    public static final double TRACK_WIDTH_METERS = 1.556;
    public static final double TRACK_WIDTH_INCHES = Units.metersToInches(TRACK_WIDTH_METERS);

    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
            TRACK_WIDTH_METERS);
    public static final double WHEEL_DIAMETER_INCHES = 6; // TODO update for actual wheel diameter
    public static final double WHEEL_CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);
    public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * DriveConstants.WHEEL_DIAMETER_METERS;

    public static final int ENCODER_CPR = 2048;
    public static final double ENCODER_DISTANCE_PER_PULSE_METERS = (WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_CPR;
    public static final double GEAR_RATIO = 10.71; // Ratio included with KOP Toughbox Mini 
    /**
    * Encoder on motor shaft (integrated encoder). Encoder CPR = 2048. Gear Ratio = 10.71.
    */
    public static final double ENCODER_CPR_PER_WHEEL_REVOLUTION = ENCODER_CPR * GEAR_RATIO;

    public static final boolean GYRO_REVERSED = true;
    
    // TODO RUN Robot characterization and update values below
    public static final double ksVolts = 0.668;
    public static final double kvVoltSecondsPerMeter = 2.82;
    public static final double kaVoltSecondsSquaredPerMeter = 0.212;

    // TODO Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 0.000368; // P for PID
  }
  
  public static final class AutonomousConstants {
    public static final double MAX_SPEED_METERS_PER_SECOND = 1;
    public static final double MAX_ACCLERATION_METERS_PER_SECOND_SQUARED = 1;
  
    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
    
  }
}
