// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  // ----- Fields -----

  // Motors on the right side of the drive train
  private final WPI_TalonFX rightMainMotor;
  private final WPI_TalonFX rightFollowMotor;

  // Motors on the left side of the drive train
  private final WPI_TalonFX leftMainMotor;
  private final WPI_TalonFX leftFollowMotor;

  // The robot's drive
  private final DifferentialDrive robotDrive;

  public DriveSubsystem() {
    // ----- Constructor -----
    rightMainMotor = new WPI_TalonFX(DriveConstants.RIGHT_MAIN_CAN_ID);
    rightFollowMotor = new WPI_TalonFX(DriveConstants.RIGHT_FOLLOW_CAN_ID);

    leftMainMotor = new WPI_TalonFX(DriveConstants.LEFT_MAIN_CAN_ID);
    leftFollowMotor = new WPI_TalonFX(DriveConstants.LEFT_FOLLOW_CAN_ID);

    robotDrive = new DifferentialDrive(leftMainMotor, rightMainMotor);

    /* Set motor controllers to Factory Default */
    leftMainMotor.configFactoryDefault();
    leftFollowMotor.configFactoryDefault();
    rightMainMotor.configFactoryDefault();
    rightFollowMotor.configFactoryDefault();

    /* Setting sensor phase not required for TalonFX due to integrated controller*/

    /* set up followers */
    leftFollowMotor.follow(leftMainMotor);
    rightFollowMotor.follow(rightMainMotor);

    /*
     * Invert motors since each side spins in opposite direction. Forward on
     * controller stick makes robot move forward and controller lights green
     */
    leftMainMotor.setInverted(false);
    rightMainMotor.setInverted(true);

    /*
     * set the invert of the followers to match their respective Main controllers
     */
    leftFollowMotor.setInverted(TalonFXInvertType.FollowMaster);
    rightFollowMotor.setInverted(TalonFXInvertType.FollowMaster);

    /* set brake/coast mode */
    leftMainMotor.setNeutralMode(NeutralMode.Brake);
    leftFollowMotor.setNeutralMode(NeutralMode.Brake);
    rightMainMotor.setNeutralMode(NeutralMode.Brake);
    rightFollowMotor.setNeutralMode(NeutralMode.Brake);
  
    /* configure deadband */
    leftMainMotor.configNeutralDeadband(DriveConstants.NEUTRAL_DEADBAND);
    leftFollowMotor.configNeutralDeadband(DriveConstants.NEUTRAL_DEADBAND);
    rightMainMotor.configNeutralDeadband(DriveConstants.NEUTRAL_DEADBAND);
    rightFollowMotor.configNeutralDeadband(DriveConstants.NEUTRAL_DEADBAND);
    
    /* configure open loop ramp rate */
    leftMainMotor.configOpenloopRamp(DriveConstants.RAMP_RATE_DRIVE);
    leftFollowMotor.configOpenloopRamp(DriveConstants.RAMP_RATE_DRIVE);
    rightMainMotor.configOpenloopRamp(DriveConstants.RAMP_RATE_DRIVE);
    rightFollowMotor.configOpenloopRamp(DriveConstants.RAMP_RATE_DRIVE);
    
    /* From talon fx java example. DJM
    /* diff drive assumes (by default) that 
			right side must be negative to move forward.
			Change to 'false' so positive/green-LEDs moves robot forward  */
    robotDrive.setRightSideInverted(false); // do not change this

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // ----- Set Methods -----

  /**
   * Basic Arcade Drive that can square the inputs.
   * @param xSpeed
   * @param zRotation
   * @param squareInput
   */
  public void setArcadeDrive(double xSpeed, double zRotation, boolean squareInput) {
    robotDrive.arcadeDrive(-xSpeed, zRotation, squareInput);
  }

}
