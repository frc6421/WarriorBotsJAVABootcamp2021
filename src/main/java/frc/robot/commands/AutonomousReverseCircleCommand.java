// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousReverseCircleCommand extends SequentialCommandGroup {

  private final DriveSubsystem driveSubsystem;

  /** Creates a new AutonomousReverseCircle. */
  public AutonomousReverseCircleCommand(DriveSubsystem driveSubSystem) {

    this.driveSubsystem = driveSubSystem;

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.DRIVE_KINEMATICS, 10);

    TrajectoryConfig reverseConfig = new TrajectoryConfig(AutonomousConstants.MAX_SPEED_METERS_PER_SECOND,
        AutonomousConstants.MAX_ACCLERATION_METERS_PER_SECOND_SQUARED)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.DRIVE_KINEMATICS)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint)
            // Set to drive in reverse
            .setReversed(true);

    // Drive in 2 meter circle in reverse
    Trajectory circleReverseTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-90))),
        List.of(new Translation2d(1, 1), new Translation2d(2, 0), new Translation2d(1, -1)),
        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-90))), reverseConfig);

    RamseteCommand circleReverseRamseteCommand = new RamseteCommand(
        circleReverseTrajectory, 
        this.driveSubsystem::getPose,
        new RamseteController(AutonomousConstants.RAMSETE_B, AutonomousConstants.RAMSETE_ZETA),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.DRIVE_KINEMATICS, 
        this.driveSubsystem::getDifferentialWheelSpeedsInMeterPerSec,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), 
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        this.driveSubsystem::setTankDriveVolts, 
        this.driveSubsystem
    );

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> this.driveSubsystem.resetOdometry(circleReverseTrajectory.getInitialPose())),
      circleReverseRamseteCommand,
      new InstantCommand(() -> this.driveSubsystem.stop())
    );
  }
}
