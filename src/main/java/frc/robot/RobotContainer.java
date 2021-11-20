// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutonomousForwardCircleCommand;
import frc.robot.commands.AutonomousForwardCommand;
import frc.robot.commands.AutonomousReverseCircleCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Fields
  // The robot's subsystems are defined here...
  private final DriveSubsystem driveSubsystem;

  // The robot's controllers and buttons are defined here...
    private XboxController driverController;
    private JoystickButton arcadeDriveTurboButton;
    private JoystickButton arcadeDriveStopSquaringButton;
  
  // The robot's commands are defined here...
    private final AutonomousForwardCommand autonomousForwardCommand;
    private final AutonomousForwardCircleCommand autonomousForwardCircleCommand;
    private final AutonomousReverseCircleCommand autonomousReverseCircleCommand;
    private final SendableChooser<Command> chooser;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem = new DriveSubsystem();

    driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);

    autonomousForwardCommand = new AutonomousForwardCommand(driveSubsystem);
    autonomousForwardCircleCommand = new AutonomousForwardCircleCommand(driveSubsystem);
    autonomousReverseCircleCommand = new AutonomousReverseCircleCommand(driveSubsystem);

    chooser = new SendableChooser<>();
    chooser.setDefaultOption("Drive Forward", autonomousForwardCommand);
    chooser.addOption("Drive Forward Circle", autonomousForwardCircleCommand);
    chooser.addOption("Drive Reverse Circle", autonomousReverseCircleCommand);
    chooser.addOption("Do nothing", null);

    SmartDashboard.putData(chooser);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Button bindings for Driver Contoller
    driveSubsystem.setDefaultCommand(
      new RunCommand(() -> 
        driveSubsystem.setArcadeDrive(
          driverController.getY(GenericHID.Hand.kLeft) * OIConstants.NERF_RATE, // Speed Back/Forth
          driverController.getX(GenericHID.Hand.kRight) * OIConstants.NERF_RATE, // Turn amount
          true), //Square inputs
        driveSubsystem));

    //Button to remove Nerf (Tubro button)
    arcadeDriveTurboButton = new JoystickButton(driverController, Button.kBumperLeft.value);
    
    arcadeDriveTurboButton.whenPressed(new RunCommand(() ->
      driveSubsystem.setArcadeDrive(
        driverController.getY(GenericHID.Hand.kLeft),
        driverController.getX(GenericHID.Hand.kRight), 
        true), 
      driveSubsystem));

    //Button to stop squaring the input to show the effect
    arcadeDriveStopSquaringButton = new JoystickButton(driverController, Button.kBumperRight.value);

    arcadeDriveStopSquaringButton.whenPressed(new RunCommand(() ->
      driveSubsystem.setArcadeDrive(
        driverController.getY(GenericHID.Hand.kLeft) * OIConstants.NERF_RATE,
        driverController.getX(GenericHID.Hand.kRight) * OIConstants.NERF_RATE, 
        false), 
      driveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return chooser.getSelected();
  }
}
