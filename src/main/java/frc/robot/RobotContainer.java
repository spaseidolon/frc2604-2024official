// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();


  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);

  /* Subsystems */
  private final SwerveSubsystem s_Swerve = new SwerveSubsystem();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));


    autoChooser.addOption("S Then Backwards", new exampleAuto(s_Swerve));
    autoChooser.addOption("DriveForwardOnly", new DriveForawrdAuto(s_Swerve));
    autoChooser.addOption("TwoBallRightForward", new TwoBallRightForwardAuto(s_Swerve));
    autoChooser.addOption("ThreeBallRight", new ThreeBallRightAuto(s_Swerve));
    autoChooser.addOption("BackupAndShoot", new BackupAndShootAuto(s_Swerve));
    autoChooser.addOption("TwoBallLeft", new TwoBallLeftAuto(s_Swerve));
    SmartDashboard.putData("Auto Selector", autoChooser);
    
    
    // Configure the button bindings
    configureButtonBindings();
  }


  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    new JoystickButton(driver, XboxController.Button.kA.value).whileTrue(new VisionAlignMovingCommand(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, true, true));
    new JoystickButton(driver, XboxController.Button.kB.value).whileTrue(new VisionAlignStopCommand(s_Swerve, true, true));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}