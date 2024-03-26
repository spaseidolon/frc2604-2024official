// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.*;
//import frc.robot.autos.*;
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
 
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  /* Subsystems */
  private final SwerveSubsystem s_Swerve = new SwerveSubsystem();
  private final OperatorSubsystem o_Operator = new OperatorSubsystem();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve));
    o_Operator.setDefaultCommand(new TeleopOperator(o_Operator));


    /*autoChooser.addOption("S Then Backwards", new exampleAuto(s_Swerve));
    autoChooser.addOption("DriveForwardOnly", new DriveForawrdAuto(s_Swerve));
    autoChooser.addOption("TwoBallRightForward", new TwoBallRightForwardAuto(s_Swerve));
    autoChooser.addOption("ThreeBallRight", new ThreeBallRightAuto(s_Swerve));
    autoChooser.addOption("BackupAndShoot", new BackupAndShootAuto(s_Swerve));
    autoChooser.addOption("TwoBallLeft", new TwoBallLeftAuto(s_Swerve));
    SmartDashboard.putData("Auto Selector", autoChooser);*/
    
    
    // Configure the button bindings
    configureButtonBindings();
    o_Operator.configOperatorParams();
  }


  private void configureButtonBindings() {
    /* Driver Buttons */
    Controls.zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));


    /* Operator I/O */
    /*Controls.shoot.onTrue(new InstantCommand(() -> o_Operator.runShooter(AuxSystems.shootpower)));
    Controls.intake.whileTrue(new InstantCommand(() -> o_Operator.runIntake(AuxSystems.intakepower)));
    Controls.intakeburp.onTrue(new InstantCommand(() -> o_Operator.runIntake(-AuxSystems.intakeburppower)));*/
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