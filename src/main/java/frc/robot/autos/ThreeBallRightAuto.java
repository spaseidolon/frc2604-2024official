// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.VisionAlignStopCommand;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallRightAuto extends SequentialCommandGroup {
  public ThreeBallRightAuto(SwerveSubsystem s_Swerve){
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);


    //------------An example trajectory to follow.  All units in meters.-------------------//
    Trajectory tarjectoryPart1 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-90))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0, -Units.inchesToMeters(15)), 

                    new Translation2d(0, -Units.inchesToMeters(30))),

            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(0, Units.inchesToMeters(-40), new Rotation2d(Units.degreesToRadians(-90))),
            config);

    Trajectory tarjectoryPart2 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, Units.inchesToMeters(-40), new Rotation2d(Units.degreesToRadians(-90))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(Units.inchesToMeters(-30), Units.inchesToMeters(-35)), 

                    new Translation2d(Units.inchesToMeters(-60), Units.inchesToMeters(-30))),
            
            
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-100), Units.inchesToMeters(-30), new Rotation2d(Units.degreesToRadians(60))),
            config);

    Trajectory tarjectoryPart3 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(Units.inchesToMeters(-100), Units.inchesToMeters(-35), new Rotation2d(Units.degreesToRadians(60))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(Units.inchesToMeters(-100), Units.inchesToMeters(-25)), 
    
                    new Translation2d(Units.inchesToMeters(-100), Units.inchesToMeters(-10))),
                
                
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-90), Units.inchesToMeters(-35), new Rotation2d(Units.degreesToRadians(60))),
            config);

    Trajectory tarjectoryPart4 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(Units.inchesToMeters(-90), Units.inchesToMeters(-35), new Rotation2d(Units.degreesToRadians(60))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(Units.inchesToMeters(-130), Units.inchesToMeters(-25)), 
        
                    new Translation2d(Units.inchesToMeters(-180), Units.inchesToMeters(-15))),
                    
                    
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-205), Units.inchesToMeters(-5), new Rotation2d(Units.degreesToRadians(0))),
            config);

            


    //------------------------The PID Controller for the actual auto------------------------//
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);


    //------------------------Making Of driving Commands----------------------------------//
    SwerveControllerCommand drivingPart1 =
        new SwerveControllerCommand(
            tarjectoryPart1,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);  
            
    SwerveControllerCommand drivingPart2 =
        new SwerveControllerCommand(
              tarjectoryPart2,
              s_Swerve::getPose,
              Constants.Swerve.swerveKinematics,
              new PIDController(Constants.AutoConstants.kPXController, 0, 0),
              new PIDController(Constants.AutoConstants.kPYController, 0, 0),
              thetaController,
              s_Swerve::setModuleStates,
              s_Swerve);   
              
    SwerveControllerCommand drivingPart3 =
        new SwerveControllerCommand(
              tarjectoryPart3,
              s_Swerve::getPose,
              Constants.Swerve.swerveKinematics,
              new PIDController(Constants.AutoConstants.kPXController, 0, 0),
              new PIDController(Constants.AutoConstants.kPYController, 0, 0),
              thetaController,
              s_Swerve::setModuleStates,
              s_Swerve);  

    SwerveControllerCommand drivingPart4 =
        new SwerveControllerCommand(
              tarjectoryPart4,
              s_Swerve::getPose,
              Constants.Swerve.swerveKinematics,
              new PIDController(Constants.AutoConstants.kPXController, 0, 0),
              new PIDController(Constants.AutoConstants.kPYController, 0, 0),
              thetaController,
              s_Swerve::setModuleStates,
              s_Swerve);  


    //---------------------The Actual Command List That will Run-----------------//

    addCommands(

        //resets odemetry
        new InstantCommand(() -> s_Swerve.resetOdometry(tarjectoryPart1.getInitialPose())),

        //go to first ball
        drivingPart1,

        //go back near ball 2 and spin
        drivingPart2,
        
        //align to shoot 
        new VisionAlignStopCommand(s_Swerve, true, true).withTimeout(2),

        //pickup ball three
        drivingPart3,

        //align to shoot again
        new VisionAlignStopCommand(s_Swerve, true, true).withTimeout(2),

        //go to terminal
        drivingPart4,

        //zeros gyro
        new InstantCommand(() -> s_Swerve.zeroGyro()).withTimeout(0.1)

    );
}
}