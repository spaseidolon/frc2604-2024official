package frc.robot.autos;

import frc.robot.Constants;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(SwerveSubsystem s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);


        //------------An example trajectory to follow.  All units in meters.-------------------//
        Trajectory tarjectoryPart1 =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);


                
        Trajectory tarjectoryPart2 =
            TrajectoryGenerator.generateTrajectory(                
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(-1, 0.5)), 
                new Pose2d(-2, 0, new Rotation2d(0)), config);

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

            


        //---------------------The Actual Command List That will Run-----------------//

        addCommands(

            //resets odemetry
            new InstantCommand(() -> s_Swerve.resetOdometry(tarjectoryPart1.getInitialPose())),

            //path part 1
            drivingPart1,

            drivingPart2
        );
    }
}