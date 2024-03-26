// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class VisionAlignStopCommand extends Command {
    // option 1 - just p
    private final double kP = 0.01;//0.0085
    private final double kI = 0.0000;
    private final double kD = 0.00;

   // option 2 - pi
   // private final double kP = 0.000425;
   // private final double kI = 0.0008;
   // private final double kD = 0.00;

   // option 3 - pd
   //private final double kP = 0.00083;
   //private final double kI = 0.0000;
   //private final double kD = 0.00005;

   private final PIDController pid = new PIDController(kP, kI, kD);
   private final SwerveSubsystem drive;
   private Translation2d translation;
   private boolean fieldRelative;
   private boolean openLoop;
   private double rotation;


   public VisionAlignStopCommand(SwerveSubsystem drive, boolean fieldRelative, boolean openLoop) {
       this.drive = drive;
       addRequirements(drive);

       this.fieldRelative = fieldRelative;
       this.openLoop = openLoop;
   }

   @Override
   public void initialize() {
       Limelight.enableTracking();
   }

   /** Returns the vision tracking error in degrees (from -27 to 27) */
   private static double getError() {
       return Limelight.getTargetAngle().x;
   }

   @Override
   public void execute() {
       //------------------------------Driving Part---------------------------------//
       double yAxis = 0;
       double xAxis = 0;
       double rAxis = pid.calculate(getError(), 0);
     
       translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);


       rotation = rAxis * Constants.Swerve.maxAngularVelocity ;

       drive.drive(translation, rotation, fieldRelative, openLoop);
   }

   @Override
   public void end(boolean wasInterrupted) {
       Limelight.disableTracking();
   }
}