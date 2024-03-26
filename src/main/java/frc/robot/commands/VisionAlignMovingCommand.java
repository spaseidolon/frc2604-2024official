package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class VisionAlignMovingCommand extends Command {
    // option 1 - just p
     private final double kP = 0.0085;//0.000625
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

    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;

    public VisionAlignMovingCommand(SwerveSubsystem drive, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.drive = drive;
        addRequirements(drive);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
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
        double yAxis = -controller.getRawAxis(translationAxis);
        double xAxis = -controller.getRawAxis(strafeAxis);
        double rAxis = pid.calculate(getError(), 0);
      
        yAxis = (Math.abs(yAxis) < Constants.Controls.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.Controls.stickDeadband) ? 0 : xAxis;
        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);


        rotation = rAxis * Constants.Swerve.maxAngularVelocity ;

        drive.drive(translation, rotation, fieldRelative, openLoop);
    }

    @Override
    public void end(boolean wasInterrupted) {
        Limelight.disableTracking();
    }
}