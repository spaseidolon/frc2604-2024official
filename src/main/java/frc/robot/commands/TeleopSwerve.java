package frc.robot.commands;

import static frc.robot.Constants.*;

import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private SwerveSubsystem s_Swerve;

    /**
     * Driver control
     */
    public TeleopSwerve(SwerveSubsystem s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.fieldRelative = Swerve.fieldRelative;
        this.openLoop = Swerve.openLoop;
    }

    @Override
    public void execute() {
        /*sets the axis to the controller sticks*/
        double yAxis = -Controls.driver.getRawAxis(Controls.translationAxis) * Swerve.axisScaler;
        double xAxis = -Controls.driver.getRawAxis(Controls.strafeAxis) * Swerve.axisScaler;
        double rAxis = -Controls.driver.getRawAxis(Controls.rotationAxis)* Swerve.axisScaler;
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Controls.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Controls.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Controls.stickDeadband) ? 0 : rAxis;

        translation = new Translation2d(yAxis, xAxis).times(Swerve.maxSpeed);
        rotation = rAxis * Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
