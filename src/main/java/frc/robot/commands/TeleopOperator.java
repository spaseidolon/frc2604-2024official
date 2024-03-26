package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.subsystems.OperatorSubsystem;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopOperator extends Command {
    

    public TeleopOperator(OperatorSubsystem o_Operator) {
        addRequirements(o_Operator);
    }
    @Override
    public void execute(){

        //default all process values
        double powerClimberA = Controls.operator.getRawAxis(Controls.rawClimberAAxis);
        double powerClimberB = Controls.operator.getRawAxis(Controls.rawClimberBAxis);
        double powerShootA = 0;
        double powerShootB = 0;
        double powerIntake = 0;
        double powerIndex  = 0;


        //calculate climber powers based on stick values
        powerClimberA = (Math.abs(powerClimberA) < Controls.stickDeadband) ? 0 : powerClimberA;
        powerClimberB = (Math.abs(powerClimberB) < Controls.stickDeadband) ? 0 : powerClimberB;
        //determine if shoot button is hit; set shooter and index powers accordingly
        powerShootA = Controls.operator.getRawButton(Controls.shoot) ? AuxSystems.shootpower : 0 ;
        powerShootB = Controls.operator.getRawButton(Controls.shoot) ? AuxSystems.shootpower : 0 ;
        powerIndex = Controls.operator.getRawButton(Controls.shoot) ? AuxSystems.shootpower : 0 ;
        //determine if ampshot button is pressed; overwrite shooter and index powers accordingly
        powerShootA = Controls.operator.getRawButton(Controls.ampshot) ? AuxSystems.ampshotpower : powerShootA ;
        powerShootB = Controls.operator.getRawButton(Controls.ampshot) ? 0 : powerShootB ;
        powerIndex = Controls.operator.getRawButton(Controls.ampshot) ? AuxSystems.ampshotpower * 1.2 : powerIndex ;
        //determine if suck button is hit; set intake and overwrite index accordingly
        powerIntake = Controls.operator.getRawButton(Controls.intake) ? AuxSystems.intakepower : powerIntake;
        powerIndex = Controls.operator.getRawButton(Controls.intake) ? AuxSystems.intakepower : powerIndex;
        //determine if burp button is hit; overwrite intake and index accordingly
        powerShootA = Controls.operator.getRawButton(Controls.intakeburp) ? -AuxSystems.shootpower : powerShootA;
        powerShootB = Controls.operator.getRawButton(Controls.intakeburp) ? -AuxSystems.shootpower : powerShootB;
        powerIntake = Controls.operator.getRawButton(Controls.intakeburp) ? -AuxSystems.intakeburppower : powerIntake;
        powerIndex = Controls.operator.getRawButton(Controls.intakeburp) ? -AuxSystems.intakeburppower : powerIndex;


        //set all outputs
        OperatorSubsystem.runShooter(powerShootA, powerShootB);
        OperatorSubsystem.runIntake(powerIntake);
        OperatorSubsystem.runIndex(powerIndex);
        OperatorSubsystem.runClimberA(powerClimberA);
        OperatorSubsystem.runClimberB(powerClimberB);

    }
}