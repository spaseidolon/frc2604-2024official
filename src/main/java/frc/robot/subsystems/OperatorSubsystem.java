package frc.robot.subsystems;

//import com.playingwithfusion.TimeOfFlight;
//import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.*;
//import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj2.command;
//import ;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class OperatorSubsystem extends SubsystemBase {
    private static CANSparkMax intakeMotor;
    private static CANSparkMax indexMotor;
    private static CANSparkMax shooterA;
    private static CANSparkMax shooterB;
    private static CANSparkMax climberA;
    private static CANSparkMax climberB;
    //private TimeOfFlight climbAlidar;
    //private TimeOfFlight climbBlidar;
    public OperatorSubsystem(){
    intakeMotor = new CANSparkMax(AuxSystems.intakeMotorID, CANSparkLowLevel.MotorType.kBrushless);
    indexMotor = new CANSparkMax(AuxSystems.indexMotorID, CANSparkLowLevel.MotorType.kBrushless);
    climberA = new CANSparkMax(AuxSystems.climbAID, CANSparkLowLevel.MotorType.kBrushless);
    climberB = new CANSparkMax(AuxSystems.climbBID, CANSparkLowLevel.MotorType.kBrushless);
    shooterA = new CANSparkMax(AuxSystems.shootAID, CANSparkLowLevel.MotorType.kBrushless);
    shooterB = new CANSparkMax(AuxSystems.shootBID, CANSparkLowLevel.MotorType.kBrushless);
    }

    
    public void configOperatorParams(){
        climberA.setSecondaryCurrentLimit(25);
        climberB.setSecondaryCurrentLimit(25);
        climberA.setIdleMode(AuxSystems.climberSparkMode);
        climberB.setIdleMode(AuxSystems.climberSparkMode);
        shooterA.setInverted(true);
        shooterB.setInverted(true);
    }

    public static void runIntake(double power){
        intakeMotor.set(power);
    }

    public static void stopIntake(){ 
        intakeMotor.set(0);
    }

    public static void runIndex(double power){
        indexMotor.set(power);
    }

    public static void runShooter(double power, double powerB){
        shooterA.set(power);
        shooterB.set(powerB);
    }
    public static void pidShooter(double power){

    }

    public static void runClimberA(double power){
        climberA.set(power);
    }
    public static void runClimberB(double power){
        climberB.set(power);
    }

}