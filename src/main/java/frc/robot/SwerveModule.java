package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.conversions.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.Resolver;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.DemandType;
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.ControlMode;
//import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public int moduleNumber;
    public double angleOffset;
    private CANVenom mAngleMotor;
    private CANVenom mDriveMotor;
    public Resolver angleEncoder;
    private boolean forceAbsolute;
    private double lastAngle;
    public PIDController anglePID;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.forceAbsolute = moduleConstants.forceAbsolute;
        
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new Resolver(moduleConstants.encoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANVenom(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANVenom(moduleConstants.driveMotorID);
        configDriveMotor();


        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setCommand(CANVenom.ControlMode.VoltageControl, percentOutput * 12);
        }
        else {
    
            double velocity = Conversions.PwFConversions.MPSTomotor(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.setCommand(CANVenom.ControlMode.SpeedControl, velocity);
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        if (forceAbsolute) {mAngleMotor.setCommand(ControlMode.VoltageControl, anglePID.calculate(angleEncoderGet(), angle));}
        else {mAngleMotor.setCommand(ControlMode.PositionControl, Conversions.PwFConversions.degreesTomotor(angle, Constants.Swerve.angleGearRatio)); }
        lastAngle = angle;
    }

    private void resetToAbsolute(){
        double absolutePosition = Conversions.PwFConversions.degreesTomotor(angleEncoderGet() - angleOffset, Constants.Swerve.angleGearRatio);
        lastAngle = absolutePosition;
    }
 
    private void configAngleEncoder(){        
       // angleEncoder.zero();
        //angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    public double angleEncoderGet(){
        angleEncoder.update();
        return angleEncoder.value - this.angleOffset;
    }

    private void configAngleMotor(){
      
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setPID(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD, Constants.Swerve.angleKF, 0);
        if (forceAbsolute) anglePID = new PIDController(Constants.Swerve.angleKP / 100, Constants.Swerve.angleKI, Constants.Swerve.angleKD);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
      //  mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setPID(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD, Constants.Swerve.driveKF, 0);
    }
/*
    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }
*/

    public SwerveModuleState getState(){
        double velocity = Conversions.PwFConversions.motorToMPS(mDriveMotor.getSpeed(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        Rotation2d angle;
        if (this.forceAbsolute) {angle = Rotation2d.fromDegrees(angleEncoderGet());}
        else {angle = Rotation2d.fromDegrees(Conversions.PwFConversions.motorToDegrees(mAngleMotor.getPosition(), Constants.Swerve.angleGearRatio));}        return new SwerveModuleState(velocity, angle);
    }
    
    public SwerveModulePosition getPosition(){
        double position = Conversions.PwFConversions.motorToMPS(mDriveMotor.getPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        Rotation2d angle;
        if (this.forceAbsolute) {angle = Rotation2d.fromDegrees(angleEncoderGet());}
        else {angle = Rotation2d.fromDegrees(Conversions.PwFConversions.motorToDegrees(mAngleMotor.getPosition(), Constants.Swerve.angleGearRatio));}
        return new SwerveModulePosition(position, angle);
    }
}