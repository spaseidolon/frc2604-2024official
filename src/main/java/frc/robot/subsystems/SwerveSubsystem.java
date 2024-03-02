package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.PigeonIMU;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.SwerveModule;
//import frc.lib.math.conversions.Conversions;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModulePosition[] swervePosAll;
    public SwerveModule[] mSwerveMods;
    public AHRS navX = new AHRS(SPI.Port.kMXP);

    public SwerveSubsystem() {
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swervePosAll = getPositions();
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), swervePosAll);

        
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), swervePosAll, pose);
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }
    public SwerveModulePosition[] getPositions(){
    
        SwerveModulePosition[] positionsAll = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods)
        {
            positionsAll[mod.moduleNumber] = mod.getPosition();
           
        }
        return positionsAll;
    }
    public void zeroGyro(){
        navX.reset();
    }




    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        ypr[0] = navX.getYaw();
        ypr[1] = navX.getPitch();
        ypr[2] = navX.getRoll();
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getPositions());  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Absolute Encoder", mod.angleEncoderGet() + mod.angleOffset);
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + "Adjusted Steering" , mod.angleEncoderGet());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated Encoder", mod.getState().angle.getDegrees());
           // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Velocity (m/s)", mod.getState().speedMetersPerSecond);  
        }
        SmartDashboard.putNumber("navX Yaw: " , navX.getYaw()); 
        SmartDashboard.putNumber("navX Pitch: " , navX.getPitch());
        SmartDashboard.putNumber("navX Roll: " , navX.getRoll());
        
    }
}