// Dear future programer:
// When I first wrote this cursed code, only god and
// I knew how it worked.
// Now only god knows!
//
// Therefore, if you are trying to optimize this subsystem 
// or make it better
// please increase the below counter as a
// warning for the next unfortunate soul who comes across this
//
// total_hours_wasted_on_swerve = too many to count
//
package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;

    public Swerve() {
        gyro = new AHRS(SPI.Port.kMXP, (byte) 1000);
        gyro.zeroYaw();
        
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, Rotation2d.fromDegrees(gyro.getFusedHeading()));
        //swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
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

    // Stuff to make the swerve command work in auton
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
        swerveOdometry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro(){
    gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
    if (gyro.isMagnetometerCalibrated()) {
     // will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(gyro.getFusedHeading());
    }

    // Need to invert for navX
    if (Constants.Swerve.invertGyro) {
    return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
    }

    else {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getStates());  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANCoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated encoder", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " WE ZOWOMIN HOW FAST", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Mod" + mod.moduleNumber + "Auton Voltages" + (mod.getState().speedMetersPerSecond / Constants.AutoConstants.kMaxSpeedMetersPerSecond * 13), 0);  
            SmartDashboard.putNumber("Mod" + mod.moduleNumber + "Angle_Current", mod.getAngleCurrent());
            SmartDashboard.putNumber("Mod" + mod.moduleNumber + "Drive_Current", mod.getDriveCurrent());

        }
    }
}