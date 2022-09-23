package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1; // He's dead Jim

    public static final class Swerve {
        public static final int pigeonID = 0;  //Can ID for Pigeon IMU.  Has the same amount of bitches I do
        public static final boolean invertGyro = false; // Is the GYRO Upside Down?  Is it being waterboarded?

        // Constants for le drive train
        public static final double trackWidth = Units.inchesToMeters(22.1875);
        public static final double wheelBase = Units.inchesToMeters(26.0);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        //Ramp time for acceleration
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        //Gear Ratios for MK4 Modules
        public static final double driveGearRatio = (8.14 / 1.0); //8.14:1
        public static final double angleGearRatio = (12.8 / 1.0); //12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        //Power Draw/Current Limits for the steering motors
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        //Power Draw/Current Limits for the drive motors
        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        //PID Values for the steering motors
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 12.0;
        public static final double angleKF = 0.0;

        //PID Values for the drive motors
        public static final double driveKP = 0.84099;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        //We are probably going to do this, but this is the characterization for the drive base
        public static final double driveKS = (0.50245 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (0.86551 / 12);
        public static final double driveKA = (0.046425 / 12);

        //Max drive speen and max spin speed
        public static final double maxSpeed = 4.5; //4.5 meters per second
        public static final double maxAngularVelocity = 11.5; //4.5 meters per second (Presumably)

        //Set the neutral mode for the angle and drive motors
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        //If the motors are inverted, make it true, but the MK4's are not inverted
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        //Is the steering CANCoder inverted?
        public static final boolean canCoderInvert = false;

        //CAN IDs and angle offsets for each module
        //Config for Module 0, Front left
        public static final class Mod0 {
            public static final int driveMotorID = 4; //CAN ID for the drive motor
            public static final int angleMotorID = 3; //CAN ID for the angle/steerer motor
            public static final int canCoderID = 10; //CAN ID for steer encoder
            public static final double angleOffset = 296.016; //Angle offset TEST THIS MIGHT BE IN DEGREES NOT RADS
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        // Config for Module 1, Front Right
        public static final class Mod1 {
            public static final int driveMotorID = 2; //CAN ID for the drive motor
            public static final int angleMotorID = 1; //CAN ID for the angle/steerer motor
            public static final int canCoderID = 9; //CAN ID for steer encoder
            public static final double angleOffset = 158.730; //Angle offset TEST THIS MIGHT BE IN DEGREES NOT RADS
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        // Config for Module 2, Back Left
        public static final class Mod2 {
            public static final int driveMotorID = 6; //CAN ID for the drive motor
            public static final int angleMotorID = 5; //CAN ID for the angle/steerer motor
            public static final int canCoderID = 11; //CAN ID for steer encoder
            public static final double angleOffset = 153.809-180.00; //Angle offset TEST THIS MIGHT BE IN DEGREES NOT RADS
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        // Config for Module 3, Back Right
        public static final class Mod3 {
            public static final int driveMotorID = 8; //CAN ID for the drive motor
            public static final int angleMotorID = 7; //CAN ID for the angle/steerer motor
            public static final int canCoderID = 12; //CAN ID for steer encoder
            public static final double angleOffset = 72.158; //Angle offset TEST THIS MIGHT BE IN DEGREES NOT RADS
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class AutoConstants {

        //How fast will this go in auto
        //NYOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOM
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        // How fast can we crack people eggs? 
        
        //IDK what this really does but it does something and it breaks without it
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        // I mean I ask the same question as to why i need something and why something else breaks without it
        // I would break without my blahaj
    
        //Controller thingys for auton    
        public static final double kPXController = 0.01;
        public static final double kPYController = 0.01;
        public static final double kPThetaController = 0.01;
    
        // Motion profile constraint for the profiled angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //OMG ITS A TRAP
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

}
