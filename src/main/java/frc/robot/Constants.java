package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.13; // He's dead Jim

    public static final class Swerve {
        public static final int pigeonID = 0;  //Can ID for Pigeon IMU. 
        public static final boolean invertGyro = false; // Is the GYRO Upside Down?

        // Drive train configuration constants
        public static final double trackWidth = Units.inchesToMeters(21.00);
        public static final double wheelBase = Units.inchesToMeters(21.0);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        //Acceleration and deceleration ramp times for the drive base
        public static final double openLoopRamp = 0.3;
        public static final double closedLoopRamp = 0.0;

        // Swerve module Gear Ratio configuration constants
        public static final double driveGearRatio = (8.14 / 1.0); //8.14:1 For MK4 L1 Modules
        public static final double angleGearRatio = (12.8 / 1.0); //12.8:1 For MK4 L1 Modules

        //Easy reference for the Swerve Drive Kinematics for future use
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        //Power Draw/Current Limits for the steering motors
        public static final int angleContinuousCurrentLimit = 25;  //Limits the continous current draw of the motors
        public static final int anglePeakCurrentLimit = 40; //Limits the peak current draw of the motors
        public static final double anglePeakCurrentDuration = 0.1; //Threshold for the peak current draw
        public static final boolean angleEnableCurrentLimit = true; //Enable or disable the software current limiting on your motors

        //Power Draw/Current Limits for the drive motors
        public static final int driveContinuousCurrentLimit = 35; //Limits the continous current draw of the motors
        public static final int drivePeakCurrentLimit = 60; //Limits the peak current draw of the motors
        public static final double drivePeakCurrentDuration = 0.1; //Threshold for the peak current draw
        public static final boolean driveEnableCurrentLimit = true; //Enable or disable the software current limiting on your motors

        //PID Values for the steering motors
        public static final double angleKP = 0.6;  //P value for motors
        public static final double angleKI = 0.0; //I value for motors
        public static final double angleKD = 12.0; //D value for motors
        public static final double angleKF = 0.0; //Feedforward value for motors

        //PID Values for the drive motors
        public static final double driveKP = 0.84099; //P value for motors
        public static final double driveKI = 0.0; //I value for motors
        public static final double driveKD = 0.0; //D value for motors
        public static final double driveKF = 0.0; //Feedforward value for motors

        //Characterizarion values as reported by SysID 
        //POSSIBLE FIXME Removing the carpet may have had an effect on the data
        public static final double driveKS = (0.50245 / 12); //divide by 12, used to convert from volts to percent output
        public static final double driveKV = (0.86551 / 12); // 0.86551
        public static final double driveKA = (0.046425 / 12);

        //Max drive speed and max angular velocity of your robot
        public static final double maxSpeed = 5; //Max speed of your bot in meters per second
        public static final double maxAngularVelocity = 5; //Max Angular Velocity of your bot, fancy way of saying radians per second of rotation

        //Set the neutral mode for the angle and drive motors
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast; //Sets angle motors to either coast or brake mode
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake; //Sets drive motors to either coast or brake mode

        //If the motors are inverted or need to be driven backwards
        public static final boolean driveMotorInvert = false; //Inverts drive motor
        public static final boolean angleMotorInvert = false; //Inverts angle motor

        //Is the steering CANCoder inverted
        public static final boolean canCoderInvert = false; //Make true if reversed

        //CAN IDs and angle offsets for each module
        //Config for Module 0, Front left
        public static final class Mod0 {
            public static final int driveMotorID = 4; //CAN ID for the drive motor
            public static final int angleMotorID = 3; //CAN ID for the angle/steerer motor
            public static final int canCoderID = 10; //CAN ID for steer encoder
            public static final double angleOffset = 296.016; //CANCoder Angle offset
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset); //Sets the constant values for the module
        }

        // Config for Module 1, Front Right
        public static final class Mod1 {
            public static final int driveMotorID = 2; //CAN ID for the drive motor
            public static final int angleMotorID = 1; //CAN ID for the angle/steerer motor
            public static final int canCoderID = 9; //CAN ID for steer encoder
            public static final double angleOffset = 158.730; //CANCoder Angle offset
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset); //Sets the constant values for the module
        }
        
        // Config for Module 2, Back Left
        public static final class Mod2 {
            public static final int driveMotorID = 6; //CAN ID for the drive motor
            public static final int angleMotorID = 5; //CAN ID for the angle/steerer motor
            public static final int canCoderID = 11; //CAN ID for steer encoder
            public static final double angleOffset = 153.809-180.00; //CANCoder Angle offset
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset); //Sets the constant values for the module
        }

        // Config for Module 3, Back Right
        public static final class Mod3 {
            public static final int driveMotorID = 8; //CAN ID for the drive motor
            public static final int angleMotorID = 7; //CAN ID for the angle/steerer motor
            public static final int canCoderID = 12; //CAN ID for steer encoder
            public static final double angleOffset = 72.158; //CANCoder Angle offset
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset); //Sets the constant values for the module
        }

    }

    public static final class AutoConstants {

        //How fast will the bot go in auto
        //NYOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOM
        public static final double kMaxSpeedMetersPerSecond = 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        
        public static final double kMaxAngularSpeedRadiansPerSecond = 1; //Changed from Math.PI for testing
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 1; //Changed from Math.PI for testing
    
        // PID Controller values for auton   
        // PID Values for the X controller 
        public static final double kPXController = 1.50; // Should only need to adjust this
        public static final double kIXController = 0.00; //Shouldn't need to adjust this right away
        public static final double kDXController = 0.00; //Shouldn't need to adjust this right away

        // PID Values for the Y controller
        public static final double kPYController = 1.75; // Should only need to adjust this
        public static final double kIYController = 0.00; //Shouldn't need to adjust this right away
        public static final double kDYController = 0.00; //Shouldn't need to adjust this right away

        // PID Values for the Theta controller
        public static final double kPThetaController = 5.00; // Should only need to adjust this
        public static final double kIThetaController = 0.00; //Shouldn't need to adjust this right away
        public static final double kDThetaController = 0.00; //Shouldn't need to adjust this right away
    
        // Motion profile constraint for the profiled angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

}
