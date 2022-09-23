package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class testingAuto extends SequentialCommandGroup {
    public testingAuto(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // Basic trajectory using the traj generator tool built into WPILib
        Trajectory testingTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start facing the +X direction at 0,0
                new Pose2d(0, 0, new Rotation2d(0)),
                // Should make an s by going through the 2 interior waypoints
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // end 3 meters gayly forward of where we started
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, Constants.AutoConstants.kIThetaController, Constants.AutoConstants.kDThetaController, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                testingTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, Constants.AutoConstants.kIXController, Constants.AutoConstants.kDXController),
                new PIDController(Constants.AutoConstants.kPYController, Constants.AutoConstants.kIYController, Constants.AutoConstants.kIYController),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(testingTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}