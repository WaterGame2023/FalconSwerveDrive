package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.BetterTalonFX;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class velocityAuto extends InstantCommand {
    private BetterTalonFX frontLeft = new BetterTalonFX(Constants.Swerve.Mod0.driveMotorID)
    .configurePID(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD)
    .configureBrakes(true);

    private BetterTalonFX frontRight = new BetterTalonFX(Constants.Swerve.Mod1.driveMotorID)
    .configurePID(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD)
    .configureBrakes(true);

    private BetterTalonFX backLeft = new BetterTalonFX(Constants.Swerve.Mod2.driveMotorID)
    .configurePID(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD)
    .configureBrakes(true);

    private BetterTalonFX backRight = new BetterTalonFX(Constants.Swerve.Mod3.driveMotorID)
    .configurePID(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD)
    .configureBrakes(true);

    private final double moveTimeout = 1;

public void execute() {
    frontLeft.setRPM(125.31885282826406); //Like 2 meters per second in theory
    frontRight.setRPM(125.31885282826406);
    backLeft.setRPM(125.31885282826406);
    backRight.setRPM(125.31885282826406);
    withTimeout(moveTimeout);
    frontLeft.stop(); //stop
    frontRight.stop();
    backLeft.stop();    
    backRight.stop();

}
    
}
