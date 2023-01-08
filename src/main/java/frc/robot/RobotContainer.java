// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BDSM license file in the root directory of this project.  

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // uwu what we doing to contowol da wowobot
  private final Joystick driver = new Joystick(0);

  // uwu dis is how we dwive de rowobot
  private final int translationAxis = XboxController.Axis.kLeftY.value; //dwive fowoard with da y stick
  private final int strafeAxis = XboxController.Axis.kLeftX.value; //slidey with da x stick
  private final int rotationAxis = XboxController.Axis.kRightX.value; //add a little rotation in there

  // the driver gets to push buttons
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

  // at this point go touch grass and make a sussystem
  private final Swerve swerveSubsystem = new Swerve();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true; // Is it field oriented?  if not say false and you will be terminated
    boolean openLoop = true;
    swerveSubsystem.setDefaultCommand(new TeleopSwerve(swerveSubsystem, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.whenPressed(new InstantCommand(() -> swerveSubsystem.zeroGyro()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // the testAuto command will run in auton
    return new testingAuto(swerveSubsystem);
  }
}
