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
  // Creates new joystick object for the driver on port 0
  private final Joystick driver = new Joystick(0);
  private final Joystick arm = new Joystick(1);

  // Creates the Axis variables mapped to various joysticks on the gamepad
  private final int translationAxis = XboxController.Axis.kLeftY.value; //Y axis on left joystick, front to back motion
  private final int strafeAxis = XboxController.Axis.kLeftX.value; //X axis on the left joystick, left to right motion
  private final int rotationAxis = XboxController.Axis.kRightX.value; //X axis on the right joystick, turns the robot

  // Creates button mappings on the controller
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value); // Y button on the controller to zero the gyro

  // Define the Swerve subsystem as swerveSubsystem
  private final Swerve swerveSubsystem = new Swerve();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true; // Do you want field oriented control?
    boolean openLoop = true; // Do you want acceleration on the robot
    //swerveSubsystem.setDefaultCommand(new TeleopSwerve(swerveSubsystem, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));  //Default command to drive the bot
    armSubsystem.setDefaultCommand(new ArmManual(armSubsystem, arm, 0, 1, 2)); //Default Manual Arm Command
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
    // the testAuto routine will run in auton
    return new testingAuto(swerveSubsystem);
  }
}
