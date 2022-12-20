// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Swerve;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class velocityAuto extends InstantCommand {

  // 2048 falcon ticks per revolution of the shft, 8.14:1 For MK4 L1 Modules drive ratio
  // Ticks per revolution of the wheel = 16671 ricks for wheel to rotate once
  private final double kDriveTick2Feet = 1.0 / 16671.0 * Swerve.wheelCircumference;
  final double setPoint = 3.0;

  final double kP = 0.5;

  private TalonFX drive0 = new TalonFX(Swerve.Mod0.driveMotorID);
  private TalonFX drive1 = new TalonFX(Swerve.Mod1.driveMotorID);
  private TalonFX drive2 = new TalonFX(Swerve.Mod2.driveMotorID);
  private TalonFX drive3 = new TalonFX(Swerve.Mod3.driveMotorID);


  public velocityAuto() {
    double currentPosition = drive0.getSelectedSensorPosition() * kDriveTick2Feet;

    double error = setPoint - currentPosition;

    double outputSpeed = kP * error;

    drive0.set(ControlMode.PercentOutput, outputSpeed);
    drive1.set(ControlMode.PercentOutput, outputSpeed);
    drive2.set(ControlMode.PercentOutput, outputSpeed);
    drive3.set(ControlMode.PercentOutput, outputSpeed);
    System.out.println("You should be driving");

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive0.getSelectedSensorPosition(0);
    drive1.getSelectedSensorPosition(0);
    drive2.getSelectedSensorPosition(0);
    drive3.getSelectedSensorPosition(0);
    drive2.setInverted(true);
    drive3.setInverted(true);
    System.out.println("Motor 2 and 4 set to inverted and sensors set to 0");
    System.out.println("LEFT SIDE STUFF:" + "Mod0 Sensor Position:" + drive0.getSelectedSensorPosition() + "Mod1 Sensor position:" + drive1.getSelectedSensorPosition());
    System.out.println("RIGHT SIDE STUFF:" + "Mod2 Sensor Position:" + drive2.getSelectedSensorPosition() + "Mod3 Sensor position:" + drive3.getSelectedSensorPosition());

  }
}
