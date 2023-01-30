// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class ArmSubsystem extends SubsystemBase {

 private CANSparkMax shoulderMotor = new CANSparkMax(21, MotorType.kBrushless);
 private CANSparkMax elbowMotor = new CANSparkMax(22, MotorType.kBrushless);
 private TalonFX wristMotor = new TalonFX(23);

  public void setSpeeds(double shoulderSpeed, double elbowSpeed, double wristSpeed) {
    shoulderMotor.set(shoulderSpeed);
    elbowMotor.set(elbowSpeed);
    wristMotor.set(ControlMode.PercentOutput, wristSpeed);

  }

public void stopMotor() {
  shoulderMotor.set(0);
  elbowMotor.set(0);
  wristMotor.set(ControlMode.PercentOutput, 0);
}
}