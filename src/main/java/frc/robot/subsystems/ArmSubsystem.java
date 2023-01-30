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

 private TalonFX shoulderFalcon = new TalonFX(21);
 private CANSparkMax elbowMotor = new CANSparkMax(22, MotorType.kBrushless);
 private CANSparkMax wristMotor = new CANSparkMax(23, MotorType.kBrushless);

  public void setSpeeds(double shoulderSpeed, double elbowSpeed, double wristSpeed) {
    shoulderFalcon.set(ControlMode.PercentOutput, shoulderSpeed);    
    elbowMotor.set(elbowSpeed);
    wristMotor.set(wristSpeed);

  }

public void stopMotor() {
  shoulderFalcon.set(ControlMode.PercentOutput, 0);
  elbowMotor.set(0);
  wristMotor.set(0);
}
}