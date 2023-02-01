// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

/** Add your docs here. */
public class ArmSubsystem extends SubsystemBase {

 private TalonFX shoulderFalcon = new TalonFX(21);
 private CANSparkMax elbowMotor = new CANSparkMax(22, MotorType.kBrushless);
 private CANSparkMax wristMotor = new CANSparkMax(23, MotorType.kBrushless);

//Set neuteral mode to brake for each motor
public void init() {
shoulderFalcon.setNeutralMode(NeutralMode.Brake);
elbowMotor.setIdleMode(IdleMode.kBrake);

//Set open and closed loop ramp rates
wristMotor.setIdleMode(IdleMode.kBrake);
shoulderFalcon.configOpenloopRamp(.25);
shoulderFalcon.configClosedloopRamp(.25);
elbowMotor.setOpenLoopRampRate(.25);
elbowMotor.setClosedLoopRampRate(.25);
wristMotor.setOpenLoopRampRate(.25);
wristMotor.setClosedLoopRampRate(25);
}

 //Add ramp speed of .25 seconds to each motor

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

public double getShoulderAngle() {
  return shoulderFalcon.getSelectedSensorPosition();
}

public double getElbowAngle() {
  return elbowMotor.getEncoder().getPosition();
}

public double getWristAngle() {
  return wristMotor.getEncoder().getPosition();
}

public double getShoulderDegrees() {
  return shoulderFalcon.getSelectedSensorPosition() * 360 / 4096; //TODO Compensate for gearbox ratio
}

public double getElbowDegrees() {
  return elbowMotor.getEncoder().getPosition() * 360 / 42; //TODO Compensate for gearbox ratio
}

public double getWristDegrees() {
  return wristMotor.getEncoder().getPosition() * 360 / 42; //TODO Compensate for gearbox ratio
}

}