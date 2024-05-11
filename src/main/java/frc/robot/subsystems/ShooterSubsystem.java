// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax leftFrontMotor;
  private final CANSparkMax rightFrontMotor;
  private final CANSparkMax rightBackMotor;
  private final CANSparkMax leftBackMotor;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    leftBackMotor = new CANSparkMax(9, MotorType.kBrushless);
    leftFrontMotor = new CANSparkMax(10, MotorType.kBrushless);
    rightBackMotor = new CANSparkMax(11, MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(12, MotorType.kBrushless);

    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);
  }

  public void run() {
    leftFrontMotor.set(ShooterConstants.MAX_SHOOTER_SPEED);
    rightFrontMotor.set(ShooterConstants.MAX_SHOOTER_SPEED);
  }

  public void stop() {
    leftFrontMotor.set(0);
    rightFrontMotor.set(0);
  }

  public void invert() {
    leftFrontMotor.setInverted(!leftFrontMotor.getInverted());
    rightFrontMotor.setInverted(!rightFrontMotor.getInverted());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
