// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax shooterLeftFront;
  private final CANSparkMax shooterRightFront;
  private final CANSparkMax shooterRightBack;
  private final CANSparkMax shooterLeftBack;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterLeftBack = new CANSparkMax(9, MotorType.kBrushless);
    shooterLeftFront = new CANSparkMax(10, MotorType.kBrushless);
    shooterRightBack = new CANSparkMax(11, MotorType.kBrushless);
    shooterRightFront = new CANSparkMax(12, MotorType.kBrushless);

    shooterLeftBack.follow(shooterLeftFront);
    shooterRightBack.follow(shooterRightFront);
  }

  public void shoot() {
    shooterLeftFront.set(ShooterConstants.MAX_SHOOTER_SPEED);
    shooterRightFront.set(ShooterConstants.MAX_SHOOTER_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
