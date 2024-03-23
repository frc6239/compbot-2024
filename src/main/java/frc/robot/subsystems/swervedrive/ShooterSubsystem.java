// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax shooterFrontLeft;
  private final CANSparkMax shooterFrontRight;
  private final CANSparkMax shooterBackRight;
  private final CANSparkMax shooterBackLeft;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterFrontLeft = new CANSparkMax(21, MotorType.kBrushless);
    shooterBackLeft = new CANSparkMax(22, MotorType.kBrushless);
    shooterFrontRight = new CANSparkMax(23, MotorType.kBrushless);
    shooterBackRight = new CANSparkMax(24, MotorType.kBrushless);

    shooterBackLeft.follow(shooterFrontLeft);
    shooterBackRight.follow(shooterFrontRight);
  }

  public void shootOnionRing() {
      shooterFrontLeft.set(ShooterConstants.MAX_SHOOTER_SPEED);
      shooterFrontRight.set(ShooterConstants.MAX_SHOOTER_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
