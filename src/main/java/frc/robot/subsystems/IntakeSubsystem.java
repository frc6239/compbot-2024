// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intakeMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(21, MotorType.kBrushless);
  }

  public void run() {
    intakeMotor.set(IntakeConstants.MAX_INTAKE_SPEED);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
