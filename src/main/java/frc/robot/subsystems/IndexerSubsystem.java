// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IndexerSubsystem extends SubsystemBase {
  private final CANSparkMax indexerMotor;
  private boolean enabled;
  
  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
      indexerMotor = new CANSparkMax(22, MotorType.kBrushless);
      enabled = false;
      indexerMotor.setInverted(false);
  }
  public void run() {
    enabled = true;
  }

  public void stop() {
    indexerMotor.set(0);
    enabled = false;
  }

  public void invert() {
    indexerMotor.setInverted(!indexerMotor.getInverted());
  }

  public boolean getIsRunning() {
    return enabled;
  }

  @Override
  public void periodic() {
    if(enabled){
    indexerMotor.set(IntakeConstants.MAX_INTAKE_SPEED);
    } else {
      indexerMotor.set(0);
    }
    // This method will be called once per scheduler run
  }
}
