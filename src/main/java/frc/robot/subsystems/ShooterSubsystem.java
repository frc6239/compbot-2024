// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax leftFrontMotor;
  private final CANSparkMax rightFrontMotor;
  private final CANSparkMax rightBackMotor;
  private final CANSparkMax leftBackMotor;
  private boolean running;
  private double speed;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    leftBackMotor = new CANSparkMax(9, MotorType.kBrushless);
    leftFrontMotor = new CANSparkMax(10, MotorType.kBrushless);
    rightBackMotor = new CANSparkMax(11, MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(12, MotorType.kBrushless);

    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);
    speed = ShooterConstants.MAX_SHOOTER_SPEED;
    running=false;
  }

  public void run() {
    leftFrontMotor.set(speed);
    rightFrontMotor.set(speed);
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
    if (running ) {
      run();
    } else { 
      stop();
    }
  }

  public void enable () {
    running = true;
  }

  public void disable () {
    running = false;
  }

  public void increaseSpeed() {
    if (speed < 0.9) {
      speed=speed+0.1;

    }
  }

  public void decreaseSpeed() {
    if (speed > 0.1 ) {
      speed = speed - 0.1;
    }
    
  }

}
