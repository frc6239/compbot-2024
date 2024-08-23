// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoDriveToTargetCommand extends Command {
  private final VisionSubsystem vision;
  private final SwerveSubsystem swerve;

  private final PhotonCamera camera;
  private PhotonPipelineResult result;

    // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(15);
  final double TARGET_HEIGHT_METERS = Units.inchesToMeters(51.875);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(44.1);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.inchesToMeters(52.25);

  // PID constants should be tuned per robot
    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
    private double range;
    private double oldrange;
    
  
  /** Creates a new AutoDriveToTargetCommand. */
  public AutoDriveToTargetCommand(VisionSubsystem vision, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision, swerve);
    this.vision = vision;
    this.swerve = swerve;
    this.camera = vision.getCamera();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed;

    result = camera.getLatestResult();
    
    if (result.hasTargets()) {
      range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));


        if(range != oldrange){
          System.out.println("range=" + range);
          oldrange = range;
        }
      
    }
    
    swerve.driveCommand(null, null, null);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(range < 1){
      System.out.println("at target");
      return true;
    } else {
      return false;
    }
  }
}
