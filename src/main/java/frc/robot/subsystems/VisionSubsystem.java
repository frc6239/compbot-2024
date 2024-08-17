// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera;
  private PhotonPipelineResult result;
  private PhotonTrackedTarget target;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    camera = new PhotonCamera("protonvision");
  }

  public PhotonCamera getCamera() {
    return camera;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
