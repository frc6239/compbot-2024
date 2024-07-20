// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera;
  private PhotonPipelineResult result;
  private boolean hasTargets;
  List<PhotonTrackedTarget> targets;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  }

  void getTargetInformation(int index) {
    int targetID = targets.get(index).getFiducialId();
    double yaw = targets.get(index).getYaw();
    double pitch = targets.get(index).getPitch();
    double area = targets.get(index).getArea();
    double skew = targets.get(index).getSkew();

    System.out.println("[" + targetID + "] Yaw: " + yaw);
    System.out.println("[" + targetID + "] Pitch: " + pitch);
    System.out.println("[" + targetID + "] Area: " + area);
    System.out.println("[" + targetID + "] Skew: " + skew);
    System.out.println();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();
    hasTargets = result.hasTargets();

    if (hasTargets) {
      targets = result.getTargets();

      for (int i = 0; i < targets.size(); i++) {
        getTargetInformation(i);
      }
    }
  }
}
