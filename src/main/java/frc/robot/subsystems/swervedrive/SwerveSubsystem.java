package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.swerve.SwerveDrive;
import frc.robot.subsystems.swervedrive.swerve.SwerveModule.Verbosity;
import frc.robot.subsystems.swervedrive.swerve.SwerveParser;
import java.io.File;

/**
 * SwerveDrive Subsystem.
 */
public class SwerveSubsystem extends SubsystemBase
{

  private final Timer       syncTimer = new Timer();
  /**
   * Swerve Drive object.
   */
  public        SwerveDrive m_drive;
  //Creates Pigeon2 Gyroscope

  public SwerveSubsystem()
  {
    syncTimer.start();

    m_drive = SwerveParser.fromJSONDirectory(new File(Filesystem.getDeployDirectory(), "swerve"));
    m_drive.zeroGyro();
    m_drive.setDeadband(0.1);

    SmartDashboard.putData(m_drive);

  }

  /**
   * Drive function
   *
   * @param forward          -1 to 1 speed to go forward.
   * @param strafe           -1 to 1 speed to strafe.
   * @param r                -1 to 1 turning rate.
   * @param fieldOrientation Field oriented drive.
   */
  public void drive(double forward, double strafe, double r, boolean fieldOrientation)
  {
    m_drive.drive(forward, strafe, r, fieldOrientation);
  }

  /**
   * Stop the swerve drive.
   */
  public void stop()
  {
    m_drive.stopMotor();
  }

  @Override
  public void periodic()
  {
//    m_drive.synchronizeEncoders();

    if (syncTimer.advanceIfElapsed(1))
    {
      m_drive.publish(Verbosity.HIGH);
    }
    if (SmartDashboard.getBoolean("Update Swerve Drive", false))
    {
      SmartDashboard.putBoolean("Update Swerve Drive", false);
      m_drive.subscribe();
    }
  }
}
