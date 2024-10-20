// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.auto.AutoAimAtTargetCommand;
import frc.robot.commands.swervedrive.auto.AutoDriveToTargetCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.io.File;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkMax;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();
  private final IndexerSubsystem Indexer = new IndexerSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);

  private GenericEntry m_shooter_max_speed;
  
  // SmartDashboard Widget
  private ComplexWidget m_autoSelectionComplexWidget;
  
  private String m_autopathselected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Holds names of paths for autonomous
  private List<String> pathNames;
  private AbsoluteDriveAdv closedAbsoluteDriveAdv;
  private Command driveFieldOrientedDirectAngle;
  private Command driveFieldOrientedAnglularVelocity;
  private Command driveFieldOrientedDirectAngleSim;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    configurePathPlannerCommands();

    // Setup Dashboards
    setupDashboards();

    closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(-driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(-driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(-driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY() * 0.75, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX() * 0.75, OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX() * 0.75);

    driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRawAxis(2));

        ChassisSpeeds a = drivebase.getRobotVelocity();
        System.out.println("vx speed" + a.vxMetersPerSecond);
        System.out.println("vy speed" + a.vyMetersPerSecond);
        System.out.println("omega speed" + a.omegaRadiansPerSecond);
  }

  private void setupDashboards() {
    // Get names of all of the paths
    pathNames = AutoBuilder.getAllAutoNames();

    // Iterate list of pathnames and add them as choices on Shuffleboard
    for (int i=0; i< pathNames.size(); i++) {
      m_chooser.addOption(pathNames.get(i),pathNames.get(i));
      //m_chooser_smartdashboard.addOption(pathNames.get(i), pathNames.get(i));
    }

    // Set default option to be the first element
    // FIXME:  Matt to create Default path and hard code Default string below
    m_chooser.setDefaultOption("Default Auto","Default Auto");
    //m_chooser_smartdashboard.setDefaultOption(pathNames.get(0), pathNames.get(0));

    
    // Put the Autonomous chooser on SmartDashboard 
    SmartDashboard.putData("Auto Command", m_chooser);
    

    
    // Construct Shuffleboard
    // Put a widget to allow driver to select speed of shooter
    m_shooter_max_speed =
        Shuffleboard.getTab("Configuration")
            .add("Shooter Max Speed", Constants.ShooterConstants.MAX_SHOOTER_SPEED)
            .withWidget("Number Slider")
            .withPosition(1, 1)
            .withSize(2, 1)
            .withProperties(Map.of("min", 0, "max", 1))
            .getEntry();

    
    
    
    // Put the Autonomous chooser on the Shuffleboard
    m_autoSelectionComplexWidget = Shuffleboard.getTab("Configuration")
      .add("Auto Path Command", m_chooser)
      .withSize(2, 1)
      .withPosition(0, 0)
      .withWidget(BuiltInWidgets.kComboBoxChooser);

  
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Drivetrain
    driverXbox.x().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    //driverXbox.x().onTrue(new AutoDriveToTargetCommand(vision, drivebase));
    // driverXbox.b().whileTrue(
    //  Commands.deferredProxy(() -> drivebase.driveToPose(
    //                         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //              ));

    // Shooter button bindings
    //driverXbox.rightBumper().onTrue(Commands.runOnce(shooter::run, shooter));
    //.rightBumper().onFalse(Commands.runOnce(shooter::stop, shooter));
    //driverXbox.y().onTrue(Commands.runOnce(shooter::invert, shooter));
    driverXbox.a().onTrue(Commands.runOnce(shooter::decreaseSpeed, shooter));
    driverXbox.y().onTrue(Commands.runOnce(shooter::increaseSpeed, shooter));

    
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    // Intake
    //driverXbox.b().toggleOnTrue(Commands.startEnd(intake::run, intake::stop, intake));
    driverXbox.rightBumper().onTrue(Commands.runOnce(intake::run, intake));
    driverXbox.rightBumper().onFalse(Commands.runOnce(intake::stop, intake));


    // Indexer button bindings
    driverXbox.leftBumper().onTrue(Commands.runOnce(Indexer::run, Indexer));
    driverXbox.leftBumper().onFalse(Commands.runOnce(Indexer::stop, Indexer));
    
    // Vision
  }

  private void configurePathPlannerCommands() {
    NamedCommands.registerCommand("runShooter", Commands.runOnce(shooter::enable));
   // NamedCommands.registerCommand("stopShooter", Commands.runOnce(shooter::disable));
    NamedCommands.registerCommand("runIndexer", Commands.runOnce(Indexer::run));
      NamedCommands.registerCommand("stopIndexer", Commands.runOnce(Indexer::stop));
      NamedCommands.registerCommand("resetGyro", Commands.runOnce(drivebase::zeroGyro));
    NamedCommands.registerCommand("stopRobot", Commands.runOnce(() -> {
      drivebase.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }));
  }


  public void setShooterSpeedFromDashboard () {
    shooter.setspeed(m_shooter_max_speed.getDouble(Constants.ShooterConstants.MAX_SHOOTER_SPEED));
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Get the user autonomous path selection from the dashboard
    m_autopathselected =m_chooser.getSelected();
   
    System.out.println("Auto Path selected: " + m_autopathselected);

    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand(m_autopathselected);
  }

  public void setDriveMode()
  {
    
    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public IntakeSubsystem getIntake() {
    return intake;
  }

  public ShooterSubsystem getShooter() {
    return shooter;
  }
}
