// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterAngle;
import frc.robot.subsystems.shooter.ShooterAngleCommands;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveCommands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

<<<<<<< HEAD
import java.io.File;
import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

=======
>>>>>>> main
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */

public class RobotContainer {
  String deployDirectory = (Robot.isSimulation()) ? "neo" : "swerve";
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), deployDirectory));

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(2);

  // A chooser for autonomous commands
  SendableChooser<String> m_chooser = new SendableChooser<>();

  XboxController driverXbox = new XboxController(0);
  XboxController operator = new XboxController(1);
  IntakeSubsystem intake = new IntakeSubsystem();
  ShooterSubsystem shooter = new ShooterSubsystem();
  VisionSubsystem visionSubsystem = new VisionSubsystem();
  ShooterAngle shooterAngle = new ShooterAngle(intake);
  IntakeCommands intakeCommands = new IntakeCommands(intake);
  ShooterAngleCommands shooterAngleCommands = new ShooterAngleCommands(shooterAngle, drivebase);
  ShooterCommands shooterCommands = new ShooterCommands(shooter, shooterAngleCommands, intakeCommands, intake, drivebase);
  SwerveCommands swerveCommands = new SwerveCommands(drivebase);
  public static final DAS das = new DAS();

  public void JointReader() {
    NTHelper.setDouble("/joints/intake",
        intake.getPivotAngle().getRadians() - Rotation2d.fromDegrees(130).getRadians());
    NTHelper.setDouble("/joints/shooter", shooterAngle.getShooterAngle().getRadians() + (Math.PI / 2));
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DAS.MotorSettings as = das.calculateAS(1.3);
    double asAngle = as.getAngle();
    double asVoltage = as.getVoltage();
    System.out.println("DASissupa");
    System.out.println(asAngle +","+ asVoltage);

    // Configure the trigger bindings
    configureBindings();
    intake.beltStop();
    SmartDashboard.putData("Intake", intake);
    SmartDashboard.putData("SwerveSubsystem", drivebase);
    SmartDashboard.putData("Shooter", shooter);
    SmartDashboard.putData("ShooterAngle", shooterAngle);

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Taxi Auto", "Taxi Auto");
    m_chooser.addOption("Yo Auto", "Yo Auto");
    m_chooser.addOption("Amp Yo Auto", "Amp Yo Auto");
    m_chooser.addOption("Feeder Yo Auto", "Feeder Yo Auto");
    m_chooser.addOption("ShootAndStayStill", "ShootAndStayStill");
    m_chooser.addOption("ShootAndStayStillFeeder", "ShootAndStayStillFeeder");
    m_chooser.addOption("ShootAndStayStillAmp", "ShootAndStayStillAmp");
    m_chooser.addOption("YoYo Auto", "YoYo Auto");
    m_chooser.addOption("Feeder YoYo Auto", "Feeder YoYo Auto");
    m_chooser.addOption("Amp YoYo Auto", "Amp YoYo Auto");
    m_chooser.addOption("Amp to Note Auto", "Amp to Note Auto");

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> {
          double y = MathUtil.applyDeadband(
              -driverXbox.getLeftY(),
              OperatorConstants.LEFT_Y_DEADBAND);
          return m_yspeedLimiter.calculate(y);
        },
        () -> {
          double x = MathUtil.applyDeadband(
              -driverXbox.getLeftX(),
              OperatorConstants.LEFT_X_DEADBAND);
          return m_xspeedLimiter.calculate(x);
        },
        () -> -driverXbox.getRightX());

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // auto commands
    // EXAMPLE: NamedCommands.registerCommand("useless",
    // exampleSubsystem.exampleCommand());
    NamedCommands.registerCommand("RevvvvvandShoot",
        new LoggedCommand(shooterCommands.shooterCommand().andThen(shooterCommands.stopIt().withTimeout(.1))
            .withName("RevvvvvandShoot auto")));

    NamedCommands.registerCommand("IntakeSlurp",
        new LoggedCommand(intakeCommands.intakeIntake().withName("IntakeSlurp auto")));
    NamedCommands.registerCommand("IntakeDown",
        new LoggedCommand(intakeCommands.intakeDown().withTimeout(0.01).withName("IntakeDown auto")));
    NamedCommands.registerCommand("IntakeUntill",
        new LoggedCommand(intakeCommands.intakeIntakeUntil().andThen(intakeCommands.beltStopCommand())
            .withName("IntakeUntill auto")));
    NamedCommands.registerCommand("IntakeUp",
        new LoggedCommand(intakeCommands.intakeUp().withTimeout(2).withName("IntakeUp auto")));
    NamedCommands.registerCommand("stopIt", new LoggedCommand(shooterCommands.stopIt().withName("stopIt auto")));

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      drivebase.getField().getObject("path").setPoses(poses);
    });
  }

  private void configureBindings() {
    new JoystickButton(driverXbox, XboxController.Button.kStart.value)
        .onTrue((new InstantCommand(drivebase::zeroGyro)));
    // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new
    // InstantCommand(drivebase::lock, drivebase)));

    new JoystickButton(driverXbox, XboxController.Button.kY.value).whileTrue(intakeCommands.intakeOuttake()); // intake.intakeOuttake
    // .onFalse(new RunCommand(intake::beltStop));

    new JoystickButton(driverXbox, XboxController.Button.kB.value)
        .whileTrue(intakeCommands.intakeIntakeUntil());
    // .onFalse(new RunCommand(intake::beltStop));

    new JoystickButton(driverXbox, XboxController.Button.kA.value).whileTrue(intakeCommands.intakeDown());
    new JoystickButton(driverXbox, XboxController.Button.kX.value).whileTrue(intakeCommands.intakeUp());
    new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value)
        .whileTrue(shooterAngleCommands.moveShooterDown());
    new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value)
        .whileTrue(shooterAngleCommands.moveShooterUp());

    new Trigger(() -> driverXbox.getRightTriggerAxis() > .5).whileTrue(shooterCommands.shooterCommand());
    shooter.setDefaultCommand(shooterCommands.stopIt());

    new Trigger(() -> operator.getPOV() == 180).whileTrue(shooterAngleCommands.shooterAngleCommand());
    new Trigger(() -> operator.getPOV() == 0).whileTrue(shooterAngleCommands.climberAngleCommand());
    new Trigger(() -> operator.getPOV() == 270).whileTrue(shooterAngleCommands.ampAngleCommand());
    new Trigger(() -> operator.getPOV() == 90).whileTrue(shooterCommands.handOffCommand());

    // new Trigger(() -> operator.getRightTriggerAxis() >
    // .5).whileTrue(shooterCommands.shootAmp());
    // shooter.setDefaultCommand(shooterCommands.stopIt());

    new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
        .whileTrue(shooterAngleCommands.moveShooterDown());
    new JoystickButton(operator, XboxController.Button.kRightBumper.value)
        .whileTrue(shooterAngleCommands.moveShooterUp());

    new JoystickButton(operator, XboxController.Button.kA.value).whileTrue(intakeCommands.intakeDown());
    new JoystickButton(operator, XboxController.Button.kB.value).whileTrue(intakeCommands.intakeIntakeUntil());
    new JoystickButton(operator, XboxController.Button.kX.value).whileTrue(intakeCommands.intakeUp());
    new JoystickButton(operator, XboxController.Button.kY.value).whileTrue(intakeCommands.intakeOuttake());
    intake.setDefaultCommand(new RunCommand(intake::beltStop, intake));

    new JoystickButton(operator, XboxController.Button.kStart.value).whileTrue(shooterCommands.autoFlopCommand());
    new JoystickButton(operator, XboxController.Button.kBack.value).whileTrue(shooterCommands.shootAmp());

    new Trigger(() -> driverXbox.getRightTriggerAxis() > .5).whileTrue(shooterCommands.shooterCommand());
    shooter.setDefaultCommand(shooterCommands.stopIt());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAuto(m_chooser.getSelected());
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void zeroGyro() {
    drivebase.zeroGyro();
  }

  public void updateSimVision() {
    // throw new UnsupportedOperationException("Unimplemented method
    // 'updateSimVision'");
    visionSubsystem.simulationPeriodic(drivebase.getPose());
    visionSubsystem.getLatestResult();
    
    // System.out.println(drivebase.getPose());

  }

  public void addVision() {
    drivebase.addCameraInput(visionSubsystem.getEstimatedRobotPose().estimatedPose.toPose2d(),
        visionSubsystem.getTimestampSeconds(), visionSubsystem.getStandardDeviations());
  }

  public void updateVision() {
    // visionSubsystem.periodic();
    Optional<Transform3d> bestResult = visionSubsystem.getLatestResult();
    if (bestResult != null && bestResult.isPresent()) {
      Transform3d transform = bestResult.get();
      NTHelper.setDouble("Measurments/april-tag-x", transform.getX());
      NTHelper.setDouble("Measurments/april-tag-y", transform.getY());
      NTHelper.setDouble("Measurments/april-tag-z", transform.getZ());
      NTHelper.setDouble("Measurments/april-tag-id", visionSubsystem.getLatestId);
    }
    // NTHelper.setDouble("Measurments/april-tag-rot", bestResult.getRotation());
  }
}
