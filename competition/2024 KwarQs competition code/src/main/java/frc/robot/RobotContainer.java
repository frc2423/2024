// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterAngle;
import frc.robot.subsystems.shooter.ShooterAngleCommands;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

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

  // CommandJoystick driverController = new
  // CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT)
  XboxController driverXbox = new XboxController(0);
  XboxController operator = new XboxController(1);
  IntakeSubsystem intake = new IntakeSubsystem();
  ShooterSubsystem shooter = new ShooterSubsystem();
  ShooterAngle shooterAngle = new ShooterAngle();
  IntakeCommands intakeCommands = new IntakeCommands(intake);
  ShooterCommands shooterCommands = new ShooterCommands(shooter, intakeCommands);
  ShooterAngleCommands shooterAngleCommands = new ShooterAngleCommands(shooterAngle);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    intake.beltStop();
    SmartDashboard.putData("Intake", intake);
    SmartDashboard.putData("SwerveSubsystem", drivebase);
    SmartDashboard.putData("Shooter", shooter);
    SmartDashboard.putData("ShooterAngle", shooterAngle);

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Taxi Auto", "Taxi Auto");
    m_chooser.addOption("Amp to Note Auto", "Amp to Note Auto");
    m_chooser.addOption("Yo Auto", "Yo Auto");
    m_chooser.addOption("YoYo Auto", "YoYo Auto");
    m_chooser.addOption("Left Yo Auto", "Left Yo Auto");
    m_chooser.addOption("Test Auto", "Test Auto");

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> {
          double y = MathUtil.applyDeadband(
              -driverXbox.getLeftY(),
              // Math.copySign(Math.pow(-driverXbox.getLeftY(), 2), -driverXbox.getLeftY()),
              OperatorConstants.LEFT_Y_DEADBAND);
          return m_yspeedLimiter.calculate(y);
        },
        () -> {
          double x = MathUtil.applyDeadband(
              -driverXbox.getLeftX(),
              // Math.copySign(Math.pow(-driverXbox.getLeftX(), 2), -driverXbox.getLeftX()),
              OperatorConstants.LEFT_X_DEADBAND);
          return m_xspeedLimiter.calculate(x);
        },
        () -> -driverXbox.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> {
          double y = MathUtil.applyDeadband(
              -driverXbox.getLeftY(),
              OperatorConstants.LEFT_Y_DEADBAND);
          return m_yspeedLimiter.calculate(y);
        },
        () -> {
          double x = MathUtil.applyDeadband(
              driverXbox.getLeftX(),
              OperatorConstants.LEFT_X_DEADBAND);
          return m_xspeedLimiter.calculate(x);
        },
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation()
            ? driveFieldOrientedAnglularVelocity
            : driveFieldOrientedDirectAngleSim);

    // auto commands
    // EXAMPLE: NamedCommands.registerCommand("useless",
    // exampleSubsystem.exampleCommand());
NamedCommands.registerCommand("RevvvvvandShoot", shooterCommands.revAndShoot());
NamedCommands.registerCommand("IntakeSlurp", intakeCommands.intakeIntake());
NamedCommands.registerCommand("IntakeDown", intakeCommands.intakeDown());
NamedCommands.registerCommand("IntakeUntill", intakeCommands.intakeIntakeUntil());

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

    new Trigger(() -> driverXbox.getRightTriggerAxis() > .5).whileTrue(shooterCommands.shooterCommand());
    shooter.setDefaultCommand(shooterCommands.stopIt());

    new Trigger(() -> operator.getRightTriggerAxis() > .5).whileTrue(shooterCommands.shootAmp());
    shooter.setDefaultCommand(shooterCommands.stopIt());

    new JoystickButton(operator, XboxController.Button.kLeftBumper.value).whileTrue(shooterAngleCommands.moveShooterDown());
    new JoystickButton(operator, XboxController.Button.kRightBumper.value).whileTrue(shooterAngleCommands.moveShooterUp());
    
    new JoystickButton(operator, XboxController.Button.kA.value).whileTrue(shooterAngleCommands.shooterAngleCommand());
    new JoystickButton(operator, XboxController.Button.kB.value).whileTrue(shooterAngleCommands.feederAngleCommand());
    new JoystickButton(operator, XboxController.Button.kX.value).whileTrue(shooterAngleCommands.ampAngleCommand());
    new JoystickButton(operator, XboxController.Button.kY.value).whileTrue(shooterAngleCommands.climberAngleCommand());
    intake.setDefaultCommand(new RunCommand(intake::beltStop, intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
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
}
