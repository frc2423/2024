// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  String deployDirectory = (Robot.isSimulation())? "neo" : "swerve";
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), deployDirectory)
  );
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);

  IntakeSubsystem intake =new IntakeSubsystem();

  ShooterSubsystem shooter = new ShooterSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    intake.beltStop();
    SmartDashboard.putData("Intake",intake);

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(
      drivebase,
      () ->
        MathUtil.applyDeadband(
          driverXbox.getLeftY(),
          OperatorConstants.LEFT_Y_DEADBAND
        ),
      () ->
        MathUtil.applyDeadband(
          driverXbox.getLeftX(),
          OperatorConstants.LEFT_X_DEADBAND
        ),
      () ->
        MathUtil.applyDeadband(
          driverXbox.getRightX(),
          OperatorConstants.RIGHT_X_DEADBAND
        ),
      driverXbox::getYButtonPressed,
      driverXbox::getAButtonPressed,
      driverXbox::getXButtonPressed,
      driverXbox::getBButtonPressed
    );

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () ->
        MathUtil.applyDeadband(
          driverXbox.getLeftY(),
          OperatorConstants.LEFT_Y_DEADBAND
        ),
      () ->
        MathUtil.applyDeadband(
          driverXbox.getLeftX(),
          OperatorConstants.LEFT_X_DEADBAND
        ),
      () -> driverXbox.getRightX(),
      () -> driverXbox.getRightY()
    );

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () ->
        MathUtil.applyDeadband(
          -driverXbox.getLeftY(),
          OperatorConstants.LEFT_Y_DEADBAND
        ),
      () ->
        MathUtil.applyDeadband(
          -driverXbox.getLeftX(),
          OperatorConstants.LEFT_X_DEADBAND
        ),
      () -> -driverXbox.getRightX()
    );

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
      () ->
        MathUtil.applyDeadband(
          -driverXbox.getLeftY(),
          OperatorConstants.LEFT_Y_DEADBAND
        ),
      () ->
        MathUtil.applyDeadband(
          driverXbox.getLeftX(),
          OperatorConstants.LEFT_X_DEADBAND
        ),
      () -> driverXbox.getRawAxis(2)
    );

    drivebase.setDefaultCommand(
      !RobotBase.isSimulation()
        ? driveFieldOrientedAnglularVelocity
        : driveFieldOrientedDirectAngleSim
    );


    //auto commands
      //EXAMPLE:  NamedCommands.registerCommand("useless", exampleSubsystem.exampleCommand());

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new JoystickButton(driverXbox, 1)
      .onTrue((new InstantCommand(drivebase::zeroGyro)));
    //new JoystickButton(driverXbox, 3)
     // .onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    //    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

    //When button 2 is pressed, move intake out and move belt motors.
   //new JoystickButton(driverXbox,2).onTrue(new InstantCommand(intake::extend)).onFalse(new InstantCommand(intake::pivotStop));

   new JoystickButton(driverXbox, XboxController.Button.kY.value).whileTrue(new RunCommand(intake::intake)).onFalse(new RunCommand(intake::beltStop));;
   new JoystickButton(driverXbox, XboxController.Button.kB.value).whileTrue(new RunCommand(intake::outtake)).onFalse(new RunCommand(intake::beltStop));;
   new JoystickButton(driverXbox, XboxController.Button.kA.value).onTrue(new RunCommand(intake::extend));
   new JoystickButton(driverXbox, XboxController.Button.kX.value).onTrue(new RunCommand(intake::retract));
   Command shooterCommand = shooter.shoot(); 
   new Trigger(() -> driverXbox.getRightTriggerAxis() > .5).onTrue(shooterCommand).onFalse(new RunCommand(() -> shooterCommand.cancel()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAuto("ExampleAuto");
  }

  public void setDriveMode() {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void zeroGyro() {
    drivebase.zeroGyro();
  }
}
