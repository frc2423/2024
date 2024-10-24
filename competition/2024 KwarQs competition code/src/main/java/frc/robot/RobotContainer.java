// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.controllers.NotVeryOldGuitarHero;
import frc.robot.subsystems.LED.KwarqsLed;
import frc.robot.subsystems.climber.ClimberCommands;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterAngle;
import frc.robot.subsystems.shooter.ShooterAngleCommands;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.shooter.ShooterFeedSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveCommands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionCommands;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.Optional;

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

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(7);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(7);

  // A chooser for autonomous commands
  SendableChooser<String> m_chooser = new SendableChooser<>();

  XboxController driverXbox = new XboxController(0);
  XboxController operator = new XboxController(1);
  NotVeryOldGuitarHero coolguy = new NotVeryOldGuitarHero();
  IntakeSubsystem intake = new IntakeSubsystem();
  ShooterSubsystem shooter = new ShooterSubsystem();
  ShooterFeedSubsystem shooterFeed = new ShooterFeedSubsystem();
  VisionSubsystem visionSubsystem = new VisionSubsystem();
  ShooterAngle shooterAngle = new ShooterAngle();
  ShooterAngleCommands shooterAngleCommands = new ShooterAngleCommands(shooterAngle, drivebase, shooter);
  IntakeCommands intakeCommands = new IntakeCommands(intake, shooterAngleCommands);
  SwerveCommands swerveCommands = new SwerveCommands(drivebase);
  ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  ClimberCommands climberCommands = new ClimberCommands(climberSubsystem);
  ShooterCommands shooterCommands = new ShooterCommands(shooter, shooterAngleCommands, intakeCommands, intake,
      drivebase, swerveCommands, shooterFeed);
  VisionCommands visionCommands = new VisionCommands(visionSubsystem, drivebase, intake, intakeCommands,
      shooterCommands, shooterAngleCommands);
  KwarqsLed ledKwarqs = new KwarqsLed(visionSubsystem, driverXbox);

  public static final DAS das = new DAS();

  public void JointReader() {
    NTHelper.setDouble("/field3d/urdf/joints/intake",
        intake.getPivotAngle().getRadians() - Rotation2d.fromDegrees(130).getRadians());
    NTHelper.setDouble("/field3d/urdf/joints/shooter", shooterAngle.getShooterAngle().getRadians() + (Math.PI / 2));
    NTHelper.setDoubleArray("/field3d/urdf/pose", NTHelper.getDoubleArrayPose2d(drivebase.getPose()));
    Pose3d cameraPose = new Pose3d(drivebase.getPose()).plus(Constants.Vision.kRobotToCam);
    NTHelper.setDoubleArray("/field3d/field/cameraPose", NTHelper.getDoubleArrayPose3d(cameraPose));
    NTHelper.setString("/field3d/field/origin", PoseTransformUtils.isRedAlliance() ? "red" : "blue");

    Rotation2d angle = drivebase.getLookAngle(PoseTransformUtils.transformXRedPose(Constants.autoAlign.middleNote));
    NTHelper.setDouble("/debug/autoRotationOverride", angle.getDegrees());


  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    intake.beltStop();
    SmartDashboard.putData("Intake", intake);
    SmartDashboard.putData("Climber", climberSubsystem);
    SmartDashboard.putData("SwerveSubsystem", drivebase);
    SmartDashboard.putData("ShooterFeed", shooterFeed);
    SmartDashboard.putData("Shooter", shooter);
    SmartDashboard.putData("ShooterAngle", shooterAngle);
    SmartDashboard.putData("VisionSubsystem", visionSubsystem);

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Taxi Auto", "Taxi Auto");

    // comp single game piece auto
    m_chooser.addOption("Comp Single Note", "Comp Single Note");

    // faster middle 2 Piece (emily and ben)
    // m_chooser.addOption("Faster Amp 2 Piece", "Faster Amp 2 Piece");
    // m_chooser.addOption("Faster Center 2 Piece", "Faster Center 2 Piece");
    // m_chooser.addOption("Faster Feeder 2 Piece", "Faster Feeder 2 Piece");
    // m_chooser.addOption("echarles Amp 3 Piece", "echarles Amp 3 Piece");
    m_chooser.addOption("Bens Amp 3 Piece", "Bens Amp 3 Piece");
    m_chooser.addOption("Bens Feeder 3 Piece", "Bens Feeder 3 Piece");

    // comp 2 piece autos
    m_chooser.addOption("Amp 2 Piece", "Amp 2 Piece");
    m_chooser.addOption("Center 2 Piece", "Center 2 Piece");
    m_chooser.addOption("Feeder 2 Piece", "Feeder 2 Piece");

    // center line autos
    m_chooser.addOption("Amp Center Wall 3 Piece", "Amp Center Wall 3 Piece");
    m_chooser.addOption("Amp Center Wall 2 Piece", "Amp Center 2 Piece");
    m_chooser.addOption("Feeder Center 2nd from Wall 2 Piece", "Feeder Center 2nd from Wall 2 Piece");
    m_chooser.addOption("Feeder Center Wall 2 Piece", "Feeder Center Wall 2 Piece");
    m_chooser.addOption("Bens 4 Piece2", "Bens 4 Piece2");
    // m_chooser.addOption("Bens 4 Piece", "Bens 4 Piece");

    // 3 piece autos
    // m_chooser.addOption("Amp 3 Piece", "Amp 3 Piece");
    // m_chooser.addOption("Feeder 3 Piece", "Feeder 3 Piece");

    m_chooser.addOption("Taxi Amp Side", "Taxi Amp Side");

    // faster 3 piece autos
    // m_chooser.addOption("Faster Feeder 3 Piece", "Faster Feeder 3 Piece");
    // m_chooser.addOption("Faster Amp 3 Piece", "Faster Amp 3 Piece");

    // Blitz autos
    // m_chooser.addOption("Blitz Center Note Auto", "Blitz Center Note Auto");

    // m_chooser.addOption("Blitz Center Line Auto", "Blitz Center Line Auto");
    m_chooser.addOption("Blitz Center Line Auto2", "Blitz Center Line Auto2");

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);

    Command driveFieldOrientedAngularVelocity = getTeleopDriveCommand();

    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    // auto commands
    // EXAMPLE: NamedCommands.registerCommand("useless",
    // exampleSubsystem.exampleCommand());
    NamedCommands.registerCommand("RevvvvvandShoot",
        shooterCommands.shooterCommand().andThen(shooterCommands.stopIt().withTimeout(.1))
            .withName("RevvvvvandShoot auto"));

    NamedCommands.registerCommand("IntakeSlurp",
        intakeCommands.intakeIntake().withName("IntakeSlurp auto"));
    NamedCommands.registerCommand("IntakeDown",
        intakeCommands.intakeDown().withTimeout(0.01).withName("IntakeDown auto"));
    NamedCommands.registerCommand("IntakeUntill",
        intakeCommands.intakeIntakeUntil().andThen(intakeCommands.beltStopCommand())
            .withName("IntakeUntill auto"));
    NamedCommands.registerCommand("IntakeUp",
        intakeCommands.intakeUp().withTimeout(2).withName("IntakeUp auto"));
    NamedCommands.registerCommand("stopIt", shooterCommands.stopIt().withName("stopIt auto"));

    NamedCommands.registerCommand("Shoot", shooterCommands.shoot());

    // NamedCommands.registerCommand("echarles Amp 3 Piece",
    // shooterCommands.shootFromIntakeAuto());

    NamedCommands.registerCommand("distanceShoot", shooterCommands.shootFromDAS());

    NamedCommands.registerCommand("shooterHandOffAngle", shooterAngleCommands.handOffAngleCommand());

    NamedCommands.registerCommand("HandOff", shooterCommands.intakeSequencePlusHandoffCommand());

    NamedCommands.registerCommand("IntakeSequence", intakeCommands.intakeSequence());

    NamedCommands.registerCommand("Week 0 Shot", intakeCommands.intakeSequence());

    NamedCommands.registerCommand("moveFeedSlowCommand", shooterCommands.moveFeedSlowCommandAuto());

    NamedCommands.registerCommand("rev Start", shooterCommands.revStartCommand().withName("rev Start"));

    NamedCommands.registerCommand("shootFromIntake", shooterCommands.shootFromIntakeAuto().withName("shootFromIntake"));

    NamedCommands.registerCommand("rev Stop", shooterCommands.revStopCommand().withName("rev Stop"));

    NamedCommands.registerCommand("Intake Slurp", shooterCommands.revStopCommand().withName("Intake Slurp"));

    NamedCommands.registerCommand("ShooterToAngle", shooterAngleCommands.setShooterAngleFromDAS().withTimeout(1.5));

    NamedCommands.registerCommand("runDAS", shooterCommands.runDAS());

    // .withTimeout(1.5)

    Command lookAtAmpNote = swerveCommands.lookAtTarget(Constants.autoAlign.ampNote, new Rotation2d());
    Command lookAtMiddleNote = swerveCommands.lookAtTarget(Constants.autoAlign.middleNote, new Rotation2d());
    Command lookAtStageNote = swerveCommands.lookAtTarget(Constants.autoAlign.stageNote, new Rotation2d());
    Command lookAtSpeaker = swerveCommands.lookAtTarget(Constants.autoAlign.speakerLocationPose,
        Rotation2d.fromDegrees(180));

    NamedCommands.registerCommand("lookAtAmpNote", lookAtAmpNote);
    NamedCommands.registerCommand("lookAtMiddleNote", lookAtMiddleNote);
    NamedCommands.registerCommand("lookAtStageNote", lookAtStageNote);
    NamedCommands.registerCommand("lookAtSpeaker", lookAtSpeaker);

    NamedCommands.registerCommand("setRotationTargetAmpNote",
        Commands.runOnce(() -> drivebase.setAutoRotationTarget(Constants.autoAlign.ampNote)));
    NamedCommands.registerCommand("setRotationTargetMiddleNote",
        Commands.runOnce(() -> drivebase.setAutoRotationTarget(Constants.autoAlign.middleNote)));
    NamedCommands.registerCommand("setRotationTargetStageNote",
        Commands.runOnce(() -> drivebase.setAutoRotationTarget(Constants.autoAlign.stageNote)));
    NamedCommands.registerCommand("setRotationTargetSpeaker",
        Commands.runOnce(() -> drivebase.setAutoRotationTarget(Constants.autoAlign.speakerLocationPose,
            Rotation2d.fromDegrees(180))));
    NamedCommands.registerCommand("setRotationTargetCenterNote1",
        Commands.runOnce(() -> drivebase.setAutoRotationTarget(Constants.autoAlign.center1Note)));
    NamedCommands.registerCommand("setRotationTargetCenterNote2",
        Commands.runOnce(() -> drivebase.setAutoRotationTarget(Constants.autoAlign.center2Note)));
    NamedCommands.registerCommand("setRotationTargetCenterNote3",
        Commands.runOnce(() -> drivebase.setAutoRotationTarget(Constants.autoAlign.center3Note)));
    NamedCommands.registerCommand("setRotationTargetCenterNote4",
        Commands.runOnce(() -> drivebase.setAutoRotationTarget(Constants.autoAlign.center4Note)));
    NamedCommands.registerCommand("setRotationTargetCenterNote5",
        Commands.runOnce(() -> drivebase.setAutoRotationTarget(Constants.autoAlign.center5Note)));

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      drivebase.getField().getObject("path").setPoses(poses);
    });
  }

  private Command getTeleopDriveCommand() {
    Command driveFieldOrientedAngularVelocity = drivebase.driveCommand(
        () -> {
          double y = MathUtil.applyDeadband(
              -driverXbox.getLeftY(),
              OperatorConstants.LEFT_Y_DEADBAND);
          if (PoseTransformUtils.isRedAlliance()) {
            y *= -1;
          }
          return m_yspeedLimiter.calculate(y);
        },
        () -> {
          double x = MathUtil.applyDeadband(
              -driverXbox.getLeftX(),
              OperatorConstants.LEFT_X_DEADBAND);
          if (PoseTransformUtils.isRedAlliance()) {
            x *= -1;
          }
          return m_xspeedLimiter.calculate(x);
        },
        () -> -driverXbox.getRightX());
    return driveFieldOrientedAngularVelocity;
  }

  private void configureBindings() {

    new JoystickButton(driverXbox, XboxController.Button.kStart.value)
        .onTrue((new InstantCommand(drivebase::zeroGyro)));
    // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new
    // InstantCommand(drivebase::lock, drivebase)));

    new JoystickButton(driverXbox, XboxController.Button.kY.value).whileTrue(intakeCommands.intakeOuttake()); // intake.intakeOuttake
    // .onFalse(new RunCommand(intake::beltStop));

    new JoystickButton(driverXbox, XboxController.Button.kB.value)
        .whileTrue(visionCommands.noteAutoAlignPickUp()
            .andThen(Commands.parallel(shooterCommands.intakeSequencePlusHandoffCommand(), getTeleopDriveCommand())));
    new JoystickButton(driverXbox, XboxController.Button.kB.value)
        .onTrue(visionCommands.noteAutoAlignPickUpHandoffAndPrepareToShoot());
    // .whileTrue(intakeCommands.intakeIntakeUntil().andThen(shooterCommands.intakeSequencePlusHandoffCommand()));
    // .onFalse(new RunCommand(intake::beltStop));

    // Command intakeOrOuttake = Commands.either(intakeCommands.intakeOuttake(),
    // intakeCommands.intakeIntake(), () -> driverXbox.getYButton());

    //new JoystickButton(driverXbox, XboxController.Button.kA.value).whileTrue(Commands.sequence(climberCommands.climbStartCommand(),
     //intakeCommands.intakeDown())).onFalse(climberCommands.climbStopCommand());
    //new JoystickButton(driverXbox, XboxController.Button.kX.value).whileTrue(Commands.sequence(intakeCommands.intakeDown(),
     //climberCommands.climbDownCommand())).onFalse(climberCommands.climbStopCommand()); for guitarhero

    // new JoystickButton(coolguy,
    // GuitarHeroController.Button.kGreen.value).whileTrue();
    new JoystickButton(driverXbox, XboxController.Button.kA.value).whileTrue(intakeCommands.intakeDown());
    // new JoystickButton(driverXbox,
    // XboxController.Button.kX.value).whileTrue(intakeCommands.intakeUp());

    new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value)
        .whileTrue(shooterAngleCommands.moveShooterUp());

    // new Trigger(() -> driverXbox.getRightTriggerAxis() >
    // .5).whileTrue(shooterCommands.shooterCommand());

    //new Trigger(() -> driverXbox.getRightTriggerAxis() > .5).whileTrue(shooterCommands.shootFromIntake())
        //.onFalse(shooterAngleCommands.handOffAngleCommand());
  new Trigger(() -> driverXbox.getRightTriggerAxis() > .5 && !intake.isHandoff()).whileTrue(shooterCommands.prepareToShoot());
    new Trigger(() -> driverXbox.getLeftTriggerAxis() > .5).whileTrue(shooterCommands.revAndShoot());
    new Trigger(() -> operator.getRightTriggerAxis() > .5).whileTrue(shooterCommands.moveFeedAmpCommand())
        .onFalse(shooterCommands.moveFeedAmpCommandEnd());
    new Trigger(() -> operator.getLeftTriggerAxis() > .5).whileTrue(shooterCommands.moveFeedAmpOppCommand());

    // new Trigger(() -> driverXbox.getRightTriggerAxis() >
    // .5).whileTrue(shooterCommands.shooterCommand());

    Command justRev = Commands.either(shooterCommands.rev(), shooterCommands.stopShooter(),
        () -> driverXbox.getRightBumper());

    shooter.setDefaultCommand(justRev);
    shooterFeed.setDefaultCommand(shooterCommands.stopFeeder());

    new Trigger(() -> operator.getPOV() == 180).whileTrue(shooterAngleCommands.shooterAngleCommand());
    new Trigger(() -> operator.getPOV() == 0).whileTrue(shooterAngleCommands.shuttleAngleCommand()); //broski
    new Trigger(() -> operator.getPOV() == 270).whileTrue(shooterAngleCommands.ampAngleCommand());
    new Trigger(() -> operator.getPOV() == 90).whileTrue(shooterAngleCommands.handOffAngleCommand());
    new Trigger(() -> driverXbox.getPOV() == 180)
        .whileTrue(swerveCommands.autoAlignShootCommand(Constants.autoAlign.shootPose));
    new Trigger(() -> driverXbox.getPOV() == 270)
        .whileTrue(swerveCommands.autoAlignAmpCommand(Constants.autoAlign.ampPose));
    new Trigger(() -> driverXbox.getPOV() == 90)
        .whileTrue(swerveCommands.autoAlignAmpCommand(Constants.autoAlign.sourceMiddlePose));
    // new Trigger(() -> driverXbox.getPOV() == 0)
    // .whileTrue(visionCommands.noteAutoAlignPickUp().andThen(shooterCommands.handOffCommand()));

    guitarHeroTriggers();

    new Trigger(intake::isBeamBroken).onTrue(Commands.run(() -> {
      operator.setRumble(RumbleType.kBothRumble, 1);
      driverXbox.setRumble(RumbleType.kBothRumble, 1);
    }).withTimeout(0.375).andThen(Commands.runOnce(() -> {
      operator.setRumble(RumbleType.kBothRumble, 0);
      driverXbox.setRumble(RumbleType.kBothRumble, 0);
    })));
    // new Trigger(() -> operator.getRightTriggerAxis() >
    // .5).whileTrue(shooterCommands.shootAmp());
    // shooter.setDefaultCommand(shooterCommands.stopIt());

    RobotModeTriggers.disabled().whileTrue(Commands.either(
        ledKwarqs.setYellow(), ledKwarqs.disable(), visionSubsystem::seesAprilTag));

    new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
        .whileTrue(shooterAngleCommands.moveShooterDown());
    new JoystickButton(operator, XboxController.Button.kRightBumper.value)
        .whileTrue(shooterAngleCommands.moveShooterUp());

        new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value)
        .whileTrue(shooterCommands.moveFeedMotorFastNoTimeout());

    new JoystickButton(operator, XboxController.Button.kA.value).whileTrue(intakeCommands.intakeDown());
    new JoystickButton(operator, XboxController.Button.kB.value)
        .whileTrue(intakeCommands.intakeIntakeUntil().andThen(shooterCommands.intakeSequencePlusHandoffCommand()));
    new JoystickButton(operator, XboxController.Button.kX.value).whileTrue(intakeCommands.intakeUp());
    new JoystickButton(operator, XboxController.Button.kY.value).whileTrue(intakeCommands.intakeOuttake());
    intake.setDefaultCommand(new RunCommand(intake::beltStop, intake));

    new JoystickButton(operator, XboxController.Button.kStart.value).onTrue(Commands.sequence(shooterAngleCommands.climbingAngleCommand(), climberSubsystem.climberDownCommand(), intakeCommands.intakeDown().withTimeout(.1))).onFalse(climberCommands.climbStopCommand());
    new JoystickButton(operator, XboxController.Button.kBack.value).onTrue(Commands.sequence(shooterAngleCommands.climbingAngleCommand(), intakeCommands.intakeDown().withTimeout(.05), climberSubsystem.climberUpCommand())).onFalse(climberCommands.climbStopCommand()); //whileTrue(shooterCommands.shootAmp());

    new JoystickButton(driverXbox, XboxController.Button.kX.value).whileTrue(intakeCommands.intakeUp());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    drivebase.setAutoRotationTarget(null);
    return drivebase.getAuto(m_chooser.getSelected());
  }

  public void guitarHeroTriggers(){
    new Trigger(() -> coolguy.getGreenButton()).onTrue(Commands.sequence(shooterAngleCommands.climbingAngleCommand(), climberSubsystem.climberDownCommand(), intakeCommands.intakeDown().withTimeout(.1))).onFalse(climberCommands.climbStopCommand()); //clibing fr //here
    new Trigger(() -> coolguy.getRedButton()).whileTrue(shooterAngleCommands.ampAngleCommand());
    new Trigger(() -> coolguy.getYellowButton()).whileTrue(shooterAngleCommands.shooterAngleCommand());
    new Trigger(() -> coolguy.getBlueButton()).whileTrue(shooterCommands.moveFeedAmpCommand());
    new Trigger(() -> coolguy.getOrangeButton()).whileTrue(shooterCommands.moveFeedAmpOppCommand());
    new Trigger(() -> coolguy.getSelectButton()).onTrue(Commands.sequence(shooterAngleCommands.climbingAngleCommand(), intakeCommands.intakeDown().withTimeout(.05), climberSubsystem.climberUpCommand())).onFalse(climberCommands.climbStopCommand()); // un climbing//figure climbing out
    new Trigger(() -> coolguy.getStartButton()).whileTrue(shooterAngleCommands.moveShooterDown()); 

    new Trigger(() -> coolguy.getUpStrum()).whileTrue(intakeCommands.intakeUp());
    new Trigger(() -> coolguy.getDownStrum()).whileTrue(Commands.run(() -> {System.out.println("STRUMMING DOWN");}));
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
    var estimatedPose = visionSubsystem.getEstimatedRobotPose();
    var std = visionSubsystem.getStandardDeviations();
    if (estimatedPose.isPresent() && std.isPresent()) {
      // var pose = estimatedPose.get().estimatedPose.toPose2d();
      // NTHelper.setDoubleArray("Measurments/estimatedPose",
      // NTHelper.getDoubleArrayPose2d(pose));
      // NTHelper.setDoubleArray("Measurments/std",
      // NTHelper.getDoubleArrayPose2d(pose));
      double distanceToSpeaker = drivebase.getDistanceToSpeaker(estimatedPose.get().estimatedPose.toPose2d());
      // Pose estimation is very inacurrate past 4 meters and is throwing off our
      // center piece autos
      boolean skipAdding = RobotState.isAutonomous() && distanceToSpeaker > 4.0 && RobotState.isEnabled();
      if (!skipAdding)
        drivebase.addCameraInput(estimatedPose.get().estimatedPose.toPose2d(),
            visionSubsystem.getTimestampSeconds(), std.get());
    }
  }

  public void updateVision() {
    // visionSubsystem.periodic();
    Optional<Transform3d> bestResult = visionSubsystem.getLatestResult();
    if (bestResult != null && bestResult.isPresent()) {
      // Transform3d transform = bestResult.get();
      // NTHelper.setDouble("Measurments/april-tag-x", transform.getX());
      // NTHelper.setDouble("Measurments/april-tag-y", transform.getY());
      // NTHelper.setDouble("Measurments/april-tag-z", transform.getZ());
      // NTHelper.setDouble("Measurments/april-tag-id", visionSubsystem.getLatestId);
      addVision();
    }
    // visionSubsystem.updateNoteV();

    // NTHelper.setDouble("Measurments/april-tag-rot", bestResult.getRotation());
    // //"idk how much that is in practical suck terms" -travis 3/5/24 6:15 pm
  }
}
