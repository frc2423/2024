package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.DAS;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveCommands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ShooterCommands {

    private ShooterSubsystem shooter;
    private ShooterFeedSubsystem shooterFeed;
    private ShooterAngleCommands shooterAngle;
    private IntakeCommands intake;
    private IntakeSubsystem iintake;
    private SwerveSubsystem drivebase;
    private SwerveCommands swerveCommands;

    public ShooterCommands(ShooterSubsystem shooter, ShooterAngleCommands shooterAngle, IntakeCommands intake,
            IntakeSubsystem iintake, SwerveSubsystem drivebase, SwerveCommands swerveCommands,
            ShooterFeedSubsystem shooterFeed) {
        this.shooter = shooter;
        this.shooterFeed = shooterFeed;
        this.intake = intake;
        this.shooterAngle = shooterAngle;
        this.iintake = iintake;
        this.drivebase = drivebase;
        this.swerveCommands = swerveCommands;
    }

    public Command shootFromIntakeAuto() {
        Command command = Commands.sequence(
                // shoot by intaking and running feeder motor. Once beam break no longer detects
                // game piece move on to next step
                intake.intakeIntake(.7)
                        .until(() -> !iintake.isBeamBroken()),
                // Game piece no longer in intake so stop intake and bring it down to prepare
                // for intaking the next game piece. Continue running feeder motor briefly since
                // game piece
                // might still be in the shooter
                Commands.sequence(intake.beltStopCommand(), intake.intakeDown()).withTimeout(.1)
        );
        command.setName("shoot in auto");
        return command;
    }

    public Command rev() {
        var command = Commands.run(() -> {
            shooter.setVoltageSpeed(-12);
            shooter.shooterOn();
        }, shooter).withTimeout(shooter.isDoneSec);
        command.setName("Revvvvv");
        return command;
    }

    public Command shootADude() {
        var command = Commands.run(() -> {
            shooter.setVoltageSpeedForHuman(-3);
            shooter.shooterOn();
        }, shooter).withTimeout(shooter.isDoneSec);
        command.setName("ShootAHuman");
        return command;
    }

    public Command runDAS() {
        Command wait = Commands.waitSeconds(.75);
        Command command = Commands.parallel(
                Commands.sequence(wait, moveFeedMotorFast()), revStartCommand(), shooterAngle.setShooterAngleFromDAS());
        command.setName("runDAS");
        return command;
    }

    public Command shootFromIntake() {
        Command command = Commands.sequence(
                Commands.parallel(
                        swerveCommands.lookAtTarget(Constants.autoAlign.speakerLocationPose,
                                Rotation2d.fromDegrees(180)),
                        // revSpeedFromDAS(),
                        // shooterAngle.setShooterAngleFromDAS().until(shooterAngle::isShooterAngleAtGoal).withTimeout(1.5)),
                        revSpeedFromDAS(), shooterAngle.setShooterAngleFromDAS().withTimeout(1.5)),
                Commands.parallel(
                        shoot(), intake.intakeHandoff()));
        command.setName("shootFromIntake");
        return command;

    }

    public Command stopShooter() {
        var command = Commands.runOnce(() -> {
            shooter.shooterOff();
        }, shooter);

        command.setName("Stop shooter!");
        return command;
    }

    public Command stopFeeder() {
        var command = Commands.runOnce(() -> {
            shooterFeed.stopFeederMotor();
        }, shooterFeed);

        command.setName("Stop feeder!");
        return command;
    }

    public Command stopIt() {
        var command = Commands.runOnce(() -> {
            shooter.shooterOff();
            shooterFeed.stopFeederMotor();
        }, shooter, shooterFeed);

        command.setName("Stop it!!");
        return command;
    }

    public Command shootAmp() {
        var command = Commands.run(() -> {
            shooterFeed.moveFeederMotorBackwards();
        }, shooterFeed);
        command.setName("Shooting amp");
        return command;
    }

    public Command moveFeedMotor() {
        var command = Commands.run(() -> shooterFeed.moveFeederMotor(), shooterFeed).withTimeout(.1);
        command.setName("Feeding");
        command.addRequirements(shooterFeed);
        return command;
    }

    public Command moveFeedMotorFast() {
        var command = Commands.run(() -> shooterFeed.moveFeederMotorFast(), shooterFeed).withTimeout(.1);
        command.setName("FeedingFast");
        return command;
    }

    public Command moveFeedMotorFastNoTimeout() {
        var command = Commands.run(() -> shooterFeed.moveFeederMotorFast(), shooterFeed);
        command.setName("FeedingFastNoTimeout");
        return command;
    }

    public Command moveFeedSlowCommand() {
        var command = Commands.run(() -> shooterFeed.moveFeederSlow(), shooterFeed).withTimeout(.3);
        command.setName("Feeding SLOW");
        return command;
    }

    public Command moveFeedSlowCommandAuto() {
        var command = Commands.run(() -> shooterFeed.moveFeederSlow(), shooterFeed).withTimeout(.5);
        command.setName("Feeding SLOW AUTO");
        return command;
    }

    public Command moveFeedSlowReverseCommand() {
        var command = Commands.run(() -> shooterFeed.moveFeederslowReverse(), shooterFeed).withTimeout(.1);
        command.setName("Feeding SLOW REVERSE");
        return command;
    }

    public Command moveFeedAmpCommand() {
        var command = Commands.run(() -> {
            shooter.moveFeederAmpOpp();
            shooterFeed.moveFeederAmpOpp();
        }, shooter, shooterFeed);
        command.setName("Feeding SLOW REVERSE");
        return command;
    }

    public Command moveFeedAmpCommandEnd() {
        var endCommand = Commands.run(() -> {
            shooter.moveFeederAmpOppEnd();
            shooterFeed.moveFeederAmpOppEnd();
        }, shooter, shooterFeed).withTimeout(.35);
        // endCommand.withInterruptBehavior(InterruptionBehavior.)
        endCommand.setName("Feeding END");
        return endCommand;
    }

    public Command moveFeedAmpOppCommand() {
        var command = Commands.run(() -> shooterFeed.moveFeederAmp(), shooterFeed);
        command.setName("Feeding SLOW");
        return command;
    }

    public Command shoot() {
        var command = Commands.run(() -> {
            shooterFeed.moveFeederMotor();
        }, shooterFeed).withTimeout(shooter.isDoneShoot);
        command.setName("Shoot");
        return command;
    }

    public Command revAndShoot() {
        var command = Commands.sequence(
                rev(),
                shoot());
        command.setName("Revvvvv and Shoot");
        return command;
    }

       public Command revAndShootTheBros() { 
        var command = Commands.sequence(
                shootADude(),
                shoot());
        command.setName("Revvvvv and Shoot the haters");
        return command;
    }

    public Command shooterOnFlop() {
        var command = Commands.run(() -> shooter.shooterOnFlop(), shooter).withTimeout(0.1);
        command.setName("sPiNiNg");
        return command;
    }

    public Command handOffCommand() {
        var command = Commands.sequence(
                Commands.parallel(shooterFeed.setFeedVoltage(6.5
                ), intake.intakeIntake(.7), shooterOnFlop()) //, shooterOnFlop()
                        .until(() -> !iintake.isBeamBroken()),
                Commands.parallel(shooterFeed.setFeedVoltage(6.5), intake.intakeIntake(.7), shooterOnFlop()).withTimeout(.5), //, shooterOnFlop())
                //shooterFeed.setFeedSpeed(-.1),
                shooterFeed.setFeedVoltage(-.5),
                Commands.waitSeconds(.3),
                Commands.runOnce(() -> {
                    shooter.shooterOff();
                    shooterFeed.feedOff();
                }));
        // intake.intakeUp().until(() -> iintake.isAngleGreat()));
        command.setName("Hand Off");
        return command;
    }

    public Command handOffCommandAuto() {
        Command shooterHandOff = shooterAngle.handOffAngleCommand();
        var command = Commands.sequence(shooterHandOff.withTimeout(0.02),
                Commands.parallel(moveFeedSlowCommand(), intake.intakeIntake(.7), shooterOnFlop())
                        .until(() -> !iintake.isBeamBroken()),
                Commands.parallel(moveFeedSlowCommand(), intake.intakeIntake(.7), shooterOnFlop()).withTimeout(.75),
                Commands.run(() -> shooterFeed.moveFeederHandoff()).withTimeout(.1),
                Commands.runOnce(() -> {
                    shooter.shooterOff();
                    shooterFeed.feedOff();
                }));
        // intake.intakeUp().until(() -> iintake.isAngleGreat()));
        command.setName("Hand off Auto");
        return command;
    }

    public Command flopAmpCommand() {

        var command = Commands.sequence(
                intake.intakeInWithRevCommand(),
                Commands.parallel(moveFeedSlowCommand(), intake.intakeOutWithFeedCommand(), shooterOnFlop()),
                intake.intakeDown().until(() -> iintake.isAngleGreat()),
                shooterAngle.ampAngleCommand());
        command.setName("Get ready to flop");
        return command;
    }

    public Command autoFlopCommand() {

        var command = Commands.sequence(
                intake.intakeInWithRevCommand(),
                Commands.parallel(moveFeedSlowCommand(), intake.intakeOutWithFeedCommand(), shooterOnFlop()),
                intake.intakeDown().until(() -> iintake.isAngleGreat()));
        command.setName("Get ready to flop");
        return command;
    }

    private Command revSpeedFromDAS() {
        return Commands.run(() -> {
            double distance = drivebase.getDistanceToSpeaker();
            DAS.MotorSettings as = RobotContainer.das.calculateAS(distance);
            shooter.setPidSpeed(-1000);//as.getVelocity()
            shooter.shooterOn();
        }, shooter).until(() -> shooter.isRevatSpeed()).withTimeout(4);
    }

    public Command revStartCommand() {
        Command command = Commands.run(() -> {
            shooter.setVoltageSpeed(-12/2.5);
            shooter.shooterOn();
        }, shooter);
        command.setName("rev Start");
        command.addRequirements(shooter);
        return command;
    }

    public Command revStopCommand() {
        Command command = Commands.run(() -> {
            shooter.setVoltageSpeed(0);
            shooter.shooterOff();
        }, shooter);
        command.setName("rev Stop");
        return command;
    }

    public Command shooterCommand() {
        Command shooterCommand = Commands.sequence(
                Commands.parallel(rev(), intake.intakeInWithRevCommand()),
                Commands.parallel(shoot(), intake.intakeOutWithShoot()));
        shooterCommand.setName("Rev,Intake,Shoot");
        return shooterCommand;
    }

    public Command shootFromDAS() {
        Command command = Commands.sequence(
                Commands.parallel(
                        swerveCommands.lookAtTarget(Constants.autoAlign.speakerLocationPose,
                                Rotation2d.fromDegrees(180)),
                        revSpeedFromDAS(), shooterAngle.setShooterAngleFromDAS().withTimeout(1.5)),
                shoot(),
                stopIt(),
                shooterAngle.handOffAngleCommand());

        command.setName("shootFromDAS");
        return command;
    }

    public Command prepareToShoot() {
        Command command = Commands.sequence(
                Commands.parallel(
                    // need to change speaker location pose to be something else that allows driver to change x, y while angle remains fixed on target
                        swerveCommands.lookAtTargetButStillMove(Constants.autoAlign.ID3Pose,
                                Rotation2d.fromDegrees(180)),
                        revStartCommand().withTimeout(150), shooterAngle.setShooterAngleFromDAS().withTimeout(150)));
                
        command.setName("shootFromDAS");
        return command;
    }

    public Command intakeSequencePlusHandoffCommand() {
        Command intakeDown = Commands.runOnce(iintake::extend);
        Command intakeStart = intake.intakeIntake().until(iintake::isBeamBroken);
        Command intakeStop = Commands.runOnce(iintake::beltStop);
        Command intakeUp = Commands.runOnce(iintake::retract);
        Command wait = Commands.waitSeconds(1);
        Command handoff = handOffCommand();

        Command bestSequence = Commands.sequence(intakeDown, intakeStart.withTimeout(20), intakeStop, intakeUp, wait,
                handoff);
        bestSequence.setName("Intake Sequence Plus Handoff");
        return bestSequence;
    }
}
