package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.DAS;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ShooterCommands {

    private ShooterSubsystem shooter;
    private ShooterAngleCommands shooterAngle;
    private IntakeCommands intake;
    private IntakeSubsystem iintake;
    private SwerveSubsystem drivebase;
    private DAS das = new DAS();

    public ShooterCommands(ShooterSubsystem shooter, ShooterAngleCommands shooterAngle, IntakeCommands intake,
            IntakeSubsystem iintake, SwerveSubsystem drivebase) {
        this.shooter = shooter;
        this.intake = intake;
        this.shooterAngle = shooterAngle;
        this.iintake = iintake;
        this.drivebase = drivebase;
    }

    public Command rev() {
        var command = Commands.run(() -> {
            shooter.shooterOn();
        }).withTimeout(shooter.isDoneSec);
        command.setName("Revvvvv");
        return command;
    }

    public Command stopIt() {
        var command = Commands.run(() -> shooter.everythingOffPlease(), shooter);
        command.setName("Stop it");
        return command;
    }

    public Command shootAmp() {
        var command = Commands.run(() -> shooter.moveFeederMotorBackwards(), shooter);
        command.setName("Shooting amp");
        return command;
    }

    public Command moveFeedMotor() {
        var command = Commands.run(() -> shooter.moveFeederMotor(), shooter).withTimeout(.1);
        command.setName("Feeding");
        return command;
    }

    public Command moveFeedSlowCommand() {
        var command = Commands.run(() -> shooter.moveFeederSlow(), shooter).withTimeout(.1);
        command.setName("Feeding SLOW");
        return command;
    }

    public Command shoot() {
        var command = Commands.run(() -> {
            shooter.moveFeederMotor();
        }, shooter).withTimeout(shooter.isDoneShoot);
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

    public Command shooterCommand() {
        Command shooterCommand = Commands.sequence(
                Commands.parallel(rev(), intake.intakeInWithRevCommand()),
                Commands.parallel(shoot(), intake.intakeOutWithShoot()));
        shooterCommand.setName("Rev,Intake,Shoot");
        return shooterCommand;
    }

    public Command shooterOnFlop() {
        var command = Commands.run(() -> shooter.shooterOnFlop()).withTimeout(0.1);
        command.setName("sPiNiNg");
        return command;
    }

    public Command handOffCommand() {

        var command = Commands.sequence(
                intake.intakeInWithRevCommand(),
                Commands.parallel(moveFeedSlowCommand(), intake.intakeOutWithFeedCommand(), shooterOnFlop()),
                intake.intakeOutOfTheWayCommand().until(() -> iintake.isAngleGreat()));
        command.setName("Hand Off");
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
            double distance = drivebase.getDistanceDAS();
            DAS.MotorSettings as = RobotContainer.das.calculateAS(distance);
            shooter.setSpeed(as.getVoltage());
            // ShooterAngle.moveShooterAngle
            // code to run while running
        }).withTimeout(0.375);
        // return new FunctionalCommand(
        // () -> {

        // // code to run on init
        // },
        // () -> {
        // double distance = drivebase.getDistanceDAS();
        // DAS.MotorSettings as = RobotContainer.das.calculateAS(distance);
        // shooter.setSpeed(as.getVoltage());
        // // ShooterAngle.moveShooterAngle
        // // code to run while running
        // },
        // (interrupted) -> {
        // // code to run when ending
        // },
        // () -> {
        // // return true when finished
        // return shooter.isRevatSpeed();
        // //return shooter.isShooterAtGoal
        // }
        // );
    }

    public Command shootFromDAS(){
        return Commands.parallel(
            revSpeedFromDAS(), shooterAngle.setShooterAngleFromDAS()
        );
    }
}
