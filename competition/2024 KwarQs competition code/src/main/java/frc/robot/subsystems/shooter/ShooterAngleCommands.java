package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.DAS;
import frc.robot.NTHelper;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ShooterAngleCommands {
  private ShooterAngle shooterAngle;
  private SwerveSubsystem drivebase;
  private ShooterSubsystem shooterSubsystem;
  private double smallVelocity = 0.02;

  public ShooterAngleCommands(ShooterAngle shooterAngle, SwerveSubsystem drivebase,
      ShooterSubsystem shooterSubsystem) {
    this.shooterAngle = shooterAngle;
    this.drivebase = drivebase;
    this.shooterSubsystem = shooterSubsystem;
  }

  public Command feederAngleCommand() {
    var command = shooterAngle.runOnce(() -> shooterAngle.setAngle(shooterAngle.feedAngle));
    // put in actual value
    command.setName("Feeder Angle");
    return command;
  }

  public Command climberAngleCommand() {
    var command = shooterAngle.runOnce(() -> shooterAngle.setAngle(shooterAngle.climbAngle));
    // put in actual value
    command.setName("Climber Angle");
    return command;
  }

  public Command shooterAngleCommand() {
    var command = shooterAngle.runOnce(() -> shooterAngle.setAngle(shooterAngle.shootAngle));
    // put in actual value
    command.setName("Shooter Angle");
    return command;
  }

  public Command ampAngleCommand() {
    var command = shooterAngle.runOnce(() -> shooterAngle.setAngle(shooterAngle.ampAngle));
    // put in actual value
    command.setName("Amp Angle");
    return command;
  }

  public Command handOffAngleCommand() {
    var command = shooterAngle.runOnce(() -> shooterAngle.setAngle(shooterAngle.handOffAngle));
    command.addRequirements(shooterAngle);
    // put in actual value
    command.setName("Hand Off Angle");
    return command;
  }

  public Command moveShooterUp() {
    var command = shooterAngle.runOnce(() -> shooterAngle.setAngle(shooterAngle.setpoint.getDegrees() + 5));
    command.setName("Move Shooter Up");
    return command;
  }

  public Command moveShooterDown() {
    var command = shooterAngle.runOnce(() -> shooterAngle.setAngle(shooterAngle.setpoint.getDegrees() - 5));
    command.setName("Move Shooter Down");
    return command;
  }

  public Command moveShooterDownReset() {
    Command command = Commands.sequence(
        moveShooterDown().until(() -> {
          boolean isStopped = false;
          boolean isAtSetpoint = false;
          return isStopped || isAtSetpoint;
        }),
        Commands.run(() -> {
          shooterAngle.rotateDown();
        }).until(() -> {
          return smallVelocity > shooterAngle.getVelocity();
        }));
    command.setName("Move Shooter Down / Reset Boom");
    return command;
  }

  public Command setShooterAngleFromDAS() {
    return Commands.run(() -> {
      double distance = drivebase.getDistanceToSpeaker();
      DAS.MotorSettings as = RobotContainer.das.calculateAS(distance);
      shooterAngle.setAngle(as.getAngle());
    }, shooterAngle);

  }

  public boolean isShooterAngleAtGoal() {
    return shooterAngle.isShooterAtGoal();
  }

}
