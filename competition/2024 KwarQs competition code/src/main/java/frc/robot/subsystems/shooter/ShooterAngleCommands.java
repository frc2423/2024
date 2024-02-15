package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ShooterAngleCommands {
    private ShooterAngle shooterAngle;

    public ShooterAngleCommands(ShooterAngle shooterAngle ) {
        this.shooterAngle = shooterAngle;
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

  public Command moveShooterUp(){
    var command = shooterAngle.runOnce(() -> shooterAngle.setAngle(shooterAngle.setpoint.getDegrees() + 5));
    command.setName("Move Shooter Up");
    return command;
  }

  public Command moveShooterDown(){
    var command = shooterAngle.runOnce(() -> shooterAngle.setAngle(shooterAngle.setpoint.getDegrees() - 5));
    command.setName("Move Shooter Down");
    return command;
  }
}
