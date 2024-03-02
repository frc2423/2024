package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.DAS;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ShooterAngleCommands {
  private ShooterAngle shooterAngle;
  private SwerveSubsystem drivebase;
  public ShooterAngleCommands(ShooterAngle shooterAngle, SwerveSubsystem drivebase) {
    this.shooterAngle = shooterAngle;
    this.drivebase = drivebase;
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

  public Command setShooterAngleFromDAS() {
    
        return new FunctionalCommand(
            () -> {

                // code to run on init
            },
            () -> {
                double distance = drivebase.getDistanceDAS();
                DAS.MotorSettings as = RobotContainer.das.calculateAS(distance);
                shooterAngle.setAngle(as.getAngle());
                // ShooterAngle.moveShooterAngle
                // code to run while running
            },
            (interrupted) -> {
                // code to run when ending
            },
            () -> {
                // return true when finished
                return shooterAngle.isShooterAtGoal();
            }
        );
    }

    

}
