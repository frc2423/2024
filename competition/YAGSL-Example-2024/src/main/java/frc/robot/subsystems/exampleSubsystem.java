package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class exampleSubsystem  extends SubsystemBase{
      public Command exampleCommand() {
    // implicitly require `this`
    return this.runOnce(() -> System.out.println("Amory Likes Ducks quakquakquakquakquak"));
  }
}
