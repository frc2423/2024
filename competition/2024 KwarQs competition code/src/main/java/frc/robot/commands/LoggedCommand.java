// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

/** An example command that uses an example subsystem. */
public class LoggedCommand extends WrapperCommand {

  private static DataLog log = DataLogManager.getLog();
  private static StringLogEntry initializedCommandLog = new StringLogEntry(log, "/commands/initialized");
  private static StringLogEntry endedCommandLog = new StringLogEntry(log, "/commands/ended");

  public LoggedCommand(Command command) {
    super(command);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<String> requirements = getRequirements().stream().map((subsystem) -> subsystem.getName())
        .collect(Collectors.toList());
    String requirementsString = "[" + String.join(",", requirements) + "]";
    initializedCommandLog
        .append(getName() + " initialized. Requirements: "
            + requirementsString);
    super.initialize();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    List<String> requirements = getRequirements().stream().map((subsystem) -> subsystem.getName())
        .collect(Collectors.toList());
    String requirementsString = "[" + String.join(",", requirements) + "]";
    endedCommandLog
        .append(getName() + " " + (interrupted ? "interrupted" : "ended") + ". Requirements: "
            + requirementsString);
    super.end(interrupted);
  }
}
