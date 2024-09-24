// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.LED.KwarqsLed;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  private final KwarqsLed led = new KwarqsLed();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData(led.disable());
    SmartDashboard.putData(led.setOrange());
    SmartDashboard.putData(led.setFirst3Green());
    SmartDashboard.putData(led.setAllGreen());
    SmartDashboard.putData(led.setEvenOdd());
    SmartDashboard.putData(led.setBlackToGreenFade());
    SmartDashboard.putData(led.setRainbow());
    SmartDashboard.putData(led.setBouncy());
    SmartDashboard.putData(led.setComet());

  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
