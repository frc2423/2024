package frc.robot.subsystems.shooter;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
//import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.devices.NeoMotor;

/** A robot arm subsystem that moves with a motion profile. */
public class ShooterAngle extends ProfiledPIDSubsystem {
  public static final double kSVolts = 1;
  public static final double kGVolts = 1;
  public static final double kVVoltSecondPerRad = 0.5;
  public static final double kAVoltSecondSquaredPerRad = 0.1;
  public static final double kMaxVelocityRadPerSecond = 3;
  public static final double kMaxAccelerationRadPerSecSquared = 10;
  public static final int kMotorPort = 20;
  public static final double kP = 0.1;
  public static final int kEncoderPPR = 256;
  public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;

  public static final double kArmOffsetRads = 0.5;
  private final NeoMotor shooter_angle_motor;
  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      kSVolts, kGVolts,
      kVVoltSecondPerRad, kAVoltSecondSquaredPerRad);

  /** Create a new ArmSubsystem. */
  public ShooterAngle() {
    super(
        new ProfiledPIDController(
            kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                kMaxVelocityRadPerSecond,
                kMaxAccelerationRadPerSecSquared)),
        0);
    shooter_angle_motor = new NeoMotor(kMotorPort,true);
    shooter_angle_motor.setConversionFactor(kEncoderDistancePerPulse);
    // Start arm at rest in neutral position
    setGoal(kArmOffsetRads);
  }
  public void setAngle(double degrees){
    var rotation = new Rotation2d(degrees);
    setGoal(rotation.getRadians());
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output

    shooter_angle_motor.setPercent((output + feedforward) / RobotController.getBatteryVoltage());

  }

  @Override
  public double getMeasurement() {
    return shooter_angle_motor.getDistance() + kArmOffsetRads;
  }

  public Command feederAngleCommand(){
    return this.runOnce(()->this.setGoal(1));
    //put in actual value
  }

  public Command climberAngleCommand(){
    return this.runOnce(()->this.setGoal(1));
    //put in actual value
  }
}
