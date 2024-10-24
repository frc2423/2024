package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import com.ctre.phoenix6.hardware.CANcoder;

/** A robot arm subsystem that moves with a motion profile. */
public class ShooterAngle extends SubsystemBase {
  private final CANSparkFlex shooter_Pivot = new CANSparkFlex(26, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkFlex shooter_Pivot2 = new CANSparkFlex(24, CANSparkLowLevel.MotorType.kBrushless);
  private static final double kSVolts = 0; // 1;
  private static final double kGVolts = 0; // 1;
  private static final double kVVoltSecondPerRad = 0;// .5;// all things for feed forward is wrong, re do pls
  private static final double kAVoltSecondSquaredPerRad = 0;// .1;
  private static final int kMotorPort = 26; // right side
  private MechanismLigament2d pivot;
  private final FlywheelSim pivotSimMotor = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);

  ProfiledPIDController shooter_pivot_PID = new ProfiledPIDController((Robot.isSimulation()) ? 0.001 : 0.6, 0, 0,
      new TrapezoidProfile.Constraints(450, 600)); // .5, 500, 400
  private double shooterPivotMotorPercent = 0;
  private Rotation2d shooterPivotAngle = new Rotation2d(0);
  private static double maxShooterPivotAngle = 323.5; // 334
  private static double minShooterPivotAngle = 140;
  private CANcoder shooterAngle; // figured out? i think

  public static double feedAngle = 323.5; // is correct number now
  public static double climbAngle = 180; // old
  public static double shuttleAngle = 270;
  public static double climbingAngle = 200; //maybe good (new)
  public static double shootAngle = 323.5; // is good
  public static double ampAngle = 141; // maybe good
  public static double handOffAngle = 303.5; // maybe good
  public double shooterSlowPivotMotorPercent = 0.02;

  public static Rotation2d setpoint = Rotation2d.fromDegrees(feedAngle); // Enter Rot2d value

  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      (Robot.isSimulation()) ? 0 : 0.0175 * 12, (Robot.isSimulation()) ? 0 : 0.01 * 12, 0, 0);
  // private final ArmFeedforward m_feedforward = new ArmFeedforward(
  // (Robot.isSimulation()) ? 0 : 0.02 * 12, (Robot.isSimulation()) ? 0 : 0.01 *
  // 12, 0, 0);

  // 25 encoder 24 26 motors

  /** Create a new ArmSubsystem. */
  public ShooterAngle() {
    shooterAngle = new CANcoder(25);

    shooter_Pivot.setInverted(true);
    shooter_Pivot.setIdleMode(IdleMode.kBrake);
    shooter_Pivot2.setIdleMode(IdleMode.kBrake);

    saveMotorSettings();

    shooter_pivot_PID.setTolerance(RobotBase.isSimulation() ? 2 : 2);

    Mechanism2d mech = new Mechanism2d(3, 3);
    // the mechanism root node
    MechanismRoot2d arm = mech.getRoot("shooter", 1.5, 0.5);
    pivot = arm.append(
        new MechanismLigament2d("pivot", Units.inchesToMeters(21), 0, 10, new Color8Bit(Color.kCyan)));
    SmartDashboard.putData("Mech2dShooterAngle", mech);
    pivotSimMotor.setInput(0);
  }

  public void setAngle(double degrees) {
    setpoint = Rotation2d.fromDegrees(degrees);
  }

  private double calculatePid(Rotation2d angle) {
    updatePivotAngle();
    double pid = shooter_pivot_PID.calculate(shooterPivotAngle.getDegrees(), angle.getDegrees());
    var setpoint = shooter_pivot_PID.getSetpoint();
    double feedforward = m_feedforward.calculate(Math.toRadians(shooterPivotAngle.getDegrees() - 70),
        setpoint.velocity);
    // return feedforward + pid;
    return (feedforward + pid) / RobotController.getBatteryVoltage();
  }

  private void saveMotorSettings() {
    try {
      Thread.sleep(200);
    } catch (Exception e) {
    }

    shooter_Pivot.burnFlash();
  }

  private void updatePivotAngle() {
    if (RobotBase.isSimulation()) {
      var encoderRateSign = 1;
      var pivotRate = pivotSimMotor.getAngularVelocityRadPerSec() * encoderRateSign;
      shooterPivotAngle = shooterPivotAngle.plus(Rotation2d.fromRadians(pivotRate * 0.02));
      if (shooterPivotAngle.getDegrees() < 0) {
        shooterPivotAngle = Rotation2d.fromDegrees(360 + shooterPivotAngle.getDegrees());
      }
    } else {
      double offset = 25.6 - 14.1 + 88.5;
      double encoderPosition = shooterAngle.getAbsolutePosition().getValueAsDouble() * 360 + 178 + offset;
      shooterPivotAngle = Rotation2d.fromDegrees(encoderPosition);
    }

  }

  public double getVelocity() {
    return shooter_Pivot2.getEncoder().getVelocity();
  }

  private void setMotorPercent() {
    if (RobotBase.isSimulation() == false) {
      shooter_Pivot.set(shooterPivotMotorPercent);
      shooter_Pivot2.set(shooterPivotMotorPercent);
      // shooter_Pivot.set(0);
      // shooter_Pivot2.set(0);
    } else {
      pivotSimMotor.setInputVoltage(shooterPivotMotorPercent * RobotController.getBatteryVoltage());
      pivotSimMotor.update(.02);
    }
  }

  public void rotateDown() {
    shooter_Pivot.set(shooterSlowPivotMotorPercent);
    shooter_Pivot2.set(shooterSlowPivotMotorPercent);
    // shooter_Pivot.set(0);
    // shooter_Pivot2.set(0);
  }

  public Rotation2d getShooterAngle() {
    return shooterPivotAngle;
  }

  @Override
  public void periodic() {
    shooterPivotMotorPercent = calculatePid(setpoint);
    pivot.setAngle(-shooterPivotAngle.getDegrees() - 90);

    // if(intake.isIntakeDown() == false){
    // shooterPivotMotorPercent = 0;
    // }

    if (shooterPivotAngle.getDegrees() > setpoint.getDegrees() + 5) {
      shooterPivotMotorPercent = Math.min(shooterPivotMotorPercent, 0);
    } else if (shooterPivotAngle.getDegrees() < setpoint.getDegrees() -5) {
      shooterPivotMotorPercent = Math.max(shooterPivotMotorPercent, 0);
    }

    if (shooterPivotAngle.getDegrees() > maxShooterPivotAngle) {
      shooterPivotMotorPercent = Math.min(shooterPivotMotorPercent, 0);
    } else if (shooterPivotAngle.getDegrees() < minShooterPivotAngle) {
      shooterPivotMotorPercent = Math.max(shooterPivotMotorPercent, 0);
    }

    // double maxShooterSpeed = 0;
    // shooterPivotMotorPercent = Math.max(Math.min(maxShooterSpeed,
    // shooterPivotMotorPercent), -maxShooterSpeed);

    setMotorPercent();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // This is used to add things to NetworkTables
    super.initSendable(builder);

    builder.addDoubleProperty("Setpoint", () -> setpoint.getDegrees(), (angle) -> {
      setpoint = Rotation2d.fromDegrees(angle);
    });
    builder.addDoubleProperty("MotorPercentage", () -> shooterPivotMotorPercent, null);
    builder.addDoubleProperty("Velocity", () -> getVelocity(), null);
    builder.addDoubleProperty("Angle", () -> shooterPivotAngle.getDegrees(), null);
    builder.addDoubleProperty("Error", () -> shooter_pivot_PID.getPositionError(), null);
    builder.addDoubleProperty("MotorVoltage", () -> shooterPivotMotorPercent * 12, null);
    builder.addDoubleProperty("ProfiledSetPoint", () -> shooter_pivot_PID.getSetpoint().position, null);

  }

  public boolean isShooterAtGoal() {
    return (shooter_pivot_PID.atGoal());
  }

}
