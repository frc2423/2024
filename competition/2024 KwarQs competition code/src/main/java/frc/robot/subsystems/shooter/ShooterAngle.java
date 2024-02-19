package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.devices.NeoMotor;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
// import com.ctre.phoenix6.sensors.CANCoder;
// import com.ctre.phoenix6.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

/** A robot arm subsystem that moves with a motion profile. */
public class ShooterAngle extends SubsystemBase {
  private final CANSparkMax shooter_Pivot = new CANSparkMax(26,CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax shooter_Pivot2 = new CANSparkMax(24,CANSparkLowLevel.MotorType.kBrushless);
  public static final double kSVolts = 1;
  public static final double kGVolts = 1;
  public static final double kVVoltSecondPerRad = 0.5;// all things for feed forward is wrong, re do pls
  public static final double kAVoltSecondSquaredPerRad = 0.1;
  public static final double kMaxVelocityRadPerSecond = 3;
  public static final double kMaxAccelerationRadPerSecSquared = 10;
  public static final int kMotorPort = 20;
  public static final double kP = 0.1;
  public static final int kEncoderPPR = 256;
  public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;
  private MechanismLigament2d pivot;
  private double pivotMotorPercent = 0;
  private Rotation2d pivotAngle = new Rotation2d(0);
  private final FlywheelSim pivotSimMotor = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
  ProfiledPIDController shooter_pivot_PID = new ProfiledPIDController((Robot.isSimulation()) ? .001 : .005, 0, 0, new TrapezoidProfile.Constraints(360, 420));
  private double shooterPivotMotorPercent = 0;
  public static Rotation2d setpoint =Rotation2d.fromRadians(0); //Enter Rot2d value
  public static double shoot = 0;
  public static double handOff = 0;
  public static double neutral = 0;
  private Rotation2d shooterPivotAngle = new Rotation2d(0);
  public static double maxShooterPivotAngle = 0;
  public static double minShooterPivotAngle = 0;
  private CANcoder shooterAngle = new CANcoder(0); //figure this out

  

  public static final double kArmOffsetRads = 0.5;
  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      kSVolts, kGVolts,
      kVVoltSecondPerRad, kAVoltSecondSquaredPerRad);

  /** Create a new ArmSubsystem. */
  public ShooterAngle() {
   // shooter_Pivot = new CANSparkMax(kMotorPort, CANSparkLowLevel.MotorType.kBrushless);
    
    shooter_pivot_PID.setTolerance(RobotBase.isSimulation() ? 5 : 5);
    // Start arm at rest in neutral position
    setpoint = Rotation2d.fromRadians(shoot); 

     Mechanism2d mech = new Mechanism2d(3, 3);
        // the mechanism root node
        MechanismRoot2d arm = mech.getRoot("shooter", 1.5, 0.5);
        pivot = arm.append(
            new MechanismLigament2d("pivot", Units.inchesToMeters(21), 0, 10, new Color8Bit(Color.kOrange)));
        SmartDashboard.putData("Mech2d", mech);
        pivotSimMotor.setInput(0);
  }

  public void setAngle(double degrees){
    setpoint = Rotation2d.fromDegrees(degrees); 
  }

  private double calculatePid(Rotation2d angle) {
    updatePivotAngle();
    double pid = shooter_pivot_PID.calculate(pivotAngle.getDegrees(), angle.getDegrees());
    var setpoint = shooter_pivot_PID.getSetpoint();
    double feedforward = m_feedforward.calculate(Math.toRadians(shooterPivotAngle.getDegrees()), setpoint.velocity);
    return feedforward + pid;
  }

  @Override
  public void simulationPeriodic(){
    super.simulationPeriodic();
    pivotSimMotor.setInputVoltage(pivotMotorPercent * RobotController.getBatteryVoltage());
    pivotSimMotor.update(.02);
    var encoderRateSign = 1;
    var pivotRate = pivotSimMotor.getAngularVelocityRadPerSec() * encoderRateSign;
    pivotAngle = pivotAngle.plus(Rotation2d.fromRadians(pivotRate * 0.02));
    pivot.setAngle(pivotAngle.getDegrees() + 90);
  }

  private void updatePivotAngle() {
        if (RobotBase.isSimulation()) {
            var encoderRateSign = 1;
            var pivotRate = pivotSimMotor.getAngularVelocityRadPerSec() * encoderRateSign;
            pivotAngle = pivotAngle.plus(Rotation2d.fromRadians(pivotRate * 0.02));
        } else {
            double encoderPosition = shooter_Pivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition();
            pivotAngle = Rotation2d.fromDegrees(encoderPosition);
        }

    }

  @Override
  public void periodic(){
    shooterPivotMotorPercent = calculatePid(setpoint);

    if (shooterPivotAngle.getDegrees() > maxShooterPivotAngle) {
      shooterPivotMotorPercent = Math.min(shooterPivotMotorPercent, 0);
    } else if (shooterPivotAngle.getDegrees() < minShooterPivotAngle) {
      shooterPivotMotorPercent = Math.max(shooterPivotMotorPercent, 0);
    }

    double maxShooterSpeed = 0;
    shooterPivotMotorPercent = Math.max(Math.min(maxShooterSpeed, shooterPivotMotorPercent), -maxShooterSpeed);

    shooter_Pivot.set(shooterPivotMotorPercent); 
    shooter_Pivot2.set(shooterPivotMotorPercent); 
  }

  // public Command feederAngleCommand(){
  //   return this.runOnce(()->this.setGoal(1));
  //   //put in actual value
  // }

  // public Command climberAngleCommand(){
  //   return this.runOnce(()->this.setGoal(1));
  //   //put in actual value
  // }
  }
