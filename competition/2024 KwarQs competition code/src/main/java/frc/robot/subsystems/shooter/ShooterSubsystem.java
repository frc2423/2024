// Function needs:
// Adjust the shooter angle
// Adjust shooter speeds
// Motors should be moving in opposite directions (counterclockwise and clockwise)
// Setup motors
// Link to the intake
// Get shooter motor speeds
// Toggle for on and off
// Two motors controlling the angle

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.devices.NeoMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class ShooterSubsystem extends SubsystemBase {

    private NeoMotor shooterMotorOne;
    private NeoMotor shooterMotorTwo;
    private double shooterSpeed = 0;
    public static Timer timer;
    public static double feederVoltage = 0;
    private final CANSparkMax feeder_Motor;
    public static final int kFeederMotorPort = 21;
    public double feederOnSec = 1.5;
    public double isDoneSec = 4.5;

    public ShooterSubsystem() {
        feeder_Motor = new CANSparkMax(kFeederMotorPort, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotorOne = new NeoMotor(28); // Correct these when we know the numbers
        shooterMotorTwo = new NeoMotor(35);
        shooterMotorTwo.setInverted(true);
        shooterMotorOne.setFollower(shooterMotorTwo);
    }

    // Shooter turns on/ shoots the note
    public void startTimer() {
        timer.start();
    }

    public void shooterOn() {
        shooterMotorOne.setSpeed(shooterSpeed);
    }

    // stops the shooter
    public void shooterOff() {
        shooterMotorOne.setSpeed(0);
    }

    public void stopMotors() {
        shooterOff();
        moveFeederMotor(0);
    }

    // Gets the current speed of the shooter
    public double getSpeed() {
        return shooterMotorOne.getSpeed();
    }

    // The goal speed for the shooter
    public void setSpeed(double speed) {
        shooterSpeed = speed;
    }

    public void periodic() {
    }

    public void moveFeederMotor(double voltage) {
        feeder_Motor.setVoltage(voltage);
    }

    public void runShooter() {
        shooterOn();

        if (timer.get() >= feederOnSec) {
            moveFeederMotor(feederVoltage);
        }
    }

    public Boolean shooted() {
        return timer.get() > isDoneSec;
    }

    // public void example() {

    // }

    // public void functionExamples() {
    // var function1 = () -> System.out.print("hello");

    // var function2 = () -> {
    // System.out.print("hello");
    // System.out.print("hello2");

    // };

    // }

    public Command shoot() {
        return new FunctionalCommand(
                () -> startTimer(),
                () -> runShooter(),
                (interupted) -> stopMotors(),
                () -> shooted(),
                this);
    }

}