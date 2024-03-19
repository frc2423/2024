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

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.NeoMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

public class ShooterSubsystem extends SubsystemBase {

    private NeoMotor shooterMotorOne;
    private NeoMotor shooterMotorTwo;
    private double shooterSpeed = -4.3;
    private double shooterSpeed2 = -4.3;
    public static Timer timer;
    public static double feederVoltage = -RobotController.getBatteryVoltage() / 3;
    public static double feederFlopVoltage = 1;
    public static double feederFlopVoltageBackwards = 4;
    private final CANSparkFlex feeder_Motor;
    public static final int kFeederMotorPort = 23;
    public double feederOnSec = 1.5;
    public double isDoneSec = 0.5; // for revving not for shooting
    public double isDoneShoot = .5; // sec

    public final SimpleMotorFeedforward feedforward1 = new SimpleMotorFeedforward(0, 0, 0);
    public final SimpleMotorFeedforward feedforward2 = new SimpleMotorFeedforward(0, 0, 0);
    PIDController pid1 = new PIDController(0, 0, 0);
    PIDController pid2 = new PIDController(0, 0, 0);



    public ShooterSubsystem() {
        feeder_Motor = new CANSparkFlex(kFeederMotorPort, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotorOne = new NeoMotor(21); // Correct these when we know the numbers
        shooterMotorTwo = new NeoMotor(22);
        shooterMotorTwo.setInverted(true);
        shooterMotorOne.setFollower(shooterMotorTwo);
    }

    private void calcShooterPID() {
        double feedforward1_calc = feedforward1.calculate(shooterSpeed);
        double feedforward2_calc = feedforward2.calculate(shooterSpeed2);
        double shooter_PID1 = pid1.calculate(shooterMotorOne.getSpeed(), shooterSpeed);
        double shooter_PID2 = pid2.calculate(shooterMotorTwo.getSpeed(), shooterSpeed2);

        shooterMotorOne.setPercent((feedforward1_calc + shooter_PID1)/ RobotController.getBatteryVoltage());
        shooterMotorTwo.setPercent((feedforward2_calc + shooter_PID2) / RobotController.getBatteryVoltage());
    }

    // public void shooterOn() {
    //     calcShooterPID();
    //     // shooterMotorOne.setSpeed(shooterSpeed/ RobotController.getBatteryVoltage());
    //     // shooterMotorTwo.setSpeed(shooterSpeed2 / RobotController.getBatteryVoltage());
    // }

    public void shooterOnFlop() {
        shooterSpeed = feederFlopVoltage;
        // shooterMotorOne.setSpeed(feederFlopVoltage / RobotController.getBatteryVoltage());
        // shooterMotorTwo.setSpeed(feederFlopVoltage / RobotController.getBatteryVoltage());
    }

    // stops the shooter
    public void shooterOff() {
        shooterSpeed = 0;
        shooterSpeed2 = 0;
    }

    public void everythingOffPlease() {
        shooterOff();
        stopFeederMotor();
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
        calcShooterPID();
    }

    public void moveFeederMotor() {
        feeder_Motor.setVoltage(feederVoltage);
    }

    public void moveFeederSlow() {
        feeder_Motor.setVoltage(-feederFlopVoltage);
    }

    public void moveFeederMotorBackwards() {
        feeder_Motor.setVoltage(feederFlopVoltage);
    }

    public void stopFeederMotor() {
        feeder_Motor.setVoltage(0);
    }
    


}