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
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.NeoMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class ShooterSubsystem extends SubsystemBase {

    private NeoMotor shooterMotorOne;
    private NeoMotor shooterMotorTwo;
    private double shooterSpeed = -.70;
    private double shooterSpeed2 = -.70;
    public static Timer timer;
    public static double feederVoltage = -3;
    private final CANSparkMax feeder_Motor;
    public static final int kFeederMotorPort = 23;
    public double feederOnSec = 1.5;
    public double isDoneSec = .5; // for revving not for shooting
    public double isDoneShoot = 2; //sec

    public ShooterSubsystem() {
        feeder_Motor = new CANSparkMax(kFeederMotorPort, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotorOne = new NeoMotor(21); // Correct these when we know the numbers
        shooterMotorTwo = new NeoMotor(22);
        shooterMotorTwo.setInverted(true);
        shooterMotorOne.setFollower(shooterMotorTwo);
    }

    // Shooter turns on/ shoots the note
    public void startTimer() {
        timer.start();
    }

    public void shooterOn() {
        shooterMotorOne.setSpeed(shooterSpeed);
        shooterMotorTwo.setSpeed(shooterSpeed2);
    }

    // stops the shooter
    public void shooterOff() {
        shooterMotorOne.setSpeed(0);
        shooterMotorTwo.setSpeed(0);
    }

    public void everythingOffPlease(){
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
    }

    public void moveFeederMotor() {
        feeder_Motor.setVoltage(feederVoltage);
    }

    public void stopFeederMotor() {
        feeder_Motor.setVoltage(0);
    }

    public Command rev() {
        Timer timer = new Timer();
        return new FunctionalCommand(
                () -> {
                timer.start(); 
                timer.restart();
                },
                () -> shooterOn(),
                (interupted) -> {},
                () -> timer.get() > isDoneSec,
                this);
    }

     public Command stopIt() {
        var command = Commands.run(() -> everythingOffPlease(), this);
        command.setName("Stop it");
        return command;
    }

    public Command shoot() {
        Timer timer = new Timer();
        return new FunctionalCommand(
                () -> {
                timer.start(); 
                timer.restart();
                },
                () -> moveFeederMotor(),
                (interupted) -> {},
                () -> timer.get() > isDoneShoot,
                this);
    }

    public Command revAndShoot() {
        return Commands.sequence(
           rev(),
           shoot()
        );
    }

}