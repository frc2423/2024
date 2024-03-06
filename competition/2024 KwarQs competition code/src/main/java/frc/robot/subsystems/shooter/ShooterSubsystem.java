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

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NTHelper;
import frc.robot.devices.NeoMotor;

public class ShooterSubsystem extends SubsystemBase {

    private NeoMotor shooterMotorOne;
    private NeoMotor shooterMotorTwo;
    private double shooterSpeed = -4.3;
    public static Timer timer;
    public static double feederVoltage = -RobotController.getBatteryVoltage() / 3;
    public static double feederFlopVoltage = 1;
    public static double feederFlopVoltageBackwards = 4;
    private final CANSparkMax feeder_Motor;
    public static final int kFeederMotorPort = 23;
    public double feederOnSec = 1.5;
    public double isDoneSec = 0.5; // for revving not for shooting
    public double isDoneShoot = 2; // sec

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
        shooterMotorOne.setSpeed(shooterSpeed / RobotController.getBatteryVoltage());
        shooterMotorTwo.setSpeed(-10 / RobotController.getBatteryVoltage());
    }

    public void shooterOnFlop() {
        shooterMotorOne.setSpeed(feederFlopVoltage / RobotController.getBatteryVoltage());
        shooterMotorTwo.setSpeed(feederFlopVoltage / RobotController.getBatteryVoltage());
    }

    // stops the shooter
    public void shooterOff() {
        shooterMotorOne.setSpeed(0);
        shooterMotorTwo.setSpeed(0);
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
    }

    public void moveFeederMotor() {
        feeder_Motor.setVoltage(feederVoltage);
    }

    public void moveFeederSlow() {
        feeder_Motor.setVoltage(-feederFlopVoltage);
    }

    public void moveFeederslowReverse() {
        feeder_Motor.setVoltage(0.5);
    }

    public void moveFeederMotorBackwards() {
        feeder_Motor.setVoltage(feederFlopVoltageBackwards);
    }

    public void stopFeederMotor() {
        feeder_Motor.setVoltage(0);
    }

    public boolean isRevatSpeed() {
        return Math.abs(shooterMotorOne.getSpeed() - shooterSpeed) < 0.2;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // This is used to add things to NetworkTables
        super.initSendable(builder);

        builder.addDoubleProperty("shooterSpeed", () -> shooterSpeed, (shooterSpeed) -> {
            this.shooterSpeed = shooterSpeed;
        });
    }

}