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

public class ShooterSubsystem extends SubsystemBase {

    private NeoMotor shooterMotorOne;
    private NeoMotor shooterMotorTwo;
    private double shooterSpeed = 0;
    public static Timer timer;
    public static double feederVoltage = 0;
    private final CANSparkMax feeder_Motor;
    public static final int kFeederMotorPort = 21;
    
    
    public ShooterSubsystem(){
        feeder_Motor = new CANSparkMax(kFeederMotorPort, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotorOne = new NeoMotor(28); // Correct these when we know the numbers
        shooterMotorTwo = new NeoMotor(35);
        shooterMotorTwo.setInverted(true);
        shooterMotorOne.setFollower(shooterMotorTwo);
    }

    //Shooter turns on/ shoots the note
    public void shooterOn() {
        shooterMotorOne.setSpeed(shooterSpeed);
    }

    //stops the shooter
    public void shooterOff() {
         shooterMotorOne.setSpeed(0);
    }

    //Gets the current speed of the shooter
    public double getSpeed() {
        return shooterMotorOne.getSpeed();
    }

    //The goal speed for the shooter
    public void setSpeed(double speed) {
        shooterSpeed = speed;
    }
    
    public void periodic() {
    }

    public void moveFeederMotor(double voltage){
        feeder_Motor.setVoltage(voltage);
    }
    
    public void runShooter(double seconds){ 
        timer.start();
        shooterOn();
        while (timer.get() < seconds) {
            moveFeederMotor(feederVoltage);
        }
        moveFeederMotor(0);
    }

}