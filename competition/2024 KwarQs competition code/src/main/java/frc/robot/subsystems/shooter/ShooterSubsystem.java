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
    private final CANSparkFlex feeder_Motor;
    public static final int kFeederMotorPort = 23;
    public double feederOnSec = 1.5;
    public double isDoneSec = 0.5; // for revving not for shooting
    public double isDoneShoot = .5; // sec

    public final SimpleMotorFeedforward feedforward1 = new SimpleMotorFeedforward(0.15, 0.00016, 0);
    public final SimpleMotorFeedforward feedforward2 = new SimpleMotorFeedforward(0.15, 0.00016, 0);
    PIDController pid1 = new PIDController(.0005, 0, 0);
    PIDController pid2 = new PIDController(.0005, 0, 0);


    public ShooterSubsystem() {
        feeder_Motor = new CANSparkFlex(kFeederMotorPort, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotorOne = new NeoMotor(21); // Correct these when we know the numbers
        shooterMotorTwo = new NeoMotor(22);
        shooterMotorTwo.setInverted(true);
        shooterMotorOne.setFollower(shooterMotorTwo);

    }

    // Shooter turns on/ shoots the note
    public void startTimer() {
        timer.start();
    }

    public double calcShooterFeedFor(double shooterSpeed) {

        double feedforward_calc = feedforward1.calculate(shooterSpeed);
        return feedforward_calc;
    }

    public double calcShooterPID2(double shooterSpeed) {
        double shooter_PID2 = pid2.calculate(shooterMotorTwo.getSpeed(), shooterSpeed);
        return shooter_PID2;
    }

    public double calcShooterPID1(double shooterSpeed) {
        double shooter_PID1 = pid1.calculate(shooterMotorOne.getSpeed(), shooterSpeed);
        return shooter_PID1;
    }

    public void shooterOn() {
        double speed = 3000;
        NTHelper.setDouble("/debug/shooterFeedforward", calcShooterFeedFor(speed));
        NTHelper.setDouble("/debug/shooterPid", calcShooterPID1(speed));
        NTHelper.setDouble("/debug/shooterSpeed", shooterMotorOne.getSpeed());
        NTHelper.setDouble("/debug/shooterSetpoint", speed);
        NTHelper.setDouble("/debug/shooterAnglePid", calcShooterPID1(speed));

        // System.out.println(calcShooterPID1(speed));
        shooterMotorOne.setPercent(calcShooterPID1(speed) + calcShooterFeedFor(speed));
        shooterMotorTwo.setPercent(calcShooterPID2(speed) + calcShooterFeedFor(speed));
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

    public void moveFeederAmp() {
        feeder_Motor.setVoltage(5);
    }

    public void moveFeederAmpOpp() {
        feeder_Motor.setVoltage(-3);
        shooterMotorOne.setSpeed(1 / RobotController.getBatteryVoltage());
        shooterMotorTwo.setSpeed(1 / RobotController.getBatteryVoltage());
        // shooterOnSource();
        // feeder_Motor.setVoltage(-0.5);
    }
   
    public void moveFeederAmpOppEnd() {
        feeder_Motor.setVoltage(0.6);
        shooterMotorOne.setSpeed(1 / RobotController.getBatteryVoltage());
        shooterMotorTwo.setSpeed(1 / RobotController.getBatteryVoltage());
    }

    public void moveFeederMotorBackwards() {
        feeder_Motor.setVoltage(feederFlopVoltageBackwards);
    }

    public double getShooterOneVelocity() {
        double speed1 = shooterMotorOne.getSpeed();
        return speed1;
    }

    public double getShooterTwoVelocity() {
        double speed2 = shooterMotorTwo.getSpeed();
        return speed2;
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
        builder.addDoubleProperty("shooterMotor1Velocity", this::getShooterOneVelocity, null);
        builder.addDoubleProperty("shooterMotor2Velocity", this::getShooterTwoVelocity, null);

        // builder.addDoubleProperty("shooterMotor1Value", shooterMotorOne::getValue, null);
        // builder.addDoubleProperty("shooterMotor2Value", shooterMotorTwo::getValue, null);
    }

}