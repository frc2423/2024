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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NTHelper;
import frc.robot.devices.NeoMotor;

public class ShooterSubsystem extends SubsystemBase {

    private NeoMotor shooterMotorOne;
    private NeoMotor shooterMotorTwo;
    private double feederVoltage = -RobotController.getBatteryVoltage() / 3;
    private double feederFlopVoltage = 1;
    private double feederFlopVoltageBackwards = 4;
    private final CANSparkFlex feeder_Motor;
    private final int kFeederMotorPort = 23;
    public double feederOnSec = 1.5;
    public double isDoneSec = 0.5; // for revving not for shooting
    public double isDoneShoot = .5; // sec

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.15, 0.000135, 0);
    private PIDController pid1 = new PIDController(.0001, 0, 0.0001);
    private PIDController pid2 = new PIDController(.0001, 0, 0.0001);

    private boolean isPidMode = false;
    private boolean shooterOn = false;
    private double shooter1Speed = -4.3;
    private double shooter2Speed = -4.3;

    public ShooterSubsystem() {
        feeder_Motor = new CANSparkFlex(kFeederMotorPort, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotorOne = new NeoMotor(21); // Correct these when we know the numbers
        shooterMotorTwo = new NeoMotor(22);
        shooterMotorTwo.setInverted(true);
        shooterMotorOne.setFollower(shooterMotorTwo);
        pid1.setTolerance(100, 100);
        pid2.setTolerance(100, 100);
    }

    public double calcShooterFeedFor(double shooterSpeed) {

        double feedforward_calc = feedforward.calculate(shooterSpeed);
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
        shooterOn = true;
    }

    public void shooterOnFlop() {
        setVoltageSpeed(feederFlopVoltage);
        shooterOn();
    }

    // stops the shooter
    public void shooterOff() {
        shooterOn = false;
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
    public void setPidSpeed(double speed) {
        isPidMode = true;
        shooter1Speed = speed;
        shooter2Speed = speed;
    }

    public void setVoltageSpeed(double speed) {
        isPidMode = false;
        shooter1Speed = speed;
        shooter2Speed = speed;
    }

    public void periodic() {
        double speed = 3000;
        NTHelper.setDouble("/debug/shooterFeedforward", calcShooterFeedFor(speed));
        NTHelper.setDouble("/debug/shooterPid", calcShooterPID1(speed));
        NTHelper.setDouble("/debug/shooterSpeed", shooterMotorOne.getSpeed());
        NTHelper.setDouble("/debug/shooterSpeed2", shooterMotorTwo.getSpeed());
        NTHelper.setDouble("/debug/shooterSetpoint", speed);
        NTHelper.setDouble("/debug/shooterAnglePid", calcShooterPID1(speed));

        if (!shooterOn) {
            shooterMotorOne.setSpeed(0);
            shooterMotorTwo.setSpeed(0);
        } else if (isPidMode) {
            // shooterMotorOne.setPercent(calcShooterPID1(shooter1Speed) +
            //         calcShooterFeedFor(shooter1Speed));
            // shooterMotorTwo.setPercent(calcShooterPID2(shooter2Speed) +
            //         calcShooterFeedFor(shooter2Speed));
            shooterMotorOne.setPercent(calcShooterPID1(speed) +
            calcShooterFeedFor(speed));
            shooterMotorTwo.setPercent(/*calcShooterPID2(speed) +*/
            calcShooterFeedFor(-speed));
        } else {
            shooterMotorOne.setSpeed(shooter1Speed / RobotController.getBatteryVoltage());
            shooterMotorTwo.setSpeed(shooter2Speed / RobotController.getBatteryVoltage());
        }
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

    public void moveFeederHandoff() {
        feeder_Motor.setVoltage(0.6);

    }

    public void moveFeederAmpOpp() {
        feeder_Motor.setVoltage(-3);
        setVoltageSpeed(1);
        shooterOn();
    }

    public void moveFeederAmpOppEnd() {
        feeder_Motor.setVoltage(0.6);
        setVoltageSpeed(1);
        shooterOn();
    }

    public void moveFeederMotorBackwards() {
        feeder_Motor.setVoltage(feederFlopVoltageBackwards);
    }

    public double getShooterOneVelocity() {
        return shooterMotorOne.getSpeed();
    }

    public double getShooterTwoVelocity() {
        return shooterMotorTwo.getSpeed();
    }

    public void stopFeederMotor() {
        feeder_Motor.setVoltage(0);
    }

    public boolean isRevatSpeed() {
        return false;
        // return Math.abs(shooterMotorOne.getSpeed() - shooterSpeed) < 0.2;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // This is used to add things to NetworkTables
        super.initSendable(builder);

        builder.addDoubleProperty("shooterSpeed", () -> shooter1Speed, (shooterSpeed) -> {
            this.shooter1Speed = shooterSpeed;
            this.shooter2Speed = shooterSpeed;
        });
        builder.addDoubleProperty("shooterMotor1Velocity", this::getShooterOneVelocity, null);
        builder.addDoubleProperty("shooterMotor2Velocity", this::getShooterTwoVelocity, null);
        builder.addBooleanProperty("PIDMode",() -> isPidMode, null);
        builder.addBooleanProperty("ShooterOn",() -> shooterOn, null);

        // builder.addDoubleProperty("shooterMotor1Value", shooterMotorOne::getValue,
        // null);
        // builder.addDoubleProperty("shooterMotor2Value", shooterMotorTwo::getValue,
        // null);
    }

}