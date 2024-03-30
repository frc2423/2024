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
    public double isDoneSec = 4; // for revving not for shooting
    public double isDoneShoot = .5; // sec

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.24, 0.00213, 0);
    private PIDController pid1 = new PIDController(.0024, 0, 0.0012);
    private PIDController pid2 = new PIDController(.0024, 0, 0.0012);


    private boolean isPidMode = false;
    private boolean shooterOn = false;
    private double shooter1Speed = -4.3;
    private double shooter2Speed = -4.3;

    public ShooterSubsystem() {
        feeder_Motor = new CANSparkFlex(kFeederMotorPort, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotorOne = new NeoMotor(21); // Correct these when we know the numbers
        shooterMotorTwo = new NeoMotor(22);
        saveMotorSettings();
        // shooterMotorTwo.setInverted(true);
        // shooterMotorOne.setFollower(shooterMotorTwo);
        pid1.setTolerance(150, 100);
        pid2.setTolerance(150, 100);
    }



    private void saveMotorSettings() {
        try {
          Thread.sleep(200);
        } catch (Exception e) {
        }
    
        shooterMotorTwo.getSparkMax().burnFlash();
      }

    private void setMotorOnePercent(double percent) {
        shooterMotorOne.setPercent(percent);
    }

    private void setMotorTwoPercent(double percent) {
        shooterMotorTwo.setPercent(-percent);
    }

    private double getMotorOneSpeed() {
        return shooterMotorOne.getSpeed();
    }
    private double getMotorTwoSpeed() {
        return -shooterMotorTwo.getSpeed();
    }

    public double calcShooterFeedFor(double shooterSpeed) {

        double feedforward_calc = feedforward.calculate(shooterSpeed);
        return feedforward_calc;
    }

    public double calcShooterPID2(double shooterSpeed) {
        double shooter_PID2 = pid2.calculate(getMotorTwoSpeed(), shooterSpeed);
        return shooter_PID2;
    }

    public double calcShooterPID1(double shooterSpeed) {
        double shooter_PID1 = pid1.calculate(getMotorOneSpeed(), shooterSpeed);
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
        // double speed = -3000;
        NTHelper.setDouble("/debug/shooterFeedforward", calcShooterFeedFor(shooter1Speed));
        NTHelper.setDouble("/debug/error1", pid1.getPositionError());
        NTHelper.setDouble("/debug/errr2", pid2.getPositionError());
        NTHelper.setDouble("/debug/shooterSpeed", getMotorOneSpeed());
        NTHelper.setDouble("/debug/shooterSpeed2", getMotorTwoSpeed());
        NTHelper.setDouble("/debug/shooterSetpoint", shooter1Speed);
        NTHelper.setDouble("/debug/shooterSetpoint2", shooter2Speed);

        if (!shooterOn) {
            setMotorOnePercent(0);
            setMotorTwoPercent(0);
        } else if (isPidMode) {

            setMotorOnePercent((calcShooterFeedFor(shooter1Speed) + calcShooterPID1(shooter1Speed)) / RobotController.getBatteryVoltage());
            setMotorTwoPercent((calcShooterFeedFor(shooter2Speed) + calcShooterPID2(shooter2Speed)) / RobotController.getBatteryVoltage());
            

        } else {
            setMotorOnePercent(shooter1Speed / RobotController.getBatteryVoltage());
            setMotorTwoPercent(shooter2Speed / RobotController.getBatteryVoltage());
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
        if(pid1.atSetpoint() && pid2.atSetpoint())
            return true;
        else
        return false;
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
        builder.addBooleanProperty("PIDMode", () -> isPidMode, null);
        builder.addBooleanProperty("ShooterOn", () -> shooterOn, null);

        // builder.addDoubleProperty("shooterMotor1Value", shooterMotorOne::getValue,
        // null);
        // builder.addDoubleProperty("shooterMotor2Value", shooterMotorTwo::getValue,
        // null);
    }

}