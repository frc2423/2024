package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterFeedSubsystem extends SubsystemBase {

    private double feederVoltage = -RobotController.getBatteryVoltage() / 3;
    private double feederFlopVoltage = 1;
    private double feederFlopVoltageBackwards = 4;
    private final CANSparkFlex feeder_Motor;
    private final int kFeederMotorPort = 23;
    public double feederOnSec = 1.5;

    public ShooterFeedSubsystem() {
        feeder_Motor = new CANSparkFlex(kFeederMotorPort, CANSparkLowLevel.MotorType.kBrushless);

    }

    public void feedOff() {
        stopFeederMotor();
    }

    public void moveFeederMotor() {
        feeder_Motor.setVoltage(feederVoltage);
    }

    public void moveFeederMotorFast() {
        feeder_Motor.setVoltage(-12);
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
        feeder_Motor.setVoltage(8);

    }

    public Command setFeedSpeed(double percent){
        return this.runOnce(() -> {feeder_Motor.set(-percent);});
    }

    public Command setFeedVoltage(double voltage){
        return this.runOnce(() -> {feeder_Motor.setVoltage(-voltage);});
    }

    public void moveFeederAmpOpp() {
        feeder_Motor.setVoltage(-3);
    }

    public void moveFeederAmpOppEnd() {
        feeder_Motor.setVoltage(0.6);
    }

    public void moveFeederMotorBackwards() {
        feeder_Motor.setVoltage(feederFlopVoltageBackwards);
    }

    public void stopFeederMotor() {
        feeder_Motor.setVoltage(0);
    }

}