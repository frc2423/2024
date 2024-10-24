package frc.robot.devices;

import com.revrobotics.CANSparkLowLevel;

/*
 * TODO:
 *  - code to add for free forward
 *  - remove any functions that are unecessary
 * 
 * 
 */




import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Robot;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
//import com.ctre.phoenix.sensors.CANCoder;

public class NeoMotor {

    public static final String ControlType = null;
    protected CANSparkMax motor;
    private IEncoder encoder;
    private PIDController pidController;
    private double voltage = 0.0;
    private double motorValue = 0.0;
    private CANSparkMax.ControlType motorControlType = CANSparkMax.ControlType.kDutyCycle;
    private NeoMotor followerMotor;
    private boolean enabled = true;

    // These are needed because getting these values from the PIDController
    // takes an excessively long time for some reason
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;

    public NeoMotor(int port, boolean isAbsoluteEncoder) {

        motor = new CANSparkMax(port, CANSparkLowLevel.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getVelocityConversionFactor();
        encoder = isAbsoluteEncoder ? new frc.robot.devices.AbsoluteEncoder(motor) : new frc.robot.devices.RelativeEncoder(motor);
        pidController = new PIDController((Robot.isSimulation()) ? .001 : .1, 0, 0);
        setPercent(0);
    }

    public NeoMotor(int port) {
        this(port, false);
    }

    public void setIZone(double zone) {
        pidController.setIZone(zone);
    }

    public CANSparkMax getSparkMax() {
        return motor;
    }

    public void setSpeed(double speed) {
        updateMotor(speed / encoder.getVelocityConversionFactor(), CANSparkMax.ControlType.kVelocity);
    }

    public double getSpeed() {
        double rate = encoder.getVelocity();
        return motor.getInverted() ? -rate : rate;

    }

    public void setPercent(double percent) {
        voltage = percent;
        updateMotor(percent, CANSparkMax.ControlType.kDutyCycle);
        if (followerMotor != null) {
            followerMotor.setPercent(percent);
        }
    }

    public double getPercent() {
        return this.voltage;
    }

    public void setDistance(double dist) {
        updateMotor(dist, CANSparkMax.ControlType.kPosition);
    }

    public void resetEncoder(double distance) {
        encoder.setPosition(distance);
    }

    public double getDistance() {
        return encoder.getPosition();
    }

    public void setConversionFactor(double factor) {
        encoder.setPositionConversionFactor(factor);
        encoder.setVelocityConversionFactor(factor / 60);
    }

    public double getConversionFactor() {
        return encoder.getPositionConversionFactor();
    }

    public void setInverted(boolean isInverted) {
        motor.setInverted(isInverted);
    }

    public boolean getInverted() {
        return motor.getInverted();
    }

    public void setPid(double kP, double kI, double kD) {
        setP(kP);
        setI(kI);
        setD(kD);
    }

    public void setPidf(double kP, double kI, double kD, double kF) {
        setP(kP);
        setI(kI);
        setD(kD);
        //setF(kF);
    }

    public void setP(double kP) {
        this.kP = kP;
        pidController.setP(kP);
    }

    public void setI(double kI) {
        this.kI = kI;
        pidController.setI(kI);
    }

    public void setD(double kD) {
        this.kD = kD;
        pidController.setD(kD);
    }

    // public void setF(double kF) {
    //     this.kF = kF;
    //     //pidController.setFF(kF);\
    //     pidController.set
    // }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getF() {
        return kF;
    }

    public void setFollower(NeoMotor follower) {
        this.followerMotor = follower;
    }

    public void setEncoderPositionAndRate(double position, double rate) {
    }

    public double getEncoderCount() {
        return getDistance() / getConversionFactor();
    }

    private void updateMotor(double newValue, CANSparkMax.ControlType newControlType) {
        if (enabled == false){
            return;
        }
        boolean hasChanged = newValue != motorValue || newControlType != motorControlType;
        if (hasChanged) {
            motorValue = newValue;
            motorControlType = newControlType;
            // pidController.setReference(motorValue, motorControlType)
            // pidController.
            // pidController.setSetpoint(motorValue, newControlType);
            motor.set(newValue);
        }
    }

    public double getValue() {
        return motorValue;
    }

    public void setBrakeMode(boolean brake) {
        if (brake) {
            motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        } else {
            motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }
    }

    public double getCurrent(){
        return motor.getOutputCurrent();
    }

    public boolean getEnabled(){
        return enabled;
    }

    public void setEnabled(boolean isEnabled){
        if(!isEnabled){
            setPercent(0);
        }
        enabled = isEnabled;
    }
    
}