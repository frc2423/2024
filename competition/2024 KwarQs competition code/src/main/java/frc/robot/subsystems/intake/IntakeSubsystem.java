package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

/*
 * TODO:
 *  - test belt code
 *  - get correct can ids
 * 
 */

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.NeoMotor;

public class IntakeSubsystem extends SubsystemBase {
    private NeoMotor pivotMotor;
    private CANSparkMax beltMotor;
    private double intakeupposition = 1;
    private double intakedownposition = 2;
    //get correct channel for digital input
    private DigitalInput beamBreak = new DigitalInput(0);

    public IntakeSubsystem() {
        pivotMotor = new NeoMotor(20);
        beltMotor = new CANSparkMax(19, CANSparkLowLevel.MotorType.kBrushless);
        pivotMotor.setPid(0.1, 00, 00);
    }

    public void extend() {
        // make it down angle
        pivotMotor.setDistance(intakedownposition);
    }

    public void retract() {
        //make it up angle
        pivotMotor.setDistance(intakeupposition);
    }

    public void intake() {
        // slurp the note
        beltMotor.set(0.4);
    }

    public void runIntake() {
        if (isBeamBroken()) {
            beltStop();
        }

        else {
            intake();
        }
    }

    public void outtake() {
        //spit the note out
        beltMotor.set(-0.4);
    }

    public void beltStop() {
        //stop the belt
        beltMotor.set(0);
    }

    public void pivotStop() {
        //stop moving up and down
        pivotMotor.setSpeed(0);
    }

    public double getPivotMotorDistance() {
        return pivotMotor.getDistance();
    }

    public double getBeltMotorSpeed() {
        return beltMotor.get();
    }

    //checks if the beam is broken
    public boolean isBeamBroken() {
        return beamBreak.get();
    }
}