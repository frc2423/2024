package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;

/*
 * TODO:
 *  - test belt code
 *  - get correct can ids
 * make the initial speed very very slow
 * 
 */

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.NeoMotor;

public class IntakeSubsystem extends SubsystemBase {
    private NeoMotor pivotMotor;
    private CANSparkMax beltMotor;
    private double intakeupposition = 1;
    private double intakedownposition = 2;
    private boolean isDown = false;
    private String intakeState = "Static"; // static, intaking, outtaking
    // get correct channel for digital input
    private DigitalInput beamBreak = new DigitalInput(0);

    public IntakeSubsystem() {
        pivotMotor = new NeoMotor(20);
        beltMotor = new CANSparkMax(19, CANSparkLowLevel.MotorType.kBrushless);
        pivotMotor.setPid(0.1, 00, 00);
    }

    public void extend() {
        // make it down angle
        pivotMotor.setDistance(intakedownposition);
        isDown = true;
        intakeState = "Intaking";
    }

    public void retract() {
        // make it up angle
        pivotMotor.setDistance(intakeupposition);
        isDown = false;
        intakeState = "Static";
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
        // spit the note out
        beltMotor.set(-0.4);
    }

    public void beltStop() {
        // stop the belt
        beltMotor.set(0);
    }

    public void pivotStop() {
        // stop moving up and down
        pivotMotor.setSpeed(0);
    }

    public double getPivotMotorDistance() {
        return pivotMotor.getDistance();
    }

    public double getBeltMotorSpeed() {
        return beltMotor.get();
    }

    // checks if the beam is broken
    public boolean isBeamBroken() {
        return beamBreak.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // This is used to add things to NetworkTables
        super.initSendable(builder);
        builder.addBooleanProperty("Is down", () -> isDown, null);
        builder.addStringProperty("Intake state", () -> intakeState, null);
        builder.addDoubleProperty("Pivot distance", pivotMotor::getDistance, null);
    }
}