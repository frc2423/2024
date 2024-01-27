package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

/*
 * TODO:
 *  - test belt code
 *  - get correct can ids
 * make the initial speed very very slow
 * 
 */

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.NeoMotor;

public class IntakeSubsystem extends ProfiledPIDSubsystem {
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;
    public static final double kMaxVelocityRadPerSecond = 3;
    public static final double kMaxAccelerationRadPerSecSquared = 10;
    public static final int kMotorPort = 20;
    public static final double kP = 0.1;
    public static final int[] kEncoderPorts = new int[] { 4, 5 };
    public static final int kEncoderPPR = 256;
    public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;

    public static final double upPositionRads = 0.5;
    private final NeoMotor m_Pivot = new NeoMotor(kMotorPort);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(
            kSVolts, kGVolts,
            kVVoltSecondPerRad, kAVoltSecondSquaredPerRad);
    private CANSparkMax beltMotor;
    private double intakedownposition = 2;
    private double intake_Offset = 0.1;
    private boolean isDown = false;
    private String intakeState = "Static"; // static, intaking, outtaking
    // get correct channel for digital input
    private DigitalInput beamBreak = new DigitalInput(0);

    public IntakeSubsystem() {
        super(
                new ProfiledPIDController(
                        kP,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(
                                kMaxVelocityRadPerSecond,
                                kMaxAccelerationRadPerSecSquared)),
                0);
        m_Pivot.setConversionFactor(kEncoderDistancePerPulse);
        // Start arm at rest in neutral position
        setGoal(upPositionRads);
        beltMotor = new CANSparkMax(19, CANSparkLowLevel.MotorType.kBrushless);
    }

    @Override
    public double getMeasurement() {
        return m_Pivot.getDistance() + upPositionRads;
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output

        m_Pivot.setPercent((output + feedforward) / RobotController.getBatteryVoltage());

    }

    public void extend() {
        // make it down angle
        isDown = true;
        intakeState = "Intaking";
        setGoal(intakedownposition);
    }

    public void retract() {
        // make it up angle
        isDown = false;
        intakeState = "Static";
        setGoal(upPositionRads);
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
        double m_Pivot_Pos = getMeasurement();
        if (m_Pivot_Pos > intakedownposition + intake_Offset && isDown) {// down position
            m_Pivot.setPercent(0);
        }
        if (m_Pivot_Pos < upPositionRads - intake_Offset && !isDown) {// down position
            m_Pivot.setPercent(0);
        }
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
        builder.addDoubleProperty("Pivot distance", m_Pivot::getDistance, null);
    }
}