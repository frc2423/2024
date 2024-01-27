package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

/*
 * TODO:
 *  - test belt code
 *  - test intake pid
 *  - get correct can id for pivot
 *  - add sim - use singlejointed arm sim
 *  - add commands in robot container to move the intake pivot
 * 
 */

import frc.robot.devices.NeoMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;


public class IntakeSubsystem extends ProfiledPIDSubsystem {
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;
    public static final double kMaxVelocityRadPerSecond = 3;
    public static final double kMaxAccelerationRadPerSecSquared = 10;
    public static final int kMotorPort = 20;
    public static final double kP = 1;
    public static final int[] kEncoderPorts = new int[] { 4, 5 };
    public static final int kEncoderPPR = 256;
    public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;

    public static final double upPositionRads = 0.4;//0.3
    private final NeoMotor m_Pivot;
    private final ArmFeedforward m_feedforward = new ArmFeedforward(
            kSVolts, kGVolts,
            kVVoltSecondPerRad, kAVoltSecondSquaredPerRad);
    private CANSparkMax beltMotor;
    private double intakedownposition = 0.7;//0.755
    private double intake_Offset = 0.05;
    private boolean isDown = false;
    private String intakeState = "Static"; // static, intaking, outtaking
    // get correct channel for digital input
    private DigitalInput beamBreak = new DigitalInput(0);
    private MechanismLigament2d pivot;
    private final FlywheelSim pivotSimMotor = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
    private Rotation2d pivotAngle = new Rotation2d(0);
    private double pivotMotorPercent = 0;

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

        m_Pivot = new NeoMotor(kMotorPort, true);
        //m_Pivot.setConversionFactor(kEncoderDistancePerPulse);
        // Start arm at rest in neutral position
        setGoal(upPositionRads);
        beltMotor = new CANSparkMax(19, CANSparkLowLevel.MotorType.kBrushless);
        m_Pivot.resetEncoder(0);


        Mechanism2d mech = new Mechanism2d(3, 3);
        // the mechanism root node
        MechanismRoot2d arm = mech.getRoot("arm", 1.5, .5);
        pivot = arm.append(
            new MechanismLigament2d("pivot", Units.inchesToMeters(21), 0, 10, new Color8Bit(Color.kOrange)));
        SmartDashboard.putData("Mech2d", mech);
        pivotSimMotor.setInput(0);


    }

    @Override
    public double getMeasurement() {
        return m_Pivot.getDistance() + upPositionRads;
    }

    public void resetEncoder() {
        // m_Pivot.resetEncoder(0);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        pivotMotorPercent = (output + feedforward) / RobotController.getBatteryVoltage();
        System.out.print(setpoint);

    }

    public void extend() {
        // make it down angle
        isDown = true;
        intakeState = "Intaking";
        this.setGoal(intakedownposition);
        this.enable();
    }

    public void retract() {
        // make it up angle
        isDown = false;
        intakeState = "Static";
        this.setGoal(upPositionRads);
        this.enable();
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
        super.periodic();
        // This method will be called once per scheduler run

        m_Pivot.setPercent(-(pivotMotorPercent));


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
        super.simulationPeriodic();
        // This method will be called once per scheduler run during simulation
        pivotSimMotor.setInputVoltage(pivotMotorPercent * RobotController.getBatteryVoltage());
        // Move simulation forward dt seconds
        pivotSimMotor.update(.02);
        var encoderRateSign = 1;

        // Get state from simulation devices (telescopeDist and shoulderAngle)
        var pivotRate = pivotSimMotor.getAngularVelocityRadPerSec() * encoderRateSign;
        pivotAngle = pivotAngle.plus(Rotation2d.fromRadians(pivotRate * 0.02));

        // mechanism2d :
        pivot.setAngle(pivotAngle.getDegrees() + 90);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // This is used to add things to NetworkTables
        super.initSendable(builder);
        builder.addBooleanProperty("Is down", () -> isDown, null);
        builder.addStringProperty("Intake state", () -> intakeState, null);
        builder.addDoubleProperty("Pivot distance", m_Pivot::getDistance, null);
        builder.addDoubleProperty("Pivot percent", m_Pivot::getPercent, null);
    }
}