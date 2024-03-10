package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
    public static final double kSVolts = Robot.isSimulation() ? 0 : 0.015;
    public static final double kGVolts = Robot.isSimulation() ? 0 : 0.02;
    public static final double kVVoltSecondPerRad = 0.00;// 0.017;
    public static final double kAVoltSecondSquaredPerRad = 0.05;
    public static final int kMotorPort = 20;

    public static final double upPositionDegrees = 290;// .4//0.3 //290
    public static Rotation2d setpoint = Rotation2d.fromDegrees(upPositionDegrees);
    private final CANSparkMax m_Pivot;
    ProfiledPIDController pivot_PID = new ProfiledPIDController((Robot.isSimulation()) ? .001 : 0.005, 0, .001,
            new TrapezoidProfile.Constraints(350, 275));// noice
    private final ArmFeedforward m_feedforward = new ArmFeedforward(
           0.004, 0.02, 0.000, 0);
    private CANSparkMax beltMotor;
    private double downPositionDegrees = 90;// .7, 0.755
    private boolean isDown = false;
    private String intakeState = "Static"; // static, intaking, outtaking
    // get correct channel for digital input
    private DigitalInput beamBreak = new DigitalInput(8);
    private MechanismLigament2d pivot;
    private final FlywheelSim pivotSimMotor = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
    private Rotation2d pivotAngle = new Rotation2d(0);
    private double pivotMotorPercent = 0;
    private double maxPivotAngle = 290; // degrees
    private double minPivotAngle = 90; // still 
    private double intakeSpeed = .25;
    public double isDoneSec = .5; // for revving not for shooting
    public double isDoneShoot = 2; //sec

    private MedianFilter beambreakFilter = new MedianFilter(5);
    private double beamBreakAverage = 0;

    public IntakeSubsystem() {
        pivot_PID.setTolerance(RobotBase.isSimulation() ? 5 : 5);

        m_Pivot = new CANSparkMax(kMotorPort, CANSparkLowLevel.MotorType.kBrushless);
        m_Pivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setZeroOffset(58);
        m_Pivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setPositionConversionFactor(360);

        // Start arm at rest in neutral position
        beltMotor = new CANSparkMax(19, CANSparkLowLevel.MotorType.kBrushless);

        Mechanism2d mech = new Mechanism2d(3, 3);
        // the mechanism root node
        MechanismRoot2d arm = mech.getRoot("arm", 1.5, .5);
        pivot = arm.append(
                new MechanismLigament2d("pivot", Units.inchesToMeters(21), 0, 10, new Color8Bit(Color.kOrange)));
        SmartDashboard.putData("Mech2dIntake", mech);
        setSetpointToCurrentAngle();
        pivotSimMotor.setInput(0);

    }

    private double calculatePid(Rotation2d angle) {
        updatePivotAngle();
        double pid = pivot_PID.calculate(pivotAngle.getDegrees(), angle.getDegrees());
        var setpoint = pivot_PID.getSetpoint();
        double feedforward = m_feedforward.calculate(Math.toRadians(pivotAngle.getDegrees() - 100), setpoint.velocity);
        return feedforward + pid;
    }

    private void setSetpointToCurrentAngle() {
        updatePivotAngle();
        setpoint = pivotAngle;
    }

    private void updatePivotAngle() {
        if (RobotBase.isSimulation()) {
            var encoderRateSign = 1;
            var pivotRate = pivotSimMotor.getAngularVelocityRadPerSec() * encoderRateSign;
            pivotAngle = pivotAngle.plus(Rotation2d.fromRadians(pivotRate * 0.02));
            if (pivotAngle.getDegrees() < 0){
                pivotAngle = Rotation2d.fromDegrees(360 + pivotAngle.getDegrees());
            }
        } else {
            double encoderPosition = m_Pivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition();
            pivotAngle = Rotation2d.fromDegrees(encoderPosition);
        }

    }

    public void extend() {
        // make it down angle
        isDown = true;
        intakeState = "Intaking";
        setpoint = Rotation2d.fromDegrees(downPositionDegrees);
    }

    public void retract() {
        // make it up angle
        isDown = false;
        intakeState = "Static";
        setpoint = Rotation2d.fromDegrees(upPositionDegrees);
    }

     public void outOfTheWay() {
        // make it up angle
        isDown = false;
        intakeState = "Static";
        setpoint = Rotation2d.fromDegrees(170);
    }

    public void intake() {
        // slurp the note
        intakeSpeed = -.3;
    }

    public void outtake() {
        // spit the note out
        intakeSpeed = .3;
    }

    public void beltStop() {
        // stop the belt
        intakeSpeed = 0;
    }

    public void pivotStop() {
        // stop moving up and down
    }

    public double getBeltMotorSpeed() {
        return beltMotor.get();
    }

    // checks if the beam is broken
    public boolean isBeamBroken() {
        return beamBreakAverage > .5;
    }
    
    public boolean getRawBeamBroken() {
        return !beamBreak.get();
    }

    public boolean isIntakeDown(){
        return getPivotAngle().getDegrees() < 180;
    }

    @Override
    public void periodic() {
        beamBreakAverage = beambreakFilter.calculate(getRawBeamBroken() ? 1 : 0);
        // This method will be called once per scheduler run moo
        pivotMotorPercent = calculatePid(setpoint);

        if (isBeamBroken() && setpoint.getDegrees() == downPositionDegrees) {
            beltMotor.set(Math.max(0, intakeSpeed));
        } else {
            beltMotor.set(intakeSpeed);
        }

        if (pivotAngle.getDegrees() > maxPivotAngle) {
            pivotMotorPercent = Math.min(pivotMotorPercent, 0);
        } else if (pivotAngle.getDegrees() < minPivotAngle) {
            pivotMotorPercent = Math.max(pivotMotorPercent, 0);
        }

        double maxSpeed = .5;
        pivotMotorPercent = Math.max(Math.min(maxSpeed, pivotMotorPercent), -maxSpeed);

        m_Pivot.set(pivotMotorPercent);

        // double m_Pivot_Pos = getMeasurement();
        // if (m_Pivot_Pos > intakedownposition + intake_Offset && isDown) {// down
        // position
        // m_Pivot.setPercent(0);
        // }
        // if (m_Pivot_Pos < upPositionRads - intake_Offset && !isDown) {// down
        // position
        // m_Pivot.setPercent(0);
        // }
    }

    @Override
    public void simulationPeriodic() {
        if (RobotState.isDisabled()) {
            return;
        }
        super.simulationPeriodic();
        // This method will be called once per scheduler run during simulation
        pivotSimMotor.setInputVoltage(pivotMotorPercent * RobotController.getBatteryVoltage());
        // Move simulation forward dt seconds
        pivotSimMotor.update(.02);
        // var encoderRateSign = 1;
        

        // Get state from simulation devices (telescopeDist and shoulderAngle)
        // var pivotRate = pivotSimMotor.getAngularVelocityRadPerSec() *
        // encoderRateSign;
        // // pivotAngle = pivotAngle.plus(Rotation2d.fromRadians(pivotRate * 0.02));

        // mechanism2d :
        pivot.setAngle(-pivotAngle.getDegrees() - 90);
    }

    public Rotation2d getPivotAngle() {

        return pivotAngle;
    }
    public boolean isAngleGreat() {
        if (pivotAngle.getDegrees() < 180) {
        return true;
    }
        else return false;
        
    }
    


    @Override
    public void initSendable(SendableBuilder builder) {
        // This is used to add things to NetworkTables
        super.initSendable(builder);
        builder.addBooleanProperty("Is down", () -> isDown, null);
        builder.addStringProperty("Intake state", () -> intakeState, null);
        builder.addDoubleProperty("Pivot angle", () -> pivotAngle.getDegrees(), null);
        builder.addDoubleProperty("Pivot raw encoder value",
                () -> m_Pivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition(), null);

        // builder.addDoubleProperty("Pivot percent", m_Pivot::gett, null);
        builder.addDoubleProperty("Setpoint velocity", () -> pivot_PID.getSetpoint().velocity, null);
        builder.addDoubleProperty("Setpoint position", () -> pivot_PID.getSetpoint().position, null);
        builder.addDoubleProperty("Setpoint", () -> setpoint.getDegrees(), (angle) -> {
            setpoint = Rotation2d.fromDegrees(angle);
        });
        builder.addDoubleProperty("Pivot Motor Percent", () -> pivotMotorPercent, null);
        builder.addDoubleProperty("Belt Motor Speed", () -> beltMotor.get(), null);
        builder.addBooleanProperty("Beam broken?", () -> isBeamBroken(), null);
        builder.addDoubleProperty("Beam break average", () -> beamBreakAverage, null);
    }
}
