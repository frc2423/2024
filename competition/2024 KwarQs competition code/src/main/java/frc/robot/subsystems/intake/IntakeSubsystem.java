package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.AbsoluteEncoder;
import frc.robot.devices.NeoMotor;
public class IntakeSubsystem extends SubsystemBase
{
    private NeoMotor pivotMotor;
    private NeoMotor beltMotor;
    private ProfiledPIDController pivotPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(360, 420));
    private double intakeupposition = 1;
    private double intakedownposition = 2;
    private CANcoder pivotEncoder= new CANcoder(20);

    public IntakeSubsystem()
    {
        pivotMotor = new NeoMotor(20);
        beltMotor = new NeoMotor(21);
    }
    public void extend()
    {
        //run pivot motor forward
        double speed = pivotPID.calculate(pivotMotor.getDistance(),intakedownposition);
        //make it actual angle
        pivotMotor.setSpeed(speed);

        //run belt motor forward
        intake();
    }
    public void retract()
    {
        pivotMotor.setSpeed(-0.4);
        outtake();
    }
    public void intake()
    {
        beltMotor.setSpeed(0.4);
    }
    public void outtake()
    {
        beltMotor.setSpeed(-0.4);
    }
    public void beltStop()
    {
        beltMotor.setSpeed(0);
    }
    public void pivotStop()
    {
        pivotMotor.setSpeed(0);
    }
    public double getPivotMotorSpeed()
    {
        return pivotMotor.getSpeed();
    }
    public double getBeltMotorSpeed()
    {
        return beltMotor.getSpeed();
    }
   // public boolean isBeamBroken()
    //{

   // }
}