package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.NeoMotor;
public class IntakeSubsystem extends SubsystemBase
{
    private NeoMotor pivotMotor;
    private NeoMotor beltMotor;

    public IntakeSubsystem()
    {
        pivotMotor = new NeoMotor(20);
        beltMotor = new NeoMotor(21);
    }
    public void extend()
    {
        //run pivot motor forward
        pivotMotor.setSpeed(0.4);
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