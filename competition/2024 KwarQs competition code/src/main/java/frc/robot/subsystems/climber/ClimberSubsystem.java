package frc.robot.subsystems.climber;

import frc.robot.devices.NeoMotor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NTHelper;
import frc.robot.devices.NeoMotor;

public class ClimberSubsystem extends SubsystemBase {
    private NeoMotor climberMotorRight;
    private NeoMotor climberMotorLeft;

    public static double CLIMB_SPEED = 4.3;


    private double climber1Speed = 25;
    private double climber2Speed = -25;

    public ClimberSubsystem(){
        climberMotorRight = new NeoMotor(30);
        climberMotorLeft = new NeoMotor(31);

    }

    public void climbStart(){
        climberMotorRight.setPercent(climber1Speed);
        climberMotorLeft.setPercent(climber2Speed);
        System.out.println("CLIMB START!");
    }

    public void climbStop() {
         climberMotorRight.setPercent(0);
        climberMotorLeft.setPercent(0);
    }




}
