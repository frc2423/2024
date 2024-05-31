package frc.robot.subsystems.climber;

import frc.robot.devices.NeoMotor;

import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

        climberMotorLeft.resetEncoder(0);
        climberMotorRight.resetEncoder(0);
    }

    public void climbStart(){
        climberMotorRight.setPercent(climber1Speed);
        climberMotorLeft.setPercent(climber2Speed);
    }

    public void climbDown(){
        climberMotorRight.setPercent(-climber1Speed);
        climberMotorLeft.setPercent(-climber2Speed);
    }

    public void climbStop() {
        climberMotorRight.setPercent(0);
        climberMotorLeft.setPercent(0);
    }

    private boolean leftClimberIsDown(){
        return climberMotorLeft.getEncoderCount() <= -50.8; //-51.2;
    }
    
    private boolean rightClimberIsDown(){
        return climberMotorRight.getEncoderCount() >= 49.2; //48.9;
    }

    private boolean leftClimberIsUp(){
        return climberMotorLeft.getEncoderCount() >= 72.8;
    }
    
    private boolean rightClimberIsUp(){
        return climberMotorRight.getEncoderCount() <= -71.6;
    }

    public void leftGoingDown(){
        if(!leftClimberIsDown()){
            climbStart();
        } else {
            climbStop();
        }
    }

    public void rightGoingDown(){
        if(!rightClimberIsDown()){
            climbStart();
        } else {
            climbStop();
        }
    }

    public void leftGoingUp(){
        if(!leftClimberIsUp()){
            climbDown();
        } else {
            climbStop();
        }
    }

    public void rightGoingUp(){
        if(!rightClimberIsUp()){
            climbDown();
        } else {
            climbStop();
        }
    }

    public Command climberUpCommand() {
        return Commands.run(() -> {
            leftGoingUp();
            rightGoingUp();
        }, this).until(() -> leftClimberIsUp() && rightClimberIsUp());
    }

    public Command climberDownCommand() {
        return Commands.run(() -> {
            leftGoingDown();
            rightGoingDown();
        }, this).until(() -> leftClimberIsDown() && rightClimberIsDown());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Left Motor Encoder Count",
                () -> climberMotorLeft.getEncoderCount(), null);
        builder.addDoubleProperty("Right Motor Encoder Count",
                () -> climberMotorRight.getEncoderCount(), null);
    }
}
