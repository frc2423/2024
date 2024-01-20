// Function needs:
// Adjust the shooter angle
// Adjust shooter speeds
// Motors should be moving in opposite directions (counterclockwise and clockwise)
// Setup motors
// Link to the intake
// Get shooter motor speeds
// Toggle for on and off
// Two motors controlling the angle

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.devices.NeoMotor;

public class Shooter extends SubsystemBase {

    private NeoMotor shooterMotorOne;
    private NeoMotor shooterMotorTwo;
    private NeoMotor shooterAngleMotor;
    private Rotation2d shooterAngle = new Rotation2d();
    private double shooterSpeed = 0;
    private CANcoder angleEncoder;
    private ProfiledPIDController shooterPID = new ProfiledPIDController((Robot.isSimulation()) ? .001 : .005, 0, 0, new TrapezoidProfile.Constraints(360, 420));
    
    
    public Shooter()
    {
        
        shooterMotorOne = new NeoMotor(28); // Correct these when we know the numbers
        shooterMotorTwo = new NeoMotor(35);
        shooterAngleMotor = new NeoMotor(37);
        shooterMotorTwo.setInverted(true);
        shooterMotorOne.setFollower(shooterMotorTwo);
        angleEncoder= new CANcoder(8);
    }
    public void shooterOn() {
        shooterMotorOne.setSpeed(shooterSpeed);
    }
    public void shooterOff() {
         shooterMotorOne.setSpeed(0);
    }

    public double getSpeed() {
        return shooterMotorOne.getSpeed();
    }

    public double getAngle() {
      return shooterAngle.getDegrees();
    }

    public void setSpeed(double speed) {
        shooterSpeed = speed;
    }

     public void setAngle(double angle) {
        // We need a PID that sets the angle
    }

    private double calculatePid(Rotation2d angle) {
        return shooterPID.calculate(angleEncoder.get(), angle.getDegrees());
    }

    public void periodic() {
        shooterAngle = Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue());
    }

}