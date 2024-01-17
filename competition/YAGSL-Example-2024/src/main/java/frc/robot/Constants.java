// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double TURN_CONSTANT = 0.75;
  }

   // ---------- Vision
    // Constants about how your camera is mounted to the robot
    public static final double CAMERA_PITCH_RADIANS =
            Units.degreesToRadians(15); // Angle "up" from horizontal
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24); // Height above floor

    // How far from the target we want to be
    public static final double GOAL_RANGE_METERS = Units.feetToMeters(10);

    // Where the 2020 High goal target is located on the field
    // See
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system
    // and https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    // (pages 4 and 5)
    public static final Pose3d TARGET_POSE =
            new Pose3d(
                    new Translation3d(
                            Units.feetToMeters(52.46),
                            Units.inchesToMeters(94.66),
                            Units.inchesToMeters(89.69)), // (center of vision target)
                    new Rotation3d(0.0, 0.0, Math.PI));
    // ----------

    // ---------- Drivetrain
    public static final int LEFT_MOTOR_CHANNEL = 0;
    public static final int RIGHT_MOTOR_CHANNEL = 1;

    // PID constants should be tuned per robot
    public static final double LINEAR_P = 0.5;
    public static final double LINEAR_I = 0;
    public static final double LINEAR_D = 0.1;

    public static final double ANGULAR_P = 0.03;
    public static final double ANGULAR_I = 0;
    public static final double ANGULAR_D = 0.003;

    // Ratio to multiply joystick inputs by
    public static final double DRIVESPEED = 0.75;

    // The following properties are necessary for simulation:

    // Distance from drivetrain left wheels to right wheels
    public static final double TRACKWIDTH_METERS = Units.feetToMeters(2.0);
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6.0);

    // The motors used in the gearbox for one drivetrain side
    public static final DCMotor DRIVE_MOTORS = DCMotor.getCIM(2);
    // The gearbox reduction, or how many motor rotations per wheel rotation
    public static final double GEARING = 8.0;

    // The drivetrain feedforward values
    public static final double LINEAR_KV = 2.0;
    public static final double LINEAR_KA = 0.5;

    public static final double ANGULAR_KV = 2.25;
    public static final double ANGULAR_KA = 0.3;
    // ----------
}
