// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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

  public static class Vision {
        public static final String kCameraName = "YOUR CAMERA NAME";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout;

        static {
            kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        }

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class Swerve {
        // Physical properties
        public static final double kTrackWidth = Units.inchesToMeters(18.5);
        public static final double kTrackLength = Units.inchesToMeters(18.5);
        public static final double kRobotWidth = Units.inchesToMeters(25 + 3.25 * 2);
        public static final double kRobotLength = Units.inchesToMeters(25 + 3.25 * 2);
        public static final double kMaxLinearSpeed = Units.feetToMeters(15.5);
        public static final double kMaxAngularSpeed = Units.rotationsToRadians(2);
        public static final double kWheelDiameter = Units.inchesToMeters(4);
        public static final double kWheelCircumference = kWheelDiameter * Math.PI;

        public static final double kDriveGearRatio = 6.75; // 6.75:1 SDS MK4 L2 ratio
        public static final double kSteerGearRatio = 12.8; // 12.8:1

        public static final double kDriveDistPerPulse = kWheelCircumference / 1024 / kDriveGearRatio;
        public static final double kSteerRadPerPulse = 2 * Math.PI / 1024;

        public enum ModuleConstants {
            FL( // Front left
                    1, 0, 0, 1, 1, 2, 3, 0, kTrackLength / 2, kTrackWidth / 2),
            FR( // Front Right
                    2, 2, 4, 5, 3, 6, 7, 0, kTrackLength / 2, -kTrackWidth / 2),
            BL( // Back Left
                    3, 4, 8, 9, 5, 10, 11, 0, -kTrackLength / 2, kTrackWidth / 2),
            BR( // Back Right
                    4, 6, 12, 13, 7, 14, 15, 0, -kTrackLength / 2, -kTrackWidth / 2);

            public final int moduleNum;
            public final int driveMotorID;
            public final int driveEncoderA;
            public final int driveEncoderB;
            public final int steerMotorID;
            public final int steerEncoderA;
            public final int steerEncoderB;
            public final double angleOffset;
            public final Translation2d centerOffset;

            private ModuleConstants(
                    int moduleNum,
                    int driveMotorID,
                    int driveEncoderA,
                    int driveEncoderB,
                    int steerMotorID,
                    int steerEncoderA,
                    int steerEncoderB,
                    double angleOffset,
                    double xOffset,
                    double yOffset) {
                this.moduleNum = moduleNum;
                this.driveMotorID = driveMotorID;
                this.driveEncoderA = driveEncoderA;
                this.driveEncoderB = driveEncoderB;
                this.steerMotorID = steerMotorID;
                this.steerEncoderA = steerEncoderA;
                this.steerEncoderB = steerEncoderB;
                this.angleOffset = angleOffset;
                centerOffset = new Translation2d(xOffset, yOffset);
            }
        }

        // Feedforward
        // Linear drive feed forward
        public static final SimpleMotorFeedforward kDriveFF =
                new SimpleMotorFeedforward( // real
                        0.25, // Voltage to break static friction
                        2.5, // Volts per meter per second
                        0.3 // Volts per meter per second squared
                        );
        // Steer feed forward
        public static final SimpleMotorFeedforward kSteerFF =
                new SimpleMotorFeedforward( // real
                        0.5, // Voltage to break static friction
                        0.25, // Volts per radian per second
                        0.01 // Volts per radian per second squared
                        );

        // PID
        public static final double kDriveKP = 1;
        public static final double kDriveKI = 0;
        public static final double kDriveKD = 0;

        public static final double kSteerKP = 20;
        public static final double kSteerKI = 0;
        public static final double kSteerKD = 0.25;
    }
}




