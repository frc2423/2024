// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  public static final class Auton {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);
    public static final double MAX_ACCELERATION = 2;
  }

  public static final class autoAlign {
    // THESE ARE FOR THE BLUE ALLIANCE
    public static final Pose2d shootPose = new Pose2d(2.34, 5.59, Rotation2d.fromDegrees(0));
    public static final Pose2d sourceRightPose = new Pose2d(14.80, 0.61, Rotation2d.fromDegrees(126.28));
    public static final Pose2d sourceMiddlePose = new Pose2d(15.42, 0.89, Rotation2d.fromDegrees(126.28));
    public static final Pose2d sourceLeftPose = new Pose2d(15.97, 1.23, Rotation2d.fromDegrees(126.28));
    public static final Pose2d speakerLocationPose = new Pose2d(0.00, 4.97, Rotation2d.fromDegrees(0));
    public static final Pose2d StageLeftPose = new Pose2d(4.09, 5.43, Rotation2d.fromDegrees(-60));
    public static final Pose2d StageRightPose = new Pose2d(3.97, 2.82, Rotation2d.fromDegrees(60));
    public static final Pose2d StageCenterPose = new Pose2d(6.55, 4.15, Rotation2d.fromDegrees(-180));

    public static final Pose3d autoCameraPose = new Pose3d(6, 9, 3, new Rotation3d(0, .4, -Math.toRadians(160)));

    public static final Pose2d ampPose = new Pose2d(1.82, 7.68, Rotation2d.fromDegrees(90));
    public static final Pose2d ampNote = new Pose2d(2.92, 6.99, Rotation2d.fromDegrees(0));
    public static final Pose2d middleNote = new Pose2d(2.89, 5.55, Rotation2d.fromDegrees(0));
    public static final Pose2d stageNote = new Pose2d(2.89, 4.10, Rotation2d.fromDegrees(0));
    public static final Pose2d center1Note = new Pose2d(8.29,7.46,Rotation2d.fromDegrees(0));
    public static final Pose2d center2Note = new Pose2d(8.29,5.81,Rotation2d.fromDegrees(0));
    public static final Pose2d center3Note = new Pose2d(8.29,4.11,Rotation2d.fromDegrees(0));
    public static final Pose2d center4Note = new Pose2d(8.29,2.45,Rotation2d.fromDegrees(0));
    public static final Pose2d center5Note = new Pose2d(8.29,0.77,Rotation2d.fromDegrees(0));



  }

  public static final class Drivebase {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 0.75;
  }

  public static class IntakeConstants {

  }

  public static class Vision {
    public static final String kCameraName = "Arducam_OV9281_USB_Camera";
    public static final String knoteCameraName = "Arducam_OV9782_USB_Camera";

    // Assumed the origin point was on the floor -(^o^)-
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(-0.229, 0, 0.394),
        new Rotation3d(0, -0.419, Math.PI));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout;

    static {
      kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    }

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }
}
