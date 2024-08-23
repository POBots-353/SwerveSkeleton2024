// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PolynomialRegression;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class GyroConstants {
    public static final int gyroID = 1;
  }

  public static class SwerveDriveConstants {
    public static final double TRACK_WIDTH = Units.inchesToMeters(24.75);
    public static final double WHEEL_BASE = Units.inchesToMeters(24.75);

    public static final Translation2d frontLeft =
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d frontRight =
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
    public static final Translation2d backLeft =
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d backRight =
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

    public static final Translation2d[] wheelLocations = {
      frontLeft, frontRight, backLeft, backRight
    };

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(wheelLocations);

    // Values referenced from 2024 POBOTS Code; will be changed during testing
    public static final double turnToAngleMaxVelocity = Units.degreesToRadians(180.0);

    // Values referenced from 2024 POBOTS Code; will be changed during testing
    public static final double maxSpeed = Units.feetToMeters(8.5);
    public static final double maxAngularSpeed = Units.degreesToRadians(180);
    
    // Values referenced from 2024 POBOTS Code; will be changed during testing
    public static final double maxTranslationalAcceleration = Units.feetToMeters(30.0);
    public static final double maxAngularAcceleration = Units.degreesToRadians(360.0);
    
    // Values referenced from 2024 POBOTS Code; will be changed during testing
    public static final double maxTourqueCurrent = 50;
  }

  public static class SwerveModuleConstants {
    public static final double WHEEL_CIRCUMFERENCE = 2;
    public static final double WHEEL_RADIUS = WHEEL_CIRCUMFERENCE/(2*3.141592);

    public static final double GEAR_RATIO = 1/10; // THIS IS A DUMMY VARIABLE; WILL BE CHANGED WHEN MORE INFO COMES OUT

    public static final int talonFXCPR = 2048;
    public static final double degreesPerCount = 360.0 / talonFXCPR;

    // Values referenced from 2024 POBOTS Code; will be changed during testing
    public static final double kV = 2.4829;
    public static final double kS = 0.22542;
    public static final double kA = 0.120;

    // Values referenced from 2024 POBOTS Code; will be changed during testing
    public static final double kP = 0.85;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final int voltageCompensation = 12;

    public static final int FLTurnMotorID = 1;
    public static final int FLDriveMotorID = 2;
    public static final int FLCanCoderID = 3;

    public static final int FRTurnMotorID = 4;
    public static final int FRDriveMotorID = 5;
    public static final int FRCanCoderID = 6;

    public static final int BLTurnMotorID = 7;
    public static final int BLDriveMotorID = 8;
    public static final int BLCanCoderID = 9;

    public static final int BRTurnMotorID = 10;
    public static final int BRDriveMotorID = 11;
    public static final int BRCanCoderID = 12;

    public static double maxCurrent;
  }

  public static class VisionConstants {
    public static final int cameraHeight = 0;
    public static final int targetHeight = 0;
    public static final int cameraPitch = 0;
    public static final int targetPitch = 0;

    // Values referenced from 2024 POBOTS Code; will be changed during testing
    public static final double[] distances =
        new double[] {
          0.50, 1.00, 1.50, 2.00, 4.95, 5.5,
        };

    // Values referenced from 2024 POBOTS Code; will be changed during testing
    public static final double[] xyStandardDeviations =
        new double[] {
          0.019,
          0.025,
          0.050,
          0.108, 
          0.130,
          1.145
        };
  
    public static PolynomialRegression xyPolynomialRegression =
    new PolynomialRegression(distances, xyStandardDeviations, 3);
  }
}
