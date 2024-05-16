// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SweveDrive. */

  private final SwerveModule frontRight;
  private final SwerveModule frontLeft;
  private final SwerveModule backRight;
  private final SwerveModule backLeft;

  private final SwerveDrivePoseEstimator poseEstimator;
  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(SwerveDriveConstants.wheelLocations);

  private final Field2d field2d = new Field2d();

  private GyroIO gyro;

  public static SwerveDrive create() {
    if (Robot.isReal()) {
      return new SwerveDrive(
        new Pigeon2IO(), 
        new KrakenModuleIO(SwerveModuleConstants.FRDriveMotorID, SwerveModuleConstants.FRTurnMotorID, SwerveModuleConstants.FRCanCoderID, "Front Right"), 
        new KrakenModuleIO(SwerveModuleConstants.FLDriveMotorID, SwerveModuleConstants.FLTurnMotorID, SwerveModuleConstants.FLCanCoderID, "Front Left"), 
        new KrakenModuleIO(SwerveModuleConstants.BRDriveMotorID, SwerveModuleConstants.BRTurnMotorID, SwerveModuleConstants.BRCanCoderID, "Back Right"), 
        new KrakenModuleIO(SwerveModuleConstants.BLDriveMotorID, SwerveModuleConstants.BLTurnMotorID, SwerveModuleConstants.BLCanCoderID, "Back Left"));
    } if (Robot.isSimulation()) {
      return new SwerveDrive(
        new NoGyro(),
        new SimModuleIO(),
        new SimModuleIO(),
        new SimModuleIO(),
        new SimModuleIO()
      );
    } else {
      return new SwerveDrive(
        new NoGyro(),
        new NoModule(),
        new NoModule(),
        new NoModule(),
        new NoModule()
      );
    }
  }
  
  public SwerveDrive(GyroIO gyro, ModuleIO frontRight, ModuleIO frontLeft, ModuleIO backRight, ModuleIO backLeft) {
    this.gyro = gyro;
    this.frontRight = new SwerveModule(frontRight, "Front Right");
    this.frontLeft = new SwerveModule(frontLeft, "Front Left");
    this.backRight = new SwerveModule(backRight, "Back Right");
    this.backLeft = new SwerveModule(backLeft, "Back Left");

    poseEstimator =
    new SwerveDrivePoseEstimator(
        kinematics,
        getHeading(),
        getModulePositions(),
        new Pose2d(0.0, 0.0, getHeading()));
  }

  public void drive(
    double forward,
    double strafe,
    double turn,
    boolean isOpenLoop) {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, -strafe, turn, getHeading());
    setChassisSpeeds(chassisSpeeds, isOpenLoop);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {
    speeds = ChassisSpeeds.discretize(speeds, 0.020);

    setModuleStates(kinematics.toSwerveModuleStates(speeds), isOpenLoop);
  }

  public void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    frontLeft.setState(states[0], isOpenLoop);
    frontRight.setState(states[1], isOpenLoop);
    backLeft.setState(states[2], isOpenLoop);
    backRight.setState(states[3], isOpenLoop);
  }

  public void lockModules() {
    setModuleStates(
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        },
        false);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getModulePosition(),
      frontRight.getModulePosition(),
      backLeft.getModulePosition(),
      backRight.getModulePosition(),
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getModuleState(),
      frontRight.getModuleState(),
      backLeft.getModuleState(),
      backRight.getModuleState(),
    };
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getYaw() {
    return gyro.yawValue();
  }

  public Rotation2d getHeading() {
    return gyro.getRotation();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
