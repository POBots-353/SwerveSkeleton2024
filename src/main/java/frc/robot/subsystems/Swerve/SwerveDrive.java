// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GyroConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.VisionSystem;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SweveDrive. */

  private final SwerveModule frontLeft = new SwerveModule(SwerveModuleConstants.FLDriveMotorID, 
                                                          SwerveModuleConstants.FLTurnMotorID, 
                                                          SwerveModuleConstants.FLCanCoderID, 
                                                          "Front Left Module", 
                                                          1, 2, 
                                                          3, 4);
  
  private final SwerveModule frontRight = new SwerveModule(SwerveModuleConstants.FRDriveMotorID, 
                                                          SwerveModuleConstants.FRTurnMotorID, 
                                                          SwerveModuleConstants.FRCanCoderID, 
                                                          "Front Right Module", 
                                                          5, 6, 
                                                          7, 8);

  private final SwerveModule backLeft = new SwerveModule(SwerveModuleConstants.BLDriveMotorID, 
                                                          SwerveModuleConstants.BLTurnMotorID, 
                                                          SwerveModuleConstants.BLCanCoderID, 
                                                          "Back Left Module", 
                                                          9, 10, 
                                                          11, 12);

  private final SwerveModule backRight = new SwerveModule(SwerveModuleConstants.BRDriveMotorID, 
                                                          SwerveModuleConstants.BRTurnMotorID, 
                                                          SwerveModuleConstants.BRCanCoderID, 
                                                          "Back Right Module", 
                                                          13, 14, 
                                                          15, 16);

  private final SwerveDrivePoseEstimator poseEstimator;

  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(SwerveDriveConstants.wheelLocations);

  private Pigeon2 pigeon = new Pigeon2(GyroConstants.gyroID);

  private VisionSystem vision;

  private Field2d field;

  StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct).publish();


  
  public SwerveDrive() {
    poseEstimator =
    new SwerveDrivePoseEstimator(
        kinematics,
        getHeading(),
        getModulePositions(),
        new Pose2d(0.0, 0.0, getHeading()));

    vision = new VisionSystem();

    field = new Field2d();
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
    frontLeft.setDesiredState(states[0], isOpenLoop);
    frontRight.setDesiredState(states[1], isOpenLoop);
    backLeft.setDesiredState(states[2], isOpenLoop);
    backRight.setDesiredState(states[3], isOpenLoop);
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
    double yaw = pigeon.getYaw().getValueAsDouble();
    return Rotation2d.fromDegrees(yaw);
  }

  public Rotation2d getHeading() {
    return pigeon.getRotation2d();
  }

  public void addVisionToPoseEstimate() {
    if (!vision.hasTargets()) return;

    poseEstimator.addVisionMeasurement(vision.getPose(), vision.getLatency(), 
    vision.getStandardDeviations(vision.getPoseFromVisionPoseEstimator(), vision.getTagTotalDistance(), 
    vision.getTagCount()));
  }

  public void updateOdometry() {
    poseEstimator.update(getHeading(), getModulePositions());
    addVisionToPoseEstimate();
  }

  public void resetOdometry(Rotation2d pose, SwerveModulePosition[] states, Pose2d drivePose) {
    poseEstimator.resetPosition(pose, states, drivePose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    posePublisher.set(getPose());

    SmartDashboard.putData("Field", field);

    updateOdometry();
    field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    frontRight.periodic();
    frontLeft.periodic();
    backRight.periodic();
    backLeft.periodic();

    field.setRobotPose(getPose());
    field.getObject("Swerve Modules").setPoses(getPose());

    SwerveModuleState[] moduleStates = {
      frontLeft.getModuleState(),
      frontRight.getModuleState(),
      backLeft.getModuleState(),
      backRight.getModuleState()
    };

    SmartDashboard.putData("Field", field);
  }
}
