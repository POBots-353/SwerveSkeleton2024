// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.fasterxml.jackson.databind.module.SimpleModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SweveDrive. */

  private final SwerveModule frontRight;
  private final SwerveModule frontLeft;
  private final SwerveModule backRight;
  private final SwerveModule backLeft;

  private final Field2d field2d = new Field2d();

  private Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
