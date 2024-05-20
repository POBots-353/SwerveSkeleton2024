// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
  public double driveVelocity = 0.0;
  public double driveVoltage = 0.0;

  public Rotation2d turnAngle = new Rotation2d();

  public SwerveModuleState moduleState = new SwerveModuleState();
  public SwerveModuleState modulePosition = new SwerveModuleState();

  public SwerveModulePosition simModulePosition = new SwerveModulePosition();
  public double simDistance = 0.0;
 
  SwerveModulePosition getModulePosition();

  SwerveModuleState getModuleState();
  
  Rotation2d getAngle();

  void setAngle(SwerveModuleState desiredState);

  void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop);

  void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop);

  void periodic();
}
