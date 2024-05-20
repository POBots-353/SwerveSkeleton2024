// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class NoModule implements ModuleIO {
  /** Creates a new NoModule. */
  public NoModule() {}

  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition();
  }

  @Override
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState();
  }
 
  @Override
  public Rotation2d getAngle() {
    return new Rotation2d();
  }

  @Override
  public void setAngle(SwerveModuleState desiredState) {}

  @Override
  public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {}

  @Override
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {}
}
