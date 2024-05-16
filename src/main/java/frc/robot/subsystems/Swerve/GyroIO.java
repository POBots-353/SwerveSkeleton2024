// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface GyroIO {
  
  boolean isConnected();

  Rotation2d yawValue();

  double yawVelocityValue();

  StatusSignal<Double> getAccelerationX();

  StatusSignal<Double> getAccelerationY();

  StatusSignal<Double> getAccelerationZ();

  Rotation3d getHeading3d();

  Rotation2d getRotation();
}
