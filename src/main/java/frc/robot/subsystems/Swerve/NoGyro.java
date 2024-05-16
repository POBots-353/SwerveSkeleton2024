// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class NoGyro implements GyroIO {
  /** Creates a new NoGyro. */
  private final StatusSignal<Double> accelX = new StatusSignal<>(null, null);
  private final StatusSignal<Double> accelY = new StatusSignal<>(null, null);
  private final StatusSignal<Double> accelZ = new StatusSignal<>(null, null);

  public boolean isConnected() {
    return false;
  }

  @Override
  public Rotation2d yawValue() {
    return new Rotation2d();
  }

  @Override
  public double yawVelocityValue() {
    return 0.0;
  }

  @Override
  public StatusSignal<Double> getAccelerationX() {
    return accelX;
  }

  @Override
  public StatusSignal<Double> getAccelerationY() {
    return accelY;
  }

  @Override
  public StatusSignal<Double> getAccelerationZ() {
    return accelZ;
  }

  @Override
  public Rotation3d getHeading3d() {
    return new Rotation3d();
  }

  @Override
  public Rotation2d getRotation() {
    return new Rotation2d();
  }
}
