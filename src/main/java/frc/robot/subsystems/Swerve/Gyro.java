// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  /** Creates a new Pigeon2IO. */
  Pigeon2 pigeon;
  StatusSignal<Double> yaw;
  private final StatusSignal<Double> yawVelocity;
  StatusSignal<Double>[] accelerationArray;

  public Gyro() {
    pigeon = new Pigeon2(1);
    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();

    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(100.0);
    yawVelocity.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();
  }

  public boolean isConnected() {
    return BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK();
  }

  public Rotation2d yawValue() {
    return Rotation2d.fromDegrees(yaw.getValueAsDouble());
  }

  public double yawVelocityValue() {
    return Units.degreesToRadians(yawVelocity.getValueAsDouble());
  }

  public StatusSignal<Double> getAccelerationX() {
    return pigeon.getAccelerationX();
  }

  public StatusSignal<Double> getAccelerationY() {
    return pigeon.getAccelerationY();
  }

  public StatusSignal<Double> getAccelerationZ() {
    return pigeon.getAccelerationZ();
  }

  public Rotation3d getHeading3d() {
    return pigeon.getRotation3d();
  }

  public Rotation2d getRotation() {
    return pigeon.getRotation2d();
  }
}
