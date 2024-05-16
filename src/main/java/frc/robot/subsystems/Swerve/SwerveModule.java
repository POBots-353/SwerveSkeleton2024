// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  
  private ModuleIO hardware;

  public String moduleName;

  public SwerveModule(ModuleIO hardware, String moduleName) {
    this.hardware = hardware;
    this.moduleName = moduleName;
  }

  public SwerveModuleState getModuleState() {
    return hardware.getModuleState();
  }

  public SwerveModulePosition getModulePosition() {
    return hardware.getModulePosition();
  }

  public Rotation2d getAngle() {
    return hardware.getAngle();
  }

  public void setAngle(SwerveModuleState desiredState) {
    hardware.setAngle(desiredState);
  }

  public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    hardware.setSpeed(desiredState, isOpenLoop);
  }

  public void setState(SwerveModuleState desiredState, boolean isOpenLoop) {
    setSpeed(desiredState, isOpenLoop);
    setAngle(desiredState);
  }

  public void setVoltage() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    String telemetryKey = "Swerve/" + moduleName + "/";

    SmartDashboard.putNumber(telemetryKey + "Position", getModulePosition().distanceMeters);

    SmartDashboard.putNumber(telemetryKey + "Velocity", getModuleState().speedMetersPerSecond);
    SmartDashboard.putNumber(telemetryKey + "Angle", getAngle().getDegrees());
  }
}
