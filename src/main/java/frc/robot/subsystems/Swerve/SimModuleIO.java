// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SimModuleIO implements ModuleIO{
  /** Creates a new ModuleIOSim. */

  private final FlywheelSim moduleRotationSimModel = new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(0.16, 0.0917),
          DCMotor.getKrakenX60(1),
          12.8
  );

  private final FlywheelSim moduleThrottleSimModel = new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(2, 1.24),
          DCMotor.getKrakenX60Foc(1),
          8.16
  );

  private Encoder simulationTurnEncoder;
  private Encoder simulationThrottleEncoder;
  private EncoderSim simulationTurnEncoderSim;
  private EncoderSim simulationThrottleEncoderSim;

  private double simTurnEncoderDistance;
  private double simThrottleEncoderDistance;

  private double driveOutput;
  private double turnOutput;

  private PIDController driveController = new PIDController(SwerveModuleConstants.kP, 0, 0);
  private PIDController turnController = new PIDController(SwerveModuleConstants.kP, 0, 0);

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveModuleConstants.kS, SwerveModuleConstants.kV, SwerveModuleConstants.kA);

  private String name;

  public SimModuleIO(String name, 
    int simThrottleChannelA,
    int simThrottleChannelB,
    int simTurnChannelA,
    int simTurnChannelB) {

    this.name = name;

    simulationThrottleEncoder = new Encoder(simThrottleChannelA, simThrottleChannelB);
    simulationTurnEncoder = new Encoder(simTurnChannelA, simTurnChannelB);

    simulationThrottleEncoderSim = new EncoderSim(simulationThrottleEncoder);
    simulationTurnEncoderSim = new EncoderSim(simulationTurnEncoder);
    
  }

  public double getAngleInRadians() {
    return simulationTurnEncoder.getDistance();
  }

  public double getVelocity() {
    return simulationThrottleEncoder.getRate();
  }

  public double getPosition() {
    return simulationThrottleEncoder.getDistance();
  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(getAngleInRadians());
  }

  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getVelocity(), getAngle());
  }

  @Override
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getPosition(), getAngle());
  }

  @Override
  public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    double desiredVelocity = desiredState.speedMetersPerSecond;
      driveOutput = driveController.calculate(getVelocity(), desiredVelocity);
      double driveFeedForward = feedforward.calculate(desiredVelocity);

      driveOutput = driveOutput + driveFeedForward;

      moduleThrottleSimModel.setInputVoltage(driveOutput / SwerveDriveConstants.maxSpeed * RobotController.getBatteryVoltage());
  }

  @Override
  public void setAngle(SwerveModuleState desiredState) {
    Rotation2d desiredAngle = desiredState.angle;

    turnOutput = turnController.calculate(getAngleInRadians(), desiredAngle.getRadians());
    double turnFeedForward = feedforward.calculate(turnController.getSetpoint());
    turnOutput = turnOutput + turnFeedForward;
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {}

  public void updateTelementry() {
    SmartDashboard.putNumber(name + " - Angle", getAngle().getDegrees());
    SmartDashboard.putNumber(name + " - Drive Velocity", getModuleState().speedMetersPerSecond);
    SmartDashboard.putNumber(name + " - Distance Travelled", getModulePosition().distanceMeters);
  }

  public void periodic() {
    moduleThrottleSimModel.setInputVoltage(driveOutput / SwerveDriveConstants.maxSpeed * RobotController.getBatteryVoltage());
    moduleRotationSimModel.setInputVoltage(turnOutput / SwerveDriveConstants.maxAngularSpeed * RobotController.getBatteryVoltage());

    moduleRotationSimModel.update(0.02);
    moduleThrottleSimModel.update(0.02);

    simTurnEncoderDistance += moduleRotationSimModel.getAngularVelocityRadPerSec() * 0.02;
    simulationTurnEncoderSim.setDistance(simTurnEncoderDistance);
    simulationTurnEncoderSim.setRate(moduleRotationSimModel.getAngularVelocityRadPerSec());

    simThrottleEncoderDistance += moduleThrottleSimModel.getAngularVelocityRadPerSec() * 0.02;
    simulationThrottleEncoderSim.setDistance(simThrottleEncoderDistance);
    simulationThrottleEncoderSim.setRate(moduleThrottleSimModel.getAngularVelocityRadPerSec());
  }
}
