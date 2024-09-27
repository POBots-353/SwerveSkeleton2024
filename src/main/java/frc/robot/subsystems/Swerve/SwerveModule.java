// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.util.Conversions;

public class SwerveModule extends SubsystemBase {
  
  private TalonFX driveMotor;
  private TalonFX turnMotor;
  private CANcoder cancoder;

  String moduleName;

  private VelocityVoltage desiredVelocity;
  private PositionVoltage anglePosition;

  private Conversions Conversions;

  private DutyCycleOut driveDutyCycle;

  private double driveSetpointMPS = 0.0;
  private double driveAppliedVolts = 0.0;
  private double driveDistanceMeters = 0.0;
  private double angleSetpointDeg = 0.0;
  private double turnAppliedVolts = 0.0;

  private double turnRelativePositionRad = 0.0;
  private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;

  private SwerveModuleState previousState = new SwerveModuleState();

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveModuleConstants.kS, 
                                                                          SwerveModuleConstants.kV, 
                                                                          SwerveModuleConstants.kA);

  private PIDController driveController = new PIDController(SwerveModuleConstants.kP, 0, 0);
  private PIDController turnController = new PIDController(SwerveModuleConstants.kP, 0, 0);
  
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
  
  public SwerveModule(int driveMotorID, 
                      int turnMotorID, 
                      int canCoderID, 
                      String moduleName) {

    this.moduleName = moduleName;

    driveMotor = new TalonFX(driveMotorID);
    turnMotor = new TalonFX(turnMotorID);
    cancoder = new CANcoder(canCoderID);

    talonConfig();
  }

  public void talonConfig() {
    driveMotor.setInverted(false);
    driveMotor.setNeutralMode(NeutralModeValue.Brake);

    turnMotor.setInverted(false);
    turnMotor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    
    Slot0Configs slot0 = new Slot0Configs();

    slot0.kS = SwerveModuleConstants.kS; 
    slot0.kV = SwerveModuleConstants.kV;
    slot0.kA = SwerveModuleConstants.kA; 
    slot0.kP = SwerveModuleConstants.kP; 
    slot0.kI = SwerveModuleConstants.kI; 
    slot0.kD = SwerveModuleConstants.kD; 

    talonConfigs.Slot0 = slot0;

    talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = SwerveModuleConstants.maxCurrent;
    talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveModuleConstants.maxCurrent;

    driveMotor.getConfigurator().apply(talonConfigs);
    turnMotor.getConfigurator().apply(talonConfigs);
  }

  public void setAngle(SwerveModuleState desiredState) {
    if (Robot.isReal()) {
      anglePosition.FeedForward = feedforward.calculate(desiredState.angle.getRotations());
      turnMotor.setControl(anglePosition);
    } else {
      angleSetpointDeg = desiredState.angle.getDegrees();
    }
  }

  public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

    if (Robot.isReal()) {
      double desiredSpeed = desiredState.speedMetersPerSecond;
    
      if (isOpenLoop) {
        driveDutyCycle.Output = desiredSpeed / SwerveDriveConstants.maxSpeed;
        driveMotor.setControl(driveDutyCycle);
      } 
      else {
        desiredVelocity.Velocity = Conversions.MPStoRPS(desiredSpeed);
        desiredVelocity.FeedForward = feedforward.calculate(desiredSpeed, (desiredSpeed - previousState.speedMetersPerSecond) / 0.02);
        driveMotor.setControl(desiredVelocity); 
      }
    } 
    
    else {
      driveSetpointMPS = desiredState.speedMetersPerSecond;
    }

  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getModuleState().angle); 
    setSpeed(desiredState, isOpenLoop);
    setAngle(desiredState);
  }

  public SwerveModulePosition getModulePosition() {
    if (Robot.isReal()) {
      return new SwerveModulePosition(Conversions.MechanismRotationsToMeters(driveMotor.getPosition().getValueAsDouble()), getAngle());
    } else {
      driveDistanceMeters = driveDistanceMeters + (moduleThrottleSimModel.getAngularVelocityRadPerSec() * 
                                                   SwerveModuleConstants.LOOP_PERIOD_SECS * 
                                                   (SwerveModuleConstants.WHEEL_CIRCUMFERENCE / (2.0 * Math.PI)));

      return new SwerveModulePosition(driveDistanceMeters, getAngle());
    }
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }

  public double getVelocity() {
    if (Robot.isReal()) {
      return Conversions.RPStoMPS(driveMotor.getVelocity().getValueAsDouble());
    } else {
      return moduleThrottleSimModel.getAngularVelocityRadPerSec() * (SwerveModuleConstants.WHEEL_CIRCUMFERENCE / (2.0 * Math.PI));
    }
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(getAngleInRadians());
  }

  public double getAngleInRadians() {
    if (Robot.isReal()) {
      return Conversions.MechanismRotationsToDegrees(turnMotor.getPosition().getValueAsDouble());
    } else {
      double angleDiffRad = moduleRotationSimModel.getAngularVelocityRadPerSec() * SwerveModuleConstants.LOOP_PERIOD_SECS;

      turnRelativePositionRad += angleDiffRad;
      turnAbsolutePositionRad += angleDiffRad;

      return turnRelativePositionRad;
    }
  }

  public Rotation2d getAbsoluteAngle() {
    if (Robot.isReal()) {
      return Rotation2d.fromDegrees(cancoder.getAbsolutePosition().getValueAsDouble());
    } else {
      double angleDiffRad = moduleRotationSimModel.getAngularVelocityRadPerSec() * SwerveModuleConstants.LOOP_PERIOD_SECS;

      turnRelativePositionRad += angleDiffRad;
      turnAbsolutePositionRad += angleDiffRad;

      return Rotation2d.fromRadians(turnAbsolutePositionRad);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isReal()) {
      String telemetryKey = "Swerve/" + moduleName + "/";

      SmartDashboard.putNumber(telemetryKey + "Position", getModulePosition().distanceMeters);

      SmartDashboard.putNumber(telemetryKey + "Velocity", getModuleState().speedMetersPerSecond);
      SmartDashboard.putNumber(telemetryKey + "Angle", getAngle().getDegrees());
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                          SIMULATION PERIODIC                                                     //
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    else {
      // SETTING THE VELOCITY IN SIMULATION

      double velocityRadPerSec = driveSetpointMPS * (2.0 * Math.PI) / (SwerveModuleConstants.WHEEL_CIRCUMFERENCE);
      driveAppliedVolts =
          feedforward.calculate(velocityRadPerSec)
              + driveController.calculate(getVelocity(), velocityRadPerSec);
      driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
      moduleThrottleSimModel.setInputVoltage(driveAppliedVolts);

      // SETTING THE TURN ANGLE IN SIMULATION

      turnAppliedVolts =
          turnController.calculate(getAngleInRadians(), angleSetpointDeg * (Math.PI / 180.0));
      turnAppliedVolts = MathUtil.clamp(turnAppliedVolts, -12.0, 12.0);
      moduleRotationSimModel.setInputVoltage(turnAppliedVolts);

      //   moduleThrottleSimModel.setInputVoltage(driveOutput / SwerveDriveConstants.maxSpeed * RobotController.getBatteryVoltage());
      //   moduleRotationSimModel.setInputVoltage(turnOutput / SwerveDriveConstants.maxAngularSpeed * RobotController.getBatteryVoltage());

      //   moduleRotationSimModel.update(0.02);
      //   moduleThrottleSimModel.update(0.02);

      //   simTurnEncoderDistance += moduleRotationSimModel.getAngularVelocityRadPerSec() * 0.02;
      //   simulationTurnEncoderSim.setDistance(simTurnEncoderDistance);
      //   simulationTurnEncoderSim.setRate(moduleRotationSimModel.getAngularVelocityRadPerSec());

      //   simThrottleEncoderDistance += moduleThrottleSimModel.getAngularVelocityRadPerSec() * 0.02;
      //   simulationThrottleEncoderSim.setDistance(simThrottleEncoderDistance);
      //   simulationThrottleEncoderSim.setRate(moduleThrottleSimModel.getAngularVelocityRadPerSec());
      // }
    }
    }
  }
