// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private DoubleSupplier forwardSpeed;
  private DoubleSupplier strafeSpeed;
  private DoubleSupplier angleX;
  private DoubleSupplier angleY;
  private BooleanSupplier turnToAngle;

  private DoubleSupplier maxTranslationalSpeed;
  private double maxAngularSpeed;

  private SlewRateLimiter forwardRateLimiter =
      new SlewRateLimiter(SwerveDriveConstants.maxTranslationalAcceleration);
  private SlewRateLimiter strafeRateLimiter =
      new SlewRateLimiter(SwerveDriveConstants.maxTranslationalAcceleration);
  private SlewRateLimiter angularRateLimiter =
      new SlewRateLimiter(SwerveDriveConstants.maxAngularAcceleration);

  private SwerveDrive swerve;

  private PIDController turnToAngleController =
      new PIDController(SwerveDriveConstants.headingP, 0, SwerveDriveConstants.headingD);

  private final boolean isOpenLoop = true;

  /** Creates a new TeleopSwerve. */
  public TeleopSwerve(
      DoubleSupplier forwardSpeed,
      DoubleSupplier strafeSpeed,
      DoubleSupplier angleX,
      DoubleSupplier angleY,
      BooleanSupplier turnToAngle,
      DoubleSupplier maxTranslationalSpeed,
      double maxAngularSpeed,
      SwerveDrive swerve) {
    this.forwardSpeed = forwardSpeed;
    this.strafeSpeed = strafeSpeed;
    this.angleX = angleX;
    this.angleY = angleY;
    this.turnToAngle = turnToAngle;

    this.maxTranslationalSpeed = maxTranslationalSpeed;
    this.maxAngularSpeed = maxAngularSpeed;

    this.swerve = swerve;

    turnToAngleController.enableContinuousInput(-Math.PI, Math.PI);
    turnToAngleController.setTolerance(Units.degreesToRadians(1.0));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  /** Creates a new TeleopSwerve. */
  public TeleopSwerve(
      DoubleSupplier forwardSpeed,
      DoubleSupplier strafeSpeed,
      DoubleSupplier angleX,
      DoubleSupplier angleY,
      BooleanSupplier turnToAngle,
      double maxTranslationalSpeed,
      double maxAngularSpeed,
      SwerveDrive swerve) {
    this(
        forwardSpeed,
        strafeSpeed,
        angleX,
        angleY,
        turnToAngle,
        () -> maxTranslationalSpeed,
        maxAngularSpeed,
        swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardMetersPerSecond =
        -forwardSpeed.getAsDouble() * maxTranslationalSpeed.getAsDouble();
    double strafeMetersPerSecond = strafeSpeed.getAsDouble() * maxTranslationalSpeed.getAsDouble();

    double angleXComponent = -angleX.getAsDouble();
    double angleYComponent = -angleY.getAsDouble();

    forwardMetersPerSecond = forwardRateLimiter.calculate(forwardMetersPerSecond);
    strafeMetersPerSecond = strafeRateLimiter.calculate(strafeMetersPerSecond);

    if (Math.abs(forwardMetersPerSecond) < Units.inchesToMeters(0.5)) {
      forwardMetersPerSecond = 0.0;
      forwardRateLimiter.reset(0.0);
    }

    if (Math.abs(strafeMetersPerSecond) < Units.inchesToMeters(0.5)) {
      strafeMetersPerSecond = 0.0;
      strafeRateLimiter.reset(0.0);
    }

    if (!turnToAngle.getAsBoolean()) {
      angleXComponent = angularRateLimiter.calculate(angleXComponent);

      if (Math.abs(angleXComponent) < Units.degreesToRadians(0.5)) {
        angleXComponent = 0.0;
        angularRateLimiter.reset(0.0);
      }

      swerve.drive(
          forwardMetersPerSecond,
          strafeMetersPerSecond,
          angleXComponent * maxAngularSpeed,
          isOpenLoop);
    } else {
      angularRateLimiter.reset(0.0);

      if (Math.abs(angleXComponent) > 0.05 || Math.abs(angleYComponent) > 0.05) {
        Rotation2d desiredAngle = new Rotation2d(-angleXComponent, -angleYComponent);

        double angularSpeed =
            turnToAngleController.calculate(
                swerve.getHeading().getRadians(), -desiredAngle.getRadians() - (Math.PI / 2));

        angularSpeed = MathUtil.clamp(angularSpeed, -0.75, 0.75);

        swerve.drive(
            forwardMetersPerSecond,
            strafeMetersPerSecond,
            angularSpeed * SwerveDriveConstants.turnToAngleMaxVelocity,
            isOpenLoop);
      } else {
        swerve.drive(
            forwardMetersPerSecond, strafeMetersPerSecond, 0.0, isOpenLoop);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    forwardRateLimiter.reset(0.0);
    strafeRateLimiter.reset(0.0);
    angularRateLimiter.reset(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}