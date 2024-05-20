// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class FieldSim{
  /** Creates a new FieldSim. */
  private Field2d field;
  private final SwerveDrive drive;

  public FieldSim(SwerveDrive drive) {
    this.drive = drive;

    field = new Field2d();
  }

  public void initSim() {
    Pose2d startPosition = new Pose2d(Units.inchesToMeters(30),Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0)));
    field.setRobotPose(startPosition);
    drive.resetOdometry(startPosition.getRotation(), drive.getModulePositions(), field.getRobotPose());
    field.getObject("trajectory").setPose(new Pose2d());
  }

  public void simulationPeriodic() {
    field.setRobotPose(drive.getPose());

    field.getObject("Swerve Modules").setPose(drive.getPose());

    SmartDashboard.putData("Field2d", field);
  }

  public Pose2d getRobotPose() {
    return field.getRobotPose();
  }

  public synchronized void resetRobotPose(Pose2d pose){
    field.setRobotPose(pose);
    drive.resetOdometry(pose.getRotation(), drive.getModulePositions(), pose);
  }
}
