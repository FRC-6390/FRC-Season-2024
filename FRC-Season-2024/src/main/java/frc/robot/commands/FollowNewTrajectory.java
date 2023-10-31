// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class FollowNewTrajectory extends CommandBase {
  static PIDConstants XY_PID = new PIDConstants(1, 0, 0);
  static PIDConstants THETA_PID = new PIDConstants(4.9, 0, 0);

  private static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    RobotContainer.driveTrain::getPose,
    RobotContainer.driveTrain::resetOdometry,
    XY_PID,
    THETA_PID,
    RobotContainer.driveTrain::swerve,
    null,
    true,
    RobotContainer.driveTrain
    );

    PathPlannerTrajectory traj3;
    Translation2d translation;

  public FollowNewTrajectory() {

  }

  @Override
  public void initialize() 
  {
    translation = RobotContainer.driveTrain.getPose().getTranslation();
    traj3 = PathPlanner.generatePath(
      new PathConstraints(4, 3), 
      new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 2),
      new PathPoint(new Translation2d(5.0, 3.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-30)));
  }

  @Override
  public void execute() 
  {
    autoBuilder.followPath(traj3);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
