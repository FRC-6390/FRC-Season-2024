// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class FollowNewTrajectory extends CommandBase {
  static PIDConstants XY_PID = new PIDConstants(1, 0, 0);
  static PIDConstants THETA_PID = new PIDConstants(4.9, 0, 0);

  //A class which handles how a swerve drivetrain follows paths
  private static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    RobotContainer.driveTrain::getPose,
    RobotContainer.driveTrain::resetOdometry,
    XY_PID,
    THETA_PID,
    RobotContainer.driveTrain::drive,
    null,
    true,
    RobotContainer.driveTrain
    );

    //A path made up of positions on the field
    PathPlannerTrajectory traj3;
    //Going to store the current coords of the robot. Translation is just like pose but without rotation
    Translation2d translation;

  public FollowNewTrajectory() {

  }

  @Override
  public void initialize() 
  {
    //Get current x and y
    translation = RobotContainer.driveTrain.getPose().getTranslation();
    //Generate a path
    traj3 = PathPlanner.generatePath(
      new PathConstraints(4, 3), 
      new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 2),
      new PathPoint(new Translation2d(5.0, 3.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-30)));
  }

  @Override
  public void execute() 
  {
    //Follow the path
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
