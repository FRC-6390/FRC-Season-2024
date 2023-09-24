// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;

public class Auto extends CommandBase {
  /** Creates a new Auto. */
  public Auto() {
    // Use addRequirements() here to declare subsystem dependencies.
    
  }
  static PIDConstants XY_PID = new PIDConstants(1, 0, 0);
  static PIDConstants THETA_PID = new PIDConstants(4.9, 0, 0);
  
  private static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
        
    ));

    private static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      RobotContainer.driveTrain::getPose,
      RobotContainer.driveTrain::resetOdometry,
      XY_PID,
      THETA_PID,
      RobotContainer.driveTrain::swerve,
      eventMap,
      true,
      RobotContainer.driveTrain
    );

    public static CommandBase runAuto(String autoSelector) {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup(autoSelector, new PathConstraints(1.5, 0.8)));
    }
}
