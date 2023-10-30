// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.vission.LimeLight;
import frc.robot.utilities.vission.LimelightConfig;


public class AutoAlign extends CommandBase {
  
  
  PIDController controller;
  double kP;
  double kI;
  double kD;

  public AutoAlign()
  {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    controller = new PIDController(kP, kI, kD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    

    if(LimeLight.hasValidTarget() == true)
    {
      Drivetrain6390.translate(0,0);
    }
    else
    {
      Drivetrain6390.translate(90, controller.calculate(LimeLight.getTargetHorizontalOffset(), 0));
    }
    SmartDashboard.putNumber("Horizontal Offset", LimeLight.getTargetHorizontalOffset());
    SmartDashboard.putBoolean("Has Target?", LimeLight.hasValidTarget());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    Drivetrain6390.translate(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
