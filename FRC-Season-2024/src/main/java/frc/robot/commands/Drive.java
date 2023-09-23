// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.subsystems.Drivetrain6390;


public class Drive extends CommandBase {
  /** Creates a new Drive. */
  public ChassisSpeeds speeds;
  public DoubleSupplier xinput;
  public DoubleSupplier yinput;
  public DoubleSupplier thetainput;
  public Drivetrain6390 drivetrain;
  private SlewRateLimiter xLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND);
  private SlewRateLimiter thetaLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND);
  public Drive(Drivetrain6390 drivetrain, DoubleSupplier xinput, DoubleSupplier yinput, DoubleSupplier thetainput) {
    this.drivetrain = drivetrain;
    this.xinput = xinput;
    this.yinput = yinput;
    this.thetainput = thetainput;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    speeds = new ChassisSpeeds(xLimiter.calculate(-xinput.getAsDouble() * Constants.SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND), yLimiter.calculate(yinput.getAsDouble() * Constants.SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND), thetaLimiter.calculate(thetainput.getAsDouble() * Constants.SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND));
    drivetrain.swerve(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.swerve(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
