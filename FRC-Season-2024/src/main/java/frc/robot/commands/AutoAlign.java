// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.vission.LimeLight;


public class AutoAlign extends CommandBase {
  
  //PID controller
  public PIDController controller;
  //Sets up a limelight - camera used for vision tracking
  public LimeLight limeLight = new LimeLight();
  //Declare the drivetrain object
  public Drivetrain6390 drivetrain = new Drivetrain6390();
  
  //PID constants
  double kP = 0;
  double kI = 0;
  double kD = 0;

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
    //If the limelight has not detected something...
    if(limeLight.hasValidTarget() != true)
    {

      drivetrain.translate(0,0);
    }
    else //But if it has...
    {
      //Use the translate command to move the chassis left or right, depending on which direction lowers the horizontal offest to target
      drivetrain.translate(90, controller.calculate(limeLight.getTargetHorizontalOffset(), 0));
    }

    //Display some data
    SmartDashboard.putNumber("Horizontal Offset", limeLight.getTargetHorizontalOffset());
    SmartDashboard.putBoolean("Has Target?", limeLight.hasValidTarget());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.translate(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
