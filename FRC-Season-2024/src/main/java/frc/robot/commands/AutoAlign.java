// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.vission.LimeLight;


public class AutoAlign extends CommandBase {
  
  //PID controller
  public PIDController controller;
  public PIDController yController;
  //Sets up a limelight - camera used for vision tracking
  public LimeLight limeLight;
  //Declare the drivetrain object
  public Drivetrain6390 drivetrain;
  
  public NetworkTable lime = NetworkTableInstance.getDefault().getTable("limelight");
  
  //PID constants
  double kP = 0.08;
  double kI = 0;
  double kD = 0;

  double kP2 = 0.08;
  double kI2 = 0;
  double kD2 = 0;

  double targetHeightMeters = 0.7112;
  public String direction;

  public AutoAlign(Drivetrain6390 drivetrain, LimeLight limeLight)
  {
    this.drivetrain = drivetrain;
    this.limeLight = limeLight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    controller = new PIDController(kP, kI, kD);
    yController = new PIDController(kP2, kI2, kD2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    NetworkTableEntry poseEntry = lime.getEntry("botpose");
    Double[] dub = {0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0};
    Double[] pose = poseEntry.getDoubleArray(dub);
    
    //This should put robot coordinates into smart dashboard.
    SmartDashboard.putNumberArray("Pose Array", pose);

    /*
    UNCOMMENT THE FOLLOWING CODE IF VALUES ARE BEING OUTPUTTED TO SMARTDASHBOARD UNDER "POSE ARRAY"
    FIRST 3 NUMBERS SHOULD REPRESENT THE X Y Z DISTANCES TO TARGET  
    */

    // if(limeLight.hasValidTarget() != true)
    // {
    //   drivetrain.drive(new ChassisSpeeds(0,0,0));
    // }
    // else //But if it has...
    // {
    //   //Use the feedback drive command to move the chassis left or right, depending on which direction lowers the horizontal offest to target
    //   drivetrain.drive(new ChassisSpeeds(yController.calculate(pose[0], 0), controller.calculate(pose[1], 0),0));
    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.drive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
