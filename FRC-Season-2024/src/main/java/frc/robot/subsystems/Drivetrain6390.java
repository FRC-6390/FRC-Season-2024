// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.utilities.swerve.SwerveModule;


public class Drivetrain6390 extends SubsystemBase {
  public static SwerveModule[] swerveModules;
  
  public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DRIVETRAIN.SWERVE_MODULE_LOCATIONS);
  
  public static SwerveModuleState[] states;
  
  static
  {
    //Motors
    swerveModules = new SwerveModule[4];
    swerveModules[0] = new SwerveModule(Constants.DRIVETRAIN.FRONT_LEFT_MODULE_CONFIG);
    swerveModules[1] = new SwerveModule(Constants.DRIVETRAIN.FRONT_RIGHT_MODULE_CONFIG);
    swerveModules[2] = new SwerveModule(Constants.DRIVETRAIN.BACK_LEFT_MODULE_CONFIG);
    swerveModules[3] = new SwerveModule(Constants.DRIVETRAIN.BACK_RIGHT_MODULE_CONFIG);
  }

  
  /** Creates a new SwerveDriveSubsystem. */
  public Drivetrain6390() 
  {

  }

  public void swerve(ChassisSpeeds speeds)
  {
   states = kinematics.toSwerveModuleStates(speeds);
   SwerveDriveKinematics.desaturateWheelSpeeds(states, SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
   swerveModules[0].setDesiredState(states[0]);
   swerveModules[1].setDesiredState(states[1]);
   swerveModules[2].setDesiredState(states[2]);
   swerveModules[3].setDesiredState(states[3]);
  }
  


  

 



 
}
