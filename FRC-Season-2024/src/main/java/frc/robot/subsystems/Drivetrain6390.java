// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.utilities.swerve.SwerveModule;


public class Drivetrain6390 extends SubsystemBase {

  public static SwerveModule[] swerveModules;
  public static SwerveModulePosition[] SwervePositions = {swerveModules[0].getPostion(), swerveModules[1].getPostion(), swerveModules[2].getPostion(), swerveModules[3].getPostion()};
  public static Pigeon2 gyro = new Pigeon2(DRIVETRAIN.PIGEON, DRIVETRAIN.CANBUS);
  public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DRIVETRAIN.SWERVE_MODULE_LOCATIONS);
  public static  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyro.getYaw()), SwervePositions);
  
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

  public static void translate(double direction, double power)
  {
    swerveModules[0].setToAngle(direction);
    swerveModules[1].setToAngle(direction);
    swerveModules[2].setToAngle(direction);
    swerveModules[3].setToAngle(direction);

    swerveModules[0].setDriveMotor(power);
    swerveModules[1].setDriveMotor(power);
    swerveModules[2].setDriveMotor(power);
    swerveModules[3].setDriveMotor(power);
  }

  public Pose2d getPose()
  {
    return odometry.getPoseMeters();
  }
  public void resetOdometry(Pose2d pose)
  {
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }
  
  public double getHeading(){
    return Math.IEEEremainder(gyro.getYaw(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  private SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    positions[0] = swerveModules[0].getPostion();
    positions[1] = swerveModules[1].getPostion();
    positions[2] = swerveModules[2].getPostion();
    positions[3] = swerveModules[3].getPostion();
    return positions;
  }
  


  

 



 
}
