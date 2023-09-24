// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.controller.DebouncedController;
import frc.robot.commands.Auto;


public class RobotContainer {
  public static Drivetrain6390 driveTrain = new Drivetrain6390();

  public static DebouncedController controller = new DebouncedController(0);

  public RobotContainer() {
    driveTrain.setDefaultCommand(new Drive(driveTrain, controller.leftX, controller.leftY, controller.rightX));
    configureBindings();
  }

  private void configureBindings() {
   
  }

   public Command getAutonomousCommand(){
    return Auto.runAuto("Test Path");
  }
}
