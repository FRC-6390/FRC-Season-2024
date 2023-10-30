// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.controller.DebouncedController;
import frc.robot.commands.*;
// import frc.robot.commands.Auto;

public class RobotContainer {
  //  public static Drivetrain6390 driveTrain = new Drivetrain6390();

  public static DebouncedController controller = new DebouncedController(0);

  

  public RobotContainer() {
    //driveTrain.setDefaultCommand(new Drive(driveTrain, controller.leftX, controller.leftY, controller.rightX));
    configureBindings();
  }

  private void configureBindings() 
  {
      controller.rightBumper.onTrue(new IntakeUp());
      controller.leftBumper.whileTrue(new ParallelCommandGroup(new IntakeDown(), new IntakeRollers(0.8)));
      controller.leftStick.whileTrue(new ParallelCommandGroup(new OutputRollers(0.65, "cone", 0), new SpinWasher(0.5, 0)));
      controller.rightStick.whileTrue(new ParallelCommandGroup(new OutputRollers(-0.65, "cone", 0), new SpinWasher(0, 0)));
      controller.a.onTrue(new GoingDown());
      controller.x.onTrue(new GoingMid());
      controller.y.onTrue(new GoingHigh());
  } 


  // public Command getAutonomousCommand(){
  //    return null; //Auto.runAuto("Test Path");
  // }
}
