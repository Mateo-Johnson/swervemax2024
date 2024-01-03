// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;


public class RobotContainer {
  //SUBSYSTEMS
  private final DriveSubsystem drivetrain = new DriveSubsystem();

  //DRIVER CONTROLLERS (ALWAYS PORT 0 FOR DRIVER AND PORT 1 FOR SECONDARY)
  CommandXboxController primaryDriver = new CommandXboxController(0);
  CommandXboxController secondaryDriver = new CommandXboxController(1);


  public RobotContainer() {

    configureButtonBindings(); //CONFIGURE BINDINGS

    ///CONFIGURE DEFAULT COMMANDS
    drivetrain.setDefaultCommand(

        //LEFT STICK IS TRANSLATION RIGHT STICK IS TURNING
        new RunCommand(() -> drivetrain.drive(
                -MathUtil.applyDeadband(primaryDriver.getLeftY(), ControllerConstants.driveDeadzone),
                -MathUtil.applyDeadband(primaryDriver.getLeftX(), ControllerConstants.driveDeadzone),
                -MathUtil.applyDeadband(primaryDriver.getRightX(), ControllerConstants.driveDeadzone),
                true, true),
            drivetrain));
  }


  private void configureButtonBindings() {
    //DEFINE ALL OF THE BUTTON BINDINGS HERE PLEASE AND THANKS
    primaryDriver.rightBumper()
        .whileTrue(new RunCommand(
            () -> drivetrain.setWheelsX(),
            drivetrain));
            
  }

  //THIS IS ALL OF THE AUTO SHIT PLEASE DON'T WRITE AUTO SHIT ANYWHERE ELSE
  public Command getAutonomousCommand() {

    return new PathPlannerAuto("New Auto");

  }


}
