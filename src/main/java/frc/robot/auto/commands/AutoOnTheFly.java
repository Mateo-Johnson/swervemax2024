// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoOnTheFly extends CommandBase {
  /** Creates a new AutoOnTheFly. */
  public AutoOnTheFly() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //CREATE A LIST OF BEZIER POINTS FROM A POSE (EACH POSE IS A WAYPOINT)
    //ROTATION IS THE DIRECTION OF TRAVEL DO NOT USE HOLONOMIC HERE
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
      new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
      new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
      //YOU CAN ADD MORE BEZIER POINTS, BUT MAKE SURE TO HAVE A COMMA AFTER ALL BUT LAST ONE
    );

    //CREATE PATH USING PLANNED BEZIER POINTS
    PathPlannerPath emergencyAutoPath = new PathPlannerPath(
      bezierPoints,
      new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), //CONSTRAINTS FOR THE PATH
      new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) //GOAL END STATE (YOU CAN USE HOLONOMIC ROTATION HERE)
    );
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
