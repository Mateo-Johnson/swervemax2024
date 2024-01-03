// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    //DRIVING PARAMS - MAX CAPABLE SPEEDS NOT MAX ALLOWED SPEEDS
    public static final double maxSpeedMetersPerSecond = 4.8;
    public static final double maxAngularSpeed = 2 * Math.PI; //RADIANS PER SECOND

    public static final double directionSlewRate = 1.2; //PPS
    public static final double magnitudeSlewRate = 1.8; //PERCENT PER SECOND (PPS) (1=100%)
    public static final double rotationalSlewRate = 2.0; //PPS

    // CHASSIS CONFIG
    public static final double trackWidth = Units.inchesToMeters(26.5);
    //DISTANCE BETWEEN CENTERS OF RIGHT AND LEFT WHEELS
    public static final double wheelBase = Units.inchesToMeters(26.5);
    // DISTANCE BETWEEN FRONT AND BACK WHEELS
    public static final SwerveDriveKinematics DriveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, trackWidth / 2),
        new Translation2d(wheelBase / 2, -trackWidth / 2),
        new Translation2d(-wheelBase / 2, trackWidth / 2),
        new Translation2d(-wheelBase / 2, -trackWidth / 2));

    // ANGULAR OFFSETS OF THE MODULES RELATIVES TO THE CHASSIS IN RADIANS
    public static final double frontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double frontRightChassisAngularOffset = 0;
    public static final double backLeftChassisAngularOffset = Math.PI;
    public static final double backRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDS

    //FRONT LEFT MODULE
    public static final int frontLeftDrivingCanId = 11;
    public static final int frontLeftTurningCanId = 10;

    //BACK LEFT MODULE
    public static final int rearLeftDrivingCanId = 13;
    public static final int rearLeftTurningCanId = 12;

    //FRONT RIGHT MODULE
    public static final int frontRightDrivingCanId = 15;
    public static final int frontRightTurningCanId = 14;

    //BACK RIGHT MODULE
    public static final int rearRightDrivingCanId = 17;
    public static final int rearRightTurningCanId = 16;

    //IS THE GYRO REVERSED??????
    public static final boolean gyroReversed = false;

    
  }

  public static final class ModuleConstants {

    public static final int low = 12;
    public static final int medium = 13;
    public static final int high = 14;
    //THIS CAN BE 12, 13 OR 14 CONSULT MECHANICAL BEFORE CHANGING, ASK FOR MODULE PINION TEETH NUMBER (IDEALLY ASK AHMED A.)
    public static final int drivingMotorPinionTeeth = medium;

    //OUTPUT SHAFT ROTATES OPPOSITE OF STEERING MOTOR
    public static final boolean turningEncoderInverted = true;

    //CALCULATIONS FOR DRIVE MOTOR CONVERSION FACTORS AND FEED FORWARD
    public static final double drivingMotorFreeSpeedRps = NeoMotorConstants.freeSpeedRpm / 60;
    public static final double wheelDiameterMeters = 0.0762;
    public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
    //45 TEETH ON BEVEL, 22 TEETH ON FIRST-STAGE SPUR, 15 TEETH ON BEVEL PINION
    public static final double drivingMotorReduction = (45.0 * 22) / (drivingMotorPinionTeeth * 15);
    public static final double driveWheelFreeSpeedRps = (drivingMotorFreeSpeedRps * wheelCircumferenceMeters)
        / drivingMotorReduction;

    public static final double drivingEncoderPositionFactor = (wheelDiameterMeters * Math.PI)
        / drivingMotorReduction; //METERS
    public static final double drivingEncoderVelocityFactor = ((wheelDiameterMeters * Math.PI)
        / drivingMotorReduction) / 60.0; //METERS PER SECOND

    public static final double turningEncoderPositionFactor = (2 * Math.PI); //RADIANS
    public static final double turningEncoderVelocityFactor = (2 * Math.PI) / 60.0; //RADIANS PER SECOND

    public static final double turningEncoderPositionPIDMinInput = 0; //RADIANS
    public static final double turningEncoderPositionPIDMaxInput = turningEncoderPositionFactor; //RADIANS

    public static final double drivingP = 0.04;
    public static final double drivingI = 0;
    public static final double drivingD = 0;
    public static final double drivingFF = 1 / driveWheelFreeSpeedRps;
    public static final double drivingMinOutput = -1;
    public static final double drivingMaxOutput = 1;

    public static final double turningP = 1;
    public static final double turningI = 0;
    public static final double turningD = 0;
    public static final double turningFF = 0;
    public static final double turningMinOutput = -1;
    public static final double turningMaxOutput = 1;

    public static final IdleMode drivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode turningMotorIdleMode = IdleMode.kBrake;

    public static final int drivingMotorCurrentLimit = 50; //AMPS
    public static final int turningMotorCurrentLimit = 20; //AMPS
  }

  public static final class ControllerConstants {
    public static final int driverControllerPort = 0;
    public static final double driveDeadzone = 0.05;
    public static final int driverSecondaryPort = 1;
  }

  public static final class AutoConstants {
    public static final double maxSpeedMetersPerSecond = 3;
    public static final double maxAccelerationMetersPerSecondSquared = 3;
    public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double PXController = 1;
    public static final double PYController = 1;
    public static final double PThetaController = 1;

    //CONSTRAINTS FOR MOTION PROFILED ROBOT ANGLE CONTROLLER
    public static final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
        maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double freeSpeedRpm = 5676;
  }
}
