// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;

public class SwerveJoystickCmd extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  /** Creates a new SwerveJoystickCmd. */
  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
          Supplier<Double> xSpdFuncion, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
          Supplier<Boolean> fieldOrientedFunction){
      this.swerveSubsystem = swerveSubsystem;
      this.xSpdFunction = xSpdFuncion;
      this.ySpdFunction = ySpdFunction;
      this.turningSpdFunction = turningSpdFunction;
      this.fieldOrientedFunction = fieldOrientedFunction;
      this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
      addRequirements(swerveSubsystem);
  }

   {
          
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //getting the real time joystick inputs using the suppliers
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    //applying the deadband, if joystick isn't centered to zero, ignore and small inputs to keep motors safe
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadBand ? turningSpeed : 0.0;

    //convert speed into m/s and rad/s but scaling them a quater of the max speed so we can control the robot better
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed);
    

    ChassisSpeeds chassisSpeeds;
    //if driver wants to operate reletive to field
    if (fieldOrientedFunction.get()){
      //Relative to fild

      //param: xspeed, yspeed, angular rate of movement, robot heading angle in radians from the gyroscope
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    }
    else {
      //Relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }



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
