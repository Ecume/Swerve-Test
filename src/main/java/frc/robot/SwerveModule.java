// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
      this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;
      absoluteEncoder = new CANcoder(absoluteEncoderId);

      driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
      turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

      driveMotor.setInverted(driveMotorReversed);
      turningMotor.setInverted(turningMotorReversed);

      driveEncoder = driveMotor.getEncoder();
      turningEncoder = driveMotor.getEncoder();

      driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
      driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
      turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
      turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

      turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
      turningPidController.enableContinuousInput(-Math.PI, Math.PI);

      resetEncoders();



    }

    public double getDrivePosition(){
      return driveEncoder.getPosition();
    }

    public double getTurningPosition(){
      return turningEncoder.getPosition();
    }

    public double getDriveVelocity(){
      return driveEncoder.getVelocity();
    }

    public double getTurningVelocity(){
      return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRadRaw(){
      double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
      return angle;
    }

    public double getAbsoluteEncoderRad() {
      double angle = this.getAbsoluteEncoderRadRaw();
      angle -= absoluteEncoderOffsetRad;
      angle = angle * (absoluteEncoderReversed ? -1.0 : 1.0);

      if (angle > 2 * Math.PI) {
        angle -= 2 * Math.PI;
      }
      else if (angle < 0){
        angle += 2 * Math.PI;
      }
      return angle;

    }

    public void resetEncoders() {
      driveEncoder.setPosition(0);
      turningEncoder.setPosition(getAbsoluteEncoderRad());

    }

    public SwerveModuleState getState() {
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void stop() {
      driveMotor.set(0);
      turningMotor.set(0);
  }

    public void setDesiredState(SwerveModuleState state){
      if (Math.abs(state.speedMetersPerSecond) < 0.001) {
        stop();
        return;
    }
      state = SwerveModuleState.optimize(state, getState().angle);
      driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
      turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }



    
  }

