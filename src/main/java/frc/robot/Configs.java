package frc.robot;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
  public static final class EasySwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI 
        / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;

      double nominalVoltage = 12.0;
      double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50);   // 70 for SparkFlex
      drivingConfig
        .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to adjust them for your own robot!
          .pid(0.02, 0, 0)
          .outputRange(-1, 1)
        .feedForward
          .kV(drivingVelocityFeedForward);

      turningConfig
      
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);   // 70 for SparkFlex
      turningConfig
        .absoluteEncoder
          // Do not invert the turning encoder, since the output shaft rotates in the same
          // direction as the steering motor in the EasySwerve Module.
          .inverted(false)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0) // radians per second
          // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
          .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

      turningConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to adjust them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public final class FloorSubsystem{

    public static final SparkMaxConfig floorMotorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig floorMotorFollowerConfig = new SparkMaxConfig();

    static {
      floorMotorConfig
        .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(Constants.FloorSubsystemConstants.kFloorP)
        .i(Constants.FloorSubsystemConstants.kFloorI)
        .d(Constants.FloorSubsystemConstants.kFloorD);

        floorMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(20); // change stall limit if needed also Coast for free roll
        floorMotorFollowerConfig.apply(floorMotorConfig).follow(Constants.FloorSubsystemConstants.kFloorFollowerMotorCanID, false); // shouldnt be inverted but can change if needed

    }

  }

  public final class IntakeSubsystem{

    public static final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig intakeFollowerMotorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig intakeRackMotorConfig = new SparkMaxConfig();

    static{
      intakeMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(Constants.IntakeSubsystemConstants.kIntakeP)
      .i(Constants.IntakeSubsystemConstants.kIntakeI)
      .d(Constants.IntakeSubsystemConstants.kIntakeD);

      intakeMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(20); 
      intakeFollowerMotorConfig.apply(intakeMotorConfig).follow(Constants.IntakeSubsystemConstants.kIntakeFollowerMotorCanID, false); 


      intakeRackMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(Constants.IntakeSubsystemConstants.kIntakeRackP)
      .i(Constants.IntakeSubsystemConstants.kIntakeRackI)
      .d(Constants.IntakeSubsystemConstants.kIntakeRackD);

      intakeRackMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(20); 

    }


}
}