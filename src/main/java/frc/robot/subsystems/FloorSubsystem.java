package frc.robot.subsystems;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FloorSubsystemConstants;
import frc.robot.Constants.FloorSubsystemConstants.FloorMotorSetPoints;
import frc.robot.Configs;

public class FloorSubsystem extends SubsystemBase {


    private final SparkMax floorMotor  = new SparkMax(FloorSubsystemConstants.kFloorMotorCanID, SparkMax.MotorType.kBrushless);;
    private RelativeEncoder floorEncoder = floorMotor.getEncoder();
    private SparkClosedLoopController floorController = floorMotor.getClosedLoopController();
    
    private final SparkMax floorFollowerMotor = new SparkMax(FloorSubsystemConstants.kFloorFollowerMotorCanID, SparkMax.MotorType.kBrushless);
  public FloorSubsystem() {

    floorMotor.configure(Configs.FloorSubsystem.floorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    floorFollowerMotor.configure(Configs.FloorSubsystem.floorMotorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    floorEncoder.setPosition(0);

  }

  public void setFloorVelocity(double v){ //v is for velocity in RPM
    floorController.setSetpoint(v, ControlType.kVelocity); 
  }

  public Command feedShooter(){

    return this.startEnd( 
        () -> {
        this.setFloorVelocity(FloorMotorSetPoints.kFloorMotorFeed);
    }, () ->{
        this.setFloorVelocity(0);
        });
    }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Floor | Floor | Velocity", floorEncoder.getVelocity());

}
} 

