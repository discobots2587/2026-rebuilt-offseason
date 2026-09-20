package frc.robot.subsystems;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FloorSubsystemConstants;
import frc.robot.Constants.ShooterSubsystemConstants;
import frc.robot.Constants.ShooterSubsystemConstants.ShooterSubsystemSetPoints;
import frc.robot.Constants.FloorSubsystemConstants.FloorMotorSetPoints;
import frc.robot.Configs;
import frc.robot.Constants;

public class FloorSubsystem extends SubsystemBase {


    private final SparkMax floorMotor  = new SparkMax(FloorSubsystemConstants.kFloorMotorCanID, SparkMax.MotorType.kBrushless);;
    private RelativeEncoder floorEncoder = floorMotor.getEncoder();
    private SparkClosedLoopController floorController = floorMotor.getClosedLoopController();
    
    private final SparkMax floorFollowerMotor = new SparkMax(FloorSubsystemConstants.kFloorFollowerMotorCanID, SparkMax.MotorType.kBrushless);

    private final SparkMax botIndexerMotor = new SparkMax(Constants.ShooterSubsystemConstants.kbotIndexerMotorCanID, SparkFlex.MotorType.kBrushless);
    private RelativeEncoder botIndexerEncoder = botIndexerMotor.getEncoder();//Move to Floor
    private SparkClosedLoopController botIndexerController = botIndexerMotor.getClosedLoopController(); //Move to Floor

    private final SparkMax botIndexerFollowerMotor = new SparkMax(Constants.ShooterSubsystemConstants.kbotFollowerIndexerMotorCanID, SparkFlex.MotorType.kBrushless);//Move To Floor

  public FloorSubsystem() {

    floorMotor.configure(Configs.FloorSubsystem.floorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    floorFollowerMotor.configure(Configs.FloorSubsystem.floorMotorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    floorEncoder.setPosition(0);

    botIndexerMotor.configure(Configs.ShooterSubsystem.botIndexerMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    botIndexerFollowerMotor.configure(Configs.ShooterSubsystem.botIndexerFollowerMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

    botIndexerEncoder.setPosition(0);

  }

  public void setFloorVelocity(double v){ //v is for velocity in RPM
    floorController.setSetpoint(v, ControlType.kDutyCycle); 
  }

  public void setBotIndexVelocity(double v){
    botIndexerController.setSetpoint(v, ControlType.kDutyCycle);
  }

  public Command feedShooter(){

    return this.startEnd( 
        () -> {
        this.setBotIndexVelocity(ShooterSubsystemSetPoints.kbotIndex);
    }, () ->{
        this.setBotIndexVelocity(0);
        });
    }

  public Command unJamINDX(){

    return this.startEnd( 
        () -> {
        this.setBotIndexVelocity(-.25);
    }, () ->{
        this.setBotIndexVelocity(0);
        });
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("INDEXER | Bot Index Speed", botIndexerEncoder.getVelocity()); //Move to Floor
    SmartDashboard.putNumber("Floor | Floor | Velocity", floorEncoder.getVelocity());
    
    SmartDashboard.putBoolean("INDEXER | INDEXER ON", botIndexerEncoder.getVelocity() > 2);

}
} 

