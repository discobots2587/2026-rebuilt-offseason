package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;



import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ShooterSubsystemConstants.ShooterSubsystemSetPoints;

public class ShooterSubsystem extends SubsystemBase {


    private final SparkFlex flywheelMotor = new SparkFlex(Constants.ShooterSubsystemConstants.kflywheelMotorCanID, SparkFlex.MotorType.kBrushless);
    private RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();
    private SparkClosedLoopController flywheelController = flywheelMotor.getClosedLoopController();

    private final SparkFlex flywheelFollowerMotor = new SparkFlex(Constants.ShooterSubsystemConstants.kflywheelFollowerMotorCanID, SparkFlex.MotorType.kBrushless);

    private final SparkMax topIndexerMotor = new SparkMax(Constants.ShooterSubsystemConstants.ktopIndexerMotorCanID, SparkFlex.MotorType.kBrushless);
    private RelativeEncoder topIndexerEncoder = topIndexerMotor.getEncoder();
    private SparkClosedLoopController topIndexerController = topIndexerMotor.getClosedLoopController();

    private final SparkMax topIndexerFollowerMotor = new SparkMax(Constants.ShooterSubsystemConstants.ktopFollowerMotorIndexerCanID, SparkFlex.MotorType.kBrushless);


    public ShooterSubsystem(){
        flywheelMotor.configure(Configs.ShooterSubsystem.flywheelMotorConfig, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);
        flywheelFollowerMotor.configure(Configs.ShooterSubsystem.flywheelFollowerMotorConfig, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);

        topIndexerMotor.configure(Configs.ShooterSubsystem.topIndexerMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        topIndexerFollowerMotor.configure(Configs.ShooterSubsystem.topIndexerFollowerMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        flywheelEncoder.setPosition(0);
        topIndexerEncoder.setPosition(0);
      


    }

    //All three are velocity controlled 
    public void setShooterVelocity(double f, double ti){

        flywheelController.setSetpoint(f, SparkFlex.ControlType.kVelocity);
        topIndexerController.setSetpoint(ti, SparkMax.ControlType.kVelocity);
      //  botIndexerController.setSetpoint(bi, SparkMax.ControlType.kDutyCycle);//Move to Floor

    }

    public Command shoot(){
        return this.startEnd( 
            () -> {
            this.setShooterVelocity(ShooterSubsystemSetPoints.kFlywheelShoot,ShooterSubsystemSetPoints.ktopIndex);
        }, () -> {
            this.setShooterVelocity(0,0);
        });
    }

    public Command stopShoot(){
        return this.startEnd( 
            () -> {
            this.setShooterVelocity(0,0);
        }, () -> {
            this.setShooterVelocity(0,0);
        });
    }

    public Command unjamST(){
        return this.startEnd( 
            () -> {
            this.setShooterVelocity(-1000,.1000);
        }, () -> {
            this.setShooterVelocity(0,0);
        });
    }
    @Override
     public void periodic() {
        SmartDashboard.putNumber("Shooter | Top Index Speed", topIndexerEncoder.getVelocity());    
        SmartDashboard.putNumber("Shooter | FLYWHEEL SPEED", flywheelEncoder.getVelocity());
        SmartDashboard.putBoolean("Shooter | FLYWHEEL ON", flywheelEncoder.getVelocity() > 2);
    
     }

    
}
