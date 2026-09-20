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

    private final SparkMax botIndexerMotor = new SparkMax(Constants.ShooterSubsystemConstants.kbotIndexerMotorCanID, SparkFlex.MotorType.kBrushless);
    private RelativeEncoder botIndexerEncoder = botIndexerMotor.getEncoder();
    private SparkClosedLoopController botIndexerController = botIndexerMotor.getClosedLoopController();

    private final SparkMax botIndexerFollowerMotor = new SparkMax(Constants.ShooterSubsystemConstants.kbotFollowerIndexerMotorCanID, SparkFlex.MotorType.kBrushless);


    public ShooterSubsystem(){
        flywheelMotor.configure(Configs.ShooterSubsystem.flywheelMotorConfig, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);
        flywheelFollowerMotor.configure(Configs.ShooterSubsystem.flywheelFollowerMotorConfig, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);

        topIndexerMotor.configure(Configs.ShooterSubsystem.topIndexerMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        topIndexerFollowerMotor.configure(Configs.ShooterSubsystem.topIndexerFollowerMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        botIndexerMotor.configure(Configs.ShooterSubsystem.botIndexerMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        botIndexerFollowerMotor.configure(Configs.ShooterSubsystem.botIndexerFollowerMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        flywheelEncoder.setPosition(0);
        topIndexerEncoder.setPosition(0);
        botIndexerEncoder.setPosition(0);


    }

    //All three are velocity controlled 
    public void setShooterVelocity(double f, double ti, double bi){

        flywheelController.setSetpoint(f, SparkFlex.ControlType.kDutyCycle);
        topIndexerController.setSetpoint(ti, SparkMax.ControlType.kDutyCycle);
        botIndexerController.setSetpoint(bi, SparkMax.ControlType.kDutyCycle);

    }

    public Command shoot(){
        return this.startEnd( 
            () -> {
            this.setShooterVelocity(ShooterSubsystemSetPoints.kFlywheelShoot,ShooterSubsystemSetPoints.ktopIndex, ShooterSubsystemSetPoints.kbotIndex);
        }, () -> {
            this.setShooterVelocity(0,0,0);
        });
    }

    @Override
     public void periodic() {
        SmartDashboard.putNumber("Shooter | Bot Index Speed", botIndexerEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter | Top Index Speed", topIndexerEncoder.getVelocity());    
        SmartDashboard.putNumber("Shooter | FLYWHEEL SPEED", flywheelEncoder.getVelocity());
     }

    
}
