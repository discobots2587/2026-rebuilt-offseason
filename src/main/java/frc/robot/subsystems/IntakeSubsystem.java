package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;



import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;





public class IntakeSubsystem extends SubsystemBase {

     private final SparkMax intakeMotor = new SparkMax(Constants.IntakeSubsystemConstants.kIntakeMotorCanID, SparkMax.MotorType.kBrushless);
     private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
     private SparkClosedLoopController intakeController = intakeMotor.getClosedLoopController();

     private final SparkMax intakeFollowerMotor = new SparkMax(Constants.IntakeSubsystemConstants.kIntakeFollowerMotorCanID, SparkMax.MotorType.kBrushless);

     private final SparkMax intakeRackMotor = new SparkMax(Constants.IntakeSubsystemConstants.kIntakeRackMotorCanID, SparkMax.MotorType.kBrushless);
     private RelativeEncoder intakeRackEncoder = intakeRackMotor.getEncoder();
     private SparkClosedLoopController intakeRackController = intakeRackMotor.getClosedLoopController();



    public IntakeSubsystem() {
       intakeMotor.configure(Configs.IntakeSubsystem.intakeMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
       intakeFollowerMotor.configure(Configs.IntakeSubsystem.intakeFollowerMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
       intakeRackMotor.configure(Configs.IntakeSubsystem.intakeRackMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

       intakeEncoder.setPosition(0);
       //the intake rack will be using an absolute encoder 

        
    }


    //this is for intake rollers
    public void setIntakeVelocity(double v){ //v is for velocity in RPM
        intakeController.setSetpoint(v, SparkMax.ControlType.kVelocity); //velocity controlled
    }

    public Command feedIntake(){
        return this.startEnd( () -> {
            this.setIntakeVelocity(Constants.IntakeSubsystemConstants.IntakeMotorSetPoints.kIntakeFeed);
        }, () -> {
            this.setIntakeVelocity(0);
        });
    }

     public Command outIntake(){
        return this.startEnd( () -> {
            this.setIntakeVelocity(Constants.IntakeSubsystemConstants.IntakeMotorSetPoints.kIntakeOut);
        }, () -> {
            this.setIntakeVelocity(0);
        });
    }

    //this is for intake rack

    public void setIntakeRackPosition(double p){ //p is for position in rotations
        intakeRackController.setSetpoint(p, ControlType.kPosition); //position controlled based on rotations

    }

    public Command outIntakeRack(){
        return this.runOnce(() -> setIntakeRackPosition(Constants.IntakeSubsystemConstants.IntakeMotorSetPoints.kIntakeRackOut));
    }

    public Command inIntakeRack(){
        return this.runOnce(() -> setIntakeRackPosition(Constants.IntakeSubsystemConstants.IntakeMotorSetPoints.kIntakeRackIn)); //should always be 0 but change if different
    }

    

     @Override
     public void periodic() {
        SmartDashboard.putNumber("Intake | Intake | Velocity",intakeEncoder.getVelocity());
        
     }
    
}
    