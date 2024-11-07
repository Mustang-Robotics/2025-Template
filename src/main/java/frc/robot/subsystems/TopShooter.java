package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TopShooter extends PIDSubsystem{

    public static ShuffleboardTab tab = Shuffleboard.getTab("TopShooter");
    //private GenericEntry ks = tab.add("ks", 0).getEntry();
    //private SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(0, ks.getDouble(0));
    public SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(0, .00217);
    //private final Encoder m_intakeEncoder = new Encoder(ShooterConstants.kEncoderPortOne, ShooterConstants.kEncoderPortTwo, ShooterConstants.kEncoderReversed);
    public GenericEntry ks = tab.add("ks", 0).getEntry();
    public final CANSparkMax topMotor = new CANSparkMax(15, MotorType.kBrushless); //One
    //public final CANSparkMax bottomMotor = new CANSparkMax(15, MotorType.kBrushless); //Follows One
    public double shooterCurrentSpeed;

    /* 
    public ShooterSubsystem() {
        
        //bottomMotor.follow(topMotor);
    }*/

    @Override
    public void periodic() {
        if (m_enabled) {
            useOutput(m_controller.calculate(getMeasurement()), getSetpoint());
          }
        // Update the odometry in the periodic block
        //Periodic methods are called every 20 ms by default (The basic update method)
        SmartDashboard.putNumber("TopShooterCurrentSpeed", getMeasurement());

        
        
    }

    // ##### Beginning of Main Functions #####
    public TopShooter() {
        super(
            new PIDController(
            0.001,
            0,
            0));
        getController().setTolerance(ShooterConstants.kShooterToTolleranceRPS);
        setSetpoint(0);
    }

    @Override
    public void useOutput(double output, double setpoint) {
        topMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
        //SmartDashboard.putNumber("useOutput", output + m_shooterFeedforward.calculate(setpoint));
        //bottomMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }
    // ##### End of Main Functions #####


    // ##### Beginning of Returns #####

    @Override
    public double getMeasurement() {
       //return m_intakeEncoder.getRate();
       return topMotor.getEncoder().getVelocity();
    }

    /*public boolean getShooterSpeedInRangeWithinLimit(double currentSpeed, double desiredSpeed, double upperLimit, double lowerLimit) {
        if((currentSpeed <= desiredSpeed + upperLimit) && (currentSpeed >= desiredSpeed - lowerLimit)) {
            return true;
        }else {
            return false;
        }
    }*/
    /*
    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }
    ;*/
    public void runFeeder(double motorSpeed) {
        /* Consistent Low Speed
        if (motorSpeed < 0.1) {
            motorSpeed = 0.1;
        } */
        topMotor.set(motorSpeed);
        //bottomMotor.set(motorSpeed);
        //topMotor.set(shooterCurrentSpeed);
    }

    public void stopFeeder() {
        topMotor.set(0);
        //bottomMotor.set(0);
    }
    // ##### End of Returns #####

    /* ##### Commands List #####
    

    */
    
}
