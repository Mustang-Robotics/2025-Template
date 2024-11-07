
//SUMMARY NOTES: Declares the modules relating to the motors. (One Motor)
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class IntakeSubsystem extends SubsystemBase {


    private final DigitalOutput sensorLight = new DigitalOutput(2);
    private final DigitalInput sensor = new DigitalInput(1);
    private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(2);
     

    // ##### Beginning of Main Functions #####
    public IntakeSubsystem() {
        sensorLight.set(true);
        
    }

    public void setSpeed(double motorSpeed) { 

            //SmartDashboard.putBoolean("taco", sensor.get());
            intakeMotor.set(motorSpeed);
        }

    public boolean sensor() {
        SmartDashboard.putBoolean("taco", sensor.get());
        return sensor.get();
        
    }    
       
        
}
    // ##### End of Main Functions #####


    // ##### Beginning of Returns #####
    // ##### End of Returns #####

    /* ##### Commands List #####
    

    */

