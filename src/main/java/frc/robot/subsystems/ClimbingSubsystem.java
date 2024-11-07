package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingSubsystem extends SubsystemBase{
    
    private final WPI_TalonSRX climbingMotor = new WPI_TalonSRX(10);

    //DigitalInput toplimitSwitch = new DigitalInput(3);
    //DigitalInput bottomlimitSwitch = new DigitalInput(4);

    public ClimbingSubsystem() {

    }

    public void SetClimbSpeed(double speed) {
        climbingMotor.set(speed);
    }

    /* 
    public boolean ReturnTopLimitSwitch() {
        return toplimitSwitch.get();
    }

    public boolean ReturnBottomLimitSwitch() {
        return bottomlimitSwitch.get();
    }
    */
}
