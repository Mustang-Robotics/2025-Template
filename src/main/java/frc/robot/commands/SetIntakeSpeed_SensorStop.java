package frc.robot.commands;
//import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeSpeed_SensorStop extends Command {
    
    public final double m_startSpeed;
    public final IntakeSubsystem m_intakeSubsystem;

    public SetIntakeSpeed_SensorStop(IntakeSubsystem intakeSubsystem, double startSpeed) {
        m_startSpeed = startSpeed;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }
    
    
    @Override
    public void execute() {
        m_intakeSubsystem.setSpeed(m_startSpeed); //Robot intake starting speed
    }
    
    @Override
    public void end(boolean interupted) {
        m_intakeSubsystem.setSpeed(0); //Robot intake ending speed
    }
    
    @Override
    public boolean isFinished() {
        return !m_intakeSubsystem.sensor();
    }
}
