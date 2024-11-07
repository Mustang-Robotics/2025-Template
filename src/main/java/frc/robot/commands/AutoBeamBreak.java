package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoBeamBreak extends Command  {
    
    public final IntakeSubsystem m_intakeSubsystem;

    public AutoBeamBreak(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }
    
    @Override
    public boolean isFinished() {
        return m_intakeSubsystem.sensor();
    } 
}
