package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeSpeed extends Command{
    
    public final double m_startSpeed;
    public final IntakeSubsystem m_intakeSubsystem;

    public SetIntakeSpeed(IntakeSubsystem intakeSubsystem, double startSpeed) {
        m_startSpeed = startSpeed;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }
    
    
    @Override
    public void initialize() {
        m_intakeSubsystem.setSpeed(m_startSpeed); //Robot intake starting speed
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
