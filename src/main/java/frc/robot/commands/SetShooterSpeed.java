package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BottomShooter;
import frc.robot.subsystems.TopShooter;

public class SetShooterSpeed extends Command{
    
    public final double m_startSpeed;
    public final TopShooter m_top;
    public final BottomShooter m_bottom;
    public final double difference;
    public final double m_tol;

    public SetShooterSpeed(TopShooter top, BottomShooter bottom, double startSpeed, double diff, double tol) {
        m_startSpeed = startSpeed;
        m_top = top;
        m_bottom = bottom;
        difference = diff;
        m_tol = tol;
        addRequirements(m_top);
        addRequirements(m_bottom);

    }
    
    
    @Override
    public void initialize() {
        //m_shooterSubsystem.runFeeder(m_startSpeed); 
        m_top.getController().setTolerance(m_tol);
        m_bottom.getController().setTolerance(m_tol);
        m_top.setSetpoint(m_startSpeed);//2800 = 50%
        m_bottom.setSetpoint(m_startSpeed - difference);
    }
    
    /*Moved this to where it is getting used in code Using ShootSpeed(m_startSpeed: 0) in the end Override
    //@Override
    //public void end(boolean interupted) { //On interrupt, set speed to 0. 
        //m_top.runFeeder(0);
        //m_bottom.runFeeder(0);
    }*/
    
    @Override
    public boolean isFinished() {
        //return true;
        if(m_startSpeed == 0) {return true;}
        else {return m_top.atSetpoint() && m_bottom.atSetpoint();}
    }
}
