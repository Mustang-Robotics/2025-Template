package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;

public class Rumble extends Command {
    private final GenericHID m_GenericHID;
    private final double m_rumbleAmount;

    public Rumble(GenericHID genericHID, double amount){
        m_GenericHID = genericHID;
        m_rumbleAmount = amount;
    }

    @Override
    public void initialize(){
        m_GenericHID.setRumble(GenericHID.RumbleType.kBothRumble, m_rumbleAmount);
    }

    @Override
    public void end(boolean interupted){
        m_GenericHID.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }
}
