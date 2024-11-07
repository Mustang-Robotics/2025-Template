package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetArm extends Command{
    private final Arm m_arm;
    private double m_setpoint;

    public SetArm(Arm arm, double setpoint) {
        m_setpoint = setpoint;
        m_arm = arm;
        addRequirements(m_arm);
    }


    @Override
    public void initialize() {
        m_arm.setGoal(m_setpoint);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
