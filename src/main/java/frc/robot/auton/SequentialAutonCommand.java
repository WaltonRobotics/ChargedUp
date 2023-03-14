package frc.robot.auton;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SequentialAutonCommand extends CommandBase {

    private Set<CommandBase> m_commandsToUse = new HashSet<>();

    /**
     * PREREQUISITES: commands must be in order
     */
    public SequentialAutonCommand(Set<CommandBase> commands) {
        m_commandsToUse = commands;
    }

    public void addNewCommand(CommandBase newCommand) {
        m_commandsToUse.add(newCommand);
    }

   
    // @Override
    // public Set<Subsystem> getRequirements() {
    //     // TODO Auto-generated method stub
    //     return null;
    // } 
}
