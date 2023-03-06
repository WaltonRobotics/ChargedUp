package frc.robot.auton;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SequentialAutonCommand extends CommandBase implements Command {

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
