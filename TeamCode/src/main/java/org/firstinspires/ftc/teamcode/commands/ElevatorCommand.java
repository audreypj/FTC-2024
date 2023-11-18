package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.Optional;

public class ElevatorCommand extends CommandBase {

    private final ElevatorSubsystem elevatorSubsystem;

    private double extension;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double extension) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.extension = extension;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorTargetInches(extension);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atTargetElevator();
    }
}
