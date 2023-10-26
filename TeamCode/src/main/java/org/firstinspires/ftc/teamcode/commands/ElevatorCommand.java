package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase {

    private ElevatorSubsystem elevatorSubsystem;

    private double targetHeight;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double targetHeight) {

        this.elevatorSubsystem = elevatorSubsystem;

        this.targetHeight = targetHeight;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorTargetInches(targetHeight);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atTarget();
    }
}
