package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class WristCommand extends CommandBase {

    private ElevatorSubsystem elevatorSubsystem;
    private double targetAngle;

    public WristCommand(ElevatorSubsystem elevatorSubsystem, double targetAngle) {
        this.elevatorSubsystem = elevatorSubsystem;

        this.targetAngle = targetAngle;

        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

}
