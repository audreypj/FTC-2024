package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class WristRateCommand extends CommandBase {

    private ElevatorSubsystem elevatorSubsystem;

    private double angle;

    public WristRateCommand(ElevatorSubsystem elevatorSubsystem, double angle) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.angle = angle;

        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setWristTargetAngle(elevatorSubsystem.getWristTargetAngle() + angle);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
