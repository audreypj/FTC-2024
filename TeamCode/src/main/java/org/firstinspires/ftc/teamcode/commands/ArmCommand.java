package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private ArmSubsystem.ArmState armState;

    public ArmCommand(ArmSubsystem armSubsystem, ArmSubsystem.ArmState armState) {
        this.armSubsystem = armSubsystem;
        this.armState = armState;

        addRequirements(this.armSubsystem);
    }

    public ArmCommand(ArmSubsystem armSubsystem, double angle) {
        this(armSubsystem, new ArmSubsystem.ArmState(angle));
    }

    @Override
    public void initialize() {
        armSubsystem.setArmState(armState);
    }
}
