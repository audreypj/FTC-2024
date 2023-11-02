package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

public class RobotContainer {

    private final GamepadEx brandon = new GamepadEx(gamepad1);

    private final DrivebaseSubsystem drivebaseSubsystem = new DrivebaseSubsystem(hardwareMap);

    public RobotContainer() {

        CommandScheduler.getInstance().registerSubsystem(drivebaseSubsystem);

        CommandScheduler.getInstance().setDefaultCommand(
                drivebaseSubsystem,
                new DefaultDriveCommand(
                        drivebaseSubsystem,
                        brandon::getLeftY,
                        brandon::getLeftX,
                        brandon::getRightX));

    }

}
