package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

public class RobotContainer {

    private final GamepadEx brandon = new GamepadEx(gamepad1);

    private final DrivebaseSubsystem drivebaseSubsystem = new DrivebaseSubsystem(hardwareMap, telemetry);

    public RobotContainer() {

        CommandScheduler.getInstance().registerSubsystem(drivebaseSubsystem);

        CommandScheduler.getInstance().setDefaultCommand(
                drivebaseSubsystem,
                new DrivebaseCommand(
                        drivebaseSubsystem,
                        brandon::getLeftY,
                        brandon::getLeftX,
                        brandon::getRightX));

    }

}
