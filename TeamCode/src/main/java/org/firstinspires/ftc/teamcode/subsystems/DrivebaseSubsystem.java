package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class DrivebaseSubsystem extends SubsystemBase {

    private HardwareMap hMap;
    private Telemetry tele;

    private Pose2d robotPose = new Pose2d();
    private MecanumDriveWheelSpeeds wheelSpeeds;

    private Motor fL,fR,bL,bR;
    private GyroEx gyro;
    private MecanumDrive mecanum;
    private MecanumDriveKinematics kinematics;
    private MecanumDriveOdometry odometry;

    private DoubleSupplier leftY, leftX, rightX;

    //FIXME placeholders. put actual positions once get finalized robot
    private Translation2d[] motorPositions = {new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d()};

    public DrivebaseSubsystem(HardwareMap hMap, Telemetry tele) {
        this.hMap = hMap;
        this.tele = tele;

        fL = new MotorEx(hMap, "fL");
        fR = new MotorEx(hMap, "fR");
        bL = new MotorEx(hMap, "bL");
        bR = new MotorEx (hMap, "bR");

        gyro = new RevIMU(hMap);
        gyro.init();

        mecanum = new MecanumDrive(true, fL, fR, bL, bR);

        leftY = () -> {return 0;};
        leftX = () -> {return 0;};
        rightX = () -> {return 0;};

        kinematics =
                new MecanumDriveKinematics(
                        motorPositions[0],
                        motorPositions[1],
                        motorPositions[2],
                        motorPositions[3]);

        odometry =
                new MecanumDriveOdometry(
                        kinematics,
                        Rotation2d.fromDegrees(0),
                        //FIXME placeholder starting values. have to fix for actual comp
                        new Pose2d(1, 2, Rotation2d.fromDegrees(0)));
    }

    public void driveMotors(DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;
    }

    private void odometryPeriodic() {
        odometry.updateWithTime();


    }

    @Override
    public void periodic() {
        mecanum.driveRobotCentric(leftY.getAsDouble(), leftX.getAsDouble(), rightX.getAsDouble());
    }

}
