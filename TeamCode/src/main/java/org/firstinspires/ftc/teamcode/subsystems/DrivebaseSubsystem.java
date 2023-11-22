package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Config.SHOW_DEBUG_DATA;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Util;

import java.util.function.DoubleSupplier;

public class DrivebaseSubsystem extends SubsystemBase {

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket packet = new TelemetryPacket();

    private MotorEx fL,fR,bL,bR;
    private RevIMU gyro;

    private MecanumDrive mecanum;
    private MecanumDriveKinematics kinematics;
    private MecanumDriveOdometry odometry;
    private Pose2d robotPose = new Pose2d();
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    private DoubleSupplier leftY, leftX, rightX;

    //FIXME placeholders. put actual positions once get finalized robot
    private final Translation2d[] motorPositions = {new Translation2d(0.17, 0.195), new Translation2d(0.17,-0.195), new Translation2d(-0.115, 0.195), new Translation2d(-0.115, -0.195)};

    public DrivebaseSubsystem(HardwareMap hMap) {

        fL = new MotorEx(hMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new MotorEx(hMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new MotorEx(hMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new MotorEx(hMap, "bR", Motor.GoBILDA.RPM_312);

        fL.resetEncoder();
        fR.resetEncoder();
        bL.resetEncoder();
        bR.resetEncoder();

        gyro = new RevIMU(hMap);
        gyro.init();

        mecanum = new MecanumDrive(true, fL, fR, bL, bR);

        leftY = () -> 0;
        leftX = () -> 0;
        rightX = () -> 0;

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

    //FIXME just a debug method - just to check if motor encoder works
    private double getMotorPos() {
        return fL.getCurrentPosition();
    }

    private MecanumDriveWheelSpeeds calculateWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
                wheelTickToMeter(fL.getCorrectedVelocity()),
                wheelTickToMeter(fR.getCorrectedVelocity()),
                wheelTickToMeter(bL.getCorrectedVelocity()),
                wheelTickToMeter(bR.getCorrectedVelocity()));
    }

    private double wheelTickToMeter(double ticks) {
        return ticks * ((0.096 * Math.PI) / 537.6);
    }

    public Rotation2d getConsistentGyroAngle() {
        return Rotation2d.fromDegrees(Util.normalizeDegrees(-gyro.getAngles()[0])); //FIXME make sure this is correct angle b/c engie flipped hub
    }

    public void zeroGyroscope() {
        gyro.reset();
    }

    public void driveRawJoystick(DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    private void odometryPeriodic() {
        this.robotPose = odometry.updateWithTime(Robot.currentTimestamp(), getConsistentGyroAngle(), calculateWheelSpeeds());
    }

    private void drivePeriodic() {
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

        mecanum.driveWithMotorPowers(
                wheelSpeeds.frontLeftMetersPerSecond,
                wheelSpeeds.frontRightMetersPerSecond,
                wheelSpeeds.rearLeftMetersPerSecond,
                wheelSpeeds.rearRightMetersPerSecond);
    }

    @Override
    public void periodic() {
        drivePeriodic();
        odometryPeriodic();

        if(SHOW_DEBUG_DATA) {
            packet.put("gyro angle", getConsistentGyroAngle());

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
