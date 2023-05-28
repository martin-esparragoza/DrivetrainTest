package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.libswerve.Drivetrain;
import org.firstinspires.ftc.teamcode.libswerve.Pose2d;
import org.firstinspires.ftc.teamcode.libswerve.SwerveModule;
import org.firstinspires.ftc.teamcode.libswerve.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.libswerve.differential.DiffySwerveModule;
import org.firstinspires.ftc.teamcode.modules.MySwerveModule;

@Autonomous(name = "Auto Test", group = "tests")
public final class AutonomousTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Two diffy swerve modules
        DiffySwerveModule m1 = new MySwerveModule(
            4, 0,
            hardwareMap.get(DcMotor.class, "dswerve_m0_0"),
            hardwareMap.get(DcMotor.class, "dswerve_m1_0")
        );
        DiffySwerveModule m2 = new MySwerveModule(
            -4, 0,
            hardwareMap.get(DcMotor.class, "dswerve_m0_1"),
            hardwareMap.get(DcMotor.class, "dswerve_m1_1")
        );

        IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
            new RevHubOrientationOnRobot(new Orientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS,
                0, 0, 0,
                0 // Apparently unused
            ))
        );
        imu.initialize(params);

        Drivetrain drive = new Drivetrain(
            0.2,
            new TwoWheelLocalizer(new Pose2d(0, 0, 0), 4, 4) {
                @Override
                public void updateValues(double dfwdEncoder, double dstrEncoder, double dTheta) {
                    this.dfwdEncoder = dfwdEncoder / 8192.0;
                    this.dstrEncoder = dstrEncoder / 8192.0;
                    this.dTheta = dTheta;
                }
            },
            new SwerveModule[]{m1, m2},
        ) {
            @Override
            public void setModules(Gamepad gamepad) {
                return;
            }
        };

        Thread hardwareFetchThread = new Thread() {
            @Override
            public void run() {
                while (true) {
                    Pose2d pose = drive.localizer.getPoseEstimate();
                    YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
                    System.out.println(angles.getYaw(AngleUnit.RADIANS));
                    ((TwoWheelLocalizer) drive.localizer).updateValues(0, 0, angles.getYaw(AngleUnit.RADIANS) - pose.h);
                }
            }
        };

        waitForStart();

        while (opModeIsActive()) {
            drive.render(FtcDashboard.getInstance());
            telemetry.addData("Heading", drive.localizer.getPoseEstimate().h);
            telemetry.update();
        }
    }
}
