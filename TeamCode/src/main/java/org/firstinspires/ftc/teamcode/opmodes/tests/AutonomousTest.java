package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.teamcode.libswerve.Drivetrain;
import org.firstinspires.ftc.teamcode.libswerve.Pose2d;
import org.firstinspires.ftc.teamcode.libswerve.SwerveModule;
import org.firstinspires.ftc.teamcode.libswerve.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.libswerve.differential.DiffySwerveModule;
import org.firstinspires.ftc.teamcode.modules.MySwerveModule;

import java.util.concurrent.ExecutorService;

@Autonomous(name = "Auto test", group = "tests")
public class AutonomousTest extends LinearOpMode {
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
            new SwerveModule[]{m1, m2}
        ) {
            @Override
            public void drive(Gamepad gamepad) {
                return;
            }
        };

        DcMotor fwdEncoder = hardwareMap.get(DcMotor.class, "fwdEncoder");
        DcMotor strEncoder = hardwareMap.get(DcMotor.class, "strEncoder");

        waitForStart();

        while (opModeIsActive()) {
            drive.render(FtcDashboard.getInstance());
            System.out.println(fwdEncoder.getCurrentPosition());
        }
    }
}
