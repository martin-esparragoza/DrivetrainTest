package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libswerve.SwerveModule;
import org.firstinspires.ftc.teamcode.libswerve.Util;
import org.firstinspires.ftc.teamcode.modules.MyDiffySwerveModule;

@Config
@Autonomous(name = "Swerve PID Tuner", group = "tests")
public class SwervePIDTuner extends LinearOpMode {
    public static double angle = 0;
    public static double fwd = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        DcMotor dswerve_m0_0 = hardwareMap.get(DcMotor.class, "dswerve_m0_0");
        DcMotor dswerve_m1_0 = hardwareMap.get(DcMotor.class, "dswerve_m1_0");
        SwerveModule m1 = new MyDiffySwerveModule(
            0, 0,
            dswerve_m0_0,
            dswerve_m1_0
        );

        waitForStart();
        while (opModeIsActive()) {
            m1.fwdPower = fwd;
            telemetry.addData("Angle (deg)", Math.toDegrees(Util.clampAngle(m1.getAngle())));
            telemetry.addData("Target (deg)", Math.toDegrees(Util.clampAngle(m1.getTargetAngle())));
            telemetry.addData("Error", Math.toDegrees(Util.clampAngle(m1.getTargetAngle() - m1.getAngle())));
            telemetry.addData("Rightpwr", dswerve_m0_0.getPower());
            telemetry.addData("Leftpwr", dswerve_m1_0.getPower());
            telemetry.update();

            m1.setTargetAngle(angle);

            m1.update();
        }
    }
}
