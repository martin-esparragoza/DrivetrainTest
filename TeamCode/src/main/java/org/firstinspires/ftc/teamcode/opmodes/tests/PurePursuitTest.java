package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libswerve.Drivetrain;
import org.firstinspires.ftc.teamcode.libswerve.OpmodeFunctions;
import org.firstinspires.ftc.teamcode.libswerve.Path;
import org.firstinspires.ftc.teamcode.libswerve.Pose2d;
import org.firstinspires.ftc.teamcode.libswerve.SwerveModule;
import org.firstinspires.ftc.teamcode.libswerve.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.libswerve.differential.DiffySwerveModule;
import org.firstinspires.ftc.teamcode.modules.MySwerveModule;

@Autonomous
public final class PurePursuitTest extends LinearOpMode {
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
            0, 0,
            new TwoWheelLocalizer(new Pose2d(0.0, 0.0, 0.0)),
            new SwerveModule[]{m1, m2},
            0.1,
            0.1
        );

        waitForStart();

        Path p = new Path();
        p.threshold = 1.2;
        p.addSpline(drive.localizer.getPoseEstimate(), new Pose2d(20, 20, 1), 1.2, 3);
        drive.executePurePursuit(
            p,
            new OpmodeFunctions() {
                @Override
                public boolean check() {
                    return opModeIsActive();
                }

                @Override
                public void update() {
                    drive.update();
                    drive.localizer.update();
                }
            }
        );
        drive.brake();
    }
}
