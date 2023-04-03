package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libswerve.SwerveModule;
import org.firstinspires.ftc.teamcode.libswerve.differential.DiffySwerveModule;

class TestSwerveModule extends DiffySwerveModule {
    public TestSwerveModule(double x, double y, DcMotor right, DcMotor left) {
        super(x, y, right, left);
    }

    @Override
    public double getAngle() {
        return (((right.getCurrentPosition() / 537.6) * (2 * Math.PI)) + ((left.getCurrentPosition() / 537.6) * (2 * Math.PI))) / (90.0 / 32.0) / 2;
    }
}

@Autonomous(name = "Swerve Angle Test", group = "tests")
public class SwerveAngleTest extends LinearOpMode {
    private SwerveModule m1;

    @Override
    public void runOpMode() throws InterruptedException {
        m1 = new TestSwerveModule(
            0, 0,
            hardwareMap.get(DcMotor.class, "dswerve_m0_0"),
            hardwareMap.get(DcMotor.class, "dswerve_m1_0")
        );

        waitForStart();
        while (opModeIsActive()) {
            m1.setPowers(0, 0.5);
            System.out.println(Math.toDegrees(m1.getAngle()));
            sleep(100);
            idle();
        }
    }
}
