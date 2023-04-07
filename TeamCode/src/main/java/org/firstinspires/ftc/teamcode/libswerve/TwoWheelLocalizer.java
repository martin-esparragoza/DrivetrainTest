package org.firstinspires.ftc.teamcode.libswerve;

public final class TwoWheelLocalizer implements Localizer {
    private Pose2d p;

    public TwoWheelLocalizer(Pose2d p) {
        this.p = p;
    }

    @Override
    public Pose2d getPoseEstimate() {
        return null;
    }

    @Override
    public void update() {
    }

    @Override
    public void setPoseEstimate() {
        return;
    }
}
