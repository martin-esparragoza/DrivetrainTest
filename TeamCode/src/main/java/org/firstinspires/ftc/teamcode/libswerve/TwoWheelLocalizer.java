package org.firstinspires.ftc.teamcode.libswerve;

// Shamelessly copied from pj!
public class TwoWheelLocalizer implements Localizer {
    private Pose2d p;
    private final double fwdy;
    private final double strx;
    // These values should be updated by another class
    public double dfwdEncoder, dstrEncoder, dtheta;

    public TwoWheelLocalizer(Pose2d p, double fwdy, double strx) {
        this.fwdy = fwdy;
        this.strx = strx;
        this.p = p;
    }

    @Override
    public Pose2d getPoseEstimate() {
        return p;
    }

    @Override
    public void update() {
        // In this case we should always use deltas
        double dfwd = dfwdEncoder - fwdy * dtheta;
        double dstr = dstrEncoder - strx * dtheta;
        p.h += dtheta;

        /*// Convert it all from relative to global ( this code is for without constant velocity arcs)
        p.x += dfwd * Math.cos(p.h) - dstr * Math.sin(p.h);
        p.y += dstr * Math.cos(p.h) + dfwd * Math.sin(p.h);*/

        // Heres the version with constant velocity arcs
        double r0 = dfwd / dtheta;
        double r1 = dstr / dtheta;

        double reldx = r0 * Math.sin(dtheta) - r1 * (1 - Math.cos(dtheta));
        double reldy = r1 * Math.sin(dtheta) + r0 * (1 - Math.cos(dtheta));

        // Now just convert relative to global
        p.x += reldx * Math.cos(p.h) - reldy * Math.sin(p.h);
        p.y += reldy * Math.cos(p.h) - reldx * Math.sin(p.h);
    }

    @Override
    public void setPoseEstimate(Pose2d p) {
        this.p = p;
    }
}
