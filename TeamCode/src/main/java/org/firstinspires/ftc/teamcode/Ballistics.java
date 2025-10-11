package org.firstinspires.ftc.teamcode;

public final class Ballistics {
    private static final double G   = 9.81;   // m/s^2
    private static final double RHO = 1.225;  // kg/m^3 (air density)

    private Ballistics(){}

    // ----- Core (no drag) -----
    public static double wheelRPM(double motorRPM, double G_ratio) { return motorRPM / G_ratio; }

    public static double exitSpeed(double wheelRPM, double r_m, double eta) {
        double wheelOmega = (2.0 * Math.PI / 60.0) * wheelRPM; // rad/s
        return eta * wheelOmega * r_m; // m/s
    }

    // d = v cosθ * ( v sinθ + sqrt((v sinθ)^2 + 2 g h) ) / g
    public static double rangeNoDrag(double v, double thetaRad, double h) {
        double vsin = v * Math.sin(thetaRad), vcos = v * Math.cos(thetaRad);
        double disc = vsin * vsin + 2.0 * G * h;
        if (disc < 0) return Double.NaN;
        double t = (vsin + Math.sqrt(disc)) / G;
        return vcos * t;
    }

    public static double distanceFromMotorRPM(
            double motorRPM, double G_ratio, double wheelRadius_m, double slipEta,
            double hoodAngle_deg, double exitHeight_m, double goalHeight_m
    ) {
        double wRPM  = wheelRPM(motorRPM, G_ratio);
        double v     = exitSpeed(wRPM, wheelRadius_m, slipEta);
        double theta = Math.toRadians(hoodAngle_deg);
        double h     = goalHeight_m - exitHeight_m;
        return rangeNoDrag(v, theta, h);
    }

    // v = sqrt( g d^2 / ( 2 cos^2θ (d tanθ - h) ) )
    public static double speedForRangeNoDrag(double d, double thetaRad, double h) {
        double denom = 2.0 * Math.pow(Math.cos(thetaRad), 2) * (d * Math.tan(thetaRad) - h);
        if (denom <= 0) return Double.NaN;
        return Math.sqrt((G * d * d) / denom);
    }

    // Back-compat alias
    public static double speedForRange(double d, double thetaRad, double h) {
        return speedForRangeNoDrag(d, thetaRad, h);
    }

    public static double motorRPMForDistance(
            double distance_m, double G_ratio, double wheelRadius_m, double slipEta,
            double hoodAngle_deg, double exitHeight_m, double goalHeight_m
    ) {
        double theta = Math.toRadians(hoodAngle_deg);
        double h     = goalHeight_m - exitHeight_m;
        double v     = speedForRangeNoDrag(distance_m, theta, h);
        if (Double.isNaN(v)) return Double.NaN;
        double wheelRPM = (60.0 / (2.0 * Math.PI)) * (v / (slipEta * wheelRadius_m));
        return wheelRPM * G_ratio;
    }

    // ----- Drag model (quadratic drag, no lift) -----
    public static double rangeWithDrag(double v0, double theta, double y0, double yGoal,
                                       double massKg, double diameter, double Cd) {
        double x = 0.0, y = y0;
        double vx = v0 * Math.cos(theta);
        double vy = v0 * Math.sin(theta);

        double area = Math.PI * Math.pow(diameter * 0.5, 2);
        double dt = 0.002; // 2 ms
        double lastX = x, lastY = y;

        for (int i = 0; i < 20000; i++) {
            double v = Math.hypot(vx, vy);
            double axDrag = (v > 1e-6) ? (-0.5 * RHO * Cd * area * v * vx) / massKg : 0.0;
            double ayDrag = (v > 1e-6) ? (-0.5 * RHO * Cd * area * v * vy) / massKg : 0.0;

            double ax = axDrag;
            double ay = ayDrag - G;

            vx += ax * dt;
            vy += ay * dt;

            lastX = x; lastY = y;
            x += vx * dt;
            y += vy * dt;

            if (vx < 0 && x > 0.1) break;

            if ((lastY - yGoal) * (y - yGoal) <= 0 && vy < 0) {
                double t = (yGoal - lastY) / (y - lastY + 1e-9);
                return lastX + t * (x - lastX);
            }
            if (y < -5) break;
        }
        return Double.NaN;
    }

    public static double distanceFromMotorRPMWithDrag(
            double motorRPM, double G_ratio, double wheelRadius_m, double slipEta,
            double hoodAngle_deg, double exitHeight_m, double goalHeight_m,
            double massKg, double diameter_m, double Cd
    ) {
        double wRPM  = wheelRPM(motorRPM, G_ratio);
        double v0    = exitSpeed(wRPM, wheelRadius_m, slipEta);
        double theta = Math.toRadians(hoodAngle_deg);
        return rangeWithDrag(v0, theta, exitHeight_m, goalHeight_m, massKg, diameter_m, Cd);
    }

    public static double motorRPMForDistanceWithDrag(
            double distance_m, double G_ratio, double wheelRadius_m, double slipEta,
            double hoodAngle_deg, double exitHeight_m, double goalHeight_m,
            double massKg, double diameter_m, double Cd,
            double minRPM, double maxRPM
    ) {
        double dMin = distanceFromMotorRPMWithDrag(minRPM, G_ratio, wheelRadius_m, slipEta,
                hoodAngle_deg, exitHeight_m, goalHeight_m, massKg, diameter_m, Cd);
        double dMax = distanceFromMotorRPMWithDrag(maxRPM, G_ratio, wheelRadius_m, slipEta,
                hoodAngle_deg, exitHeight_m, goalHeight_m, massKg, diameter_m, Cd);

        if (Double.isNaN(dMin) && Double.isNaN(dMax)) return Double.NaN;
        if (!Double.isNaN(dMin) && dMin > distance_m) return Double.NaN;
        if (!Double.isNaN(dMax) && dMax < distance_m) return Double.NaN;

        double lo = minRPM, hi = maxRPM;
        for (int it = 0; it < 40; it++) {
            double mid = 0.5 * (lo + hi);
            double dMid = distanceFromMotorRPMWithDrag(mid, G_ratio, wheelRadius_m, slipEta,
                    hoodAngle_deg, exitHeight_m, goalHeight_m, massKg, diameter_m, Cd);

            if (Double.isNaN(dMid)) lo = mid;
            else if (dMid < distance_m) lo = mid;
            else hi = mid;
        }
        return 0.5 * (lo + hi);
    }
}
