package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class KalmanPoseEstimator {

    private double x, y, heading;

    private double[][] P = new double[3][3];

    private double processNoise = 0.5;
    private double visionNoise = 2.0;

    public KalmanPoseEstimator(Pose2D initialPose) {
        x = initialPose.getX(DistanceUnit.INCH);
        y = initialPose.getY(DistanceUnit.INCH);
        heading = initialPose.getHeading(AngleUnit.RADIANS);

        P[0][0] = 1;
        P[1][1] = 1;
        P[2][2] = 0.1;
    }

    public Pose2D update(Pose2D odoPose, Pose2D visionPose) {


        x = odoPose.getX(DistanceUnit.INCH);
        y = odoPose.getY(DistanceUnit.INCH);
        heading = odoPose.getHeading(AngleUnit.RADIANS);

        P[0][0] += processNoise;
        P[1][1] += processNoise;
        P[2][2] += processNoise * 0.1;


        if (visionPose != null) {

            double vx = visionPose.getX(DistanceUnit.INCH);
            double vy = visionPose.getY(DistanceUnit.INCH);
            double vh = visionPose.getHeading(AngleUnit.RADIANS);

            double[] innovation = {
                    vx - x,
                    vy - y,
                    angleWrap(vh - heading)
            };

            double[] K = new double[3];

            K[0] = P[0][0] / (P[0][0] + visionNoise);
            K[1] = P[1][1] / (P[1][1] + visionNoise);
            K[2] = P[2][2] / (P[2][2] + visionNoise);

            x += K[0] * innovation[0];
            y += K[1] * innovation[1];
            heading += K[2] * innovation[2];

            P[0][0] *= (1 - K[0]);
            P[1][1] *= (1 - K[1]);
            P[2][2] *= (1 - K[2]);
        }

        return new Pose2D(
                DistanceUnit.INCH,
                x,
                y,
                AngleUnit.RADIANS,
                heading
        );

    }

    private double angleWrap(double angle) {
        return ((angle + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
    }

}