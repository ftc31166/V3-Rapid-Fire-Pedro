
package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class Pedro extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }



    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(110.000, 136.000),

                                    new Pose(87.758, 88.152)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-78))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.758, 88.152),
                                    new Pose(92.588, 63.754),
                                    new Pose(108.000, 57.200),
                                    new Pose(135.000, 58.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(135.000, 58.000),
                                    new Pose(112.007, 63.877),
                                    new Pose(128.009, 69.166)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .setReversed()
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.009, 69.166),
                                    new Pose(100.742, 66.315),
                                    new Pose(87.682, 87.910)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-28))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.682, 87.910),
                                    new Pose(94.682, 84.209),
                                    new Pose(112.097, 75.419),
                                    new Pose(129.156, 81.384)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.156, 81.384),

                                    new Pose(88.038, 87.867)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-101))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.038, 87.867),
                                    new Pose(76.282, 30.711),
                                    new Pose(110.559, 31.393),
                                    new Pose(138.028, 34.744)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(138.028, 34.744),

                                    new Pose(87.588, 88.028)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();
        }
    }



    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                follower.followPath(paths.Path1);
                pathState = 1;
                break;
            case 1:
                follower.followPath(paths.Path2);
                pathState = 2;
                break;
            case 2:
                follower.followPath(paths.Path3);
                pathState = 3;
                break;
            case 3:
                follower.followPath(paths.Path4);
                pathState = 4;
                break;
            case 4:
                follower.followPath(paths.Path5);
                pathState = 5;
                break;
            case 5:
                follower.followPath(paths.Path6);
                pathState = 6;
                break;
            case 6:
                follower.followPath(paths.Path7);
                pathState = 7;
                break;
            case 7:
                follower.followPath(paths.Path8);
                pathState = -1;
                break;
        }

    }
}
