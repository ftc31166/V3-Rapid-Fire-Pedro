
package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Subsystems.Poses;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red 12", group = "Autonomous")
@Configurable // Panels
public class PedroRed12 extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ElapsedTime pathTimer, actionTimer, opmodeTimer;

    public static final Pose start = new Pose(110, 136, Math.toRadians(90));
    public static final Pose shootPoint = new Pose(87.758, 88.152);
    public static final Pose Path2ControlPoint1 = new Pose(92.588, 63.754);
    public static final Pose Path2ControlPoint2 = new Pose(108.000, 57.200);
    public static final Pose intakeFirst = new Pose(135.000, 58.000);
    public static final Pose Path3ControlPoint1 = new Pose(112.007, 63.877);
    public static final Pose gateOpenPose = new Pose(128.009, 69.166);
    public static final Pose Path4ControlPoint1 = new Pose(100.742, 66.315);
    public static final Pose Path5ControlPoint1 = new Pose(100.900, 83.032);
    public static final Pose intakeSecond = new Pose(127.644, 83.736);
    public static final Pose Path7ControlPoint1 =  new Pose(76.282, 30.711);
    public static final Pose Path7ControlPoint2 = new Pose(109.215, 34.754);

    public static final Pose intakeThird = new Pose(134.668, 34.408);
    Robot robot;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();
        opmodeTimer.reset();
        follower = Constants.createFollower(hardwareMap);
        robot =new Robot(hardwareMap, start, Poses.redGoal);
    }
    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        if (pathState >= 0) {
            robot.turret.setTargetAngle(
                    robot.turret.autoAim(follower.getPose(), Poses.redGoal)
            );
            robot.flywheels.setTargetRPM(3200);
            robot.hood.setPosition(
                    robot.hood.distanceToRPM(follower.getPose(), Poses.redGoal)
            );
        } else {
            // Final safe state
            robot.turret.setTargetAngle(0);
            robot.flywheels.setTargetRPM(0);
            robot.gate.gateClosed();
        }
        robot.turret.update();
        robot.flywheels.update();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }



    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    start,
                                    shootPoint
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-78))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shootPoint,
                                    Path2ControlPoint1,
                                    Path2ControlPoint2,
                                    intakeFirst
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    intakeFirst,
                                    Path3ControlPoint1,
                                    gateOpenPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .setReversed()
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    gateOpenPose,
                                    Path4ControlPoint1,
                                    shootPoint
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-20))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shootPoint,
                                    Path5ControlPoint1,
                                    intakeSecond
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    intakeSecond,
                                    shootPoint
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-101))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shootPoint,
                                    Path7ControlPoint1,
                                    Path7ControlPoint2,
                                    intakeThird
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    intakeThird,
                                    shootPoint
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();
        }
    }



    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                if (advance()){
                robot.intake.slowIntake();
                follower.followPath(paths.Path1, true);
                robot.gate.gateOpen();
                if (pathTimer.milliseconds() > 1000) {
                    setPathState(1);
                }
                else{

                }
                }
                break;
            case 1:
                if (advance()) {
                    robot.gate.gateClosed();
                    robot.intake.intakeBalls();
                    follower.followPath(paths.Path2, true);
                    setPathState(2);

                }
                break;
            case 2:
                if (advance()) {
                    follower.followPath(paths.Path3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (advance()) {
                    robot.intake.slowIntake();
                    follower.followPath(paths.Path4, true);
                    robot.gate.gateOpen();
                    if (pathTimer.milliseconds() > 1000){
                        setPathState(4);
                    }

                }
                break;
            case 4:
                if (advance()) {
                    robot.gate.gateClosed();
                    robot.intake.intakeBalls();
                    follower.followPath(paths.Path5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (advance()) {
                    robot.intake.slowIntake();
                    follower.followPath(paths.Path6, true);
                    robot.gate.gateOpen();
                    if (pathTimer.milliseconds() > 1000) {
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if (advance()) {
                    robot.gate.gateClosed();
                    robot.intake.intakeBalls();
                    follower.followPath(paths.Path7, true);
                    setPathState(7);


                }
                break;
            case 7:
                if (advance()) {
                    robot.intake.slowIntake();
                    follower.followPath(paths.Path8, true);
                    robot.gate.gateOpen();
                    if (pathTimer.milliseconds() > 1000){
                        setPathState(-1);
                    }
                }
                break;
        }

    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    private boolean advance() {
        return !follower.isBusy() || pathTimer.seconds() > 8;
    }
}
