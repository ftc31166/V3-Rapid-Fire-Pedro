
package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Poses;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red 15", group = "Autonomous")
@Configurable // Panels
public class PedroRed15 extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ElapsedTime pathTimer, actionTimer, opmodeTimer;

    public static final Pose start = new Pose(127,121,Math.toRadians(36.4));
    public static final Pose shootPoint = new Pose(87.758, 88.152);
    public static final Pose Path2ControlPoint1 = new Pose(92.588, 63.754);
    public static final Pose Path2ControlPoint2 = new Pose(108.000, 57.200);
    public static final Pose intakeFirst = new Pose(135.000, 58.000);
    public static final Pose Path3ControlPoint1 = new Pose(100.742, 66.315);
    public static final Pose Path4ControlPoint1 = new Pose(104.921, 71.037);
    public static final Pose gateOpenPose = new Pose(125, 70.512);
    public static final Pose gateIntake = new Pose(129, 59.540);
    public static final Pose Path7ControlPoint1 = new Pose(84.515, 30.207);
    public static final Pose Path7ControlPoint2 = new Pose(110.223, 34.754);
    public static final Pose intakeSecond = new Pose(134.836, 35.080);
    public static final Pose Path9ControlPoint1 = new Pose(98.439, 81.479);
    public static final Pose intakeThird = new Pose(128.277, 83.130);
    public static final Pose Path10ControlPoint1 = new Pose(98.477, 81.571);
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
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

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
                                    shootPoint
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(2), Math.toRadians(-43))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shootPoint,
                                    Path4ControlPoint1,
                                    gateOpenPose
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    gateOpenPose,
                                    gateIntake
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    gateIntake,
                                    shootPoint
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-101))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shootPoint,
                                    Path7ControlPoint1,
                                    Path7ControlPoint2,
                                    intakeSecond
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    intakeSecond,
                                    shootPoint
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shootPoint,
                                    Path9ControlPoint1,
                                    intakeThird
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    intakeThird,
                                    Path10ControlPoint1,
                                    shootPoint
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

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
                    robot.intake.slowIntake();
                    follower.followPath(paths.Path3, true);
                    robot.gate.gateOpen();
                    if (pathTimer.milliseconds() > 1000){
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if (advance()){
                    robot.gate.gateClosed();
                    follower.followPath(paths.Path4, true);
                    robot.intake.intakeBalls();
                    setPathState(4);
                }
                break;
            case  4:
                if (advance()){
                    follower.followPath(paths.Path5, true);
                    if (pathTimer.milliseconds() > 1000) {
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (advance()){
                    robot.intake.slowIntake();
                    follower.followPath(paths.Path6, true);
                    robot.gate.gateOpen();
                    if (pathTimer.milliseconds() > 1000){
                        setPathState(6);
                    }
                }
                break;
            case 6:
               if ((advance())){
                   robot.intake.intakeBalls();
                   follower.followPath(paths.Path7, true);
                   setPathState(7);
               }
               break;
            case 7:
                if (advance()){
                    robot.intake.slowIntake();
                    follower.followPath(paths.Path8, true);
                    robot.gate.gateOpen();
                    if (pathTimer.milliseconds() > 1000){
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if (advance()){
                    robot.gate.gateClosed();
                    robot.intake.intakeBalls();
                    follower.followPath(paths.Path9, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (advance()){
                    robot.intake.slowIntake();
                    follower.followPath(paths.Path10, true);
                    robot.gate.gateOpen();
                    if (pathTimer.milliseconds() > 1000){
                        setPathState(-1);
                    }
                }
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
