
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

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroRed15 extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ElapsedTime pathTimer, actionTimer, opmodeTimer;

    Robot robot;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();
        opmodeTimer.reset();
        follower = Constants.createFollower(hardwareMap);
        robot =new Robot(hardwareMap,new Pose(72, 8, Math.toRadians(90)), Poses.redGoal);
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
        public PathChain Path1;

        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8, Path9, Path10;

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
                                    new Pose(100.742, 66.315),
                                    new Pose(87.682, 87.910)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-28))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.682, 87.910),
                                    new Pose(104.921, 71.037),
                                    new Pose(128.594, 70.512)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.594, 70.512),

                                    new Pose(132.817, 59.540)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.817, 59.540),

                                    new Pose(88.038, 87.867)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-101))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.038, 87.867),
                                    new Pose(84.515, 30.207),
                                    new Pose(110.223, 34.754),
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

            Path9 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.588, 88.028),
                                    new Pose(98.439, 81.479),
                                    new Pose(128.277, 83.130)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.277, 83.130),
                                    new Pose(98.477, 81.571),
                                    new Pose(88.235, 87.357)
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
                actionTimer.reset();
                if (actionTimer.milliseconds() > 1000) {
                    robot.gate.gateOpen();
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
                    actionTimer.reset();
                    if (actionTimer.milliseconds() > 1000) {
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (advance()) {
                    robot.intake.slowIntake();
                    follower.followPath(paths.Path3, true);
                    actionTimer.reset();
                    if (actionTimer.milliseconds() > 1000){
                        robot.gate.gateOpen();
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
                    actionTimer.reset();
                    if (actionTimer.milliseconds() > 1000) {
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (advance()){
                    robot.intake.slowIntake();
                    follower.followPath(paths.Path6, true);
                    actionTimer.reset();
                    if (actionTimer.milliseconds() > 1000){
                        robot.gate.gateOpen();
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
                    actionTimer.reset();
                    if (actionTimer.milliseconds() > 1000){
                        robot.gate.gateOpen();
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if (advance()){
                    robot.gate.gateClosed();
                    robot.intake.intakeBalls();
                    follower.followPath(paths.Path9, true);
                }
                break;
            case 9:
                if (advance()){
                    robot.intake.slowIntake();
                    follower.followPath(paths.Path10, true);
                    actionTimer.reset();
                    if (actionTimer.milliseconds() > 1000){
                        robot.gate.gateOpen();
                        setPathState(-1);
                    }
                }
        }

    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
        actionTimer.reset();
    }

    private boolean advance() {
        return !follower.isBusy() || pathTimer.seconds() > 8;
    }
}
