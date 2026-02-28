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

@Autonomous(name = "Red Far No Spike", group = "Autonomous")
@Configurable // Panels
public class RedFarNoSpike extends OpMode {

    private Follower follower;
    private ElapsedTime pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public static final Pose start = new Pose(87,9,Math.toRadians(0));
    public static final Pose pose1 = new Pose(135,9,Math.toRadians(0));




    public static final Pose park = new Pose(107,15,Math.toRadians(0));
    PathChain path1,path2,path3;
    Robot robot;
    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
        opmodeTimer.reset();
        follower = Constants.createFollower(hardwareMap);
        robot =new Robot(hardwareMap,start, Poses.redGoal);

        buildPaths();
        follower.setStartingPose(start);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        if (pathState < 12) {
            robot.turret.setTargetAngle(
                    robot.turret.autoAim(follower.getPose(), Poses.redGoal)
            );
            robot.flywheels.setTargetRPM(4500);
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
    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(0);
    }
    public void buildPaths() {

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(start, pose1))
                .setLinearHeadingInterpolation(start.getHeading(), pose1.getHeading())
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(pose1, start))
                .setLinearHeadingInterpolation(pose1.getHeading(), start.getHeading())
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(start, park))
                .setTangentHeadingInterpolation()
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                if(pathTimer.seconds()<3 && pathTimer.seconds() > .5){
                    robot.gate.gateOpen();
                    robot.intake.slowIntake();
                }
                else{
                    robot.gate.gateClosed();
                    robot.intake.intakeBalls();
                    follower.followPath(path1, false);
                    setPathState(1);

                }

                break;
            case 1:
            /* You could check for
            - Follower State: "if(advance()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(advance()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    robot.intake.stopIntake();
                    follower.followPath(path2,false);
                    setPathState(2);
                }

                break;
            case 2:
                if(advance()) {
                    if (pathTimer.seconds() < 3) {
                        robot.gate.gateOpen();
                        robot.intake.slowIntake();
                    } else {
                        robot.gate.gateClosed();
                        robot.intake.intakeBalls();
                        follower.followPath(path1, false);
                        setPathState(3);

                    }
                }

                break;
            case 3:
            /* You could check for
            - Follower State: "if(advance()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(advance()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    robot.intake.stopIntake();
                    follower.followPath(path2,false);
                    setPathState(4);
                }

                break;
            case 4:
                if(advance()) {
                    if (pathTimer.seconds() < 3) {
                        robot.gate.gateOpen();
                        robot.intake.slowIntake();
                    } else {
                        robot.gate.gateClosed();
                        robot.intake.intakeBalls();
                        follower.followPath(path1, false);
                        setPathState(5);

                    }
                }

                break;
            case 5:
            /* You could check for
            - Follower State: "if(advance()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(advance()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    robot.intake.stopIntake();
                    follower.followPath(path2,false);
                    setPathState(6);
                }

                break;
            case 6:
                if(advance()) {
                    if (pathTimer.seconds() < 3) {
                        robot.gate.gateOpen();
                        robot.intake.slowIntake();
                    } else {
                        robot.gate.gateClosed();
                        robot.intake.intakeBalls();
                        follower.followPath(path1, false);
                        setPathState(7);

                    }
                }

                break;
            case 7:
            /* You could check for
            - Follower State: "if(advance()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(advance()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    robot.intake.stopIntake();
                    follower.followPath(path2,false);
                    setPathState(8);
                }

                break;

            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(advance()) {

                    if(pathTimer.seconds()<3){
                        robot.gate.gateOpen();
                        robot.intake.slowIntake();
                    }
                    else{
                        robot.gate.gateClosed();
                        robot.intake.intakeBalls();
                        follower.followPath(path3, true);
                        setPathState(11);
                    }
                }

                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(advance()) {



                    setPathState(12);


                }

                break;
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }
    private boolean advance() {
        return !follower.isBusy() || pathTimer.seconds() > 8;
    }
}