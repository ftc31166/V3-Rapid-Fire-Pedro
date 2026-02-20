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

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class RedFar extends OpMode {

    private Follower follower;
    private ElapsedTime pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public static final Pose start = new Pose(87,9,Math.toRadians(0));
    public static final Pose pose1 = new Pose(135,9,Math.toRadians(0));

    public static final Pose pose2 = new Pose(125,9,Math.toRadians(0));
    public static final Pose pose3 = new Pose(135,9,Math.toRadians(0));
    public static final Pose pose4 = new Pose(85,15,Math.toRadians(45));
    public static final Pose pose5 = new Pose(135,36,Math.toRadians(0));
    public static final Pose pose5cp1 = new Pose(106,37,Math.toRadians(0));
    public static final Pose pose6 = new Pose(85,15,Math.toRadians(10));
    public static final Pose pose7 = new Pose(135,24,Math.toRadians(0));
    public static final Pose pose8 = new Pose(125,24,Math.toRadians(0));
    public static final Pose pose9 = new Pose(135,24,Math.toRadians(0));
    public static final Pose pose10 = new Pose(85,15,Math.toRadians(0));
    public static final Pose pose11 = new Pose(107,15,Math.toRadians(0));
    PathChain path1,path2,path3,path4,path5,path6,path7,path8,path9,path10,path11;
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
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
                .build();
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(pose3, pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .build();
        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(pose4, pose5cp1, pose5))

                .setTangentHeadingInterpolation()
                .build();
        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(pose5, pose6))
                .setLinearHeadingInterpolation(pose5.getHeading(), pose6.getHeading())
                .build();
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(pose6, pose7))
                .setLinearHeadingInterpolation(pose6.getHeading(), pose7.getHeading())
                .build();
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(pose7, pose8))
                .setLinearHeadingInterpolation(pose7.getHeading(), pose8.getHeading())
                .build();
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(pose8, pose9))
                .setTangentHeadingInterpolation()
                .build();
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(pose9, pose10))
                .setLinearHeadingInterpolation(pose9.getHeading(), pose10.getHeading())
                .build();
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(pose10, pose11))
                .setTangentHeadingInterpolation()
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.gate.gateOpen();
                if(pathTimer.seconds()<3){
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
                    follower.followPath(path2,false);
                    setPathState(2);
                }
               
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(advance()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(path3,false);
                    setPathState(3);
                }
               
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(advance()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(path4,true);
                    setPathState(4);
                }

                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(advance()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    robot.gate.gateOpen();
                    if(pathTimer.seconds()<3){
                        robot.intake.slowIntake();
                    }
                    else{
                        robot.gate.gateClosed();
                        robot.intake.intakeBalls();
                        follower.followPath(path5, false);
                        setPathState(5);
                    }
                }

                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(advance()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(path6,true);
                    setPathState(6);
                }

                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(advance()) {
                    robot.gate.gateOpen();
                    if(pathTimer.seconds()<3){
                        robot.intake.slowIntake();
                    }
                    else{
                        robot.gate.gateClosed();
                        robot.intake.intakeBalls();
                        follower.followPath(path7, false);
                        setPathState(7);
                    }

                }

                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(advance()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */


                        follower.followPath(path8);
                        setPathState(8);

                }

                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(advance()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(path9, false);
                    setPathState(9);
                }

                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(advance()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(path10, true);
                    setPathState(10);
                }

                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(advance()) {
                    robot.gate.gateOpen();
                    if(pathTimer.seconds()<3){
                        robot.intake.slowIntake();
                    }
                    else{
                        robot.gate.gateClosed();
                        robot.intake.intakeBalls();
                        follower.followPath(path11, true);
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