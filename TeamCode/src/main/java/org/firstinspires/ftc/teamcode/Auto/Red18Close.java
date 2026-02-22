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

@Autonomous(name = "Red 18 close", group = "Autonomous")
@Configurable // Panels
public class Red18Close extends OpMode {

    private Follower follower;
    private ElapsedTime pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public static final Pose start = new Pose(127,121,Math.toRadians(36.4));
    public static final Pose scorePreload = new Pose(88,78,Math.toRadians(-30));

    public static final Pose get2ndSpike = new Pose(120,60,Math.toRadians(-30));
    public static final Pose shoot2ndSpike = new Pose(88,78,Math.toRadians(-20));
    public static final Pose emptyGate1 = new Pose(133,60,Math.toRadians(30));
    public static final Pose scoreGate1 = new Pose(88,78,Math.toRadians(-20));
    public static final Pose emptyGate2 = new Pose(133,60,Math.toRadians(30));
    public static final Pose scoreGate2 = new Pose(88,78,Math.toRadians(-55));
    public static final Pose get3rdSpike = new Pose(120,35,Math.toRadians(-55));
    public static final Pose shoot3rdSpike = new Pose(88,78,Math.toRadians(10));
    public static final Pose get1stSpike = new Pose(120,84,Math.toRadians(10));
    public static final Pose shoot1stSpike = new Pose(88,78,Math.toRadians(0));
    public static final Pose park = new Pose(120,72,Math.toRadians(0));
    PathChain path1,path2,path3,path4,path5,path6,path7,path8,path9,path10,path11,path12;
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
        Pose followPose = follower.getPose();
        if (pathState < 13) {
            robot.turret.setTargetAngle(
                    robot.turret.autoAim(followPose, Poses.redGoal)
            );
            robot.flywheels.setTargetRPM(robot.flywheels.distanceToRPM(followPose, Poses.redGoal,0));
            robot.hood.setPosition(
                    robot.hood.distanceToRPM(followPose, Poses.redGoal)
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
        telemetry.addData("x", followPose.getX());
        telemetry.addData("y", followPose.getY());
        telemetry.addData("heading", followPose.getHeading());
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
                .addPath(new BezierLine(start, scorePreload))
                .setLinearHeadingInterpolation(start.getHeading(), scorePreload.getHeading())
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePreload, get2ndSpike))
                .setLinearHeadingInterpolation(scorePreload.getHeading(), get2ndSpike.getHeading())
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(get2ndSpike, shoot2ndSpike))
                .setLinearHeadingInterpolation(get2ndSpike.getHeading(), shoot2ndSpike.getHeading())
                .build();
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2ndSpike, emptyGate1))
                .setLinearHeadingInterpolation(shoot2ndSpike.getHeading(), emptyGate1.getHeading())
                .build();
        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(emptyGate1,scoreGate1))
                .setLinearHeadingInterpolation(shoot2ndSpike.getHeading(), scoreGate1.getHeading())
                .setTangentHeadingInterpolation()
                .build();
        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(scoreGate1, emptyGate2))
                .setLinearHeadingInterpolation(scoreGate1.getHeading(), emptyGate2.getHeading())
                .build();
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(emptyGate2, scoreGate2))
                .setLinearHeadingInterpolation(emptyGate2.getHeading(), scoreGate2.getHeading())
                .build();
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(scoreGate2, get3rdSpike))
                .setLinearHeadingInterpolation(scoreGate2.getHeading(), get3rdSpike.getHeading())
                .build();
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(get3rdSpike, shoot3rdSpike))
                .setLinearHeadingInterpolation(get3rdSpike.getHeading(), shoot3rdSpike.getHeading())
                .build();
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(shoot3rdSpike, get1stSpike))
                .setLinearHeadingInterpolation(shoot3rdSpike.getHeading(), get1stSpike.getHeading())
                .build();
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(get1stSpike, shoot1stSpike))
                .setLinearHeadingInterpolation(get1stSpike.getHeading(), shoot1stSpike.getHeading())
                .build();
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1stSpike, park))
                .setLinearHeadingInterpolation(shoot1stSpike.getHeading(), park.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                follower.followPath(path1, true);
                setPathState(1);



                break;
            case 1:
            /* You could check for
            - Follower State: "if(advance()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(followPose.getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(advance()) {
                    robot.intake.intakeBalls();
                    if(pathTimer.seconds()<1){
                        robot.gate.gateOpen();

                    }
                    else{
                        robot.gate.gateClosed();

                        follower.followPath(path2, false);
                        setPathState(2);
                    }
                }

                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(advance()) {
                    follower.followPath(path3,true);
                    setPathState(3);
                }

                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(advance()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    robot.intake.intakeBalls();
                    if(pathTimer.seconds()<1){
                        robot.gate.gateOpen();
                    }
                    else{
                        robot.gate.gateClosed();

                        follower.followPath(path4, false);
                        setPathState(4);
                    }
                }

                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(advance()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                        follower.followPath(path5, true);
                        setPathState(5);

                }

                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(advance()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    robot.intake.intakeBalls();
                    if(pathTimer.seconds()<1){
                        robot.gate.gateOpen();

                    }
                    else{
                        robot.gate.gateClosed();
                        follower.followPath(path6, false);
                        setPathState(6);
                    }
                }

                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(advance()) {

                        follower.followPath(path7, true);
                        setPathState(7);


                }

                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(advance()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */

                    robot.intake.intakeBalls();
                    if(pathTimer.seconds()<1){
                        robot.gate.gateOpen();

                    }
                    else{
                        robot.gate.gateClosed();

                        follower.followPath(path8, false);
                        setPathState(8);
                    }

                }

                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(advance()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(path9, true);
                    setPathState(9);
                }

                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(advance()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    robot.intake.intakeBalls();
                    if(pathTimer.seconds()<1){
                        robot.gate.gateOpen();

                    }
                    else{
                        robot.gate.gateClosed();

                        follower.followPath(path10, false);
                        setPathState(10);
                    }
                }

                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(advance()) {

                        follower.followPath(path11, true);
                        setPathState(11);
                    
                }

                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(advance()) {
                    robot.intake.intakeBalls();
                    if(pathTimer.seconds()<1){
                        robot.gate.gateOpen();

                    }
                    else {
                        robot.gate.gateClosed();

                        follower.followPath(path12, false);
                        setPathState(12);
                    }
                }

                break;
            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(advance()) {
                   setPathState(13);
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