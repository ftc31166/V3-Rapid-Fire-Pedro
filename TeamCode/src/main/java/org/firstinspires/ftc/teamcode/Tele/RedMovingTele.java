package org.firstinspires.ftc.teamcode.Tele;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Subsystems.Poses;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.util.List;
@TeleOp
public class RedMovingTele extends OpMode {
    public enum states{
        BASE,
        INTAKEON,
        GATEOPEN,
        SHOOT,
        REINIT,
    }
    states fsm = states.BASE;
    Robot robot;
    TelemetryManager telemetryM;

    ElapsedTime loopTime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime reinittimer = new ElapsedTime();
    ElapsedTime autoLocalizeTimer = new ElapsedTime();

    private double slowModeMultiplier = 0.5;
    double rpm = 0;
    double target = 0;
    Servo light;
    
    @Override
    public void init(){
        robot = new Robot(hardwareMap, Poses.redAutoEnd,Poses.redGoal);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry.addData("cam connected",
        robot.cam.isConnected());
        telemetry.addData("cam on", robot.cam.isRunning());
        light = hardwareMap.get(Servo.class, "light");

    }
    @Override
    public void start(){
        autoLocalizeTimer.reset();
    }
    @Override
    public void loop(){
        loopTime.reset();
        robot.pinpointDriver.update();
        Pose2D pinpointPose = robot.pinpointDriver.getPosition();
        Pose drivepose = new Pose(pinpointPose.getX(DistanceUnit.INCH), pinpointPose.getY(DistanceUnit.INCH), pinpointPose.getHeading(AngleUnit.RADIANS) );

//        if(autoLocalizeTimer.milliseconds()>2000){
//            robot.cam.updateRobotOrientation(robot.pinpointDriver.getHeading(AngleUnit.RADIANS));
//            LLResult result = robot.cam.getLatestResult();
//            Pose3D pose;
//            if((result!=null)&&result.isValid()) {
//                pose = result.getBotpose();
//                robot.pinpointDriver.setPosition(new Pose2D(DistanceUnit.INCH,pose.getPosition().x*39.3701+72, pose.getPosition().y*39.3701+72, AngleUnit.DEGREES,pose.getOrientation().getYaw(AngleUnit.DEGREES)-90));
//                telemetry.addData("reinintpose", pose.toString());
//            }
//            else {
//                telemetry.addLine("notagslol");
//            }
//            autoLocalizeTimer.reset();
//        }
        if (!gamepad1.left_bumper) robot.driveRoboCentric(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x*.7
        );
            //This is how it looks with slowMode on
        else robot.driveRoboCentric(
                gamepad1.left_stick_y * slowModeMultiplier,
                gamepad1.left_stick_x * slowModeMultiplier,
                gamepad1.right_stick_x * slowModeMultiplier// Robot Centric
        );





        if(gamepad1.dpadLeftWasPressed()){
            robot.turret.turretOffset += 5;

        }
        if(gamepad1.dpadRightWasPressed()){
            robot.turret.turretOffset -= 5;

        }
        if(gamepad1.x){
            robot.cam.updateRobotOrientation(robot.pinpointDriver.getHeading(AngleUnit.RADIANS));
            LLResult result = robot.cam.getLatestResult();
            Pose3D pose;

             if((result!=null)&&result.isValid()) {
                pose = result.getBotpose();
                robot.pinpointDriver.setPosition(new Pose2D(DistanceUnit.INCH,pose.getPosition().x*39.3701+72, pose.getPosition().y*39.3701+72, AngleUnit.DEGREES,pose.getOrientation().getYaw(AngleUnit.DEGREES)-90));
                 telemetry.addData("reinintpose", pose.toString());
            }
            else {
                telemetry.addLine("notagslol");
            }

        }

        switch(fsm){
            case BASE:

                robot.gate.gateClosed();
                robot.intake.stopIntake();


                if(gamepad1.a){
                    fsm = states.INTAKEON;
                }
                if(gamepad1.right_bumper){
                    fsm = states.GATEOPEN;
                }
                if(gamepad1.start ){
                    reinittimer.reset();
                    fsm = states.REINIT;

                }
                break;
            case REINIT:

                robot.pinpointDriver.setPosition(new Pose2D(DistanceUnit.INCH,Poses.redReinit.getX(),Poses.redReinit.getY(),AngleUnit.RADIANS,Poses.redReinit.getHeading()));
                if(reinittimer.milliseconds()>300){
                    fsm = states.BASE;
                }
                break;
            case INTAKEON:

                robot.gate.gateClosed();
                robot.intake.intakeBalls();

                if(gamepad1.b){
                    fsm = states.BASE;
                }
                if(gamepad1.right_bumper){

                    fsm = states.GATEOPEN;
                }
                break;

            case GATEOPEN:
                robot.gate.gateOpen();
                timer.reset();
                fsm = states.SHOOT;
            case SHOOT:



                if (drivepose.getY() < 50) {
                    robot.intake.slowIntake();
                } else {
                    robot.intake.intakeBalls();
                }

                if(gamepad1.b){
                    fsm = states.BASE;
                }
                break;
        }
        double lightPosition = (fsm == states.SHOOT) ? .5 : 0;
        light.setPosition(lightPosition);
        target = robot.turret.autoAim(drivepose,Poses.redGoal);
        rpm = robot.flywheels.distanceToRPM(drivepose,Poses.redGoal,0);
        robot.hood.setPosition(robot.hood.distanceToRPM(drivepose,Poses.redGoal));
        robot.turret.setTargetAngle(target);
        robot.flywheels.setTargetRPM(rpm);
        robot.turret.update();
        robot.flywheels.update();
        double dist = Math.hypot(Poses.redGoal.getY()- drivepose.getY(),Poses.redGoal.getX() - drivepose.getX());
        telemetry.addData("loopTime", loopTime.milliseconds());
        telemetry.addData("x", drivepose.getX());
        telemetry.addData("y", drivepose.getY());
        telemetry.addData("Heading", Math.toDegrees(drivepose.getHeading()));
        telemetry.addData("Target Angle", target);
        telemetry.addData("Subtracted", target-Math.toDegrees(drivepose.getHeading()));
        telemetry.addData("Distance",dist) ;
        telemetry.addData("currentState", fsm);
        telemetry.addData("target rpm", rpm);
        telemetry.update();
    }
}
