package org.firstinspires.ftc.teamcode.Tele;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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

    private double slowModeMultiplier = 0.5;
    double rpm = 0;
    double target = 0;
    
    @Override
    public void init(){
        robot = new Robot(hardwareMap, Poses.redAutoEnd,Poses.redGoal);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){
        loopTime.reset();
        robot.pinpointDriver.update();
        Pose2D pinpointPose = robot.pinpointDriver.getPosition();
        Pose drivepose = new Pose(pinpointPose.getX(DistanceUnit.INCH), pinpointPose.getY(DistanceUnit.INCH), pinpointPose.getHeading(AngleUnit.RADIANS) );

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
                if(gamepad1.start){
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



                    robot.intake.intakeBalls();

                if(gamepad1.b){
                    fsm = states.BASE;
                }
                break;
        }
        double[] autoAimMovingVals = robot.autoAimMove(drivepose);
        target = (fsm == states.REINIT) ?0:autoAimMovingVals[0];
        rpm = autoAimMovingVals[1];

        robot.turret.setTargetAngle(target);
        robot.flywheels.setTargetRPM(rpm);
        robot.hood.setPosition(autoAimMovingVals[2]);
        robot.turret.update();
        robot.flywheels.update();

        telemetry.addData("loopTime", loopTime.milliseconds());
        telemetry.addData("x", drivepose.getX());
        telemetry.addData("y", drivepose.getY());
        telemetry.addData("Heading", Math.toDegrees(drivepose.getHeading()));
        telemetry.addData("Target Angle", target);
        telemetry.addData("Subtracted", target-Math.toDegrees(drivepose.getHeading()));
        telemetry.addData("Distance", Math.hypot(Poses.redGoal.getY()- drivepose.getY(),Poses.redGoal.getX() - drivepose.getX()));
        telemetry.addData("loopTime", loopTime.milliseconds());
        telemetry.addData("currentState", fsm);
        telemetry.addData("target rpm", rpm);
        telemetry.addData("stored rpm", robot.flywheels.targetRPM);
        telemetry.update();
    }
}
