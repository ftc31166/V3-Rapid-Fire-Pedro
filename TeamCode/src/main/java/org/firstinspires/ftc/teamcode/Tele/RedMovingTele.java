package org.firstinspires.ftc.teamcode.Tele;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Poses;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.util.List;

public class RedMovingTele extends OpMode {
    public enum states{
        BASE,
        INTAKEON,
        GATEOPEN,
        FLYWHEELON,
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
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }
    @Override
    public void start(){
        robot.drive.startTeleOpDrive(true);
    }
    @Override
    public void loop(){
        loopTime.reset();
        robot.drive.update();
        Pose drivepose = robot.drive.getPose();
        telemetryM.update();
        if (!gamepad1.left_bumper) robot.drive.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );
            //This is how it looks with slowMode on
        else robot.drive.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMultiplier,
                -gamepad1.left_stick_x * slowModeMultiplier,
                -gamepad1.right_stick_x * slowModeMultiplier,
                true // Robot Centric
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
                    fsm = states.FLYWHEELON;
                }
                if(gamepad1.start){
                    reinittimer.reset();
                    fsm = states.REINIT;

                }
                break;
            case REINIT:

                robot.drive.setPose(Poses.redReinit);
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
                    fsm = states.FLYWHEELON;

                }
                break;
            case FLYWHEELON:

                robot.gate.gateClosed();
                robot.intake.stopIntake();

//                if(drivepose.getX() < 20){
//                    target = robot.turret.autoAim(drivepose, Poses.redGoal);
//                    rpm = robot.flywheels.distanceToRPM(drivepose, Poses.redGoal,0);
//                }
//                else{
//                    target= 70;
//                    rpm = 4350;
//                }

                if(gamepad1.b){
                    fsm = states.BASE;
                }

                if( gamepad1.right_trigger>.3){

                    fsm = states.GATEOPEN;
                }
                break;
            case GATEOPEN:
                robot.gate.gateOpen();
                timer.reset();
                fsm = states.SHOOT;
            case SHOOT:

                if (drivepose.getX() > 20 && (timer.milliseconds() > 300)){
                    robot.intake.slowIntake();
                }
                else if(drivepose.getX() < 20){

                    robot.intake.intakeBalls();
                }
                else{

                }
                if(gamepad1.b){
                    fsm = states.BASE;
                }
                break;
        }
        double[] autoAimMovingVals = robot.autoAimMove();
        target = (fsm == states.REINIT) ?0:autoAimMovingVals[0];
        rpm = (fsm == states.FLYWHEELON || fsm == states.GATEOPEN||fsm == states.SHOOT) ? autoAimMovingVals[1] : 2000;

        robot.turret.setTargetAngle(target);
        robot.flywheels.setTargetRPM(rpm);
        robot.hood.setPosition(autoAimMovingVals[2]);
        robot.turret.update();
        robot.flywheels.update();

        telemetryM.debug("loopTime", loopTime.milliseconds());
        telemetryM.debug("x", drivepose.getX());
        telemetryM.debug("y", drivepose.getY());
        telemetryM.debug("Heading", Math.toDegrees(drivepose.getHeading()));
        telemetryM.debug("Target Angle", target);
        telemetryM.debug("Subtracted", target-Math.toDegrees(drivepose.getHeading()));
        telemetryM.debug("Distance", Math.hypot(Poses.redGoal.getY()- drivepose.getY(),Poses.redGoal.getX() - drivepose.getX()));
        telemetryM.debug("loopTime", loopTime.milliseconds());
        telemetryM.debug("currentState", fsm);
        telemetryM.debug("target rpm", rpm);
        telemetryM.debug("stored rpm", robot.flywheels.targetRPM);
    }
}
