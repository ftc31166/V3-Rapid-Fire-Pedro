//package org.firstinspires.ftc.teamcode.Tele;
//
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Poses;
//import org.firstinspires.ftc.teamcode.Subsystems.Robot;
//
//import java.util.List;
//
//public class RedTele extends OpMode {
//    public enum states{
//        BASE,
//        INTAKEON,
//        GATEOPEN,
//        FLYWHEELON,
//        SHOOT
//    }
//    states fsm = states.BASE;
//    Robot robot;
//    TelemetryManager telemetryM;
//    ElapsedTime timer2 = new ElapsedTime();
//    ElapsedTime loopTime = new ElapsedTime();
//    ElapsedTime timer = new ElapsedTime();
//
//    private double slowModeMultiplier = 0.5;
//    double rpm = 0;
//    double target = 0;
//    @Override
//    public void init(){
//        robot = new Robot(hardwareMap, Poses.redAutoEnd,Poses.redGoal);
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//
//    }
//    @Override
//    public void start(){
//        robot.drive.startTeleOpDrive(true);
//    }
//    @Override
//    public void loop(){
//        loopTime.reset();
//        robot.drive.update();
//        telemetryM.update();
//        if (!gamepad1.left_bumper) robot.drive.setTeleOpDrive(
//                -gamepad1.left_stick_y,
//                -gamepad1.left_stick_x,
//                -gamepad1.right_stick_x,
//                true // Robot Centric
//        );
//            //This is how it looks with slowMode on
//        else robot.drive.setTeleOpDrive(
//                -gamepad1.left_stick_y * slowModeMultiplier,
//                -gamepad1.left_stick_x * slowModeMultiplier,
//                -gamepad1.right_stick_x * slowModeMultiplier,
//                true // Robot Centric
//        );
//
//        if(gamepad1.start){
//            robot.turret.setTargetAngle(0);
//            robot.turret.update();
//            robot.drive.setPose(Poses.redReinit);
//        }
//
//
//
//
//        if(gamepad1.dpad_left && timer2.milliseconds() > 200){
//            robot.turret.turretOffset += 5;
//            timer2.reset();
//        }
//        if(gamepad1.dpad_right && timer2.milliseconds() > 200){
//            robot.turret.turretOffset -= 5;
//            timer2.reset();
//        }
//
//        switch(fsm){
//            case BASE:
//
//                robot.gate.gateClosed();
//                robot.intake.stopIntake();
//                rpm = 1000;
//                target = 0;
//                if(gamepad1.a){
//                    fsm = states.INTAKEON;
//                }
//                if(gamepad1.right_bumper){
//                    fsm = states.FLYWHEELON;
//                }
//                break;
//            case INTAKEON:
//
//                robot.gate.gateClosed();
//                robot.intake.intakeBalls();
//                rpm = 1000;
//                target = 0;
//                if(gamepad1.b){
//                    fsm = states.BASE;
//                }
//                if(gamepad1.right_bumper){
//                    fsm = states.FLYWHEELON;
//
//                }
//                break;
//            case FLYWHEELON:
//
//                robot.gate.gateClosed();
//                robot.intake.stopIntake();
//                if(robot.drive.getPose().getX() < 20){
//                    target = robot.turret.autoAim(robot.drive.getPose(), Poses.redGoal);
//                    rpm = robot.flywheels.distanceToRPM(robot.drive.getPose(), Poses.redGoal,0);
//                }
//                else{
//                    target= 70;
//                    rpm = 4350;
//                }
//                if(gamepad1.b){
//                    fsm = states.BASE;
//                }
//
//                if( gamepad1.right_trigger>.3){
//
//                    fsm = states.GATEOPEN;
//                }
//                break;
//            case GATEOPEN:
//                robot.gate.gateOpen();
//                timer.reset();
//                fsm = states.SHOOT;
//            case SHOOT:
//
//
//
//
//                if (robot.drive.getPose().getX() > 20 && (timer.milliseconds() > 300)){
//                    rpm=4350;
//                    robot.intake.slowIntake();
//                }
//                else if(robot.drive.getPose().getX() < 20){
//                    rpm = robot.flywheels.distanceToRPM(robot.drive.getPose(), Poses.redGoal,0);
//                    robot.intake.intakeBalls();
//                }
//                else{
//
//                }
//                if(gamepad1.b){
//                    fsm = states.BASE;
//                }
//                break;
//        }
//        robot.flywheels.setTargetRPM(rpm);
//        robot.hood.setPosition(robot.hood.distanceToRPM(robot.drive.getPose(),Poses.redGoal));
//        robot.turret.setTargetAngle(target);
//        robot.turret.update();
//        robot.flywheels.update();
//
//        telemetryM.debug("loopTime", loopTime.milliseconds());
//        telemetryM.debug("x", robot.drive.getPose().getX());
//        telemetryM.debug("y", robot.drive.getPose().getY());
//        telemetryM.debug("Heading", Math.toDegrees(robot.drive.getPose().getHeading()));
//        telemetryM.debug("Target Angle", target);
//        telemetryM.debug("Subtracted", target-Math.toDegrees(robot.drive.getPose().getHeading()));
//        telemetryM.debug("Distance", Math.hypot(Poses.redGoal.getY()- robot.drive.getPose().getY(),Poses.redGoal.getX() - robot.drive.getPose().getX()));
//        telemetryM.debug("loopTime", loopTime.milliseconds());
//        telemetryM.debug("currentState", fsm);
//        telemetryM.debug("target rpm", rpm);
//        telemetryM.debug("stored rpm", robot.flywheels.targetRPM);
//    }
//}
