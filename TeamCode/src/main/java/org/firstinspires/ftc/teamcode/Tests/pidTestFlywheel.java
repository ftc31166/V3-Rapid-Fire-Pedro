package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Flywheels;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;


@TeleOp
public class pidTestFlywheel extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Flywheels flywheels = new Flywheels(hardwareMap);
        Servo hood = hardwareMap.get(Servo.class, "hood");
        Intake intake = new Intake(hardwareMap);
        hood.setPosition(0.35);
         double kF = 1.0/5600;
         double kP = 0.002;
         double kI = 0;
         double kD = 0;
         double[] telems = {0,0};
         double targetRPM = 3000;

        ElapsedTime timer = new ElapsedTime();
        if (isStopRequested()) return;
        waitForStart();
        while (opModeIsActive()){


            flywheels.kD = kD;
            flywheels.kP = kP;
            flywheels.kI = kI;
            flywheels.kF = kF;
            flywheels.setCoeffs();
            flywheels.setTargetRPM(targetRPM);


            telems = flywheels.update();
            targetRPM = Math.max(0,Math.min(targetRPM,6000));
            if(gamepad1.b){

                flywheels.stop();
            }
            if(gamepad1.a){

                flywheels.stop();
            }
            if(gamepad1.left_bumper&&timer.milliseconds()>200){

                targetRPM -=100;
                timer.reset();
            }
            if(gamepad1.right_bumper&&timer.milliseconds()>200){

                targetRPM +=100;
                timer.reset();
            }
            if(gamepad1.dpad_up&&timer.milliseconds()>200){
                kP += .0001;
                timer.reset();
            }
            if(gamepad1.dpad_down&&timer.milliseconds()>200){
                kP -= .0001;
                timer.reset();
            }
            if(gamepad1.dpad_right&&timer.milliseconds()>200){
                kD += .0001;
                timer.reset();
            }
            if(gamepad1.dpad_left&&timer.milliseconds()>200){
                kD -= .0001;
                timer.reset();
            }

            if (gamepad1.yWasPressed()) {
                hood.setPosition(hood.getPosition() + 0.05);
            } else if (gamepad1.xWasPressed()) {
                hood.setPosition(hood.getPosition() - 0.05);
            }

            if(gamepad1.left_trigger>.5){
                intake.intakeBalls();
            }
            if(gamepad1.right_trigger>.5){
                intake.intakeBalls();
            }

            telemetry.addData("current", telems[0]);
            telemetry.addData("target", telems[1]);
            telemetry.addData("power", telems[2]);
            telemetry.addData("kP", kP);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.addLine();
            telemetry.addData("hood position", hood.getPosition());
            telemetry.update();
        }
    }
}
