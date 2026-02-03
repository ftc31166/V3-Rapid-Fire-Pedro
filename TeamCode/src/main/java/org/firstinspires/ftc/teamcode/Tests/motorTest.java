package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class motorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.get(DcMotor.class,"motor");
        DcMotor motor1 = hardwareMap.get(DcMotor.class,"motor1");
        ElapsedTime timer = new ElapsedTime();
        if (isStopRequested()) return;
        double powerOut = 0;
        double powerIn = 0;
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.a){
                motor.setPower(powerOut);
            }if(gamepad1.b){
                motor.setPower(0);
            }
            if(gamepad1.dpad_right && timer.milliseconds()>300){
               powerOut += .1;
               timer.reset();
            }
            if(gamepad1.dpad_left && timer.milliseconds()>300){
                powerOut -= .1;
                timer.reset();
            }
            if (gamepad1.x){
                motor1.setPower(powerIn);
            }
            if (gamepad1.y){
                motor1.setPower(0);
            }
            if (gamepad1.dpad_up && timer.milliseconds()>300){
                powerIn += .1;
                timer.reset();
            }
            if (gamepad1.dpad_down  && timer.milliseconds()>300){
                powerIn -= .1;
                timer.reset();
            }
            telemetry.addData("Outtake Motor power", powerOut);
            telemetry.addData("Intake Motor power", powerIn);
            telemetry.update();
        }
    }
}
