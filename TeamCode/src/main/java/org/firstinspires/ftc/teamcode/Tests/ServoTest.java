package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class,"servo");
        ElapsedTime timer = new ElapsedTime();
        if (isStopRequested()) return;
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.dpad_up&&timer.milliseconds()>300){
                servo.setPosition(servo.getPosition()+.1);
                timer.reset();

            }if(gamepad1.dpad_down&&timer.milliseconds()>300){
                servo.setPosition(servo.getPosition()-.1);
                timer.reset();

            }
            if(gamepad1.dpad_right&&timer.milliseconds()>300){
                servo.setPosition(servo.getPosition()+.05);
                timer.reset();
            }
            if(gamepad1.dpad_left&&timer.milliseconds()>300){
                servo.setPosition(servo.getPosition()-.5);
                timer.reset();

            }
            if(gamepad1.left_bumper&&timer.milliseconds()>300){
                servo.setPosition(servo.getPosition()+.01);
                timer.reset();
            }
            if(gamepad1.right_bumper&&timer.milliseconds()>300){
                servo.setPosition(servo.getPosition()+.01);
                timer.reset();
            }
            telemetry.addData("Servo Pos", servo.getPosition());
            telemetry.update();
        }
    }
}
