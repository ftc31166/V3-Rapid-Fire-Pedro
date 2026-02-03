package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp
public class pidTestTurret extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap);

        double kP = .0015;
        double kI = 0;
        double kD = .000;
        double[] telems;

        double targetAng = 0;
        ElapsedTime timer = new ElapsedTime();
        if (isStopRequested()) return;
        waitForStart();
        while (opModeIsActive()){
            turret.kD = kD;
            turret.kP = kP;
            turret.kI = kI;
            turret.setCoeffs();


            turret.setTargetAngle(targetAng);

            telems = turret.update();


            if(gamepad1.b){
                turret.stop();

            }
            if(gamepad1.left_bumper&&timer.milliseconds()>200){

                targetAng -=5;
                timer.reset();
            }
            if(gamepad1.right_bumper&&timer.milliseconds()>200){

                targetAng +=5;
                timer.reset();
            }
            if(gamepad1.left_trigger>0&&timer.milliseconds()>200){

                targetAng -=20;
                timer.reset();
            }
            if(gamepad1.right_trigger>0&&timer.milliseconds()>200){

                targetAng +=20;
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
                kD += .000001;
                timer.reset();
            }
            if(gamepad1.dpad_left&&timer.milliseconds()>200){
                kD -= .000001;
                timer.reset();
            }
            telemetry.addData("current", telems[0]);
            telemetry.addData("target", telems[1]);
            telemetry.addData("power", telems[2]);
            telemetry.addData("target angle", targetAng);
            telemetry.addData("currang", telems[0]*360/turret.TICKS_PER_REV);
            telemetry.addData("kP", kP);
            telemetry.addData("kD", String.format("%.8f",kD));
            telemetry.update();
        }
    }
}
