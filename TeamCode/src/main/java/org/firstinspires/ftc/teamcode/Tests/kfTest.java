package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Flywheels;

@TeleOp
public class kfTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Flywheels flywheels = new Flywheels(hardwareMap);


        ElapsedTime timer = new ElapsedTime();
        if (isStopRequested()) return;
        waitForStart();
        while (opModeIsActive()){
            flywheels.lfly.setPower(1);
            flywheels.rfly.setPower(1);
            telemetry.addData("rpm", flywheels.getRPM());
            telemetry.update();
        }
    }
}
