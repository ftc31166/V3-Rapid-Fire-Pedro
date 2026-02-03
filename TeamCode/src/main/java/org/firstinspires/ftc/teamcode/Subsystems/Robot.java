package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.PinpointDrive;

public class Robot {


    public PinpointDrive drive;
    public Flywheels flywheels;
    public Gate gate;
    public Intake intake;
    public Turret turret;
    public Hood hood;

    Pose2d goal;

    public Robot(HardwareMap hardwareMap, Pose2d initPose, Pose2d goal){
        drive = new PinpointDrive(hardwareMap, initPose);
        flywheels = new Flywheels(hardwareMap);
        gate = new Gate( hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        hood = new Hood(hardwareMap);
        this.goal = goal;

    }
    public void driveFieldCentric(double leftX, double leftY, double rightX, boolean resetImu) {
        double y = leftY; // Remember, Y stick value is reversed
        double x = leftX;
        double rx = rightX;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if(resetImu){
            drive.resetImu();
        }

        double botHeading = drive.pose.heading.toDouble();

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        drive.customDrivePowers(frontLeftPower,backLeftPower,backRightPower,frontRightPower);

    }



}
