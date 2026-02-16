package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.PinpointDrive;

public class AutonFuncs  {
    ElapsedTime oscillate = new ElapsedTime();
    int oscCounter = 0;
    Robot robot;
    public PinpointDrive drive;
    Pose2d goal;
    public AutonFuncs(HardwareMap hardwareMap, Pose2d start, Pose2d Goal){
        robot = new Robot(hardwareMap,start,Goal );
        drive = robot.drive;
        goal = Goal;
    }

    public Action intakeOn(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.intake.intakeBalls();
                return false;
            }
        };
    }

    public Action intakeSlow(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.intake.slowIntake();
                return false;
            }
        };
    }

    public Action slowIntake(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.intake.slowIntake();
                return false;
            }
        };
    }



    public Action updateTurretRed(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.turret.setTargetAngle(45);
                robot.turret.update();
                return true;
            }
        };
    }
    public Action updateTurretBlue(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.turret.setTargetAngle(-45);
                robot.turret.update();
                return true;
            }
        };
    }

    public Action updateTurretRedFar(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.turret.setTargetAngle(70);
                robot.turret.update();
                return true;
            }
        };
    }
    public Action updateTurretBlueFar(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.turret.setTargetAngle(-70);
                robot.turret.update();
                return true;
            }
        };
    }




    public Action updateFlywheel(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.flywheels.setTargetRPM(3200);
                robot.flywheels.update();
                return true;
            }
        };
    }

    public Action updateFlywheelFar(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.flywheels.setTargetRPM(4350);
                robot.flywheels.update();
                return true;
            }
        };
    }





    public Action gateOpen(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.gate.gateOpen();
                return false;
            }
        };
    }

    public Action gateClose(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.gate.gateClosed();
                return false;
            }
        };
    }

    public Action updatePose() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                robot.drive.updatePoseEstimate();
                return true;
            }
        };
    }
    public Action zeroturret() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                robot.turret.setTargetAngle(0);
                robot.turret.update();
                return (robot.turret.turret.getCurrentPosition()!= 0) ;
            }
        };
    }
    public Action zeroturretBlue() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                robot.turret.setTargetAngle(5);
                robot.turret.update();
                return (robot.turret.turret.getCurrentPosition()!= 5) ;
            }
        };
    }

}
