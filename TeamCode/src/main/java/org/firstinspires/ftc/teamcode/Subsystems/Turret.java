package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.PIDController;


public class Turret {

    public double kP = 0.002;
    public double kI = 0;
    public double kD = 0.000003;
    public double turretOffset = 0;
     double targetTicks = 0;
    public double TICKS_PER_REV = 1700;
    DcMotorEx turret;
    PIDController pid;



    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "tur");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setCoeffs();
    }
    public void stop(){
        turret.setPower(0);
    }
    public void setCoeffs(){
        pid = new PIDController(kP, kI, kD);
    }

    public void setTargetAngle(double angleDeg) {
        double angle =  ((angleDeg + 180) % 360 + 360) % 360 - 180;
        targetTicks = angle * TICKS_PER_REV/ 360.0 ;
    }
    public double autoAim(Pose2d drive, Pose2d goal){
        double dx = goal.position.x - drive.position.x;
        double dy = goal.position.y - drive.position.y;
        double degrees = Math.toDegrees(Math.atan2(dy,dx));
        double heading = Math.toDegrees(drive.heading.toDouble());

        return degrees-heading+turretOffset;
    }

    public double[] update() {
        double currentTicks = turret.getCurrentPosition();
        double power = pid.calculate(currentTicks, targetTicks);
        power = Math.max(-.5,Math.min(.5,power));
        turret.setPower(power);
        double[] ret = {currentTicks, targetTicks, power};
        return ret;
    }
}
