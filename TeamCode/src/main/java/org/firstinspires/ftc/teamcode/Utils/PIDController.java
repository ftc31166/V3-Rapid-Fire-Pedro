package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double kP, kI, kD;

    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;

    private final ElapsedTime timer = new ElapsedTime();

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        lastTime = timer.seconds();
    }


    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        lastTime = timer.seconds();
    }


    public double calculate(double current, double target) {
        double error = target - current;

        double now = timer.seconds();
        double dt = now - lastTime;
        lastTime = now;

        if (dt <= 0) return 0;

        // Integral
        integral += error * dt;

        // Derivative
        double derivative = (error - lastError) / dt;
        lastError = error;
        // PIDF output
        return (kP * error)
                + (kI * integral)
                + (kD * derivative);
    }
}
