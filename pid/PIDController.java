package pid;

public class PIDController {
    private double kp;  // Proportional gain
    private double ki;  // Integral gain
    private double kd;  // Derivative gain

    private double previousError;
    private double integral;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.previousError = 0;
        this.integral = 0;
    }

    public double calculatePIDOutput(double point, double processVariable) {
        double error = point - processVariable;
        // Proportional term
        double proportional = kp * error;

        // Integral term
        integral += ki * error;

        // Derivative term
        double derivative = kd * (error - previousError);

        // PID output
        double output = proportional + integral + derivative;

        // Update previous error
        previousError = error;

        return output;
    }

    public double calculatePDOutput(double point, double processVariable) {
        double error = point - processVariable;
        // Proportional term
        double proportional = kp * error;

        // Derivative term
        double derivative = kd * (error - previousError);

        // PID output
        double output = proportional + derivative;

        // Update previous error
        previousError = error;

        return output;
    }
}