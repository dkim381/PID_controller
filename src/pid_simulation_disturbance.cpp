#include <iostream>
#include <fstream>
#include <algorithm>

// ================= PID Controller =================
class PID {
public:
    PID(double kp, double ki, double kd, double dt)
        : Kp(kp), Ki(ki), Kd(kd), dt(dt) {}

    void setSaturation(double umin, double umax) {
        u_min = umin;
        u_max = umax;
        use_saturation = true;
    }

    void setAntiWindupGain(double kaw) {
        K_aw = kaw;
    }

    // ===== Added getters for logging =====
    double getIntegrator() const { return I; }
    double getLastUnsat() const { return last_u_unsat; }
    double getLastSat()   const { return last_u_sat; }

    double compute(double reference, double measurement) {
        // 1. Error
        double e = reference - measurement;

        // 2. Derivative
        double D = (e - e_prev) / dt;

        // 3. Unsaturated control
        double u_unsat = Kp * e + Ki * I + Kd * D;

        // 4. Saturation
        double u_sat = u_unsat;
        if (use_saturation) {
            u_sat = std::clamp(u_unsat, u_min, u_max);
        }

        // ===== Store values for logging =====
        last_u_unsat = u_unsat;
        last_u_sat = u_sat;

        // 5. Anti-windup (back-calculation)
        I += (e + K_aw * (u_sat - u_unsat)) * dt;

        // 6. Save state
        e_prev = e;

        return u_sat;
    }

private:
    // Gains
    double Kp, Ki, Kd;
    double K_aw = 0.0;

    // State
    double I = 0.0;
    double e_prev = 0.0;

    // Timing
    double dt;

    // Saturation
    bool use_saturation = false;
    double u_min = -1e9;
    double u_max =  1e9;

    // ===== Added for logging =====
    double last_u_unsat = 0.0;
    double last_u_sat   = 0.0;
};

// ================= Main Simulation =================
int main() {
    // -------- Design Specifications --------
    const double dt = 0.001;       // 1 ms
    const double sim_time = 20.0;  // seconds
    const double reference = 3.0;

    // Plant parameters: x_dot = -x + (u + disturbance)
    const double a = 1.0;
    const double b = 1.0;

    // PID gains (example tuned values)
    double Kp = 4.0;
    double Ki = 3.0;
    double Kd = 0.02;
    double Kaw = 5.0;

    // Actuator limits
    double u_min = -3.0;
    double u_max =  3.0;

    // -------- Controller --------
    PID pid(Kp, Ki, Kd, dt);
    pid.setSaturation(u_min, u_max);
    pid.setAntiWindupGain(Kaw);

    // -------- Plant state --------
    double x = 0.0;
    double y = 0.0;

    // -------- Log file --------
    std::ofstream log("pid_log_ref3_lim3_dis1_kaw5.csv");
    log << "time,reference,output,u_sat,u_unsat,integrator,disturbance\n";

    // -------- Simulation Loop --------
    int steps = static_cast<int>(sim_time / dt);
    for (int i = 0; i < steps; i++) {
        double t = i * dt;

        // Disturbance (pulse)
        double disturbance = 0.0;
        if (t >= 5.0 && t < 10.0) {
            disturbance = -1.0;
        }

        // Control
        double u_sat = pid.compute(reference, y);
        double u_unsat = pid.getLastUnsat();
        double I = pid.getIntegrator();

        // Plant update (Forward Euler)
        double x_dot = -a * x + b * (u_sat + disturbance);
        x += x_dot * dt;
        y = x;

        // Log
        log << t << "," << reference << "," << y << ","
            << u_sat << "," << u_unsat << "," << I << ","
            << disturbance << "\n";
    }

    log.close();
    std::cout << "Simulation finished. Results saved.\n";
    return 0;
}
