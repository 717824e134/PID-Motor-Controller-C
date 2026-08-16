#include <stdio.h>
#include "pico/stdlib.h"

// --- 1. The Motor Model (Digital Twin) ---
float simulate_motor(float pwm_power) {
    static float current_speed = 0.0;
    current_speed = (current_speed * 0.95) + (pwm_power * 0.05);
    return current_speed;
}

// --- 2. The PID Controller Logic ---
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral_sum;
    float previous_error;
    float out_max;
    float out_min;
} PID_Controller;

float compute_pid(PID_Controller *pid, float setpoint, float measured_value, float dt) {
    float error = setpoint - measured_value;
    
    float P = pid->Kp * error;
    
    pid->integral_sum += (error * dt);
    float I = pid->Ki * pid->integral_sum;
    
    float D = pid->Kd * ((error - pid->previous_error) / dt);
    
    float output = P + I + D;
    
    // Clamp output so we don't send impossible power levels
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;
    
    pid->previous_error = error;
    return output;
}

// --- 3. The Main System ---
int main() {
    stdio_init_all();
    
    // Initialize our PID Controller with some starter tuning values
    PID_Controller motor_pid = {
        .Kp = 2.0,   // Proportional (Pushes hard when error is large)
        .Ki = 0.5,   // Integral (Pushes gently to fix tiny, lingering errors)
        .Kd = 0.1,   // Derivative (Pushes back against fast changes to prevent overshoot)
        .integral_sum = 0.0,
        .previous_error = 0.0,
        .out_min = 0.0,
        .out_max = 100.0 // Max PWM power
    };
    
    float target_rpm = 75.0; // The speed we WANT (Try changing this later!)
    float current_pwm = 0.0;
    float dt = 0.1; // 100ms time step (0.1 seconds)

    while (true) {
        // 1. Read the sensor (our digital twin actual speed)
        float actual_speed = simulate_motor(current_pwm);
        
        // 2. Calculate new power using the PID algorithm
        current_pwm = compute_pid(&motor_pid, target_rpm, actual_speed, dt);
        
        // 3. Print the results to the serial monitor
        printf("Target: %.2f | Actual: %.2f RPM | PWM Output: %.2f\n", target_rpm, actual_speed, current_pwm);
        
        // 4. Wait for the next time step
        sleep_ms(100);
    }
}
