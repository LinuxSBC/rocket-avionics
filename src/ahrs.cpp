// AHRS LOOP
// steps: Single-file AHRS loop: q0 -> bias-corrected sensors -> gyro propagate (q1) ->
// normalise (q2) -> Madgwick correction (q3) -> normalise (q4 final orientation)
// Replace the placeholder sensor reads functions with our IMU reads. DOUBLE CHECK I HAVE THE RIGHT ONES!!

#ifdef ARDUINO
#include <Arduino.h>
#include <Adafruit_LSM6DS.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#else
// Non-Arduino build: provide minimal stubs so static analysis / host builds succeed.
// These stubs intentionally do not provide sensor functionality; they only allow compilation.
#include <cstdint>
#include <cmath>
#include <vector>
#include <array>
#include <iostream>
#include <string>

// Minimal Serial stub used in this file
struct SerialClass {
    void begin(int) {}
    void println(const char* s){ std::cout << s << std::endl; }
    void println(const std::string& s){ std::cout << s << std::endl; }
    template<typename... Args> void printf(const char* fmt, ...) {}
     operator bool() const { return true; }
};
static SerialClass Serial;

// Minimal time stub
static unsigned long micros(){ return 0; }

// delay stub
static void delay(int ms) {}

// Basic sensors_event_t shape used by this file
struct sensors_event_t {
    struct { double x=0,y=0,z=0; } gyro;
    struct { double x=0,y=0,z=0; } acceleration;
    struct { double x=0,y=0,z=0; } magnetic;
};

// Minimal Adafruit-like stubs used by the code
class Adafruit_LSM6DS {
public:
    bool getEvent(sensors_event_t* accel_ev, sensors_event_t* gyro_ev){ (void)accel_ev; (void)gyro_ev; return false; }
    bool begin_I2C(){ return true; }
    void setAccelRange(int) {}
    void setGyroRange(int) {}
};
class Adafruit_LIS3MDL {
public:
    bool getEvent(sensors_event_t* mag_ev){ (void)mag_ev; return false; }
    bool begin_I2C(){ return true; }
    void setPerformanceMode(int) {}
    void setOperationMode(int) {}
};

// Provide simple definitions for constants referenced in the code
#define LSM6DS_ACCEL_RANGE_8_G 8
#define LSM6DS_GYRO_RANGE_2000_DPS 2000
#define LIS3MDL_ULTRA_HIGHMODE 1
#define LIS3MDL_CONTINUOUSMODE 1

#endif

#include <vector>
#include <cmath>

// Component Identifications
// 3D Vectors Containers
using Vec3 = std::array<double,3>;

// Quaternions Containers
using Quat = std::array<double,4>; // [w, x, y, z]

// Squaring, norm, clamp helpers
static inline double sq(double x){ return x*x; }

// 3D vector norm
static inline double norm3(const Vec3 &v){ return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }

// Limit values for gain stability(sensor values and noise handling)
static inline double clamp(double v, double lo, double hi){ return v < lo ? lo : (v > hi ? hi : v); }

//  Quaternion Utitlities/Data Set-up
Quat quatMultiply(const Quat &p, const Quat &q){

    // Hamilton product p ⊗ q
    Quat r;
    r[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
    r[1] = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2];
    r[2] = p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1];
    r[3] = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0];
    return r;
}

// quaternion conjugate
Quat quatConjugate(const Quat &q){
    return Quat{q[0], -q[1], -q[2], -q[3]};
}

// quaternion norm
double quatNorm(const Quat &q){
    return std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
}

// normalise quaternion
void quatNormalise(Quat &q){
    double n = quatNorm(q);
    if(n <= 1e-15) { q = {1,0,0,0}; return; }
    q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
}

// convert quaternion using rotation matrix (body -> earth if q maps body->earth)
std::array<std::array<double,3>,3> quatToMatrix(const Quat &q){
    double w=q[0], x=q[1], y=q[2], z=q[3];
    std::array<std::array<double,3>,3> R{};
    R[0][0] = 1.0 - 2.0*(y*y + z*z);
    R[0][1] = 2.0*(x*y - w*z);
    R[0][2] = 2.0*(x*z + w*y);
    R[1][0] = 2.0*(x*y + w*z);
    R[1][1] = 1.0 - 2.0*(x*x + z*z);
    R[1][2] = 2.0*(y*z - w*x);
    R[2][0] = 2.0*(x*z - w*y);
    R[2][1] = 2.0*(y*z + w*x);
    R[2][2] = 1.0 - 2.0*(x*x + y*y);
    return R;
}

// rotate a body-vector v_b into earth frame using quaternion q (body->earth)
Vec3 rotateBodyToEarth(const Quat &q, const Vec3 &v_b){
    // p = q ⊗ [0,v_b] ⊗ q*
    Quat p{0, v_b[0], v_b[1], v_b[2]};
    Quat t = quatMultiply(q, p);
    Quat res = quatMultiply(t, quatConjugate(q));
    return Vec3{res[1], res[2], res[3]}; // last 3 are vector part
}

// small helper: axis-angle -> quaternion exact
Quat axisAngleToQuat(const Vec3 &axis, double angle){
    double half = angle*0.5;
    double s = std::sin(half);
    return Quat{std::cos(half), axis[0]*s, axis[1]*s, axis[2]*s};
}

// build delta quaternion(propagation) from angular rate omega (rad/s) and dt
Quat deltaQuatFromGyro(const Vec3 &omega, double dt){
    double wmag = norm3(omega);
    if(wmag < 1e-12){
        // tiny rotation -> small-angle approx: q ≈ [1, 0.5*ω*dt]
        return Quat{1.0, 0.5*omega[0]*dt, 0.5*omega[1]*dt, 0.5*omega[2]*dt};
    } else {
        Vec3 axis{omega[0]/wmag, omega[1]/wmag, omega[2]/wmag};
        double theta = wmag * dt;
        return axisAngleToQuat(axis, theta);
    }
}

// Madgwick correction step, I dont know how to get the library from adafruit so I just did it manually/mathmatically!

// q_pred: predicted quaternion (gyro-propagated & normalised) -> q2
// a: accelerometer (bias-corrected) in body frame (no normalising required; we can normalise inside)
// m: magnetometer (bias-corrected & soft-iron corrected) in body frame
// beta: algorithm gain (0.0 = no correction, larger faster)
// dt: time step (s)
// returns corrected sensor fused quaternion as quaternion q3 (unnormalised)
Quat madgwickCorrectionStep(const Quat &q_pred, const Vec3 &a, const Vec3 &m, const Vec3 &gyro, double beta, double dt){

// normalise accelerometer measurement
    Vec3 a_n = a;
    double na = norm3(a_n);
    if(na < 1e-12) return q_pred; // can't correct without accel
    a_n[0]/=na; a_n[1]/=na; a_n[2]/=na;

// normalise magnetometer measurement
    Vec3 m_n = m;
    double nm = norm3(m_n);
    if(nm < 1e-12) return q_pred;
    m_n[0]/=nm; m_n[1]/=nm; m_n[2]/=nm;

    double q1=q_pred[0], q2=q_pred[1], q3=q_pred[2], q4=q_pred[3];

// Reference direction of Earth's magnetic field (in body frame) --- Madgwick derivation uses this.
    // Compute objective function gradient (following Madgwick 2010 equations, check paper).
    // For full derivation see Madgwick's report; here we implement the standard gradient step.
// variables
    double _2q1=2.0*q1, _2q2=2.0*q2, _2q3=2.0*q3, _2q4=2.0*q4;
    double _4q1=4.0*q1, _4q2=4.0*q2, _4q3=4.0*q3;
    double _8q2=8.0*q2, _8q3=8.0*q3;
    double _4q4 = 4.0 * q4;
    double q1q1=q1*q1, q2q2=q2*q2, q3q3=q3*q3, q4q4=q4*q4;

// Gradient of f (accel) part (from Madgwick): need to use specific q# combinations
    // math behind thw values= f = predicted_gravity - measured_gravity; predicted_gravity = [2(q2 q4 - q1 q3), 2(q1 q2 + q3 q4), q1^2 - q2^2 - q3^2 + q4^2]
    Vec3 f;
    f[0] = 2.0*(q2*q4 - q1*q3) - a_n[0];
    f[1] = 2.0*(q1*q2 + q3*q4) - a_n[1];
    f[2] = q1q1 - q2q2 - q3q3 + q4q4 - a_n[2];

// Jacobian J (3x4) lines combined into gradient g = J^T * f  (this expands to 4 components, as we need it to build a quaternion)
    // Madgwick 2010 paper's compact expression:
    Vec3 g_accel_vec;
    double grad1 = _4q1 * f[2] - _2q3 * f[0] + _2q2 * f[1];
    double grad2 = _4q2 * f[2] + _2q4 * f[0] + _2q1 * f[1] - _8q2 * f[2];
    double grad3 = _4q3 * f[2] - _2q1 * f[0] + _2q4 * f[1] - _8q3 * f[2];
    double grad4 = _4q4 * f[2] + _2q2 * f[0] + _2q3 * f[1];

// For magnetometer part we should compute reference direction and its gradient.
    // compute Earth's magnetic field in body frame and gradient.
    // Compute h = q ⊗ m_n ⊗ q*  (magnetic field in earth/body frame)
    Quat p_m{0, m_n[0], m_n[1], m_n[2]};
    Quat t = quatMultiply(q_pred, p_m);
    Quat hq = quatMultiply(t, quatConjugate(q_pred));
    Vec3 h{hq[1], hq[2], hq[3]};

// Projection of h onto x-y plane of Earth (reference)
    Vec3 b; // b = [bx, 0, bz] as in Madgwick
    b[0] = std::sqrt(h[0]*h[0] + h[1]*h[1]);
    b[1] = 0.0;
    b[2] = h[2];

// compute the gradient of magnetometer using a simplified combined gradient:
    // Compute an approximate mag error vector between predicted and measured (in body frame!)
    Vec3 m_pred = h; // already in body frame from above calculation
    Vec3 mag_err = { m_pred[0]/(norm3(m_pred)+1e-12) - m_n[0], m_pred[1]/(norm3(m_pred)+1e-12) - m_n[1], m_pred[2]/(norm3(m_pred)+1e-12) - m_n[2] };

// Build a simple magnetometer gradient approximation using cross products of predicted vs measured
    // This is less exact than full Madgwick derivation but should work well in practice as a correction term, simpler for us
    Vec3 g_mag_vec = {
        2.0*( q3*mag_err[2] - q4*mag_err[1] + q1*mag_err[0] - q2*mag_err[2] ),
        2.0*( q4*mag_err[0] - q1*mag_err[2] + q2*mag_err[1] - q3*mag_err[0] ),
        2.0*( q1*mag_err[1] - q2*mag_err[0] + q3*mag_err[2] - q4*mag_err[1] )
    };
// Convert this vector into 4-component approximate gradient (spread across all axes)
    double magGrad1 = 0.0;
    double magGrad2 = g_mag_vec[0];
    double magGrad3 = g_mag_vec[1];
    double magGrad4 = g_mag_vec[2];

// Combine gradients (accel-heavy + mag small contribution)
    double g1 = grad1 + magGrad1;
    double g2 = grad2 + magGrad2;
    double g3 = grad3 + magGrad3;
    double g4 = grad4 + magGrad4;

// Normalise gradient
    double gn = std::sqrt(g1*g1 + g2*g2 + g3*g3 + g4*g4);
    if(gn > 0.0){
        g1/=gn; g2/=gn; g3/=gn; g4/=gn;
    } else {
        // if mag fails
        g1 = grad1; g2 = grad2; g3 = grad3; g4 = grad4;
        double gn2 = std::sqrt(g1*g1 + g2*g2 + g3*g3 + g4*g4);
        if(gn2>0){ g1/=gn2; g2/=gn2; g3/=gn2; g4/=gn2; }
    }

// Compute quaternion rate from gyroscope (already bias-corrected), math is q_dot = 0.5 * q ⊗ [0,ω]
    Quat omega_q = {0.0, gyro[0], gyro[1], gyro[2]};
    Quat q_dot = quatMultiply(q_pred, omega_q);
    q_dot[0] *= 0.5; q_dot[1] *= 0.5; q_dot[2] *= 0.5; q_dot[3] *= 0.5;

// Apply gradient descent corrective step scaled by beta: q_dot = q_dot - beta * gradient
    q_dot[0] -= beta * g1;
    q_dot[1] -= beta * g2;
    q_dot[2] -= beta * g3;
    q_dot[3] -= beta * g4;

// Integrate to get corrected quaternion (Simple Euler integration)
    Quat q_corr;
    q_corr[0] = q_pred[0] + q_dot[0] * dt;
    q_corr[1] = q_pred[1] + q_dot[1] * dt;
    q_corr[2] = q_pred[2] + q_dot[2] * dt;
    q_corr[3] = q_pred[3] + q_dot[3] * dt;

    return q_corr;
}

// Bias computation (mean across samples)
Vec3 computeBiasMean(const std::vector<Vec3> &samples){
    Vec3 s{0,0,0};
    if(samples.empty()) return s;
    for(const auto &v: samples){ s[0]+=v[0]; s[1]+=v[1]; s[2]+=v[2]; }
    double N = double(samples.size());
    return Vec3{ s[0]/N, s[1]/N, s[2]/N };
}

//  Hardware Sensor Read Functions, instead of the library, this is direct senor readings which is why I have unit conversions here
Adafruit_LSM6DS sox;
Adafruit_LIS3MDL lis3mdl;

bool readIMU(Vec3 &gyro, Vec3 &accel, Vec3 &mag){
    sensors_event_t gyro_event, accel_event, mag_event;

// Read LSM6DSOX (gyro + accel)
    if(!sox.getEvent(&accel_event, &gyro_event)){
        return false;
    }

// Read LIS3MDL (magnetometer)
    if(!lis3mdl.getEvent(&mag_event)){
        return false;
    }

// Converting to proper units
    gyro = {
        gyro_event.gyro.x,      // rad/s
        gyro_event.gyro.y,
        gyro_event.gyro.z
    };

    accel = {
        accel_event.acceleration.x,  // m/s²
        accel_event.acceleration.y,
        accel_event.acceleration.z
    };

    mag = {
        mag_event.magnetic.x,    // uT
        mag_event.magnetic.y,
        mag_event.magnetic.z
    };

    return true;
}

// Main Loop AHRS function Using the above utilities

// AHRS Current State
struct AHRSState {
    Quat q = {1.0, 0.0, 0.0, 0.0};  // Current orientation quaternion
    Vec3 gyroBias = {0.0, 0.0, 0.0};
    Vec3 accelBias = {0.0, 0.0, 0.0};
    Vec3 magBias = {0.0, 0.0, 0.0};
    double beta = 0.1;  // Madgwick filter gain
    unsigned long lastUpdate = 0;
    bool isCalibrated = false;
    unsigned long calibrationStart = 0;
    std::vector<Vec3> gyroSamples, accelSamples, magSamples;
};

AHRSState ahrs;

void updateAHRS(AHRSState &state){
    Vec3 gyroRaw, accelRaw, magRaw;

    if(!readIMU(gyroRaw, accelRaw, magRaw)){
        Serial.println("IMU read failed!");
        return;
    }
// Used to Calculate delta time
    unsigned long now = micros();
    double dt = (now - state.lastUpdate) / 1000000.0;
    state.lastUpdate = now;

    if(dt <= 0 || dt > 0.1){
        dt = 0.01;
    }

// Auto-calibration phase (first 3 seconds or longer if needed)
    if(!state.isCalibrated){
        if(state.calibrationStart == 0){
            state.calibrationStart = now;
            Serial.println("Auto-calibration started - keep IMU stationary...");
        }
// Collect calibration samples for 3 seconds; we can adjust this duration as needed??
        if(now - state.calibrationStart < 3000000){
            state.gyroSamples.push_back(gyroRaw);
            state.accelSamples.push_back(accelRaw);
            state.magSamples.push_back(magRaw);

// Display calibration progress
            static unsigned long lastCalibPrint = 0;
            if(now - lastCalibPrint > 500000){
                float progress = (now - state.calibrationStart) / 3000000.0 * 100.0;
                Serial.printf("Calibrating... %.0f%%\n", progress);
                lastCalibPrint = now;
            }
            return;
        } else {

// Calibration complete from the steps above - now compute biases
            state.gyroBias = computeBiasMean(state.gyroSamples);
            state.accelBias = computeBiasMean(state.accelSamples);
            state.magBias = computeBiasMean(state.magSamples);

// Adjustment for rocket launch position (Z-up coordinate system/nose up)
            state.accelBias[2] += 9.81;

            state.isCalibrated = true;
            state.gyroSamples.clear();
            state.accelSamples.clear();
            state.magSamples.clear();

            Serial.println("Calibration complete!");
            Serial.printf("Gyro bias: %.4f, %.4f, %.4f rad/s\n",
                         state.gyroBias[0], state.gyroBias[1], state.gyroBias[2]);
            Serial.printf("Accel bias: %.4f, %.4f, %.4f m/s²\n",
                         state.accelBias[0], state.accelBias[1], state.accelBias[2]);
        }
    }

//  COMPLETE Q0 → Q4
// Step 0: Initial quaternion (q0) - previous orientation
    Quat q0 = state.q;

// Step 1: Bias correction and sensor adjustment
    Vec3 gyroCorrected = {
        gyroRaw[0] - state.gyroBias[0],
        gyroRaw[1] - state.gyroBias[1],
        gyroRaw[2] - state.gyroBias[2]
    };

    Vec3 accelCorrected = {
        accelRaw[0] - state.accelBias[0],
        accelRaw[1] - state.accelBias[1],
        accelRaw[2] - state.accelBias[2]
    };

    Vec3 magCorrected = {
        magRaw[0] - state.magBias[0],
        magRaw[1] - state.magBias[1],
        magRaw[2] - state.magBias[2]
    };

// Step 2: Gyro propagation (q1) - integrate angular rates
    Quat delta_q = deltaQuatFromGyro(gyroCorrected, dt);
    Quat q1 = quatMultiply(q0, delta_q);

// Step 3: Normalise (q2) - maintain quaternion unit length
    Quat q2 = q1;
    quatNormalise(q2);

// Step 4: Madgwick sensor fusion correction (q3) - fuse with accelerometer and magnetometer
    Quat q3 = madgwickCorrectionStep(q2, accelCorrected, magCorrected, gyroCorrected, state.beta, dt);

// Step 5: Final normalisation (q4) - ensure valid quaternion
    Quat q4 = q3;
    quatNormalise(q4);

// Update state with final quaternion, final state
    state.q = q4;

// EARTH FRAME CONVERSIONS
// Convert body-frame measurements to earth frame for control systems
    Vec3 earthAccel = rotateBodyToEarth(q4, accelCorrected);
    Vec3 earthGyro = rotateBodyToEarth(q4, gyroCorrected);
    Vec3 earthMag = rotateBodyToEarth(q4, magCorrected);

// CONTINUOUS ORIENTATION MONITORING/Active Tracking
    static unsigned long lastPrint = 0;
    if(now - lastPrint > 100000){

// Convert quaternion to Euler angles
        double roll = atan2(2.0 * (q4[0] * q4[1] + q4[2] * q4[3]),
                     1.0 - 2.0 * (q4[1] * q4[1] + q4[2] * q4[2]));
        double pitch = asin(2.0 * (q4[0] * q4[2] - q4[3] * q4[1]));
        double yaw = atan2(2.0 * (q4[0] * q4[3] + q4[1] * q4[2]),
                    1.0 - 2.0 * (q4[2] * q4[2] + q4[3] * q4[3]));

        // Display orientation and earth-frame data
        Serial.println("ROCKET ORIENTATION");
        Serial.printf("Quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n",
                     q4[0], q4[1], q4[2], q4[3]);
        Serial.printf("Euler: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\n",
                     roll * 180/M_PI, pitch * 180/M_PI, yaw * 180/M_PI);
        Serial.printf("Earth Accel: X=%.2f, Y=%.2f, Z=%.2f m/s²\n",
                     earthAccel[0], earthAccel[1], earthAccel[2]);
        Serial.printf("Earth Gyro: X=%.2f, Y=%.2f, Z=%.2f rad/s\n",
                     earthGyro[0], earthGyro[1], earthGyro[2]);
        Serial.println("==========================");

        lastPrint = now;
    }
}
// End of AHRS update, final loop main part , debugging prints
void setup(){
    Serial.begin(115200);
    while(!Serial) delay(10);

    Serial.println("AHRS SYSTEM INITIALISING");
    Serial.println("==================================");

    // Initialise IMUs
    if(!sox.begin_I2C()){
        Serial.println("Failed to find LSM6DSOX chip!");
        while(1) delay(10);
    }
    sox.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
    sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    Serial.println("LSM6DSOX initialised");

    if(!lis3mdl.begin_I2C()){
        Serial.println("Failed to find LIS3MDL chip!");
        while(1) delay(10);
    }
    lis3mdl.setPerformanceMode(LIS3MDL_ULTRA_HIGHMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    Serial.println("LIS3MDL initialised");

    ahrs.lastUpdate = micros();
    Serial.println("AHRS- continuous orientation tracking");
    Serial.println("=========================================================");
}
// Continuous AHRS loop, to keep q0 -> q4 going
void loop(){
    updateAHRS(ahrs);

//Rocket control inside loop, AHRS loops continuously after each task and keeps orientation current for the next one,
// I will leave that to you guys to implement as per our rocket target requirements

    // Use ahrs.q for current orientation
    // Launch detection, apogee detection, attitude control logic, data logging, and recovery system, would go here inside the loop

}
