// File:          my_controller.cpp
// Date:          2024-8-29
// Description:
// Author:        HenningHe
// Modifications:

#include <array>
#include <chrono>
#include <cassert>
#include <Eigen/Dense>
#include "shared_data.hpp"
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

using namespace webots;

// [ty, tx] = f1(tML, tMR, d, L1, h1, h2)                   已有
// [\dot{ty}, \dot{tx}] = J [\dot{tML}, \dot{tMR}]          已有
// [tML, tMR] = f2(tx, ty, d, L1, h1, h2)                   已有
// [Tau_tML, Tau_tMR] = J^T [Tau_ty, Tau_tx]                已有
// [\dot{tML}, \dot{tMR}] = J2 [\dot{tx}, \dot{ty}]         已有
// [Tau_ty, Tau_tx] = J2^T [Tau_tML, Tau_tMR]               已有

std::array<double, 2> ankle_ik(double d, double L1, double h1, double h2, double tx, double ty) {
    double cx = cos(tx);
    double sx = sin(tx);
    double cy = cos(ty);
    double sy = sin(ty);

    double AL = -L1 * L1 * cy + L1 * d * sx * sy;
    double BL = -L1 * L1 * sy + L1 * h1 - L1 * d * sx * cy;
    double CL = -(L1 * L1 + d * d - d * d * cx - L1 * h1 * sy - d * h1 * sx * cy);
    double LenL = sqrt(AL * AL + BL * BL);

    double AR = -L1 * L1 * cy - L1 * d * sx * sy;
    double BR = -L1 * L1 * sy + L1 * h2 + L1 * d * sx * cy;
    double CR = -(L1 * L1 + d * d - d * d * cx - L1 * h2 * sy + d * h2 * sx * cy);
    double LenR = sqrt(AR * AR + BR * BR);

    if (LenL <= abs(CL) || LenR <= abs(CR)) {
        return {0.0, 0.0};
    } else {
        double tL_1 = asin(CL / LenL) - asin(AL / LenL);
        double tR_1 = asin(CR / LenR) - asin(AR / LenR);
        return {tL_1, tR_1};
    }
}

double calculateTx(double mL, double mR, double d, double L1, double h1, double h2) {
    double halfSumAngles = (mL / 2.0) + (mR / 2.0);
    double sinHalfSum = std::sin(halfSumAngles);
    double cosHalfSum = std::cos(halfSumAngles);

    double numerator = L1 * (h2 - h1) * sinHalfSum
                     + L1 * h1 * std::sin(mL)
                     - L1 * h2 * std::sin(mR);

    double denominator = d * (h1 + h2) * cosHalfSum;

    if (denominator == 0) {
        throw std::runtime_error("Denominator in tx calculation is zero. Check input values.");
    }

    // Calculate the arcsine value and return it
    double sinTx = numerator / denominator;
    // Clamp the value to the range [-1, 1] to avoid domain errors in asin due to numerical precision issues
    if (sinTx < -1.0) sinTx = -1.0;
    if (sinTx > 1.0) sinTx = 1.0;

    return asin(sinTx);
}

double calculateTxML(double tML, double tMR, double d, double L1, double h1, double h2) {
    if (std::abs(tML) > 1.52) {
        tML = (tML / std::abs(tML)) * 1.52;
    }

    if (std::abs(tMR) > 1.52) {
        tMR = (tMR / std::abs(tMR)) * 1.52;
    }

    static double pre_result = 0;

    double b_t19_tmp;
    double t12;
    double t14;
    double t15;
    double t17;
    double t19;
    double t19_tmp;
    double t5;
    double tXtML_tmp;

    t5 = 1.0 / d;
    t12 = 1.0 / (h1 + h2);
    t14 = tML / 2.0 + tMR / 2.0;
    t15 = std::cos(t14);
    t14 = std::sin(t14);
    t17 = 1.0 / (t15 * t15);
    t19_tmp = L1 * (h1 - h2);
    b_t19_tmp = L1 * h1;
    t19 = (L1 * h2 * std::sin(tMR) - b_t19_tmp * std::sin(tML)) + t19_tmp * t14;
    tXtML_tmp = t5 * t12;
    
    double result = -1.0 / std::sqrt(-(t5 * t5) * (t12 * t12) * t17 * (t19 * t19) + 1.0) *
                    (tXtML_tmp * (t19_tmp * t15 / 2.0 - b_t19_tmp * std::cos(tML)) / t15 +
                    tXtML_tmp * t14 * t17 * t19 / 2.0);
    if(std::isnan(result)) {
        result = pre_result;
    } else {
        pre_result = result;
    }
}

double calculateTxMR(double tML, double tMR, double d, double L1, double h1, double h2) {
    double b_t19_tmp;
    double t12;
    double t14;
    double t15;
    double t17;
    double t19;
    double t19_tmp;
    double t5;
    double tXtMR_tmp;

    t5 = 1.0 / d;
    t12 = 1.0 / (h1 + h2);
    t14 = tML / 2.0 + tMR / 2.0;
    t15 = std::cos(t14);
    t14 = std::sin(t14);
    t17 = 1.0 / (t15 * t15);
    t19_tmp = L1 * (h1 - h2);
    b_t19_tmp = L1 * h2;
    t19 = (b_t19_tmp * std::sin(tMR) - L1 * h1 * std::sin(tML)) + t19_tmp * t14;
    tXtMR_tmp = t5 * t12;
    return -1.0 / std::sqrt(-(t5 * t5) * (t12 * t12) * t17 * (t19 * t19) + 1.0) *
           (tXtMR_tmp * (t19_tmp * t15 / 2.0 + b_t19_tmp * std::cos(tMR)) / t15 +
            tXtMR_tmp * t14 * t17 * t19 / 2.0);
}

static double rt_powd_snf(double u0, double u1)
{
    double y;
    if (std::isnan(u0) || std::isnan(u1)) {
        y = NAN;
    } else {
        double d;
        double d1;
        d = std::abs(u0);
        d1 = std::abs(u1);
        if (std::isinf(u1)) {
            if (d == 1.0) {
                y = 1.0;
            } else if (d > 1.0) {
                if (u1 > 0.0) {
                    y = INFINITY;
                } else {
                    y = 0.0;
                }
            } else if (u1 > 0.0) {
                y = 0.0;
            } else {
                y = INFINITY;
            }
        } else if (d1 == 0.0) {
            y = 1.0;
        } else if (d1 == 1.0) {
            if (u1 > 0.0) {
                y = u0;
            } else {
                y = 1.0 / u0;
            }
        } else if (u1 == 2.0) {
            y = u0 * u0;
        } else if ((u1 == 0.5) && (u0 >= 0.0)) {
            y = std::sqrt(u0);
        } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
            y = NAN;
        } else {
            y = std::pow(u0, u1);
        }
    }
    return y;
}

double computeTmLtX(double tx, double ty, double d, double L1, double h1)
{
    double t10;
    double t15;
    double t2;
    double t21;
    double t26;
    double t29;
    double t3;
    double t5;
    double t6;
    double t7;
    double t9;

    t2 = std::cos(tx);
    t3 = std::cos(ty);
    t5 = std::sin(ty);
    t6 = L1 * L1;
    t7 = d * d;
    t9 = L1 * t5;
    t10 = d * std::sin(tx);
    t15 = t3 * t10;
    t21 = (t9 - h1) + t15;
    t29 = L1 * t3 - t5 * t10;
    t26 = (((h1 * t9 - t6) - t7) + t2 * t7) + h1 * t15;
    t7 = t29 * t29;
    t9 = t6 * t7 + t6 * (t21 * t21);
    t15 = 1.0 / t9;
    t29 = 1.0 / rt_powd_snf(t9, 1.5);
    return -1.0 / std::sqrt(-(t26 * t26) * t15 + 1.0) *
               (d / std::sqrt(t9) * (t10 + t2 * t3 * -h1) +
                d * t2 * t6 * t26 * t29 * (t10 + t3 * -h1)) -
           rt_powd_snf(L1, 3.0) * d * t2 * t21 * t29 /
               std::sqrt(-t6 * t7 * t15 + 1.0) * (L1 + t5 * -h1);
}

double computeTmLtY(double tx, double ty, double d, double L1, double h1)
{
    double t20;
    double t21;
    double t22;
    double t25;
    double t25_tmp;
    double t3;
    double t30;
    double t31;
    double t4;
    double t5;
    double t6;
    double t7;
    double t8;

    t3 = std::cos(ty);
    t4 = std::sin(tx);
    t5 = std::sin(ty);
    t6 = L1 * h1;
    t7 = L1 * L1;
    t8 = d * d;
    t31 = L1 * d;
    t20 = t5 * t7 + t31 * t3 * t4;
    t25_tmp = d * h1;
    t25 = (((t5 * t6 - t7) - t8) + std::cos(tx) * t8) + t25_tmp * t3 * t4;
    t21 = t3 * t7 - t31 * t4 * t5;
    t7 = -t6 + t20;
    t22 = t21 * t21;
    t8 = t22 + t7 * t7;
    t30 = 1.0 / t8;
    t31 = 1.0 / std::sqrt(t8);
    t8 = t20 * t21 * 2.0 - t21 * t7 * 2.0;
    t7 = rt_powd_snf(t31, 3.0);
    return 1.0 / std::sqrt(-(t25 * t25) * t30 + 1.0) *
               (t31 * (t3 * t6 - t25_tmp * t4 * t5) + t25 * t7 * t8 / 2.0) -
           (t20 * t31 - t21 * t7 * t8 / 2.0) / std::sqrt(-t22 * t30 + 1.0);
}

double computeTmRtX(double tx, double ty, double d, double L1, double h2)
{
    double t10;
    double t12;
    double t2;
    double t21;
    double t25;
    double t28;
    double t3;
    double t5;
    double t6;
    double t7;
    double t9;

    t2 = std::cos(tx);
    t3 = std::cos(ty);
    t5 = std::sin(ty);
    t6 = L1 * L1;
    t7 = d * d;
    t9 = L1 * t5;
    t10 = d * std::sin(tx);
    t12 = t3 * t10;
    t28 = L1 * t3 + t5 * t10;
    t21 = (h2 + t12) - t9;
    t25 = (((t6 + t7) + h2 * t12) - h2 * t9) - t2 * t7;
    t7 = t6 * (t28 * t28);
    t9 = t7 + t6 * (t21 * t21);
    t12 = 1.0 / t9;
    t28 = 1.0 / rt_powd_snf(t9, 1.5);
    return -1.0 / std::sqrt(-(t25 * t25) * t12 + 1.0) *
               (d / std::sqrt(t9) * (t10 + h2 * t2 * t3) -
                d * t2 * t6 * t25 * t28 * (t10 + h2 * t3)) -
           rt_powd_snf(L1, 3.0) * d * t2 * t21 * t28 /
               std::sqrt(-t7 * t12 + 1.0) * (L1 - h2 * t5);
}

double computeTmRtY(double tx, double ty, double d, double L1, double h2)
{
    double t11;
    double t13;
    double t20;
    double t21;
    double t22;
    double t23;
    double t25_tmp;
    double t3;
    double t4;
    double t5;
    double t6;
    double t7;
    double t8;

    t3 = std::cos(ty);
    t4 = std::sin(tx);
    t5 = std::sin(ty);
    t6 = L1 * h2;
    t7 = L1 * L1;
    t8 = d * d;
    t11 = t5 * t7;
    t23 = L1 * d;
    t13 = t23 * t3 * t4;
    t20 = t3 * t7 + t23 * t4 * t5;
    t21 = t11 - t13;
    t22 = t20 * t20;
    t23 = (t6 + t13) - t11;
    t25_tmp = d * h2;
    t8 = (((t7 + t8) + t25_tmp * t3 * t4) - t5 * t6) - std::cos(tx) * t8;
    t11 = t22 + t23 * t23;
    t23 = t20 * t21 * 2.0 + t20 * t23 * 2.0;
    t7 = 1.0 / t11;
    t11 = 1.0 / std::sqrt(t11);
    t13 = rt_powd_snf(t11, 3.0);
    return 1.0 / std::sqrt(-(t8 * t8) * t7 + 1.0) *
               (t11 * (t3 * t6 + t25_tmp * t4 * t5) - t8 * t13 * t23 / 2.0) -
           (t21 * t11 - t20 * t13 * t23 / 2.0) / std::sqrt(-t22 * t7 + 1.0);
}

int main(int argc, char **argv) {
    auto *robot = new Robot();
    int timeStep = (int)robot->getBasicTimeStep();

    SharedMemoryObject<SharedData> shm;
    shm.Attach(SHARED_MEMORY_NAME);
    shm.Init(false);

    auto motor1 = robot->getMotor("ankle_l_motor");
    auto motor2 = robot->getMotor("ankle_r_motor");
    auto motor_pitch = robot->getMotor("ankle_motor_pitch");
    auto motor_roll = robot->getMotor("ankle_motor_roll");

    auto *motor_pos1 = robot->getPositionSensor("ankle_l_motor_pos");
    auto *motor_pos2 = robot->getPositionSensor("ankle_r_motor_pos");
    auto *motor_pitch_pos = robot->getPositionSensor("ankle_pos_pitch");
    auto *motor_roll_pos = robot->getPositionSensor("ankle_pos_roll");

    motor_pos1->enable(timeStep);
    motor_pos2->enable(timeStep);
    motor_pitch_pos->enable(timeStep);
    motor_roll_pos->enable(timeStep);
    motor_pitch->enableTorqueFeedback(timeStep);
    motor_roll->enableTorqueFeedback(timeStep);

    int count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto end_time = std::chrono::high_resolution_clock::now();

    bool test_kinematics = true;

    double pre_pos1 = 0;
    double pre_pos2 = 0;
    double pre_pos_pitch = 0;
    double pre_pos_roll = 0;

    while (robot->step(timeStep) != -1) {
        end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_time = end_time - start_time;
        // std::cout << elapsed_time.count() << std::endl;

        double pos1 = motor_pos1->getValue();
        double pos2 = motor_pos2->getValue();
        double pitch_pos = motor_pitch_pos->getValue();
        double roll_pos = motor_roll_pos->getValue();
        auto pitch_torque = motor_pitch->getTorqueFeedback();
        auto roll_torque = motor_roll->getTorqueFeedback();

        double vel1 = (pos1 - pre_pos1) / (timeStep * 0.001);
        double vel2 = (pos2 - pre_pos2) / (timeStep * 0.001);
        double vel_pitch = (pitch_pos - pre_pos_pitch) / (timeStep * 0.001);
        double vel_roll = (roll_pos - pre_pos_roll) / (timeStep * 0.001);

        pre_pos1 = pos1;
        pre_pos2 = pos2;
        pre_pos_pitch = pitch_pos;
        pre_pos_roll = roll_pos;

        count += timeStep;
        double des_pos_s = std::sin(count * 0.001 * 0.2) * 0.4;
        double des_pos_c = std::cos(count * 0.001 * 0.2) * 0.4;
        auto des_motor_pos = ankle_ik(0.06, 0.12, 0.49, 0.37, des_pos_s, des_pos_c);

        // test for forward kinematics
        shm().sim2ros.data[4] = 0.5 * (des_motor_pos[0] + des_motor_pos[1]);
        shm().sim2ros.data[5] = calculateTx(pos1, pos2, 0.06, 0.12, 0.49, 0.37);//atan((0.12 / 0.06) * sin(0.5 * (-des_motor_pos[1] + des_motor_pos[0])));

        shm().sim2ros.data[2] = pitch_pos;
        shm().sim2ros.data[6] = des_pos_c;
        shm().sim2ros.data[3] = roll_pos;
        shm().sim2ros.data[7] = des_pos_s;

        shm().sim2ros.data[0] = pos1;
        shm().sim2ros.data[8] = des_motor_pos[0];
        shm().sim2ros.data[1] = pos2;
        shm().sim2ros.data[9] = des_motor_pos[1];
        shm().sim2ros.data[10] = calculateTxML(pos1, pos2, 0.06, 0.12, 0.49, 0.37);
        shm().sim2ros.data[11] = calculateTxMR(pos1, pos2, 0.06, 0.12, 0.49, 0.37);

        shm().sim2ros.data[12] = vel1;
        shm().sim2ros.data[13] = vel2;
        shm().sim2ros.data[14] = vel_pitch;
        shm().sim2ros.data[15] = vel_roll;
        shm().sim2ros.data[16] = 0.5 * vel1 + 0.5 * vel2;
        shm().sim2ros.data[17] = shm().sim2ros.data[10] * vel1 + shm().sim2ros.data[11] * vel2;

        shm().sim2ros.data[18] = pitch_torque;
        shm().sim2ros.data[19] = roll_torque;
        shm().sim2ros.data[20] = -(pitch_torque * 0.5 + roll_torque * shm().sim2ros.data[10]);
        shm().sim2ros.data[21] = -(pitch_torque * 0.5 + roll_torque * shm().sim2ros.data[11]);
        shm().sim2ros.data[22] = 2 * des_pos_s;
        shm().sim2ros.data[23] = 10 * des_pos_c;

        shm().sim2ros.data[24] = vel_roll * computeTmLtX(roll_pos, pitch_pos, 0.06, 0.12, 0.49) + vel_pitch * computeTmLtY(roll_pos, pitch_pos, 0.06, 0.12, 0.49);
        shm().sim2ros.data[25] = vel_roll * computeTmRtX(roll_pos, pitch_pos, 0.06, 0.12, 0.37) + vel_pitch * computeTmRtY(roll_pos, pitch_pos, 0.06, 0.12, 0.37);

        Eigen::Matrix2d J = Eigen::Matrix2d::Zero();
        J << 0.5, 0.5,
             shm().sim2ros.data[10], shm().sim2ros.data[11];
        auto J_inv = J.inverse();

        shm().sim2ros.data[26] = vel_roll * J_inv(0, 1) + vel_pitch * J_inv(0, 0);
        shm().sim2ros.data[27] = vel_roll * J_inv(1, 1) + vel_pitch * J_inv(1, 0);

        if (test_kinematics) {
            motor1->setPosition(des_motor_pos[0]);
            motor2->setPosition(des_motor_pos[1]);

            motor_pitch->setTorque(0);
            motor_roll->setTorque(0);
        } else {
            motor1->setTorque(shm().sim2ros.data[22]);
            motor2->setTorque(shm().sim2ros.data[23]);

            motor_pitch->setPosition(-0.2);
            motor_roll->setPosition(-0.2);
        }

        shm.RobotIsDone();
        start_time = std::chrono::high_resolution_clock::now();
    }

    delete robot;
    shm.Detach();
    return 0;
}
