#include "mbed.h"
#include "coordinate.hpp"
#include "PID_new.hpp"
#include "mecanum.hpp"
#include "C620.hpp"
#include <array>

bool readline(BufferedSerial &serial, char *buffer, size_t size, bool is_integar = false, bool is_float = false);
float duration_to_sec(const std::chrono::duration<float> &duration);

constexpr int motor_amount = 4;
class MecanumMotor
{
public:
    MecanumMotor(const std::array<bit::Coordinate, motor_amount> motor_pos, CAN &can) : c620_(can), mecanum_(motor_pos)
    {
        for (int i = 0; i < motor_amount; ++i)
        {
            pid_[i] = Pid({gain_, -1, 1});
            pid_[i].reset();
        }
        c620_.set_max_output(max_motor_pwr);
    }
    void read_motor_rpm()
    {
        for (int i = 0; i < motor_amount; ++i)
        {
            c620_.read_data();
        }
    }
    bool set_mecanum_output(const bit::Velocity &vel, const float elapsed)
    {
        float motor_vel[motor_amount] = {0};
        mecanum_.calc(vel, motor_vel);
        // printf("%.2f, %.2f, %.2f, %.2f\n", motor_vel[0], motor_vel[1], motor_vel[2], motor_vel[3]);
        for (int i = 0; i < motor_amount; ++i)
        {
            c620_.read_data();
        }
        bool is_succeed[motor_amount];
        for (int i = 0; i < motor_amount; ++i)
        {
            // printf("[%d]\n", i);
            float goal_ang_vel = motor_vel[i] / motor_radius * 19; //C620の減速比である19を掛け, 入力値と測定rpmの符号が逆になるので反転
            if (i == 0 || i == 4)
            {
                goal_ang_vel *= -1;
            }
            is_succeed[i] = pid(goal_ang_vel, elapsed, i + 1);
            // printf("goal: %.4f\n", goal_ang_vel);
        }
        if (!(is_succeed[0] && is_succeed[1] && is_succeed[2] && is_succeed[3]))
        {
            return false;
        }
        // printf("\nnow: %d, %d, %d, %d\n", c620_.get_current(1), c620_.get_current(2), c620_.get_current(3), c620_.get_current(4));
        return c620_.write();
    }

private:
    bool pid(const float goal, const float elapsed, const int id)
    {
        constexpr float k = 2 * M_PI / 60;
        const float now = c620_.get_rpm(id) * k;
        const float percent = pid_[id - 1].calc(goal, now, elapsed);
        // printf("now: %.4f\n", now);
        c620_.set_output_percent(percent, id);
        return true;
    }

    std::array<Pid, motor_amount> pid_;
    const PidGain gain_ = {0.001, 0.001, 0.0}; // Example PID gains
    dji::C620 c620_;
    bit::Mecanum mecanum_;
    const float motor_radius = 48.65;
    const int max_motor_pwr = 8000;
};

int main()
{
    BufferedSerial pc(USBTX, USBRX, 115200);
    BufferedSerial esp(PB_6, PA_10, 115200);
    CAN can(PB_12, PB_13, 1e6);
    std::array<bit::Coordinate, 4> motor_pos =
        {
            bit::Coordinate(337.5, 382.5),
            bit::Coordinate(-337.5, 382.5),
            bit::Coordinate(-337.5, -382.5),
            bit::Coordinate(337.5, -382.5)
        };
    MecanumMotor mecanum(motor_pos, can);

    bit::Velocity robot_vel = {0, 0, 0};
    DigitalOut led(LED1);

    while (1)
    {
        auto now = HighResClock::now();
        static auto pre = now;

        char received[15] = "";
        if (readline(esp, received, sizeof(received)) == 0)
        {
            if (strcmp(received, "vel") == 0)
            {
                led = !led;
                for (int i = 0; i < 3; i++)
                {
                    char data_vel[8] = "";
                    if (readline(esp, data_vel, sizeof(data_vel), false, true) == 0)
                    {
                        switch (i)
                        {
                        case 0:
                            robot_vel.x = atof(data_vel);
                            break;
                        case 1:
                            robot_vel.y = atof(data_vel);
                            break;
                        case 2:
                            robot_vel.ang = atof(data_vel);
                            break;
                        }
                    }
                }
                // printf("vel: %f, %f, %f\n", robot_vel.x, robot_vel.y, robot_vel.ang);
            }
        }
        mecanum.read_motor_rpm();
        if (now - pre > 10ms)
        {
            float elapsed = duration_to_sec(now - pre);
            // printf("elapsed: %f\n", elapsed);
            // printf("set_mecanum_output: %d\n", mecanum.set_mecanum_output(robot_vel, elapsed));
            mecanum.set_mecanum_output(robot_vel, elapsed);
            // printf("x: %.2f, y: %.2f, ang: %.2f\n", robot_vel.x, robot_vel.y, robot_vel.ang);
            pre = now;
        }
    }
}

bool readline(BufferedSerial &serial, char *buffer, const size_t size, const bool is_integar, const bool is_float)
{
    int i = 0;       // 繰り返し変数
    char buff = '0'; // シリアル受信

    if (not serial.readable())
    {
        return 1;
    }

    while ((buff != '\n') and i < (int)size)
    {
        serial.read(&buff, sizeof(buff)); // シリアル受信
        // printf("%c", buff);

        if (buff != '\n' && buff != '\r')
        {
            buffer[i] = buff; // 受信データ保存

            if (is_integar)
            {

                if (((buff < '0' || buff > '9') && buff != '-'))
                {
                    printf("error\n");
                    return 1;
                }
            }
            if (is_float)
            {

                if (((buff < '0' || buff > '9') && buff != '.' && buff != '-'))
                {
                    printf("error\n");
                    return 1;
                }
            }
        }
        i++;
    }
    // printf("\n");
    return 0;
}

float duration_to_sec(const std::chrono::duration<float> &duration)
{
    return duration.count();
}