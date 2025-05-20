#include "mbed.h"
#include "coordinate.hpp"
#include "PID_new.hpp"
#include "mecanum.hpp"
#include "C620.hpp"
#include <array>

CAN can(PB_12, PB_13, 1e6);

// class DjiMotor<4>;

int main()
{

}

constexpr int motor_amount = 4;
class MecanumMotor
{
    public:
    MecanumMotor(const std::array<bit::Coordinate, motor_amount> motor_pos) : c620_(can), mecanum_(motor_pos)
    {
        for(int i = 0; i < motor_amount; ++i)
        {
            pid_[i] = Pid({gain_, -1, 1});
            pid_[i].reset();
        }
    }
    bool set_mecanum_output(const bit::Velocity& vel, const float elapsed)
    {
        float motor_vel[motor_amount] = {0};
        mecanum_.calc(vel, motor_vel);
        for (int i = 0; i < motor_amount; ++i)
        {
            c620_.read_data();
        }
        bool is_succeed[motor_amount];
        for (int i = 0; i < motor_amount; ++i)
        {
            is_succeed[i] = pid(motor_vel[i], elapsed, i + 1);
        }
        if(!(is_succeed[0] && is_succeed[1] && is_succeed[2] && is_succeed[3]))
        {
            return false;
        }
        return c620_.write();
    }

    private:
    bool pid(const float goal, const float elapsed, const int id)
    {
        constexpr int k = 2 * M_PI / 60;
        const float now = c620_.get_rpm(id) * k;
        const float percent = pid_[id - 1].calc(goal, now, elapsed);
        c620_.set_output_percent(percent, id);
    }

    std::array<Pid, motor_amount> pid_;
    const PidGain gain_ = {1.0f, 0.1f, 0.01f}; // Example PID gains
    dji::C620 c620_;
    bit::Mecanum mecanum_;
};