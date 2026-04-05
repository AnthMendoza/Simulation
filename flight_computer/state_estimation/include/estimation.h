#ifndef ESTIMATION_H
#define ESTIMATION_H

#include "state_estimation_base.h"
#include <actual_state.h>

namespace avionics::estimation{

class estimation : public estimation_base{

    private:

    protected:

    std::vector<avionics::scheduler_tasks> get_routine() override;

    public:


    estimation( sensor_dbuf& sensor_buff,state_dbuf& state_buff ) : estimation_base(sensor_buff,state_buff){

    }

    void get_sensor_data();

    void calculate_state();

};

}

#endif