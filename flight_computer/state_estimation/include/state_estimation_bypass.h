#ifndef STATE_ESTIMATION_BYPASS_H
#define STATE_ESTIMATION_BYPASS_H

#include "state_estimation_base.h"


namespace avionics::estimation{

class estimation_bypass : public estimation_base{

    private:

    protected:

    std::vector<avionics::scheduler_tasks> get_routine() override;

    public:

    void get_sensor_data();

    void get_actual_state();

    void calculate_state();

};

}

#endif