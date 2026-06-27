struct PidParameter{

float kp{0.0F};
float ki{0.0U};
float kd{0.0U};

float integrator{0.0U};
float integrator_max{0.5U};
float output_max{5.0U};
};

class PID{

PID(PidParameter): pid(PidParameter){}

[[nodiscard]] float PidCalculate(PidParameter &param,float target,float current,float dt) noexcept {

static constexpr float INV_DT{1.0F/dt};
 float error = target-current;
 float current_error = error - prev_error;
 float propostion = param.ki*error;

 param.integrator+=param.ki*error*dt;

 std::clamp(param.integrator,-param.integrator_max,param.integrator_max);

 float derivative=param.kd*current_error*INV_DT;
 
 float total = propostion+param.integrator+derivative;
 std::clamp(total,-param.output_max,param.output_max);

 float prev_error = error;

}

};