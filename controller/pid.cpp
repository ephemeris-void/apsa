struct PidParameter{

float kp{0.0F};
float ki{0.0F};
float kd{0.0F};


float integrator{0.0F};
float integrator_max{0.5F};
float output_max{5.0F};
float prev_error{0.0F};
};

class PID{

public:

explicit PID(PidParameter& parameter): param(parameter){}

[[nodiscard]] float PidCalculate(float target,float current,float dt) noexcept {

//ZERO DIVISION CHECK 
 const float safe_dt{std::max(dt,0.000001F)};
 const float INV_DT{1.0F/safe_dt};


//ERROR CALCULATE 
 float error {target-current};


 //PROPORTIONAL CALCULATE
 float proportional {param.kp*error};

 //INTEGRATOR CALCULATE WITH RANGE USING std::clamp
 float next_integrator {std::fma(param.ki*error,safe_dt,param.integrator)};
 param.integrator = std::clamp(next_integrator,-param.integrator_max,param.integrator_max); 

 //DERIVATIVE CALCULATE 
 float current_error{error - param.prev_error};
 float derivative{param.kd*current_error*INV_DT};

  //FINAL CALCULATE
 float total {proportional+param.integrator+derivative};

 //ERROR UPDATE
  param.prev_error = error;

 return std::clamp(total,-param.output_max,param.output_max);

}
 private:
 PidParameter& param;
};