#pragma once
#include <cstdint>

struct Vec3{
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
};

struct ImuFrame{
    Vec3 accel;
    Vec3 gyro;

    uint32_t time_stamp{0U};
};

//class defination
class RingBuffer{


public:

//push and check if full or not
[[nodiscard]] bool push(const ImuFrame& frame) noexcept {

     uint8_t next_tail=(tail+1U) & (SIZE-1U);
     
     if(next_tail==head)[[unlikely]]{
        return false;
     }
    storage[tail]=frame;
    tail=next_tail;
    return true;

}

//pop 
[[nodiscard]] bool pop(ImuFrame& out) noexcept{

    if(tail==head)[[unlikely]]{
        return false;
    }

    out=storage[head];
    
   head=(head+1U)&(SIZE-1U);

    return true;
}


private :

static constexpr uint8_t SIZE{4U};
alignas(4) ImuFrame storage[SIZE];

uint8_t head{0U};
uint8_t tail{0U};
};
