static constexpr uint8_t BIT_CLEAR{1U<<5U};
static constexpr uint32_t DMA2_LIFCR_ADDR {0x40026408U};
static constexpr uint32_t TIM2_CNT_ADDR   {0x40000024U};

//FOR STORE DATA ,ALIGNAS BECAUSE DMA ACCES ONLY ALINGNED 4 BYTE
alignas(4) static volatile int16_t dm_rx_buff[7];

//FOR CONVERT G TO MS^-1,SIMPLY CONVERT TO USEABLE ACCELERATION & GYRO
static constexpr float INV_ACCEL{1.0F/16384.OF};
static constexpr float INV_GYRO{1.0F/131.0F};

// push failed counter 
std::atomic<uint32_t> imu_fault_count{0U};

//object create 
static RingBuffer imu_buffer;
static TaskHandle_t apsa_task_handle{nullptr};

//FUNCTION START FROM HERE 
extern "c"
void DMA_Stream0_IRQHandler() noexcept{

//FLAG CLEAR FIRST 
*reinterpret_cast<volatile uint32_t* const>DMA2_LIFCR_ADDR=BIT_CLEAR;

extern ImuFrame frame{};
static constexpr uint32_t TIM2_CNT_ADDR   {0x40000024U};

//dm_rxz_buff give int16_t ,but frame want float so static_cast<float>
frame.accel.x= static_cast<float>dm_rx_buff[0]*INV_ACCEL;
frame.accel.y= static_cast<float>dm_rx_buff[1]*INV_ACCEL;
frame.accel.z= static_cast<float>dm_rx_buff[2]*INV_ACCEL;
frame.gyro.x= static_cast<float>dm_rx_buff[4]*INV_GYRO;
frame.gyro.y= static_cast<float>dm_rx_buff[5]*INV_GYRO;
frame.gyro.x= static_cast<float>dm_rx_buff[6]*INV_GYRO;

frame.time_stamp=*reinterpret_cast<volatile uint32_t* const>(TIM2_CNT_ADDR);


//push to ringbuffer

if(!imu_buffer.push(frame)) [[unlikely]]{

  imu_fault_count.fetch_add(1U,std::memory_order_relaxed);
}

BaseType_t woke{PdFALSE};

vTaskNotifyGivenFROMISR (apsa_task_handle,&woke);
portYIELD_FROM_ISR(woke);

}
