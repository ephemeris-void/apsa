/*
SPI :
1) PAR -> WHERE TO PICK
2) MOAR -> WHERE TO STORE
3) NDTR -> WHAT SIZE 
4) CR -> CONFIGURE & ENABLE FLAG
*/
static constexpr uint32_t DMA2_S0_PAR {0x40026418U};
static constexpr uint32_t SPI2_DR_ADDR{0x4000380CU};
static constexpr uint32_t DMA2_S0_M0AR{0x4002641CU};
static constexpr uint32_t DMA2_S0_NDTR{0x40026414U};
static constexpr uint8_t  BIT_SIZE{7u};
static constexpr uint32_t DMA2_S0_CR  {0x40026410U};

void DMA_Stream0_IRQHandler() noexcept {


 *reinterpret_cast<volatile uint32_t* const> DMA2_S0_PAR=SPI2_DR_ADDR;

 // MOAR ACCEPT UINT32_t but dm_rx_buff sent int16_t ,so type_cast and remove volatile for buff
 
 *reinterpret_cast<volatile uint32_t* const> DMA2_S0_M0AR = reinterpret_cast<uint32_t>(const_cast<int16_t*>(dm_rx_buff));

 //BIT SIZE FOR NDTR

 *reinterpret_cast<volatile uint32_t* const> DMA2_S0_NDTR= BIT_SIZE;

 // NOW TIME FOR CONFIGURE AND ENABLE FLAG
/*
BIT 0 -> ENABLE
BIT 4 -> TRANSFER COMPLETE INTERRUPT
BIT 8 -> PERIPHERAL SIZE
BIT 11 -> MEMORY SIZE

*/
 *reinterpret_cast<volatile uint32_t* const>DMA2_S0_CR=
 (0b01U<<11U) |
 (0b01U<<8U)  |
 (1U<<4U)     |
 (1U<<0U);
}