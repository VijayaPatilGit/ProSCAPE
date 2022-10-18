#include "SerialFLASH_S25FLL.h"
#include "SerialFLASH_S25FLL_private.h"
#include <stdlib.h>
#include <stdbool.h>
LLR
#define LCD_X_RESOLUTION 480 
#define LCD_Y_RESOLUTION 480 
#define SERIAL_FLASH_DEV_NUMBER  0 
#define START_OF_COMP_DATA_OFFSET   8
#define GAUGE_BUFFERS_BASEADDR                       0xA9000000
#define SCREEN_BUFFER_SIZE_IN_BYTES                  (LCD_X_RESOLUTION * LCD_Y_RESOLUTION * 4 )  // 480px by 480px @ 32-bit
#define SCRATCH_BUFF_1_DDR_BUFFER_BASEADDR            ( GAUGE_BUFFERS_BASEADDR + SCREEN_BUFFER_SIZE_IN_BYTES )
#define SCRATCH_BUFF_2_DDR_BUFFER_BASEADDR            ( SCRATCH_BUFF_1_DDR_BUFFER_BASEADDR + SCREEN_BUFFER_SIZE_IN_BYTES )
#define GAUGE_SPECIFIC_BUFFERS_BASEADDR              ( SCRATCH_BUFF_2_DDR_BUFFER_BASEADDR + SCREEN_BUFFER_SIZE_IN_BYTES )
LLR

int main() {
    
    int32_t status2;
    
    
    bool status = ( EXIT_SUCCESS == SerialFLASH_S25FLL_Initialize ( SERIAL_FLASH_DEV_NUMBER ) ) ? false : true ;
    
    status2 = SerialFLASH_S25FLL_EraseAll ( SERIAL_FLASH_DEV_NUMBER ) ;
    
    while ( 1 ) { 
    
    
            uint32_t numBytesToErase = 4096 ; // Must be a multiple of 4096


            status2 = SerialFLASH_S25FLL_Erase (SERIAL_FLASH_DEV_NUMBER,
                                                0, 
                                                numBytesToErase) ; 


            uint32_t numBytesToRead = 4096 ; 

            status2 = SerialFLASH_S25FLL_Read ( SERIAL_FLASH_DEV_NUMBER , 
                                                START_OF_COMP_DATA_OFFSET, 
                                                numBytesToRead, 
                                                ( uint8_t* ) GAUGE_SPECIFIC_BUFFERS_BASEADDR
                                                ) ;
            uint32_t numBytesToWrite = 4096 ;

            status2 = SerialFLASH_S25FLL_Write ( SERIAL_FLASH_DEV_NUMBER , 
                                                START_OF_COMP_DATA_OFFSET, 
                                                numBytesToRead, 
                                                ( uint8_t* ) GAUGE_SPECIFIC_BUFFERS_BASEADDR
                                                ) ;
    
    } 
    
    return 0;
}
