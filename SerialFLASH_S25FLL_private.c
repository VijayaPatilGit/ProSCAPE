/**
 *  @file SerialFLASH_S25FLL_private.c
 *  @brief Contains code for communicating with the Cypress S25FL-L serial FLASH memory chips
 *
 *  @date 04Aug2021
 *  @author Brett Augsburger
 *
 *  @par <b> Description: </b>
 *
 *  @par <b> Table of Contents: </b>
 *
 *  @par <b> Change History: </b> <BR>
 *     <table>
 *        <tr><td> <i> Date  <td> Release <td> Author <td> Description </i>
 *        <tr><td> 04Aug2021 <td> <center>Draft</center>   <td> <center>BA</center> <td> Initial Draft
 *  </table>
 *  <BR>
 *
 *  @note <i> All rights reserved.  Copyright 2021.  Archangel Systems, Inc. </i> <BR>
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "SerialFLASH_S25FLL_private.h"
#include "plib_sqi1.h"
#include "sys_time.h"

/**************  Type Definitions  ************************/

#define FLASH_NUM_SECTORS_PER_BLOCK (16)     // Number of serial FLASH sectors per block

#define SQI1_PAGE_SIZE              (256)   // Largest number of data bytes that a single SQI1 Buffer Descriptor can reference

#define NUM_SQI_DATA_BUFF_DESC      (FLASH_BLOCK_SIZE / SQI1_PAGE_SIZE)   // Number of SQI data buffer descriptors needed to write or read up to block of FLASH memory
#define NUM_SQI_CMD_BUFF_DESC       5  // Number of SQI command buffer descriptors
#define MAX_NUM_SQI_DEVICES         2  // SQI1 supports up to two devices (using CS0 and CS1)

#define SQI_CMD_DUMMY_BUFF_LEN      16
#define SQI_ADDRESS_BUFF_LEN        16
#define SQI_DATABUF_LEN             16    // Length of the temporary KSEG1 data buffer for read/write commands that don't pass a KSEG1 address

#define ADDRESS_NOT_USED            0xFFFFFFFF
#define BUFFER_NOT_USED             (uint8_t*)0

typedef enum AddressModes_t
{
   THREE_BYTE_ADDR_MODE,
   FOUR_BYTE_ADDR_MODE,
} AddressModes ;

typedef struct SerialFlashStatus_t
{
   bool isInQPImode ; /* Indicates whether QPI mode is active */
   bool isInQUADmode ; /* Indicates whether QUAD mode is active */
   AddressModes addressMode ; /* Indicates whether 4 byte address mode is active */
} SerialFlashStatus ;

/**************  Local Variables ************************/

const uint32_t maxSQIreadSize = 0x10000 ; // 64kByte maximum read size using PIC32MZ SQI in DMA mode

const uint32_t maxWriteTimeAnyRegister_ms = 750 ; // Maximum amount of time to write any register (in ms), typical 145 ms
const uint32_t maxChipEraseTime_ms = 360000 ; // Maximum amount of time to wait for chip erase (in ms), typical 140 seconds
const uint32_t maxBlockEraseTime_ms = 725 ; // Maximum amount of time to wait for block erase (in ms), typical 270 ms
const uint32_t maxHalfBlockEraseTime_ms = 363 ; // Maximum amount of time to wait for half block erase (in ms), typical 190 ms
const uint32_t maxSectorEraseTime_ms = 250 ; // Maximum amount of time to wait for sector erase (in ms), typical 50 ms
const uint32_t maxPageWriteTime_ms = 2 ; // Maximum amount of time to wait for write operation to complete (in ms), typical 300 us

const uint32_t maxReadTimePerSQIread = 100 ; // Maximum amount of time to wait for SQI read
const uint32_t maxReadTimePerSQIwrite = 100 ; // Maximum amount of time to wait for SQI write

static volatile bool xfer_done ; /* SQI transfer status */

static SerialFlashStatus serialFlashStatuses[MAX_NUM_SQI_DEVICES] ;

static bool isModuleInitialized[MAX_NUM_SQI_DEVICES] = { false, false } ;

/* SQI1 Buffer Descriptors. These must be located in KSEG1. */
static sqi_dma_desc_t CACHE_ALIGN sqiCmdBuffDesc[NUM_SQI_CMD_BUFF_DESC] ; // SQI1 buffer descriptors for serial FLASH commands
static sqi_dma_desc_t CACHE_ALIGN sqiDataBufDesc[NUM_SQI_DATA_BUFF_DESC] ; // SQI1 buffer descriptors for serial FLASH data

/* Variables passed to SQI1 peripheral using buffer descriptors. These must be aligned on 32-bit boundaries and in KSEG1. */
static uint8_t CACHE_ALIGN __ALIGNED ( 4 ) sqi_cmd_dummy[SQI_CMD_DUMMY_BUFF_LEN] ; // Dummy bytes
static uint8_t CACHE_ALIGN __ALIGNED ( 4 ) sqi_address[SQI_ADDRESS_BUFF_LEN] ; // Address bytes
static uint8_t CACHE_ALIGN __ALIGNED ( 4 ) sqi_cmd ;
static uint8_t CACHE_ALIGN __ALIGNED ( 4 ) sqi_modebit_char ;
static uint8_t CACHE_ALIGN __ALIGNED ( 4 ) sqi_databuffer[SQI_DATABUF_LEN] ; // Used for certain write commands that are not passed a KSEG1 buffer address

/**************  Function Prototypes **************/
static SLLD_STATUS SerialFLASH_S25FLL_DetermineAddressMode ( const uint8_t device_num ) ;
static SLLD_STATUS SerialFLASH_S25FLL_DetermineIfQUADmodeActive ( const uint8_t device_num ) ;
static SLLD_STATUS SerialFLASH_S25FLL_DetermineIfQPImodeActive ( const uint8_t device_num ) ;

/**************  Functions ************************/

static void SQI1_EventHandler ( uintptr_t context )
{
   xfer_done = true ;

   return ;
}

/***************************************************************************
 * 
 * SerialFLASH_S25FLL_Init_Private - Initializes the private portion of the serial FLASH library
 * 
 * input : none
 * 
 * return value : status of the initialization - FAIL or SUCCESS
 * 
 ***************************************************************************/
SLLD_STATUS SerialFLASH_S25FLL_Init_Private ( const uint8_t device_num )
{
   SLLD_STATUS status = SLLD_OK ;

   if ( device_num >= MAX_NUM_SQI_DEVICES )
   {
      return SLLD_E_HAL_ERROR ; // Error: Invalid device number specified
   }

   SQI1_RegisterCallback ( SQI1_EventHandler, ( uintptr_t ) NULL ) ;

   //   uint32_t count ;
   //   for ( count = 0 ; count < MAX_NUM_SQI_DEVICES ; count++ )
   //   {
   serialFlashStatuses[device_num].isInQPImode = false ;
   serialFlashStatuses[device_num].isInQUADmode = false ;
   serialFlashStatuses[device_num].addressMode = THREE_BYTE_ADDR_MODE ;
   //   }

   isModuleInitialized[device_num] = true ;

   status = SerialFLASH_S25FLL_DetermineAddressMode ( device_num ) ; // Determine whether 3-byte or 4-byte addressing is active and configure driver accordingly

   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_DetermineIfQUADmodeActive ( device_num ) ;
   }

   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_DetermineIfQPImodeActive ( device_num ) ;
   }

   if ( SLLD_OK != status )
   {
      isModuleInitialized[device_num] = false ;
   }

   return status ;
}

/* Determines whether 3-byte or 4-byte addressing mode is active and configures the driver accordingly.
 * Returns SLLD_OK if mode is read and driver is successfully configured. Otherwise, returns the fault condition. */
static SLLD_STATUS SerialFLASH_S25FLL_DetermineAddressMode ( const uint8_t device_num )
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;

   // Determine if address length (ADP) is set to 3-byte (0) or 4-byte (1)
   uint8_t CR2Vreadback ;

   status = SerialFLASH_S25FLL_RDCR2Cmd ( device_num, /* device number */
         &CR2Vreadback ) ; /* variable in which to store read data */

   if ( SLLD_OK == status )
   {
      if ( 1 == ( ( CR2Vreadback >> CR2_ADS_BITPOS ) & 0x1 ) )
      {
         // 4-byte addressing mode
         serialFlashStatuses[device_num].addressMode = FOUR_BYTE_ADDR_MODE ;
      }
      else
      {
         // 3-byte addressing mode
         serialFlashStatuses[device_num].addressMode = THREE_BYTE_ADDR_MODE ;
      }
   }

   return status ;
}

/* Determines whether QUAD mode is active and configures the driver accordingly.
 * Returns SLLD_OK if mode is read and driver is successfully configured. Otherwise, returns the fault condition. */
static SLLD_STATUS SerialFLASH_S25FLL_DetermineIfQUADmodeActive ( const uint8_t device_num )
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;
   uint8_t CR1VreadVal ;

   status = SerialFLASH_S25FLL_RDCR1Cmd ( device_num, /* device number */
         &CR1VreadVal ) ; /* variable in which to store read data */

   if ( SLLD_OK == status )
   {
      if ( 1 == ( ( CR1VreadVal >> CR1_QUAD_BITPOS ) & 0x1 ) )
      {
         serialFlashStatuses[device_num].isInQUADmode = true ;
      }
      else
      {
         serialFlashStatuses[device_num].isInQUADmode = false ;
      }
   }

   return status ;
}

/* Determines whether QPI mode is active and configures the driver accordingly. 
 * Returns SLLD_OK if mode is read and driver is successfully configured. Otherwise, returns the fault condition. */
static SLLD_STATUS SerialFLASH_S25FLL_DetermineIfQPImodeActive ( const uint8_t device_num )
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;
   uint8_t CR2VreadVal ;

   status = SerialFLASH_S25FLL_RDCR2Cmd ( device_num, /* device number */
         &CR2VreadVal ) ; /* variable in which to store read data */

   if ( SLLD_OK == status )
   {
      if ( 1 == ( ( CR2VreadVal >> CR2_QPI_BITPOS ) & 0x1 ) )
      {
         serialFlashStatuses[device_num].isInQPImode = true ;
      }
      else
      {
         serialFlashStatuses[device_num].isInQPImode = false ;
      }
   }

   return status ;
}

/***************************************************************************
 * SerialFLASH_S25FLL_SQIread - reads data from serial flash
 * 
 * input : device_num            device number to which operation will be done
 *         command               specifies the command to send to flash
 *         sys_addr              system address to be used
 *         data_buffer           Pointer to the data buffer where to store the read data
 *         numBytesToRead        number of bytes to be read
 * 
 * NOTE: If numBytesToRead > SQI_DATABUF_LEN then data_buffer must be a KSEG1 address.
 * 
 * return value : status of the operation - SLLD_OK on success
 ***************************************************************************/
static SLLD_STATUS SerialFLASH_S25FLL_SQIread ( const uint8_t device_num, /* device number to which operation will be done */
                                                const uint8_t command, /* specifies the command to send to flash */
                                                const uint32_t sys_addr, /* system address to be used */
                                                uint8_t * const data_buffer, /* Pointer to the data buffer where to store the read data */
                                                const int32_t numBytesToRead ) /* number of bytes to be read */
{
   SLLD_STATUS status = SLLD_OK ;

   if ( ( ( 0 != numBytesToRead ) && ( NULL == data_buffer ) ) ||
        ( device_num >= MAX_NUM_SQI_DEVICES ) )
   {
      return SLLD_E_HAL_ERROR ; // Error: Attempting to store read data to a null buffer or invalid device number specified
   }

   SQI1CMDTHRbits.TXCMDTHR = 0x1 ; // Set SQI TX command threshold to 1 byte to prevent problems related to TX size

   uint32_t sqi_bdctrl_CSmask = SQI_BDCTRL_SQICS_CS0 ; /* SQI buffer descriptor mask for chip select of device */
   uint32_t sqi_bdctrl_LaneMode = SQI_BDCTRL_MODE_SINGLE_LANE ; /* SQI buffer descriptor mask for lane mode to use while sending instruction */

   /* Determine SQI buffer descriptor mask for chip select of device */
   switch ( device_num )
   {
      case 0:
         sqi_bdctrl_CSmask = SQI_BDCTRL_SQICS_CS0 ;
         break ;

      case 1:
         sqi_bdctrl_CSmask = SQI_BDCTRL_SQICS_CS1 ;
         break ;

      default:
         return SLLD_E_HAL_ERROR ; // Should never happen
         break ;
   }

   sqi_cmd = command ; // Copy to uncached memory location

   /* Determine SQI buffer descriptor mask for lane mode to use while sending command instruction */
   switch ( sqi_cmd )
   {
      case SPI_RDSR2_CMD:
      case SPI_RDCR1_CMD:
      case SPI_RDCR2_CMD:
      case SPI_RDCR3_CMD:
      case SPI_READ_CMD:
      case SPI_4READ_CMD:
      case SPI_FAST_READ_CMD:
      case SPI_4FAST_READ_CMD:
      case SPI_DOR_CMD:
      case SPI_4DOR_CMD:
      case SPI_QOR_CMD:
      case SPI_4QOR_CMD:
      case SPI_DIOR_CMD:
      case SPI_4DIOR_CMD:
      {
         if ( true == serialFlashStatuses[device_num].isInQPImode )
         {
            return SLLD_E_HAL_ERROR ; // Attempted to execute a non-QPI compatible command while in QPI mode
         }
         sqi_bdctrl_LaneMode = SQI_BDCTRL_MODE_SINGLE_LANE ;
         break ;
      }

      case SPI_RDQID_CMD:
      case SPI_QIOR_CMD:
      case SPI_4QIOR_CMD:
      case SPI_DDRQIOR_CMD:
      case SPI_4DDRQIOR_CMD:
      {
         if ( true == serialFlashStatuses[device_num].isInQPImode )
         {
            sqi_bdctrl_LaneMode = SQI_BDCTRL_MODE_QUAD_LANE ;
         }
         else if ( true == serialFlashStatuses[device_num].isInQUADmode )
         {
            sqi_bdctrl_LaneMode = SQI_BDCTRL_MODE_SINGLE_LANE ;
         }
         else
         {
            return SLLD_E_HAL_ERROR ; // Attempted to execute a QUAD/QPI command while not in QPI or QUAD mode
         }
         break ;
      }

      default:
      {
         if ( true == serialFlashStatuses[device_num].isInQPImode )
         {
            sqi_bdctrl_LaneMode = SQI_BDCTRL_MODE_QUAD_LANE ;
         }
         else
         {
            sqi_bdctrl_LaneMode = SQI_BDCTRL_MODE_SINGLE_LANE ;
         }
         break ;
      }
   }

   // Write the command to the device
   uint32_t cmdBDcurrIndex = 0 ;

   sqiCmdBuffDesc[cmdBDcurrIndex].bd_ctrl = ( SQI_BDCTRL_BUFFLEN_VAL ( 1 ) | SQI_BDCTRL_DIR_WRITE |
                                              sqi_bdctrl_LaneMode | sqi_bdctrl_CSmask | SQI_BDCTRL_DESCEN ) ;
   sqiCmdBuffDesc[cmdBDcurrIndex].bd_bufaddr = ( uint32_t * ) KVA_TO_PA ( &sqi_cmd ) ;
   sqiCmdBuffDesc[cmdBDcurrIndex].bd_stat = 0 ;

   // Write the address to the device
   if ( ADDRESS_NOT_USED != sys_addr )
   {
      sqiCmdBuffDesc[cmdBDcurrIndex].bd_nxtptr = ( sqi_dma_desc_t * ) KVA_TO_PA ( &sqiCmdBuffDesc[cmdBDcurrIndex + 1] ) ; // Point previous BD to address BD
      cmdBDcurrIndex++ ;

      /* For certain commands, change the lane mode for sending out the address bytes */
      switch ( sqi_cmd )
      {
         case SPI_DIOR_CMD:
         case SPI_4DIOR_CMD:
            sqi_bdctrl_LaneMode = SQI_BDCTRL_MODE_DUAL_LANE ;
            break ;

         case SPI_QIOR_CMD:
         case SPI_4QIOR_CMD:
         case SPI_DDRQIOR_CMD:
         case SPI_4DDRQIOR_CMD:
            sqi_bdctrl_LaneMode = SQI_BDCTRL_MODE_QUAD_LANE ;
            break ;

         default:
            break ; // No change to lane mode required
      }

      size_t numAddrBytes ;
      switch ( sqi_cmd )
      {
            // 4-byte address only commands
         case SPI_4FAST_READ_CMD:
         case SPI_4READ_CMD:
         case SPI_4DOR_CMD:
         case SPI_4QOR_CMD:
         case SPI_4DIOR_CMD:
         case SPI_4QIOR_CMD:
         case SPI_4DDRQIOR_CMD:
         case SPI_4IBLRD_CMD:
            numAddrBytes = 4 ;
            sqi_address[0] = ( uint8_t ) ( ( sys_addr >> 24 ) & 0x000000FF ) ;
            sqi_address[1] = ( uint8_t ) ( ( sys_addr >> 16 ) & 0x000000FF ) ;
            sqi_address[2] = ( uint8_t ) ( ( sys_addr >> 8 ) & 0x000000FF ) ;
            sqi_address[3] = ( uint8_t ) ( sys_addr & 0x000000FF ) ;
            break ;

         default:
            if ( THREE_BYTE_ADDR_MODE == serialFlashStatuses[device_num].addressMode )
            {
               numAddrBytes = 3 ;
               sqi_address[0] = ( uint8_t ) ( ( sys_addr >> 16 ) & 0x000000FF ) ;
               sqi_address[1] = ( uint8_t ) ( ( sys_addr >> 8 ) & 0x000000FF ) ;
               sqi_address[2] = ( uint8_t ) ( sys_addr & 0x000000FF ) ;

            }
            else if ( FOUR_BYTE_ADDR_MODE == serialFlashStatuses[device_num].addressMode )
            {
               numAddrBytes = 4 ;
               sqi_address[0] = ( uint8_t ) ( ( sys_addr >> 24 ) & 0x000000FF ) ;
               sqi_address[1] = ( uint8_t ) ( ( sys_addr >> 16 ) & 0x000000FF ) ;
               sqi_address[2] = ( uint8_t ) ( ( sys_addr >> 8 ) & 0x000000FF ) ;
               sqi_address[3] = ( uint8_t ) ( sys_addr & 0x000000FF ) ;
            }
            else
            {
               return SLLD_E_HAL_ERROR ; // Shouldn't happen
            }
            break ;
      }

      // Setup buffer descriptor to send address
      sqiCmdBuffDesc[cmdBDcurrIndex].bd_ctrl = ( SQI_BDCTRL_BUFFLEN_VAL ( numAddrBytes ) | SQI_BDCTRL_DIR_WRITE |
                                                 sqi_bdctrl_LaneMode | sqi_bdctrl_CSmask | SQI_BDCTRL_DESCEN ) ;

      sqiCmdBuffDesc[cmdBDcurrIndex].bd_bufaddr = ( uint32_t * ) KVA_TO_PA ( sqi_address ) ;
      sqiCmdBuffDesc[cmdBDcurrIndex].bd_stat = 0 ;
   }

   // Write mode byte to the device
   switch ( sqi_cmd )
   {
      case SPI_DIOR_CMD:
      case SPI_4DIOR_CMD:
      case SPI_QIOR_CMD:
      case SPI_4QIOR_CMD:
      case SPI_DDRQIOR_CMD:
      case SPI_4DDRQIOR_CMD:
      {
         sqiCmdBuffDesc[cmdBDcurrIndex].bd_nxtptr = ( sqi_dma_desc_t * ) KVA_TO_PA ( &sqiCmdBuffDesc[cmdBDcurrIndex + 1] ) ;
         cmdBDcurrIndex++ ;

         sqiCmdBuffDesc[cmdBDcurrIndex].bd_ctrl = ( SQI_BDCTRL_BUFFLEN_VAL ( 1 ) | SQI_BDCTRL_DIR_WRITE |
                                                    sqi_bdctrl_LaneMode | sqi_bdctrl_CSmask | SQI_BDCTRL_DESCEN ) ;
         sqiCmdBuffDesc[cmdBDcurrIndex].bd_bufaddr = ( uint32_t * ) KVA_TO_PA ( &sqi_modebit_char ) ;
         sqiCmdBuffDesc[cmdBDcurrIndex].bd_stat = 0 ;
         break ;
      }

      default:
         break ; // No mode byte to write
   }

   // Write the dummy bytes to the device
   uint32_t numDummyBytes = 0 ;
   switch ( sqi_cmd )
   {
      case SPI_RSFDP_CMD:
      case SPI_RDAR_CMD:
      case SPI_SECRR_CMD:
      case SPI_IBLRD_CMD:
      case SPI_4IBLRD_CMD:
      {
         if ( true == serialFlashStatuses[device_num].isInQPImode )
         {
            numDummyBytes = 4 ;
         }
         else
         {
            numDummyBytes = 1 ;
         }
         break ;
      }

      case SPI_FAST_READ_CMD:
      case SPI_4FAST_READ_CMD:
      case SPI_DOR_CMD:
      case SPI_4DOR_CMD:
      case SPI_QOR_CMD:
      case SPI_4QOR_CMD:
      {
         numDummyBytes = 1 ;
         break ;
      }

      case SPI_DIOR_CMD:
      case SPI_4DIOR_CMD:
      {
         numDummyBytes = 2 ;
         break ;
      }

      case SPI_QIOR_CMD:
      case SPI_4QIOR_CMD:
      {
         numDummyBytes = 4 ;
         break ;
      }

      case SPI_DDRQIOR_CMD:
      case SPI_4DDRQIOR_CMD:
      {
         numDummyBytes = 8 ;
         break ;
      }

      case SPI_RUID_CMD:
      {
         if ( true == serialFlashStatuses[device_num].isInQPImode )
         {
            numDummyBytes = 16 ;
         }
         else
         {
            numDummyBytes = 4 ;
         }
         break ;
      }

         /* These commands are incompatible with the PIC32MZ's SQI module because they require a single dummy clock cycle
         and there is no way to accomplish this with this peripheral */
      case SPI_DLPRD_CMD: // Data learning pattern read
      case SPI_IRPRD_CMD: // IRP register read
      case SPI_PRRD_CMD: // Protection register read
      case SPI_PASSRD_CMD: // Password read
      {
         return SLLD_E_HAL_ERROR ; // Error: command not supported on this target
      }

      default:
      {
         // Default to no dummy bytes written
         numDummyBytes = 0 ;
         break ;
      }
   }

   if ( 0 != numDummyBytes )
   {
      sqiCmdBuffDesc[cmdBDcurrIndex].bd_nxtptr = ( sqi_dma_desc_t * ) KVA_TO_PA ( &sqiCmdBuffDesc[cmdBDcurrIndex + 1] ) ; // Point previous BD to dummy BD
      cmdBDcurrIndex++ ;

      sqiCmdBuffDesc[cmdBDcurrIndex].bd_ctrl = ( SQI_BDCTRL_BUFFLEN_VAL ( numDummyBytes ) | SQI_BDCTRL_DIR_READ |
                                                 sqi_bdctrl_LaneMode | sqi_bdctrl_CSmask | SQI_BDCTRL_DESCEN ) ;
      sqiCmdBuffDesc[cmdBDcurrIndex].bd_bufaddr = ( uint32_t * ) KVA_TO_PA ( &sqi_cmd_dummy ) ;
      sqiCmdBuffDesc[cmdBDcurrIndex].bd_stat = 0 ;
   }

   // Read the data from the serial flash device
   if ( 0 != numBytesToRead )
   {
      /* Some commands don't pass a KSEG1 address. In these cases, read into a KSEG1 buffer first and then copy later. */
      uint8_t* readBuffer ;
      if ( false == IS_KVA1 ( data_buffer ) )
      {
         if ( numBytesToRead <= sizeof ( sqi_databuffer ) )
         {
            readBuffer = sqi_databuffer ;
         }
         else
         {
            return SLLD_E_HAL_ERROR ; // Error: non-KSEG1 address and number of bytes to write exceeds the size of sqi_databuffer[]
         }
      }
      else // Use passed in address directly
      {
         readBuffer = data_buffer ;
      }

      sqiCmdBuffDesc[cmdBDcurrIndex].bd_nxtptr = ( sqi_dma_desc_t * ) KVA_TO_PA ( &sqiDataBufDesc[0] ) ; // Point previous BD to address BD

      uint32_t dataBDcurrIndex = 0 ;
      uint32_t numBytes ;
      uint32_t pendingBytes = numBytesToRead ;

      while ( ( dataBDcurrIndex < NUM_SQI_DATA_BUFF_DESC ) && ( pendingBytes > 0 ) )
      {
         if ( pendingBytes > FLASH_PAGE_SIZE )
         {
            numBytes = FLASH_PAGE_SIZE ;
         }
         else
         {
            numBytes = pendingBytes ;
         }

         sqiDataBufDesc[dataBDcurrIndex].bd_ctrl = ( SQI_BDCTRL_BUFFLEN_VAL ( numBytes ) | SQI_BDCTRL_DIR_READ |
                                                     sqi_bdctrl_LaneMode | sqi_bdctrl_CSmask | SQI_BDCTRL_DESCEN ) ;
         sqiDataBufDesc[dataBDcurrIndex].bd_bufaddr = ( uint32_t * ) KVA_TO_PA ( readBuffer ) ;
         sqiDataBufDesc[dataBDcurrIndex].bd_stat = 0 ;
         sqiDataBufDesc[dataBDcurrIndex].bd_nxtptr = ( sqi_dma_desc_t * ) KVA_TO_PA ( &sqiDataBufDesc[dataBDcurrIndex + 1] ) ;

         pendingBytes -= numBytes ;
         readBuffer += numBytes ;

         dataBDcurrIndex++ ;
      }

      /* The last descriptor must indicate the end of the descriptor list */
      sqiDataBufDesc[dataBDcurrIndex - 1].bd_ctrl |= ( SQI_BDCTRL_PKTINTEN | SQI_BDCTRL_LASTPKT | SQI_BDCTRL_LASTBD | SQI_BDCTRL_DEASSERT ) ;
      sqiDataBufDesc[dataBDcurrIndex - 1].bd_nxtptr = 0x00000000 ;
   }
   else // No bytes to read
   {
      // Last command buffer descriptor is last BD to send, so mark it as such
      sqiCmdBuffDesc[cmdBDcurrIndex].bd_ctrl |= ( SQI_BDCTRL_PKTINTEN | SQI_BDCTRL_LASTPKT | SQI_BDCTRL_LASTBD | SQI_BDCTRL_DEASSERT ) ;
      sqiCmdBuffDesc[cmdBDcurrIndex].bd_nxtptr = 0x00000000 ;
   }

   xfer_done = false ;

   SQI1_DMATransfer ( ( sqi_dma_desc_t * ) KVA_TO_PA ( &sqiCmdBuffDesc[0] ) ) ;

   const uint32_t startSysTimeInCounts = SYS_TIME_CounterGet ( ) ; // System timer value before polling commences
   uint32_t elapsedTimeMS = 0 ;

   while ( xfer_done == false ) // Wait for SQI transfer to finish
   {
      elapsedTimeMS = SYS_TIME_CountToMS ( SYS_TIME_CounterGet ( ) - startSysTimeInCounts ) ;

      if ( elapsedTimeMS >= maxReadTimePerSQIread )
      {
         status = SLLD_E_HAL_ERROR ; // Error: timed out
         break ;
      }
   }

   //   SQI1CFGbits.RXBUFRST = 1;  // TEMP TEMP TEMP TEMP

   // If the read buffer that was passed was not a KSEG1 address, then copy from the temporary KSEG1 buffer into the original read buffer
   if ( ( SLLD_OK == status ) && ( 0 != numBytesToRead ) && ( false == IS_KVA1 ( data_buffer ) ) )
   {
      memcpy ( data_buffer, sqi_databuffer, numBytesToRead ) ;
   }

   return status ;
}

/***************************************************************************
 * SerialFLASH_S25FLL_SQIwrite - writes data to serial flash
 *
 * input : device_num               device number to which operation will be done
 *         command                  specifies the command to send to flash
 *         sys_addr                 system address to be used
 *         data_buffer              Pointer to the data buffer where to store the written data
 *         numBytesToWrite          number of bytes to be written
 * 
 * NOTE: If numBytesToWrite > SQI_DATABUF_LEN then data_buffer must be a KSEG1 address.
 * 
 * return value : status of the operation - SLLD_OK on success
 ****************************************************************************/

static SLLD_STATUS SerialFLASH_S25FLL_SQIwrite ( const uint8_t device_num, /* device number to which operation will be done */
                                                 const uint8_t command, /* specifies the command to send to flash */
                                                 const uint32_t sys_addr, /* system address to be used */
                                                 const uint8_t * const data_buffer, /* Pointer to the data buffer containing data to be written */
                                                 const int32_t numBytesToWrite ) /* number of bytes to be written */
{
   SLLD_STATUS status = SLLD_OK ;

   if ( ( ( 0 != numBytesToWrite ) && ( NULL == data_buffer ) ) ||
        ( device_num >= MAX_NUM_SQI_DEVICES ) )
   {
      return SLLD_E_HAL_ERROR ; // Error: Attempting to read data from a null buffer or invalid device number specified
   }

   SQI1CMDTHRbits.TXCMDTHR = 0x1 ; // Set SQI TX command threshold to 1 byte to prevent problems related to TX size

   uint32_t sqi_bdctrl_CSmask = SQI_BDCTRL_SQICS_CS0 ; /* SQI buffer descriptor mask for chip select of device */
   uint32_t sqi_bdctrl_LaneMode = SQI_BDCTRL_MODE_SINGLE_LANE ; /* SQI buffer descriptor mask for lane mode to use while sending instruction */

   /* Determine SQI buffer descriptor mask for chip select of device */
   switch ( device_num )
   {
      case 0:
         sqi_bdctrl_CSmask = SQI_BDCTRL_SQICS_CS0 ;
         break ;

      case 1:
         sqi_bdctrl_CSmask = SQI_BDCTRL_SQICS_CS1 ;
         break ;

      default:
         return SLLD_E_HAL_ERROR ; // Should never happen
         break ;
   }

   sqi_cmd = command ; // Copy to volatile memory location

   /* Determine SQI buffer descriptor mask for lane mode to use while sending instruction */
   switch ( sqi_cmd )
   {
      case SPI_QPIEN_CMD:
         if ( true == serialFlashStatuses[device_num].isInQPImode )
         {
            return SLLD_E_HAL_ERROR ; // Error: Attempting to execute a non-QPI compatible command while in QPI mode
         }
         sqi_bdctrl_LaneMode = SQI_BDCTRL_MODE_SINGLE_LANE ;
         break ;

      case SPI_QPIEX_CMD:
         if ( false == serialFlashStatuses[device_num].isInQPImode )
         {
            return SLLD_E_HAL_ERROR ; // Error: Attempting to execute a QPI command while not in QPI mode
         }
         sqi_bdctrl_LaneMode = SQI_BDCTRL_MODE_QUAD_LANE ;
         break ;

      case SPI_QPP_CMD:
      case SPI_4QPP_CMD:
         if ( false == serialFlashStatuses[device_num].isInQUADmode )
         {
            return SLLD_E_HAL_ERROR ; // Error: Attempting to execute a QUAD command while not in QUAD mode
         }
         sqi_bdctrl_LaneMode = SQI_BDCTRL_MODE_SINGLE_LANE ;
         break ;

      default:
         if ( true == serialFlashStatuses[device_num].isInQPImode )
         {
            sqi_bdctrl_LaneMode = SQI_BDCTRL_MODE_QUAD_LANE ;
         }
         else
         {
            sqi_bdctrl_LaneMode = SQI_BDCTRL_MODE_SINGLE_LANE ;
         }
         break ;
   }

   // Write the command to the device
   uint32_t cmdBDcurrIndex = 0 ; // Holds the current index into the SQI command buffer descriptor array
   sqiCmdBuffDesc[0].bd_ctrl = ( SQI_BDCTRL_BUFFLEN_VAL ( 1 ) | SQI_BDCTRL_DIR_WRITE |
                                 sqi_bdctrl_LaneMode | sqi_bdctrl_CSmask | SQI_BDCTRL_DESCEN ) ;
   sqiCmdBuffDesc[0].bd_bufaddr = ( uint32_t * ) KVA_TO_PA ( &sqi_cmd ) ;
   sqiCmdBuffDesc[0].bd_stat = 0 ;

   // Write the address to the device
   if ( sys_addr != ADDRESS_NOT_USED )
   {
      sqiCmdBuffDesc[cmdBDcurrIndex].bd_nxtptr = ( sqi_dma_desc_t * ) KVA_TO_PA ( &sqiCmdBuffDesc[cmdBDcurrIndex + 1] ) ; // Point previous BD to address BD
      cmdBDcurrIndex++ ;

      size_t numAddrBytes ;
      switch ( sqi_cmd )
      {
            // 4-byte address only commands
         case SPI_4PP_CMD:
         case SPI_4QPP_CMD:
         case SPI_4SE_CMD:
         case SPI_4HBE_CMD:
         case SPI_4BE_CMD:
         case SPI_4IBL_CMD:
         case SPI_4IBUL_CMD:
         case SPI_4SPRP_CMD:
            numAddrBytes = 4 ;
            sqi_address[0] = ( uint8_t ) ( ( sys_addr >> 24 ) & 0x000000FF ) ;
            sqi_address[1] = ( uint8_t ) ( ( sys_addr >> 16 ) & 0x000000FF ) ;
            sqi_address[2] = ( uint8_t ) ( ( sys_addr >> 8 ) & 0x000000FF ) ;
            sqi_address[3] = ( uint8_t ) ( sys_addr & 0x000000FF ) ;
            break ;

         default:
            if ( THREE_BYTE_ADDR_MODE == serialFlashStatuses[device_num].addressMode )
            {
               numAddrBytes = 3 ;
               sqi_address[0] = ( uint8_t ) ( ( sys_addr >> 16 ) & 0x000000FF ) ;
               sqi_address[1] = ( uint8_t ) ( ( sys_addr >> 8 ) & 0x000000FF ) ;
               sqi_address[2] = ( uint8_t ) ( sys_addr & 0x000000FF ) ;

            }
            else if ( FOUR_BYTE_ADDR_MODE == serialFlashStatuses[device_num].addressMode )
            {
               numAddrBytes = 4 ;
               sqi_address[0] = ( uint8_t ) ( ( sys_addr >> 24 ) & 0x000000FF ) ;
               sqi_address[1] = ( uint8_t ) ( ( sys_addr >> 16 ) & 0x000000FF ) ;
               sqi_address[2] = ( uint8_t ) ( ( sys_addr >> 8 ) & 0x000000FF ) ;
               sqi_address[3] = ( uint8_t ) ( sys_addr & 0x000000FF ) ;
            }
            else
            {
               return SLLD_E_HAL_ERROR ; // Shouldn't happen
            }
            break ;
      }

      // Setup buffer descriptor to send address
      sqiCmdBuffDesc[cmdBDcurrIndex].bd_ctrl = ( SQI_BDCTRL_BUFFLEN_VAL ( numAddrBytes ) | SQI_BDCTRL_DIR_WRITE |
                                                 sqi_bdctrl_LaneMode | sqi_bdctrl_CSmask | SQI_BDCTRL_DESCEN ) ;

      sqiCmdBuffDesc[cmdBDcurrIndex].bd_bufaddr = ( uint32_t * ) KVA_TO_PA ( sqi_address ) ;
      sqiCmdBuffDesc[cmdBDcurrIndex].bd_stat = 0 ;
   }

   sqi_dma_desc_t * firstBuffDesc = ( sqi_dma_desc_t * ) KVA_TO_PA ( &sqiCmdBuffDesc[0] ) ;

   // Write the data to the serial flash device
   if ( 0 != numBytesToWrite )
   {
      /* Some commands don't pass a KSEG1 address. In these cases, copy to a KSEG1 buffer, if there is room. */
      uint8_t* writeBuffer ;
      if ( false == IS_KVA1 ( data_buffer ) )
      {
         if ( numBytesToWrite <= sizeof (sqi_databuffer ) )
         {
            memcpy ( sqi_databuffer, data_buffer, numBytesToWrite ) ;
            writeBuffer = sqi_databuffer ;
         }
         else
         {
            return SLLD_E_HAL_ERROR ; // Error: non-KSEG1 address specified and number of bytes to write exceeds the size of sqi_databuffer[]
         }
      }
      else // Use passed in address directly
      {
         writeBuffer = ( uint8_t* ) data_buffer ;
      }

      /* If the number of bytes to write is divisible by four then send the current SQI buffer descriptors and
       * create a new buffer descriptor operation with the SQI TX command threshold set to 0x20 */
      if ( ( numBytesToWrite % 4 ) == 0 )
      {
         // Last command buffer descriptor is last BD to send, so mark it as such. Don't deassert chip select as a second operation will occur.
         sqiCmdBuffDesc[cmdBDcurrIndex].bd_ctrl |= ( SQI_BDCTRL_PKTINTEN | SQI_BDCTRL_LASTPKT | SQI_BDCTRL_LASTBD ) ;
         sqiCmdBuffDesc[cmdBDcurrIndex].bd_nxtptr = 0x00000000 ;

         xfer_done = false ;

         SQI1_DMATransfer ( ( sqi_dma_desc_t * ) KVA_TO_PA ( &sqiCmdBuffDesc[0] ) ) ;

         const uint32_t startSysTimeInCounts = SYS_TIME_CounterGet ( ) ; // System timer value before polling commences
         uint32_t elapsedTimeMS = 0 ;

         while ( ( xfer_done == false ) && // Waits for SQI PKTCOMP or BDDONE flag
                 ( 0 != SQI1BDTXDSTATbits.TXBUFCNT ) ) // Not truly done until this is empty
         {
            elapsedTimeMS = SYS_TIME_CountToMS ( SYS_TIME_CounterGet ( ) - startSysTimeInCounts ) ;

            if ( elapsedTimeMS >= maxReadTimePerSQIwrite )
            {
               return SLLD_E_HAL_ERROR ; // Error: timed out
            }
         }

         SQI1CMDTHRbits.TXCMDTHR = 0x20 ;
         firstBuffDesc = ( sqi_dma_desc_t * ) KVA_TO_PA ( &sqiDataBufDesc[0] ) ;
      }
      else
      {
         sqiCmdBuffDesc[cmdBDcurrIndex].bd_nxtptr = ( sqi_dma_desc_t * ) KVA_TO_PA ( &sqiDataBufDesc[0] ) ; // Point previous BD to address BD
      }

      /* For certain commands, change the lane mode for sending out the data to be written */
      switch ( sqi_cmd )
      {
         case SPI_QPP_CMD:
         case SPI_4QPP_CMD:
            sqi_bdctrl_LaneMode = SQI_BDCTRL_MODE_QUAD_LANE ;
            break ;

         default:
            break ; // No change to lane mode required
      }

      uint32_t dataBDcurrIndex = 0 ; // Holds the current index into the SQI data buffer descriptor array
      uint32_t numBytes ;
      uint32_t pendingBytes = numBytesToWrite ;

      while ( ( dataBDcurrIndex < NUM_SQI_DATA_BUFF_DESC ) && ( pendingBytes > 0 ) )
      {
         if ( pendingBytes > FLASH_PAGE_SIZE )
         {
            numBytes = FLASH_PAGE_SIZE ;
         }
         else
         {
            numBytes = pendingBytes ;
         }

         sqiDataBufDesc[dataBDcurrIndex].bd_ctrl = ( SQI_BDCTRL_BUFFLEN_VAL ( numBytes ) | SQI_BDCTRL_DIR_WRITE |
                                                     sqi_bdctrl_LaneMode | sqi_bdctrl_CSmask | SQI_BDCTRL_DESCEN ) ;
         sqiDataBufDesc[dataBDcurrIndex].bd_bufaddr = ( uint32_t * ) KVA_TO_PA ( writeBuffer ) ;
         sqiDataBufDesc[dataBDcurrIndex].bd_stat = 0 ;
         sqiDataBufDesc[dataBDcurrIndex].bd_nxtptr = ( sqi_dma_desc_t * ) KVA_TO_PA ( &sqiDataBufDesc[dataBDcurrIndex + 1] ) ;

         pendingBytes -= numBytes ;
         writeBuffer += numBytes ;

         dataBDcurrIndex++ ;
      }

      /* The last descriptor must indicate the end of the descriptor list */
      sqiDataBufDesc[dataBDcurrIndex - 1].bd_ctrl |= ( SQI_BDCTRL_PKTINTEN | SQI_BDCTRL_LASTPKT | SQI_BDCTRL_LASTBD | SQI_BDCTRL_DEASSERT ) ;
      sqiDataBufDesc[dataBDcurrIndex - 1].bd_nxtptr = 0x00000000 ;
   }
   else // No bytes to write
   {
      // Last command buffer descriptor is last BD to send, so mark it as such
      sqiCmdBuffDesc[cmdBDcurrIndex].bd_ctrl |= ( SQI_BDCTRL_PKTINTEN | SQI_BDCTRL_LASTPKT | SQI_BDCTRL_LASTBD | SQI_BDCTRL_DEASSERT ) ;
      sqiCmdBuffDesc[cmdBDcurrIndex].bd_nxtptr = 0x00000000 ;
   }

   xfer_done = false ;

   SQI1_DMATransfer ( firstBuffDesc ) ;

   const uint32_t startSysTimeInCounts = SYS_TIME_CounterGet ( ) ; // System timer value before polling commences
   uint32_t elapsedTimeMS = 0 ;

   while ( xfer_done == false ) // Waits for SQI PKTCOMP or BDDONE flag
   {
      elapsedTimeMS = SYS_TIME_CountToMS ( SYS_TIME_CounterGet ( ) - startSysTimeInCounts ) ;

      if ( elapsedTimeMS >= maxReadTimePerSQIwrite )
      {
         status = SLLD_E_HAL_ERROR ; // Error: timed out
         break ;
      }
   }

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_RDIDCmd - Read Identification from SPI Flash
 *
 * This function issues the RDID command to SPI Flash and reads out the ID.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS SerialFLASH_S25FLL_RDIDCmd ( const uint8_t device_num, /* device number */
                                         uint32_t * const mfgDeviceID ) /* address of where to store manufacturer device ID */
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   uint8_t tempBuff[JEDEC_ID_LENGTH] ;

   SLLD_STATUS status = SLLD_OK ;
   status = SerialFLASH_S25FLL_SQIread ( device_num, SPI_RDID_CMD, ( uint32_t ) ADDRESS_NOT_USED, tempBuff, JEDEC_ID_LENGTH ) ;

   if ( SLLD_OK == status )
   {
      *mfgDeviceID = ( tempBuff[3] << 24 ) + ( tempBuff[2] << 16 ) + ( tempBuff[1] << 8 ) + tempBuff[0] ;
   }

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_RDSRCmd - Read from Status Register
 *
 * This function issues the RDSR command to SPI Flash and reads from status register.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS SerialFLASH_S25FLL_RDSRCmd ( const uint8_t device_num, //device number
                                         uint8_t * const target )/* variable in which to store read data */
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;
   status = SerialFLASH_S25FLL_SQIread ( device_num, SPI_RDSR1_CMD, ( uint32_t ) ADDRESS_NOT_USED, target, 1 ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_RDSR2Cmd - Read from Status Register-2
 *
 * This function issues the RDSR2 command to SPI Flash and reads from status register.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS SerialFLASH_S25FLL_RDSR2Cmd ( const uint8_t device_num, /* device number */
                                          uint8_t * const target ) /* variable in which to store read data */
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;
   status = SerialFLASH_S25FLL_SQIread ( device_num, SPI_RDSR2_CMD, ( uint32_t ) ADDRESS_NOT_USED, target, 1 ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_RDCR1Cmd - Read from the configuration register 1
 *
 * This function issues the RDCR1 command to the SPI Flash.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS SerialFLASH_S25FLL_RDCR1Cmd ( const uint8_t device_num, /* device number */
                                          uint8_t * const target ) /* variable in which to store read data */
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;
   status = SerialFLASH_S25FLL_SQIread ( device_num, SPI_RDCR1_CMD, ADDRESS_NOT_USED, target, 1 ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_RDCR2Cmd - Read from the configuration register 2
 *
 * This function issues the RDCR2 command to the SPI Flash.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS SerialFLASH_S25FLL_RDCR2Cmd ( const uint8_t device_num, /* device number */
                                          uint8_t * const target ) /* variable in which to store read data */
{

   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;
   status = SerialFLASH_S25FLL_SQIread ( device_num, SPI_RDCR2_CMD, ADDRESS_NOT_USED, target, 1 ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_RDCR3Cmd - Read from the configuration register 3
 *
 * This function issues the RDCR3 command to the SPI Flash.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS SerialFLASH_S25FLL_RDCR3Cmd ( const uint8_t device_num, /* device number */
                                          uint8_t * const target )/* variable in which to store read data */
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;
   status = SerialFLASH_S25FLL_SQIread ( device_num, SPI_RDCR3_CMD, ADDRESS_NOT_USED, target, 1 ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_RDARCmd - Read any device register-non-volatile and volatile.
 *
 * This function issues the read any register command and reads the requested data in 1-bytes
 *
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS SerialFLASH_S25FLL_RDARCmd ( const uint8_t device_num, /* device number */
                                         const uint32_t reg_addr, /* register address given by device*/
                                         uint8_t * const target )/*variable in which to store read data*/
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;
   status = SerialFLASH_S25FLL_SQIread ( device_num, SPI_RDAR_CMD, reg_addr, target, 1 ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_WRENCmd - Issue the write enable command
 *
 * This function issues the WREN command to the SPI Flash.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
static SLLD_STATUS SerialFLASH_S25FLL_WRENCmd ( const uint8_t device_num ) //device number
{
   SLLD_STATUS status = SLLD_OK ;
   status = SerialFLASH_S25FLL_SQIwrite ( device_num, SPI_WREN_CMD, ( uint32_t ) ADDRESS_NOT_USED, BUFFER_NOT_USED, 0 ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_WRARCmd - Write any device register-non-volatile and volatile.
 *
 * This function issues the write any register command and write 1-byte
 *
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
static SLLD_STATUS SerialFLASH_S25FLL_WRARCmd ( const uint8_t device_num, /* device number */
                                                const uint32_t reg_addr, /*register address given by device*/
                                                const uint8_t * const data_buf ) /*variable containing data to program*/
{
   SLLD_STATUS status = SLLD_OK ;
   status = SerialFLASH_S25FLL_SQIwrite ( device_num, SPI_WRAR_CMD, reg_addr, data_buf, 1 ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_StatusGet - Determines Flash Status
 *
 * This function gets the device status from the SPI flash.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
static SLLD_STATUS SerialFLASH_S25FLL_StatusGet ( const uint8_t device_num, /* device number */
                                                  DEVSTATUS * const dev_status_ptr ) /* variable to store device status */
{
   SLLD_STATUS status = SLLD_OK ;
   uint8_t poll_data ;

   status = SerialFLASH_S25FLL_RDSRCmd ( device_num, &poll_data ) ;
   if ( status != SLLD_OK )
      return status ;

   // Check if steady state
   if ( poll_data & B0_MASK )
   { // If b0 = 1 then device is busy
      status = SerialFLASH_S25FLL_RDSR2Cmd ( device_num, &poll_data ) ;
      if ( status != SLLD_OK )
      {
         return status ;
      }

      // If b5 = 1 then there was an erase error
      if ( ( STATUS_ERROR_FLAGS & HAS_STATBIT5_ERROR ) && ( poll_data & B5_MASK ) )
      {
         *dev_status_ptr = dev_erase_error ;
      }
         // If b6 = 1 then there was a program error
      else if ( ( STATUS_ERROR_FLAGS & HAS_STATBIT6_ERROR ) && ( poll_data & B6_MASK ) )
      {
         *dev_status_ptr = dev_program_error ;
      }
      else
      {
         *dev_status_ptr = dev_busy ;
      }
   }
   else
   {
      // If b0 = 0 then device is not busy
      *dev_status_ptr = dev_not_busy ;
   }

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_Poll - Polls flash device for embedded operation completion
 *
 * This function polls the Flash device to determine when an embedded
 * operation is finished.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
static SLLD_STATUS SerialFLASH_S25FLL_Poll ( const uint8_t device_num, /* device number */
                                             DEVSTATUS * const dev_status_ptr, /* variable to store device status */
                                             const uint32_t maxPollWaitMS ) /* Maximum amount of time to spend polling in ms */
{
   SLLD_STATUS status = SLLD_OK ;

   *dev_status_ptr = dev_status_unknown ;

   const uint32_t startSysTimeInCounts = SYS_TIME_CounterGet ( ) ; // System timer value before polling commences
   uint32_t elapsedTimeMS = 0 ;

   do
   {
      status = SerialFLASH_S25FLL_StatusGet ( device_num, dev_status_ptr ) ;
      elapsedTimeMS = SYS_TIME_CountToMS ( SYS_TIME_CounterGet ( ) - startSysTimeInCounts ) ;
   }
   while ( ( *dev_status_ptr == dev_busy ) &&
           ( elapsedTimeMS < maxPollWaitMS ) &&
           ( SLLD_OK == status ) ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_WRDICmd - Issue the write disable command
 *
 * This function issues the WRDI command to the SPI Flash.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
static SLLD_STATUS SerialFLASH_S25FLL_WRDICmd ( const uint8_t device_num ) //device number
{
   SLLD_STATUS status = SLLD_OK ;
   status = SerialFLASH_S25FLL_SQIwrite ( device_num, SPI_WRDI_CMD, ( uint32_t ) ADDRESS_NOT_USED, BUFFER_NOT_USED, 0 ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_WRAR_Op - Write any device register-non-volatile and volatile operation
 *
 * This function issues the write any register command and write 1-byte
 *
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS SerialFLASH_S25FLL_WRAR_Op ( const uint8_t device_num, /* device number */
                                         const uint32_t reg_addr, /*register address given by device*/
                                         const uint8_t * const data_buf, /*variable containing data to program*/
                                         DEVSTATUS * const dev_status_ptr ) /* variable to store device status */
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;

   status = SerialFLASH_S25FLL_WRENCmd ( device_num ) ;

   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_WRARCmd ( device_num,
            reg_addr,
            data_buf ) ;
   }

   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_Poll ( device_num,
            dev_status_ptr,
            maxWriteTimeAnyRegister_ms ) ;
   }

   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_WRDICmd ( device_num ) ; /* just in case PPOp is operated on protected area */
   }

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_RSTENCmd - Reset Enable
 *
 * This function issues the Reset Enable command which required immediately before a
 * Reset command(RST).
 *
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS SerialFLASH_S25FLL_RSTENCmd ( const uint8_t device_num )
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;
   status = SerialFLASH_S25FLL_SQIwrite ( device_num, SPI_RSTEN_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0 ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_RSTCmd - Reset command
 *
 * This function issues the Software Reset command immediately following a RSTEN command,
 * initiates the software reset process.
 *
 *
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS SerialFLASH_S25FLL_RSTCmd ( const uint8_t device_num )
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;
   status = SerialFLASH_S25FLL_SQIwrite ( device_num, SPI_RST_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0 ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_CECmd - Chip erase
 *
 * This function issues the Chip Erase command to SPI Flash.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
static SLLD_STATUS SerialFLASH_S25FLL_CECmd ( const uint8_t device_num ) //device number
{
   SLLD_STATUS status = SLLD_OK ;
   status = SerialFLASH_S25FLL_SQIwrite ( device_num, SPI_CE_CMD, ADDRESS_NOT_USED, BUFFER_NOT_USED, 0 ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_CEOp - Performs a chip erase Operation
 *
 * Function erases all data in a device.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS SerialFLASH_S25FLL_CEOp ( const uint8_t device_num, /* device number */
                                      DEVSTATUS * const dev_status_ptr ) /* variable to store device status */
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;

   status = SerialFLASH_S25FLL_WRENCmd ( device_num ) ;

   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_CECmd ( device_num ) ;
   }

   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_Poll ( device_num,
            dev_status_ptr,
            maxChipEraseTime_ms ) ;
   }

   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_WRDICmd ( device_num ) ; /* just in case CEOp is operated on protected area */
   }

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_BECmd - Block erase
 *
 * This function issues the block erase command to SPI Flash. The block erase
 * command sets all bits in the addressed 64 KB block to 1 (all bytes are FFh).
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
static SLLD_STATUS SerialFLASH_S25FLL_BECmd ( const uint8_t device_num, //device number
                                              const uint32_t sys_addr )
{
   SLLD_STATUS status = SLLD_OK ;
   status = SerialFLASH_S25FLL_SQIwrite ( device_num, SPI_BE_CMD, sys_addr, BUFFER_NOT_USED, 0 ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_BEOp - Performs a block erase Operation.
 *
 * Function erase all data in a block.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS SerialFLASH_S25FLL_BEOp ( const uint8_t device_num, /* device number */
                                      const uint32_t sys_addr, /* device address given by system */
                                      DEVSTATUS * const dev_status_ptr )/* variable to store device status */
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;

   status = SerialFLASH_S25FLL_WRENCmd ( device_num ) ;

   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_BECmd ( device_num,
            sys_addr ) ;
   }

   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_Poll ( device_num,
            dev_status_ptr,
            maxBlockEraseTime_ms ) ;
   }

   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_WRDICmd ( device_num ) ; /* just in case BEOp is operated on protected area */
   }

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_HBECmd - Half block erase
 *
 * This function issues the half block Erase command to SPI Flash.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
static SLLD_STATUS SerialFLASH_S25FLL_HBECmd ( const uint8_t device_num, //device number
                                               const uint32_t sys_addr )/* device address given by system */
{
   SLLD_STATUS status = SLLD_OK ;

   status = SerialFLASH_S25FLL_SQIwrite ( device_num, SPI_HBE_CMD, sys_addr, BUFFER_NOT_USED, 0 ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_HBEOp - Performs a half block erase operation
 *
 * Function erases all data in a specified half block.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS SerialFLASH_S25FLL_HBEOp ( const uint8_t device_num, /* device number */
                                       const uint32_t sys_addr, /* device address given by system */
                                       DEVSTATUS * const dev_status_ptr ) /* variable to store device status */
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;

   status = SerialFLASH_S25FLL_WRENCmd ( device_num ) ;

   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_HBECmd ( device_num,
            sys_addr ) ;
   }

   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_Poll ( device_num,
            dev_status_ptr,
            maxHalfBlockEraseTime_ms ) ;
   }

   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_WRDICmd ( device_num ) ; /* just in case HBEOp is operated on a protected area */
   }

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_SECmd - Sector erase
 *
 * This function issues the sector erase command to SPI Flash.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
static SLLD_STATUS SerialFLASH_S25FLL_SECmd ( const uint8_t device_num, //device number
                                              const uint32_t sys_addr )/* device address given by system */
{
   SLLD_STATUS status = SLLD_OK ;

   status = SerialFLASH_S25FLL_SQIwrite ( device_num, SPI_SE_CMD, sys_addr, BUFFER_NOT_USED, 0 ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_SEOp - Performs a sector erase operation
 *
 * Function erases specified sector.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS SerialFLASH_S25FLL_SEOp ( const uint8_t device_num, /* device number */
                                      const uint32_t sys_addr, /* device address given by system */
                                      DEVSTATUS * const dev_status_ptr ) /* variable to store device status */
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;

   status = SerialFLASH_S25FLL_WRENCmd ( device_num ) ;
   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_SECmd ( device_num,
            sys_addr ) ;
   }

   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_Poll ( device_num,
            dev_status_ptr,
            maxSectorEraseTime_ms ) ;
   }

   if ( SLLD_OK == status )
   {
      status = SerialFLASH_S25FLL_WRDICmd ( device_num ) ; /* just in case SEOp is operated on a protected area */
   }

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_PPCmd - Page Program
 *
 * This function issues a Page Program command to SPI Flash.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
static SLLD_STATUS SerialFLASH_S25FLL_PPCmd ( const uint8_t device_num, //device number
                                              const uint32_t sys_addr, /* device address given by system */
                                              const uint8_t * const data_buf, /* variable containing data to program */
                                              const uint32_t len_in_bytes ) /* number of bytes to operate */
{
   SLLD_STATUS status = SLLD_OK ;
   status = SerialFLASH_S25FLL_SQIwrite ( device_num, SPI_PP_CMD, sys_addr, data_buf, len_in_bytes ) ;

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_ReadOp - Performs a read operation
 *
 * Function reads the specified length of bytes from the specified serial FLASH address
 * and stores the data at the given buffer address.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS SerialFLASH_S25FLL_ReadOp ( const uint8_t device_num, /* device number */
                                        const uint32_t sys_addr, /* device address given by system */
                                        uint8_t * const target, /* variable in which to store read data */
                                        const uint32_t len_in_bytes ) /* number of bytes to read */
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;
   uint32_t length ;

   sqi_modebit_char = 0 ; // Make sure that modebit is not Axh

   uint32_t bytesRemaining = len_in_bytes ;
   uint32_t currAddr = sys_addr ;
   uint8_t * targetAddr = ( uint8_t* ) target ;

   while ( bytesRemaining > 0 )
   {
      if ( bytesRemaining < maxSQIreadSize )
      {
         length = bytesRemaining ;
      }
      else
      {
         length = maxSQIreadSize ;
      }

      status = SerialFLASH_S25FLL_SQIread ( device_num, SPI_QIOR_CMD, currAddr, targetAddr, length ) ;
      if ( SLLD_OK != status ) break ;

      bytesRemaining -= length ;
      currAddr += length ;
      targetAddr += length ;
   }

   return status ;
}

/******************************************************************************
 *
 * SerialFLASH_S25FLL_WriteOp - Performs a write operation
 *
 * Function writes the specified length of bytes from the given buffer address to
 * the specified serial FLASH address.
 * Function issues all required commands and polls for completion.
 *
 * RETURNS: SLLD_OK or SLLD_E_HAL_ERROR
 *
 */
SLLD_STATUS SerialFLASH_S25FLL_WriteOp ( const uint8_t device_num, /* device number */
                                         const uint32_t sys_addr, /* device address given by system */
                                         const uint8_t * const data_buf, /* variable containing data to program */
                                         const uint32_t len_in_bytes, /* number of bytes to program */
                                         DEVSTATUS * const dev_status_ptr ) /* variable to store device status */
{
   if ( ( device_num >= MAX_NUM_SQI_DEVICES ) ||
        ( false == isModuleInitialized[device_num] ) )
   {
      return SLLD_ERROR ;
   }

   SLLD_STATUS status = SLLD_OK ;
   uint32_t length ;

   const uint8_t * srcAddr = ( uint8_t* ) data_buf ;
   uint32_t bytesLeftToWrite = len_in_bytes ;
   uint32_t currAddr = sys_addr ;

   /* Write up to one page at a time */
   while ( bytesLeftToWrite > 0 )
   {
      if ( ( currAddr % FLASH_PAGE_SIZE ) != 0 )
      {
         /* Not on page boundary. Write up to the next page boundary if enough bytes are left to be written or set the length
          * to the number of bytesLeftToWrite otherwise. */
         length = FLASH_PAGE_SIZE - ( currAddr % FLASH_PAGE_SIZE ) ;
         length = ( length > bytesLeftToWrite ) ? bytesLeftToWrite : length ;
      }
      else
      {
         /* Starting on page boundary. Write up to one full page at a time. */
         length = ( bytesLeftToWrite < FLASH_PAGE_SIZE ) ? bytesLeftToWrite : FLASH_PAGE_SIZE ;
      }

      status = SerialFLASH_S25FLL_WRENCmd ( device_num ) ;
      if ( SLLD_OK != status ) break ;

      status = SerialFLASH_S25FLL_PPCmd ( device_num,
            currAddr,
            srcAddr,
            length ) ;
      if ( SLLD_OK != status ) break ;

      status = SerialFLASH_S25FLL_Poll ( device_num,
            dev_status_ptr,
            maxPageWriteTime_ms ) ;
      if ( SLLD_OK != status ) break ;

      status = SerialFLASH_S25FLL_WRDICmd ( device_num ) ;
      if ( SLLD_OK != status ) break ;

      bytesLeftToWrite -= length ;
      currAddr += length ;
      srcAddr += length ;
   }

   return status ;
}

/* End of SerialFLASH_S25FLL_private.c source file. */
