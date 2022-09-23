/**
 *  @file SerialFLASH_S25FLL.c
 *  @brief Contains code for communicating with the Cypress S25FL-L serial FLASH memory chips
 *
 *  @date 19Feb2021
 *  @author Brett Augsburger
 *
 *  @par <b> Description: </b>
 *
 *  @par <b> Table of Contents: </b>
 *
 *  @par <b> Change History: </b> <BR>
 *     <table>
 *        <tr><td> <i> Date  <td> Release <td> Author <td> Description </i>
 *        <tr><td> 19Feb2021 <td> <center>Draft</center>   <td> <center>BA</center> <td> Initial Draft
 *  </table>
 *  <BR>
 *
 *  @note <i> All rights reserved.  Copyright 2021.  Archangel Systems, Inc. </i> <BR>
 */

/**************  Included Files  ************************/
#include <stdbool.h>
#include <stdlib.h>
#include "SerialFLASH_S25FLL.h"
#include "SerialFLASH_S25FLL_private.h"
#include "plib_coretimer.h"

/**************  Type Definitions  ************************/
#define SR1NV_SETVAL    0x00        // SRP0_NV, SEC_NV, TBPROT_NV, and BP_NV0-2 should all be 0s
#define CR1NV_SETVAL    0x02        // CMP_NV, LB0-3, SRP1_D should all be 0s. QUAD_NV should be 1.
#define CR2NV_SETVAL    0x62        // IO3R_NV, QPI_NV, and WPS_NV should all be 0. ADP_NV should be 1. OI_NV should be 11b.
#define CR3NV_SETVAL    0x78        // WL_NV should be 11b, WE_NV should be 1, RL_NV should be 1000b
#define SR1NV_MASK      0xFC
#define CR1NV_MASK      0x7F
#define CR2NV_MASK      0xEE
#define CR3NV_MASK      0x7F

#define S25FLL_RESET_DELAY_US 100   // Reset wait time per datasheet in microseconds

/**************  Local Variables ************************/



/**************  Function Prototypes ************************/
static int32_t SerialFLASH_S25FLL_VerifyNVregisters ( const uint8_t device_num, bool* areRegsValid ) ;
static int32_t SerialFLASH_S25FLL_SetNVregisters ( const uint8_t device_num ) ;
static int32_t SerialFLASH_S25FLL_VerifyVolatileRegisters ( const uint8_t device_num, bool* areRegsValid ) ;

/**************  Functions ************************/

/* Initializes the serial FLASH device. Takes SQI device number of serial FLASH. If serial FLASH initialization is successful,
 * returns EXIT_SUCCESS. Otherwise, returns EXIT_FAILURE. */
int32_t SerialFLASH_S25FLL_Initialize ( const uint8_t device_num ) // PIC32-SQI device number
{
   int32_t success = EXIT_SUCCESS ;

   SLLD_STATUS slld_status ;

   slld_status = SerialFLASH_S25FLL_Init_Private ( device_num ) ; // Initialize driver

   // Read JEDICid and verify that it is as expected
   uint32_t serialFlash_JEDECid = 0 ;
   if ( SLLD_OK == slld_status )
   {
      slld_status = SerialFLASH_S25FLL_RDIDCmd ( device_num, /* device number */
            & serialFlash_JEDECid ) ; /* variable in which to store read data */
   }

   // TODO temporary still support 128Mb chip since Joe has a proto with that chip. Eventually remove support by editing the next statement
   if ( ( ( ( S25FL128L_JEDEC_ID & JEDEC_ID_MASK ) != ( serialFlash_JEDECid & JEDEC_ID_MASK ) ) &&
          ( ( S25FL256L_JEDEC_ID & JEDEC_ID_MASK ) != ( serialFlash_JEDECid & JEDEC_ID_MASK ) ) ) ||
        ( SLLD_OK != slld_status ) )
   {
      return EXIT_FAILURE ; // Error: Unexpected JEDEC ID or error communicating with serial FLASH
   }

   // Check non-volatile register configuration
   bool areNVregistersCorrect ;

   if ( EXIT_FAILURE == SerialFLASH_S25FLL_VerifyNVregisters ( device_num, &areNVregistersCorrect ) )
   {
      return EXIT_FAILURE ;
   }

   if ( false == areNVregistersCorrect )
   {
      // NV register configuration incorrect. Attempt to set to correct values.
      if ( EXIT_FAILURE == SerialFLASH_S25FLL_SetNVregisters ( device_num ) )
      {
         return EXIT_FAILURE ; // Error: serial FLASH NV configuration is incorrect and could not be changed to correct values
      }
   }

   //  Verify volatile register configuration
   bool areVolRegistersCorrect ;
   if ( EXIT_FAILURE == SerialFLASH_S25FLL_VerifyVolatileRegisters ( device_num, &areVolRegistersCorrect ) )
   {
      return EXIT_FAILURE ;
   }

   success = ( true == areVolRegistersCorrect ) ? EXIT_SUCCESS : EXIT_FAILURE ;

   return success ;
}

/* Determines whether non-volatile serial FLASH registers match specified configuration values. Sets *areRegsValid to TRUE if values are correct,
 * otherwise sets it to FALSE. If no serial FLASH error is encountered, the function returns EXIT_SUCCESS. Otherwise, the function
 * returns EXIT_FAILURE and *areRegsValid is set to FALSE. */
static int32_t SerialFLASH_S25FLL_VerifyNVregisters ( const uint8_t device_num, // PIC32-SQI device number
                                                      bool* areRegsValid )
{
   /* Default all read values to something other than the specified configuration values */
   uint8_t SR1NVreadVal = SR1NV_SETVAL ^ SR1NV_MASK ;
   uint8_t CR1NVreadVal = CR1NV_SETVAL ^ CR1NV_MASK ;
   uint8_t CR2NVreadVal = CR2NV_SETVAL ^ CR2NV_MASK ;
   uint8_t CR3NVreadVal = CR3NV_SETVAL ^ CR3NV_MASK ;

   SLLD_STATUS slld_status ;

   // Read SR1NV, CR1NV, CR2NV and CR3NV and verify values
   slld_status = SerialFLASH_S25FLL_RDARCmd ( device_num, /* device number */
         SR1NV, /* register address given by device*/
         &SR1NVreadVal ) ; /*variable in which to store read data*/

   if ( SLLD_OK == slld_status )
   {
      slld_status = SerialFLASH_S25FLL_RDARCmd ( device_num, /* device number */
            CR1NV, /* register address given by device*/
            &CR1NVreadVal ) ; /*variable in which to store read data*/
   }

   if ( SLLD_OK == slld_status )
   {
      slld_status = SerialFLASH_S25FLL_RDARCmd ( device_num, /* device number */
            CR2NV, /* register address given by device*/
            &CR2NVreadVal ) ; /*variable in which to store read data*/
   }

   if ( SLLD_OK == slld_status )
   {
      slld_status = SerialFLASH_S25FLL_RDARCmd ( device_num, /* device number */
            CR3NV, /* register address given by device*/
            &CR3NVreadVal ) ; /*variable in which to store read data*/
   }

   if ( ( ( SR1NV_SETVAL & SR1NV_MASK ) == ( SR1NVreadVal & SR1NV_MASK ) ) &&
        ( ( CR1NV_SETVAL & CR1NV_MASK ) == ( CR1NVreadVal & CR1NV_MASK ) ) &&
        ( ( CR2NV_SETVAL & CR2NV_MASK ) == ( CR2NVreadVal & CR2NV_MASK ) ) &&
        ( ( CR3NV_SETVAL & CR3NV_MASK ) == ( CR3NVreadVal & CR3NV_MASK ) ) &&
        ( ( SLLD_OK == slld_status ) ) )
   {
      *areRegsValid = true ;
   }
   else
   {
      *areRegsValid = false ;
   }

   int32_t success = ( SLLD_OK == slld_status ) ? EXIT_SUCCESS : EXIT_FAILURE ;

   return success ;
}

/* Determines whether volatile serial FLASH registers match specified configuration values. Sets *areRegsValid to TRUE if values are correct,
 * otherwise sets it to FALSE. If no serial FLASH error is encountered, the function returns EXIT_SUCCESS. Otherwise, the function
 * returns EXIT_FAILURE and *areRegsValid is set to FALSE. */
static int32_t SerialFLASH_S25FLL_VerifyVolatileRegisters ( const uint8_t device_num, // PIC32-SQI device number
                                                            bool* areRegsValid )
{
   /* Default all read values to something other than the specified configuration values */
   uint8_t SR1VreadVal = SR1NV_SETVAL ^ SR1NV_MASK ;
   uint8_t CR1VreadVal = CR1NV_SETVAL ^ CR1NV_MASK ;
   uint8_t CR2VreadVal = CR2NV_SETVAL ^ CR2NV_MASK ;
   uint8_t CR3VreadVal = CR3NV_SETVAL ^ CR3NV_MASK ;

   SLLD_STATUS slld_status ;

   slld_status = SerialFLASH_S25FLL_RDSRCmd ( device_num, //device number
         &SR1VreadVal ) ; /* variable in which to store read data */

   if ( SLLD_OK == slld_status )
   {
      slld_status = SerialFLASH_S25FLL_RDCR1Cmd ( device_num, /* device number */
            &CR1VreadVal ) ; /* variable in which to store read data */
   }

   if ( SLLD_OK == slld_status )
   {
      slld_status = SerialFLASH_S25FLL_RDCR2Cmd ( device_num, /* device number */
            &CR2VreadVal ) ; /* variable in which to store read data */
   }

   if ( SLLD_OK == slld_status )
   {
      slld_status = SerialFLASH_S25FLL_RDCR3Cmd ( device_num, /* device number */
            &CR3VreadVal ) ; /* variable in which to store read data */
   }

   if ( ( ( SR1NV_SETVAL & SR1NV_MASK ) == ( SR1VreadVal & SR1NV_MASK ) ) &&
        ( ( CR1NV_SETVAL & CR1NV_MASK ) == ( CR1VreadVal & CR1NV_MASK ) ) &&
        ( ( CR2NV_SETVAL & CR2NV_MASK ) == ( CR2VreadVal & CR2NV_MASK ) ) &&
        ( ( CR3NV_SETVAL & CR3NV_MASK ) == ( CR3VreadVal & CR3NV_MASK ) ) &&
        ( ( SLLD_OK == slld_status ) ) )
   {
      *areRegsValid = true ;
   }
   else
   {
      *areRegsValid = false ;
   }

   int32_t success = ( SLLD_OK == slld_status ) ? EXIT_SUCCESS : EXIT_FAILURE ;

   return success ;
}

/* Sets non-volatile serial FLASH registers to specified configuration values and then verifies. Returns TRUE if read-back values
 * are correct. Returns FALSE otherwise. */
static int32_t SerialFLASH_S25FLL_SetNVregisters ( const uint8_t device_num ) // PIC32-SQI device number
{
   const uint8_t SR1NVsetVal = SR1NV_SETVAL & SR1NV_MASK ;
   const uint8_t CR1NVsetVal = CR1NV_SETVAL & CR1NV_MASK ;
   const uint8_t CR2NVsetVal = CR2NV_SETVAL & CR2NV_MASK ;
   const uint8_t CR3NVsetVal = CR3NV_SETVAL & CR3NV_MASK ;

   DEVSTATUS devStatus ;
   SLLD_STATUS slld_status ;

   slld_status = SerialFLASH_S25FLL_WRAR_Op ( device_num, /* device number */
         SR1NV, /* register address given by device*/
         &SR1NVsetVal, /*variable containing data to program */
         &devStatus ) ; /* Pointer to the device status value after polling end */

   if ( SLLD_OK == slld_status )
   {
      slld_status = SerialFLASH_S25FLL_WRAR_Op ( device_num, /* device number */
            CR1NV, /* register address given by device*/
            &CR1NVsetVal, /*variable containing data to program */
            &devStatus ) ; /* Pointer to the device status value after polling end */
   }

   if ( SLLD_OK == slld_status )
   {
      slld_status = SerialFLASH_S25FLL_WRAR_Op ( device_num, /* device number */
            CR2NV, /* register address given by device*/
            &CR2NVsetVal, /*variable containing data to program */
            &devStatus ) ; /* Pointer to the device status value after polling end */
   }

   if ( SLLD_OK == slld_status )
   {
      slld_status = SerialFLASH_S25FLL_WRAR_Op ( device_num, /* device number */
            CR3NV, /* register address given by device*/
            &CR3NVsetVal, /*variable containing data to program */
            &devStatus ) ; /* Pointer to the device status value after polling end */
   }

   /* Reset serial FLASH */
   if ( SLLD_OK == slld_status )
   {
      slld_status = SerialFLASH_S25FLL_RSTENCmd ( device_num ) ; // Execute software reset enable command. Must come immediately before software reset command
   }

   if ( SLLD_OK == slld_status )
   {
      slld_status = SerialFLASH_S25FLL_RSTCmd ( device_num ) ; // Execute software reset of serial FLASH device
   }

   /* Wait for reset to complete (per serial flash datasheet) */
   CORETIMER_DelayUs ( S25FLL_RESET_DELAY_US ) ;

   if ( SLLD_OK == slld_status )
   {
      slld_status = SerialFLASH_S25FLL_Init_Private ( device_num ) ; // Initialize driver
   }

   int32_t success = EXIT_SUCCESS ;

   /* Verify NV status register values */
   if ( SLLD_OK == slld_status )
   {
      bool areNVregistersCorrect ;
      success = SerialFLASH_S25FLL_VerifyNVregisters ( device_num, &areNVregistersCorrect ) ;
      success = ( false == areNVregistersCorrect ) ? EXIT_FAILURE : success ;
   }
   else
   {
      success = EXIT_FAILURE ;
   }

   return success ;
}

/* Erases (sets to FFh) all bytes inside the flash memory array.
 * If erase operation is successful, returns EXIT_SUCCESS. Otherwise, returns EXIT_FAILURE. */
int32_t SerialFLASH_S25FLL_EraseAll ( const uint8_t device_num ) // PIC32-SQI device number
{
   DEVSTATUS dev_status ; /* variable to store device status */

   int32_t status = ( SLLD_OK == SerialFLASH_S25FLL_CEOp ( device_num, /* device number */
                      &dev_status ) ) /* variable to store device status */
         ? EXIT_SUCCESS : EXIT_FAILURE ;

   return status ;
}

/* Erases (sets to FFh) the specified number of bytes of serial FLASH memory starting at the specified address. The starting address must
 * be on a FLASH sector boundary and the size must be a multiple of the FLASH sector size. This function uses as a combination
 * of block, half-block, and sector erases to erase the defined memory region in the most efficient manner possible.
 * 
 * NOTE: Block size > Half-block size > Sector size. Block and half-block sizes are exact multiples of sector size.
 * 
 * If erase operation is successful, returns EXIT_SUCCESS. Otherwise, returns EXIT_FAILURE. */
int32_t SerialFLASH_S25FLL_Erase ( const uint8_t device_num, // PIC32-SQI device number
                                   const uint32_t flashAddr, // serial FLASH address to begin erasing from
                                   const uint32_t numBytesToErase ) // number of bytes of serial FLASH to erase
{
   if ( ( ( flashAddr % FLASH_SECTOR_SIZE ) != 0 ) ||
        ( ( numBytesToErase % FLASH_SECTOR_SIZE ) != 0 ) ||
        ( ( FLASH_SIZE - flashAddr ) < numBytesToErase ) ) // Verify address range is within serial FLASH
   {
      return EXIT_FAILURE ; // Error: Invalid function arguments
   }

   DEVSTATUS dev_status ; /* variable to store device status */
   SLLD_STATUS slld_status = SLLD_OK ;

   uint32_t currAddr = flashAddr ;
   uint32_t numBytesLeftToErase = numBytesToErase ;

   /* If not on half block boundary, erase sectors up to half block boundary or until there are no more bytes needing to be erased */
   while ( ( ( currAddr % FLASH_HALF_BLOCK_SIZE ) != 0 ) &&
           ( numBytesLeftToErase >= FLASH_SECTOR_SIZE ) &&
           ( slld_status == SLLD_OK ) )
   {
      slld_status = SerialFLASH_S25FLL_SEOp ( device_num, /* device number */
            currAddr, /* device address given by system */
            &dev_status ) ; /* variable to store device status */

      currAddr += FLASH_SECTOR_SIZE ;
      numBytesLeftToErase -= FLASH_SECTOR_SIZE ;
   }

   /* If on half block boundary but not block boundary and there is at least one half block worth of bytes to erase then erase the next half block */
   if ( ( ( currAddr % FLASH_BLOCK_SIZE ) != 0 ) &&
        ( numBytesLeftToErase >= FLASH_HALF_BLOCK_SIZE ) &&
        ( slld_status == SLLD_OK ) )
   {
      slld_status = SerialFLASH_S25FLL_HBEOp ( device_num, /* device number */
            currAddr, /* device address given by system */
            &dev_status ) ; /* variable to store device status */

      currAddr += FLASH_HALF_BLOCK_SIZE ;
      numBytesLeftToErase -= FLASH_HALF_BLOCK_SIZE ;
   }

   /* Erase full blocks. If there is at least a full block's worth of memory to erase then we must be on a block boundary. */
   while ( ( numBytesLeftToErase >= FLASH_BLOCK_SIZE ) &&
           ( slld_status == SLLD_OK ) )
   {
      slld_status = SerialFLASH_S25FLL_BEOp ( device_num, /* device number */
            currAddr, /* device address given by system */
            &dev_status ) ; /* variable to store device status */

      currAddr += FLASH_BLOCK_SIZE ;
      numBytesLeftToErase -= FLASH_BLOCK_SIZE ;
   }

   /* Erase up to one half block. If there is at least a half block's worth of memory to erase then we must be on a half block boundary. */
   if ( ( numBytesLeftToErase >= FLASH_HALF_BLOCK_SIZE ) &&
        ( slld_status == SLLD_OK ) )
   {
      slld_status = SerialFLASH_S25FLL_HBEOp ( device_num, /* device number */
            currAddr, /* device address given by system */
            &dev_status ) ; /* variable to store device status */

      currAddr += FLASH_HALF_BLOCK_SIZE ;
      numBytesLeftToErase -= FLASH_HALF_BLOCK_SIZE ;
   }

   /* Erase any remaining sectors that need to be erased. */
   while ( ( numBytesLeftToErase >= FLASH_SECTOR_SIZE ) &&
           ( slld_status == SLLD_OK ) )
   {
      slld_status = SerialFLASH_S25FLL_SEOp ( device_num, /* device number */
            currAddr, /* device address given by system */
            &dev_status ) ; /* variable to store device status */

      currAddr += FLASH_SECTOR_SIZE ;
      numBytesLeftToErase -= FLASH_SECTOR_SIZE ;
   }

   int32_t status = ( SLLD_OK == slld_status ) ? EXIT_SUCCESS : EXIT_FAILURE ;

   return status ;
}

/* Reads the specified number of serial FLASH memory data bytes, starting at the specified serial FLASH address, from the serial
 * FLASH and copies to the destination buffer. The serial FLASH addresses must be within the range of the serial FLASH. * 
 * 
 * If read operation is successful, returns EXIT_SUCCESS. Otherwise, returns EXIT_FAILURE. */
int32_t SerialFLASH_S25FLL_Read ( const uint8_t device_num, // PIC32-SQI device number
                                  const uint32_t flashAddr, // serial FLASH address to read from
                                  const uint32_t numBytesToRead, // number of bytes to read from serial FLASH
                                  uint8_t * const destAddr ) // address to write bytes to
{
   if ( ( FLASH_SIZE - flashAddr ) < numBytesToRead ) // Verify address range is within serial FLASH
   {
      return EXIT_FAILURE ; // Error: Invalid function arguments
   }

   int32_t status = ( SLLD_OK == SerialFLASH_S25FLL_ReadOp ( device_num, /* device number */
                      flashAddr, /* device address given by system */
                      destAddr, /* variable in which to store read data */
                      numBytesToRead ) ) /* number of bytes to read */
         ? EXIT_SUCCESS : EXIT_FAILURE ;

   return status ;
}

/* Writes the specified number of serial FLASH memory data bytes, starting at the specified serial FLASH address with the data
 * copied from the source buffer. The serial FLASH addresses must be within the range of the serial FLASH. * 
 * 
 * If write operation is successful, returns EXIT_SUCCESS. Otherwise, returns EXIT_FAILURE. */
int32_t SerialFLASH_S25FLL_Write ( const uint8_t device_num, // PIC32-SQI device number
                                   const uint32_t flashAddr, // serial FLASH address to write to
                                   const uint32_t numBytesToWrite, // number of bytes to write to serial FLASH
                                   uint8_t * const srcAddr ) // source address of bytes to be written to serial FLASH
{
   if ( ( FLASH_SIZE - flashAddr ) < numBytesToWrite ) // Verify address range is within serial FLASH
   {
      return EXIT_FAILURE ; // Error: Invalid function arguments
   }

   uint8_t* currAddr = srcAddr ;
   DEVSTATUS dev_status ; /* variable to store device status */

   int32_t status = ( SLLD_OK == SerialFLASH_S25FLL_WriteOp ( device_num, /* device number */
                      flashAddr, /* device address given by system */
                      currAddr, /* variable containing data to program */
                      numBytesToWrite, /* number of bytes to program */
                      &dev_status ) ) /* variable to store device status */
         ? EXIT_SUCCESS : EXIT_FAILURE ;

   return status ;
}


/* End of SerialFLASH_S25FLL.c source file. */
