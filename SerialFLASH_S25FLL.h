/**
 *  @file SerialFLASH_S25FLL.h
 *  @brief Header file for SerialFLASH_S25FLL.c
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

#ifndef SERIALFLASH_S25FLL_H
#define SERIALFLASH_S25FLL_H

/**************  Included Files  ************************/
#include <stdint.h>


#ifdef __cplusplus
extern "C"
{
#endif



   /**************  Definitions  ************************/




   /**************  Function Prototypes ************************/

   /* Initializes the serial FLASH device. Takes SQI device number of serial FLASH. If serial FLASH initialization is successful,
    * returns EXIT_SUCCESS. Otherwise, returns EXIT_FAILURE. */
   int32_t SerialFLASH_S25FLL_Initialize ( const uint8_t device_num ) ; // PIC32-SQI device number

   /* Erases (sets to FFh) all bytes inside the flash memory array.
    * If erase operation is successful, returns EXIT_SUCCESS. Otherwise, returns EXIT_FAILURE. */
   int32_t SerialFLASH_S25FLL_EraseAll ( const uint8_t device_num ) ; // PIC32-SQI device number

   /* Erases (sets to FFh) the specified number of bytes of serial FLASH memory starting at the specified address. The starting address must
    * be on a FLASH sector boundary and the size must be a multiple of the FLASH sector size. This function uses as a combination
    * of block, half-block, and sector erases to erase the defined memory region in the most efficient manner possible.
    * 
    * NOTE: Block size > Half-block size > Sector size. Block and half-block sizes are exact multiples of sector size.
    * 
    * If erase operation is successful, returns EXIT_SUCCESS. Otherwise, returns EXIT_FAILURE. */
   int32_t SerialFLASH_S25FLL_Erase ( const uint8_t device_num, // PIC32-SQI device number
                                      const uint32_t flashAddr, // serial FLASH address to begin erasing from
                                      const uint32_t numBytesToErase ) ; // number of bytes of serial FLASH to erase

   /* Reads the specified number of serial FLASH memory data bytes, starting at the specified serial FLASH address, from the serial
    * FLASH and copies to the destination buffer. The serial FLASH addresses must be within the range of the serial FLASH. * 
    * 
    * If read operation is successful, returns EXIT_SUCCESS. Otherwise, returns EXIT_FAILURE. */
   int32_t SerialFLASH_S25FLL_Read ( const uint8_t device_num, // PIC32-SQI device number
                                     const uint32_t flashAddr, // serial FLASH address to read from
                                     const uint32_t numBytesToRead, // number of bytes to read from serial FLASH
                                     uint8_t * const destAddr ) ; // address to write bytes to

   /* Writes the specified number of serial FLASH memory data bytes, starting at the specified serial FLASH address with the data
    * copied from the source buffer. The serial FLASH addresses must be within the range of the serial FLASH. * 
    * 
    * If write operation is successful, returns EXIT_SUCCESS. Otherwise, returns EXIT_FAILURE. */
   int32_t SerialFLASH_S25FLL_Write ( const uint8_t device_num, // PIC32-SQI device number
                                      const uint32_t flashAddr, // serial FLASH address to write to
                                      const uint32_t numBytesToWrite, // number of bytes to write to serial FLASH
                                      uint8_t * const srcAddr ) ; // source address of bytes to be written to serial FLASH


#ifdef	__cplusplus
}
#endif

#endif

/* End of SerialFLASH.h header file. */
