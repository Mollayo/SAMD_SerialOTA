/*
  Copyright (c) 2017 Arduino LLC.  All right reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "SAMD_SerialOTA.h"

#include <SerialPackets.h>
#include <FlashStorage.h>

#include "SAMD_OTAInternalFlash.h"

// For CRC32 checksum https://github.com/bakercp/CRC32
#include <CRC32.h>


#if defined(__SAMD51__)
// https://github.com/SapientHetero/TRNG-for-ATSAMD51J19A-Adafruit-Metro-M4-
// Only for SAMD51
extern "C" {
#include <trngFunctions.h>    // For true random number generator 
}
#endif

char SAMD_SerialOTA::version[]={0x00};

OTAInternalFlash my_internal_storage;

SerialPackets packetsOTA;
HardwareSerial* _hwSerial=nullptr;
#ifdef SOFTWARESERIAL
SoftwareSerial* _swSerial=nullptr;
#endif 
Stream* _debugPort=nullptr;

bool doLoop=true;
bool errorDownloadingFirmware=false;

// For CRC computation
CRC32 crc;


uint8_t flash_buff[OTAInternalFlash::BLOCK_SIZE]={0x00};
uint32_t flash_buff_size=0;
uint32_t flash_offset=0;
uint32_t firmware_size=0;
uint8_t firmware_cmd=0;
#define CMD_WRITE_FIRMWARE 1
#define CMD_REBOOT         2

unsigned long ledTime=millis();

///////////////////////////////////////////////////////
// Read and print the flash at the specified address //
///////////////////////////////////////////////////////
uint32_t getFlashContent(uint32_t addr)
{
  uint32_t buff_offset=addr%OTAInternalFlash::BLOCK_SIZE;
  addr=addr-buff_offset;
  if (addr>=my_internal_storage.get_flash_size()+((uint32_t)my_internal_storage.get_flash_address()))
  {
    packetsOTA.printf("ADDR OUT OF RANGE\n");
    return 0;
  }
  my_internal_storage.readAtAbsoluteAddr(addr, flash_buff, OTAInternalFlash::BLOCK_SIZE);

  uint32_t val=0x00;
  uint8_t len=sizeof(val);
  if (buff_offset+len>OTAInternalFlash::BLOCK_SIZE)
    len=OTAInternalFlash::BLOCK_SIZE-buff_offset;
  for (int i = 0; i < len; i++) 
  {
    ((uint8_t*)&val)[len-i-1]=flash_buff[buff_offset+i];
  }
  return val;
}


bool writeFirmwareBufferToFlash()
{
  if (errorDownloadingFirmware)
    return false;

  // Check if enough space to write the buffer
  if (flash_offset+OTAInternalFlash::BLOCK_SIZE<=my_internal_storage.get_flash_size())
  {    
    uint8_t nbTrials=0;
    while(nbTrials<3)
    {
      // Write the buffer to the flash
      my_internal_storage.write(flash_offset,(const void *)flash_buff,OTAInternalFlash::BLOCK_SIZE);
      // Compare the content of the flash with the one from the buffer
      if (my_internal_storage.sameContent(flash_offset,(const void *)flash_buff,OTAInternalFlash::BLOCK_SIZE))
        break;
      nbTrials++;
      if (nbTrials>=3)
      {
        packetsOTA.printf("ERROR WRITING TO FLASH\n");
        errorDownloadingFirmware=true;
        return false;
      }
    }
  }
  else
  {
    // Send the message flash full
    packetsOTA.printf("ERROR NO SPACE LEFT\n");
    errorDownloadingFirmware=true;
    return false;
  }
  flash_offset=flash_offset+flash_buff_size;
  flash_buff_size=0;

  return true;
}

bool firmwarePacketBegin(uint8_t * payload, uint8_t payloadSize)
{
  // Init variables for uploading the firmware
  flash_buff_size=0;
  flash_offset=0;
  errorDownloadingFirmware=false;

  // If payload is defined, it should contain the FIRMWARE value
  if (payloadSize>0)
  {
    // Potential bug. SHould check the exact length
    if (!(payloadSize==strlen("FIRMWARE") && memcmp(payload,"FIRMWARE",strlen("FIRMWARE"))!=0))
    {
      // assume that payloadSize <=MAX_PAYLOAD_SIZE
      payload[payloadSize]=0x00;
      packetsOTA.printf("NOT A FIRMWARE \"%s\"",payload);
      errorDownloadingFirmware=true;
      return false;
    }
  }

  return true;
}

void firmwarePacketEnd(uint8_t * payload, uint8_t payloadSize)
{
  if (errorDownloadingFirmware)
    return;

  // Write down the remaining data to the flash if any
  bool OK=true;
  if (flash_buff_size>0)
    OK=writeFirmwareBufferToFlash();
  // Save the firmware size
  firmware_size=flash_offset;
  // Write the firmware
  if (OK)
  {
    firmware_cmd=CMD_WRITE_FIRMWARE;
  }
}

//////////////////////////////////////////////////////////////////////////////
// Getting the firmware data from the serial and write it down to the flash //
//////////////////////////////////////////////////////////////////////////////
bool firmwarePacketReceived(uint8_t * payload, uint8_t payloadSize)
{
  // Shows some activities with the builtin led
  if (millis()-ledTime>100)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    ledTime=millis();
  }

  if (errorDownloadingFirmware)
    return false;

  // In the case of receiving the first packet, we check the signature
  if (flash_offset==0 && flash_buff_size==0)
  {
    if (payloadSize<4)
    {
      packetsOTA.printf("FIRMWARE SIZE TOO SMALL\n");
      errorDownloadingFirmware=true;
      return false;
    }
    uint32_t currentSketchSignature=getFlashContent((uint32_t)my_internal_storage.get_sketch_start_address());
    uint8_t newSketchSignature[4]={payload[3],payload[2],payload[1],payload[0]};
    if (memcmp(&currentSketchSignature,newSketchSignature,sizeof(currentSketchSignature))!=0)
    {
      packetsOTA.printf("FIRMWARE SIGNATURES DONT MATCH: 0x%08X vs 0x%02X%02X%02X%02X\n",
                        currentSketchSignature,newSketchSignature[0],newSketchSignature[1],
                                               newSketchSignature[2],newSketchSignature[3]);

      errorDownloadingFirmware=true;
      return false;
    }
  }

  for(uint16_t i=0; i<payloadSize; i++)
  {
    // If the buffer is full
    if (flash_buff_size>=OTAInternalFlash::BLOCK_SIZE)
    {
      bool OK=writeFirmwareBufferToFlash();
      if (!OK)
        return false;
    }
    // Fill the buffer for the flash
    flash_buff[flash_buff_size]=payload[i];
    flash_buff_size++;
  }
  return true;
}

void scanFlashForErrors()
{
  packetsOTA.printf("START SCANNING FLASH\n");
  
  bool error=false;
  uint32_t flash_offset=0;

  // Generate values for the writing
  char c=0x30;
  for (uint16_t i=0;i<OTAInternalFlash::BLOCK_SIZE;i++)
  {
    flash_buff[i]=c;
    c++;
    if (c>0x39)
      c=0x30;
  }

  while(flash_offset<my_internal_storage.get_flash_size())
  {
    // Write the data to the flash
    my_internal_storage.write(flash_offset,(const void*)flash_buff,OTAInternalFlash::BLOCK_SIZE);
    
    // Compare the data
    if (!my_internal_storage.sameContent(flash_offset,(const void *)flash_buff,OTAInternalFlash::BLOCK_SIZE))
    {
      packetsOTA.printf("ERROR AT 0x%08X\n",my_internal_storage.get_flash_address()+flash_offset);
      error=true;
    }
    packetsOTA.printf("0x%08X\n",my_internal_storage.get_flash_address()+flash_offset);
    flash_offset=flash_offset+OTAInternalFlash::BLOCK_SIZE;
  }
  if (error==false)
    packetsOTA.printf("NO ERROR FOUND\n");
  else
    packetsOTA.printf("FINISHED SCANNING\n");  
}

// https://forum.arduino.cc/t/hex-string-to-byte-array/563827/3
byte nibble(char c)
{
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  return 0;  // Not a valid hexadecimal character
}

void hexCharacterStringToBytes(byte *byteArray, const char *hexString)
{
  bool oddLength = strlen(hexString) & 1;

  byte currentByte = 0;
  byte byteIndex = 0;

  for (byte charIndex = 0; charIndex < strlen(hexString); charIndex++)
  {
    bool oddCharIndex = charIndex & 1;

    if (oddLength)
    {
      // If the length is odd
      if (oddCharIndex)
      {
        // odd characters go in high nibble
        currentByte = nibble(hexString[charIndex]) << 4;
      }
      else
      {
        // Even characters go into low nibble
        currentByte |= nibble(hexString[charIndex]);
        byteArray[byteIndex++] = currentByte;
        currentByte = 0;
      }
    }
    else
    {
      // If the length is even
      if (!oddCharIndex)
      {
        // Odd characters go into the high nibble
        currentByte = nibble(hexString[charIndex]) << 4;
      }
      else
      {
        // Odd characters go into low nibble
        currentByte |= nibble(hexString[charIndex]);
        byteArray[byteIndex++] = currentByte;
        currentByte = 0;
      }
    }
  }
}

//////////////////////////////////////////////////
// Compute the CRC of the firmware in the flash //
//////////////////////////////////////////////////
uint32_t crcFlash() 
{
  CRC32 crc;
  uint32_t flash_offset=0;
  uint32_t buff_size=0;
  while(flash_offset<firmware_size)
  {
    buff_size=OTAInternalFlash::BLOCK_SIZE;
    if (buff_size+flash_offset>firmware_size)
      buff_size=firmware_size-flash_offset;
    my_internal_storage.read(flash_offset, flash_buff, buff_size);
    
    for (int i = 0; i < buff_size; i++) 
      crc.update(flash_buff[i]);
    flash_offset=flash_offset+buff_size;
  }
  return crc.finalize();
}

void packetReceived(uint8_t *payload, uint8_t payloadSize)
{
  char* cmd=(char*)payload;
  
  // Execute the commande
  if (strstr(cmd,"BAUD ")!=nullptr)
  {
    // Change the baud rate
    uint32_t bd=atoi(&cmd[strlen("BAUD ")]);
#ifdef SOFTWARESERIAL
  if (_hwSerial)
    _hwSerial->begin(bd);
  else
    _swSerial->begin(bd);
#else
    _hwSerial->begin(bd);
#endif
  }
  else if (strcmp(cmd,"FIRMWARE_CRC")==0)
  {
    // CRC32 from the flash
    uint32_t crc=crcFlash();
    packetsOTA.printf("%08X\n",crc);
  }
  else if (strstr(cmd,"PRINT_FLASH ")!=nullptr)
  {
    hexCharacterStringToBytes(flash_buff, &(cmd[strlen("PRINT_FLASH ")]));
    uint8_t tmp[4]={flash_buff[3],flash_buff[2],flash_buff[1],flash_buff[0]};
    uint32_t addr=((uint32_t*)tmp)[0];
    packetsOTA.printf("Addr: 0x%08X\n",addr);
    uint32_t val=getFlashContent(addr);
    packetsOTA.printf("Content: 0x%08X\n",val);
    return;
  }
  else if (strcmp(cmd,"SCAN_FLASH")==0)
  {
    scanFlashForErrors();
    return;
  }
  else if (strcmp(cmd,"FIRMWARE_SIZE")==0)
  {
    packetsOTA.printf("%d\n",firmware_size);
  }
  else if (strcmp(cmd,"BLOCK_SIZE")==0)
  {
    // CRC from the received serial data
    packetsOTA.printf("%d\n",OTAInternalFlash::BLOCK_SIZE);
  }
  else if (strcmp(cmd,"REBOOT")==0)
  {
    // Reboot the MCU
    firmware_cmd=CMD_REBOOT;
    return;
  }
  else if (strcmp(cmd,"EXIT")==0)
  {
    // Continue to the main loop
    doLoop=false;
    packetsOTA.printf("EXITING OTA\n");
    return;
  }
  else if (strcmp(cmd,"SKETCH_ADDR")==0)
  {
    packetsOTA.printf("0x%08X\n",my_internal_storage.get_sketch_start_address());
  }
  else if (strcmp(cmd,"FLASH_ADDR")==0)
  {
    packetsOTA.printf("0x%08X\n",my_internal_storage.get_flash_address());
  }
  else if (strcmp(cmd,"FLASH_SIZE")==0)
  {
    packetsOTA.printf("%d\n",my_internal_storage.get_flash_size());
  }
  else if (strcmp(cmd,"WRITE_FIRMWARE")==0)
  {
    // The firmware should have been uploaded first (with no reboot in between)
    if (firmware_size==0)
      packetsOTA.printf("FIRMWARE NOT UPLOADED\n");
    else
      my_internal_storage.writeFirmwareAndReboot(firmware_size);
  }
  else if (strcmp(cmd,"VERSION")==0)
  {
    packetsOTA.printf("%s\n", SAMD_SerialOTA::version);
  }
  else
  {
    // Send echo to the unknown cmd
    // cmd is supposed to be a readable string
    packetsOTA.printf("?%s\n",cmd);
  }
}



void packetError(uint8_t *payload, uint8_t payload_size)
{
  if (_debugPort)
    _debugPort->printf("samd: error sending packet\n");
}

SAMD_SerialOTA::SAMD_SerialOTA(const char vers[])
{
  if (strlen(vers)<sizeof(version))
    memcpy(version,vers,strlen(vers));
}

void SAMD_SerialOTA::begin(HardwareSerial& hwSerial)
{
  _hwSerial=&hwSerial;
#ifdef SOFTWARESERIAL
  _swSerial=nullptr;
#endif

#if defined(__SAMD51__)
  // Initialize the True Random Number Generator
  trngInit();
  packetsOTA.begin(*_hwSerial, trngGetRandomNumber());
#else
  packetsOTA.begin(*_hwSerial);
#endif
}

#ifdef SOFTWARESERIAL
void SAMD_SerialOTA::begin(SoftwareSerial& swSerial)
{
  _swSerial=&swSerial;
  _hwSerial=nullptr;
#if defined(__SAMD51__)
  // Initialize the True Random Number Generator
  trngInit();
  packetsOTA.begin(*_swSerial, trngGetRandomNumber());
#else
  packetsOTA.begin(*_swSerial);
#endif
}
#endif

void SAMD_SerialOTA::setDebugPort(Stream& stream)
{
  _debugPort=&stream;
}

void SAMD_SerialOTA::loop() 
{
  doLoop=true;
  pinMode(LED_BUILTIN, OUTPUT);

  packetsOTA.setReceiveCallback(packetReceived);
  packetsOTA.setErrorCallback(packetError);

  // For firmware upload
  packetsOTA.setOpenFileCallback(firmwarePacketBegin);
  packetsOTA.setReceiveFileDataCallback(firmwarePacketReceived);
  packetsOTA.setCloseFileCallback(firmwarePacketEnd);
  if (_debugPort)
    packetsOTA.setDebugPort(*_debugPort);

  packetsOTA.printf("ENTERING OTA\n");

  while(doLoop)
  {
    if (millis()-ledTime>1000)
    {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      ledTime=millis();
    }
    // Process serial data
    packetsOTA.update();
    // To prevent triggering the WDT
    yield();

    // Write the firmware and reboot if a new firmware is available
    if (firmware_cmd==CMD_WRITE_FIRMWARE)
    {
      packetsOTA.printf("WRITE THE FIRMWARE AND REBOOT NOW\n");
      my_internal_storage.writeFirmwareAndReboot(firmware_size);
      // Will actually never reach this point
    }
    else if (firmware_cmd==CMD_REBOOT)
    {
      packetsOTA.printf("REBOOT NOW\n");
      NVIC_SystemReset();
    }
  }
}


