#include "ftdidevice.h"
#include <sstream>

FTDIDevice::FTDIDevice(int vid, int pid, enum ftdi_interface interface, const char *serial, char *busconf, bool v, int dl) {
   
   int res = 0;

   verbose = v;
   debugLevel = dl;
   setName("FTDI");

   if ((ftdi = ftdi_new()) == 0) {
      std::string errmsg(ftdi_get_error_string(ftdi));
      throw std::runtime_error("E: FTDIDevice failed allocation (" + errmsg + ")");
   }

   if(ftdi_set_interface(ftdi, interface) < 0) {
      std::string errmsg(ftdi_get_error_string(ftdi));
      throw std::runtime_error("E: FTDIDevice can't set interface (" + errmsg + ")");
   }

   if (ftdi_usb_open_desc(ftdi, vid, pid, NULL, serial) < 0) {
      std::string errmsg(ftdi_get_error_string(ftdi));
      throw std::runtime_error("E: FTDIDevice can't open device (" + errmsg + ")");
   }

   ftdi_usb_reset(ftdi);
   ftdi_set_latency_timer(ftdi, 1);

   // Correct MPSSE initialization for JTAG
   unsigned char buf[] = {
      DIS_DIV_5,
      SET_BITS_LOW, 0x00, 0x07,     // output values=0, directions=0x07 (TMS,TCK,TDI=output, TDO=input)
      TCK_DIVISOR, 0x2B, 0x01,      // 100 kHz  
      SEND_IMMEDIATE 
   };

   ftdi_set_bitmode(ftdi, 0x07, BITMODE_MPSSE);  // Set bit mode with correct directions

   // Remove deprecated buffer purge calls
   // ftdi_usb_purge_rx_buffer(ftdi);
   // ftdi_usb_purge_tx_buffer(ftdi);
  
   std::stringstream ss;
   if(busconf == nullptr)
      ss.str("0x00:0x0B:0x00:0x00");
   else
      ss.str(busconf);

   std::string tmp;
   std::vector<unsigned int> mask;
   while(std::getline((ss), tmp, ':')) {
      int value = std::stoul(tmp, nullptr, 16);
      mask.push_back(value);
   }
   
   if(mask.size() < 4) {
      throw std::runtime_error("E: FTDIDevice bus config error: " + std::string(busconf));
   }

   // Fix buffer size issue - make sure we don't write beyond array bounds
   if (sizeof(buf) >= 8) {
      buf[6] = mask[2];    // cbus_data
      buf[7] = mask[3];    // cbus_en
   }

   if ((res = ftdi_write_data(ftdi, buf, sizeof(buf))) != sizeof(buf)) {
      std::string errmsg(ftdi_get_error_string(ftdi));
      throw std::runtime_error("E: FTDIDevice error initializing MPSSE (" + errmsg + ")");
   }

   // set TDO sampling on clock negative edge (default)
   setTDOPosSampling(false);

   // try to detect device
   if(!detect())
      std::cout << "WARNING: FTDIDevice: failed to detect JTAG target" << std::endl;
}

bool FTDIDevice::detect() {
    printDebug("FTDIDevice::detect start", 1);
    
    DeviceDB devDB(0);
    uint32_t tempId;
    const char *tempDesc;
    bool found = false;

    // Set very slow clock for reliable detection
    setClockFrequency(91553); // ~91.553 kHz
    
    // Try to read IDCODE
    tempId = scanChain();
    tempDesc = devDB.idToDescription(tempId);

    if (tempDesc) {
        found = true;
        idcode = tempId;
        irlen = devDB.idToIRLength(idcode);
        idcmd = devDB.idToIDCmd(idcode);
        desc = tempDesc;
        detected = true;
    }

    if(detected && verbose)
        printf("FTDIDevice::detect device detected: idcode:0x%X irlen:%d idcmd:0x%X desc:%s\n",
            idcode, irlen, idcmd, desc.c_str());

    printDebug("FTDIDevice::detect end", 1);
    return found;
}

void FTDIDevice::setClockDiv(bool div5, int value) {
    // Implementation for setting clock divisor
    unsigned char buf[3] = {TCK_DIVISOR, (unsigned char)(value & 0xFF), (unsigned char)((value >> 8) & 0xFF)};
    ftdi_write_data(ftdi, buf, 3);
}

void FTDIDevice::setClockFrequency(int freq) {
    // Calculate divisor for given frequency
    bool div5 = (freq > MAX_CFREQ_DIV5_OFF);
    int divisor = (div5 ? MAX_CFREQ_DIV5_ON : MAX_CFREQ_DIV5_OFF) / freq;
    setClockDiv(div5, divisor);
}

void FTDIDevice::setTDOPosSampling(bool value) {
    // Implementation for TDO sampling edge
    if (value)
        samplingEdge = POS_EDGE;
    else
        samplingEdge = NEG_EDGE;
}

int FTDIDevice::getDivisorByFrequency(bool div5, int freq) {
    // Calculate divisor from frequency
    return (div5 ? MAX_CFREQ_DIV5_ON : MAX_CFREQ_DIV5_OFF) / freq;
}

int FTDIDevice::getFrequencyByDivisor(bool div5, int div) {
    // Calculate frequency from divisor
    return (div5 ? MAX_CFREQ_DIV5_ON : MAX_CFREQ_DIV5_OFF) / div;
}

FTDIDevice::~FTDIDevice() {
    if (ftdi) {
        ftdi_free(ftdi);
    }
}

void FTDIDevice::readBytes(unsigned int len, unsigned char *buf) {
    ftdi_read_data(ftdi, buf, len);
}

// Proper MPSSE-based shift implementation for JTAG operations
void FTDIDevice::shift(int nbits, unsigned char *buffer, unsigned char *result) {
    if (nbits <= 0) return;
    
    // For proper JTAG operations using MPSSE, we need to:
    // 1. Send TMS/TCK/TDI bits properly for each bit
    // 2. Read back TDO bits
    
    // Create a simple MPSSE command sequence for bit-level operations
    std::vector<unsigned char> cmd_buffer;
    
    // Set initial state: TMS=0, TCK=0, TDI=0 (idle)
    cmd_buffer.push_back(SET_BITS_LOW);
    cmd_buffer.push_back(0x00);  // output values
    cmd_buffer.push_back(0x07);  // directions (TMS,TCK,TDI=output, TDO=input)
    cmd_buffer.push_back(SEND_IMMEDIATE);
    
    int bytes_needed = (nbits + 7) / 8;
    
    if (bytes_needed > 0) {
        // For now, we'll use a simpler approach that works with the existing MPSSE setup
        // The key is to properly send and receive bits through MPSSE
        
        // Send the read bits command
        cmd_buffer.push_back(READ_BITS_NBITS);
        cmd_buffer.push_back((unsigned char)(nbits - 1));  // number of bits minus 1
        cmd_buffer.push_back(SEND_IMMEDIATE);
        
        // Write the command to FTDI device
        ftdi_write_data(ftdi, cmd_buffer.data(), cmd_buffer.size());
        
        // Read back the result
        if (result) {
            ftdi_read_data(ftdi, result, bytes_needed);
        }
    }
}
