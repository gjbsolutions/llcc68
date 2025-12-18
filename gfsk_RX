from machine import SPI, Pin
import time

# Initialize SPI
spi = SPI(1, baudrate=1000000, polarity=0, phase=0, bits=8, firstbit=SPI.MSB, 
          sck=Pin(4), mosi=Pin(6), miso=Pin(5))
cs = Pin(7, Pin.OUT)  

RESET = Pin(2, Pin.OUT)
BUSY = Pin(3, Pin.IN)
DIO1 = Pin(10, Pin.IN)
TXEN = Pin(1, Pin.OUT, value=0)
RXEN = Pin(0, Pin.OUT, value=0)

def hardware_reset():
    RESET.value(0)          
    time.sleep_us(200)      
    RESET.value(1)          
    time.sleep(0.01)        

def wait_if_busy():
    while BUSY.value() == 1:
        pass
    
def write_opcode(data):
    wait_if_busy()  
    cs.value(0)  
    spi.write(bytearray(data))  
    cs.value(1)  

def write_32bit_opcode(opcode, value):
    wait_if_busy()  
    cs.value(0)
    data = bytearray([
        opcode,
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF
    ])
    spi.write(data)
    cs.value(1)
    
def write_bufferbaseaddr():
    write_opcode([0x8F,0x00,0x00])
    
def write_modparameters():
    """
    RadioLib defaults:
    - Bit rate: 48 kbps (BR = 21333)
    - Freq dev: 50 kHz (52428 in register format)
    - RX BW: 156.2 kHz (0x1A)
    - Pulse shape: Gaussian 0.5 (0x09)
    """
    # Calculate bit rate: BR = 32 * Fxtal / bitrate
    # For 48 kbps: 32 * 32000000 / 48000 = 21333.33 = 0x005355
    br = 21333
    
    # Calculate freq deviation: Fdev = (freqDev * 2^25) / Fxtal
    # For 50 kHz: (50000 * 33554432) / 32000000 = 52428.8 = 0x00CCCD
    fdev = 52428
    
    data = bytearray([
        0x8B,
        (br >> 16) & 0xFF,   # BR byte 1
        (br >> 8) & 0xFF,    # BR byte 2
        br & 0xFF,           # BR byte 3
        0x09,                # Pulse shape: Gaussian BT 0.5
        0x1A,                # RX_BW: 156.2 kHz
        (fdev >> 16) & 0xFF, # Fdev byte 1
        (fdev >> 8) & 0xFF,  # Fdev byte 2
        fdev & 0xFF          # Fdev byte 3
    ])
    write_opcode(data)
    
def write_packetparameters():
    """
    RadioLib defaults:
    - Preamble: 16 bytes (0x0010)
    - Preamble detector: 16 bits (0x05)
    - Sync word: 2 bytes (16 bits = 0x10)
    - Variable length packet (0x01)
    - CRC: 2-byte inverted (0x06)
    - Whitening: disabled (0x00)
    """
    data = bytearray([
        0x8C,
        0x00, 0x10,  # Preamble length: 16 bytes
        0x05,        # Preamble detector: 16 bits
        0x10,        # Sync word length: 16 bits (2 bytes)
        0x00,        # Address filtering: disabled
        0x01,        # Variable length packet
        0xFF,        # Max payload length: 255 bytes
        0x06,        # CRC: 2-byte inverted (CCITT)
        0x00         # Whitening: disabled
    ])
    write_opcode(data)
    
def write_setdioirq():
    """Configure IRQ for RxDone, Timeout, and CRC error on DIO1"""
    data = bytearray([
        0x08,       # SetDioIrqParams opcode
        0x02, 0x42, # IrqMask: RxDone + CrcErr + Timeout
        0x02, 0x42, # DIO1Mask: map all to DIO1
        0x00, 0x00, # DIO2Mask
        0x00, 0x00  # DIO3Mask
    ])
    write_opcode(data)
    
def write_syncword():
    """
    Set sync word to RadioLib default: 0x12AD
    Need to write 2 bytes at 0x06C0 and 0x06C1
    """
    data = bytearray([
        0x0D,       # WriteRegister opcode
        0x06, 0xC0, # Start address
        0x12, 0xAD  # Sync word: 0x12, 0xAD
    ])
    write_opcode(data)

def write_crc_config():
    """
    Configure CCITT CRC (2-byte inverted)
    Initial value: 0xFFFF
    Polynomial: 0x1021
    """
    # Set CRC initial value
    data = bytearray([
        0x0D,       # WriteRegister opcode
        0x06, 0xBC, # CRC initial value MSB address
        0x1D, 0x0F  # Initial value: 0xFFFF (CCITT standard)
    ])
    write_opcode(data)
    time.sleep_us(100)
    # Set CRC polynomial
    data = bytearray([
        0x0D,       # WriteRegister opcode
        0x06, 0xBE, # CRC polynomial MSB address
        0x10, 0x21  # Polynomial: 0x1021 (CCITT standard)
    ])
    write_opcode(data)
    
def write_setrx():
    """Enter RX mode with 10 second timeout"""
    data = bytearray([
        0x82,
        0x09, 0xC4, 0x00  # ~10s timeout
    ])
    write_opcode(data)

def write_clearirq():
    """Clear all IRQ flags"""
    data = bytearray([
        0x02,
        0xFF, 0xFF  # Clear all IRQs
    ])
    write_opcode(data)

def freq_to_reg(freq_hz):
    return int((freq_hz / 32000000.0) * (1 << 25))

def calibrate_image_868():
    """Calibrate for 863-870MHz band"""
    write_opcode([0x98,0xD7,0xDB])
    
def calibrate_image_433():
    """Calibrate for 430-440MHz band"""
    write_opcode([0x98,0x6B,0x6F])

def get_irq_status():
    wait_if_busy()
    cs.value(0)
    tx = bytearray([0x12, 0x00, 0x00, 0x00])
    rx = bytearray(4)
    spi.write_readinto(tx, rx)
    cs.value(1)
    irq = (rx[2] << 8) | rx[3]
    return irq, rx[1]

def get_rx_buffer_status():
    wait_if_busy()
    cs.value(0)
    tx = bytearray([0x13, 0x00, 0x00, 0x00])
    rx = bytearray(4)
    spi.write_readinto(tx, rx)
    cs.value(1)
    return rx[2], rx[3]  # payload_len, start_addr

def read_rx_buffer(length, start_address):
    wait_if_busy()
    cs.value(0)
    tx = bytearray([0x1E, start_address] + [0x00] * (1 + length))
    rx = bytearray(len(tx))
    spi.write_readinto(tx, rx)
    cs.value(1)
    return rx[3:3 + length]

def enable_rx_boosted_gain():
    wait_if_busy()
    cs.value(0)
    data = bytearray([
        0x0D,       # WriteRegister opcode
        0x08, 0xAC, # Address 0x08AC
        0x96        # Value for Rx Boosted gain
    ])
    spi.write(data)
    cs.value(1)
    time.sleep(0.001)

def get_packet_status_fsk():
    """
    Get FSK packet status (RSSI values)
    Returns: RxStatus, RssiSync, RssiAvg
    
    From datasheet Table 13-80:
    - RxStatus: bit flags (preamble err, sync err, crc err, etc.)
    - RssiSync: RSSI latched at sync word detection
    - RssiAvg: Average RSSI over payload
    
    Actual RSSI in dBm = -RssiValue / 2
    """
    wait_if_busy()
    cs.value(0)
    
    tx = bytearray([0x14, 0x00, 0x00, 0x00, 0x00])
    rx = bytearray(5)
    spi.write_readinto(tx, rx)
    cs.value(1)
    
    rx_status = rx[2]  # Status bits
    rssi_sync = rx[3]  # RSSI at sync word
    rssi_avg = rx[4]   # Average RSSI over packet
    
    # Convert to dBm (datasheet: actual power = -RssiValue/2)
    rssi_sync_dbm = -rssi_sync / 2.0
    rssi_avg_dbm = -rssi_avg / 2.0
    
    return rx_status, rssi_sync_dbm, rssi_avg_dbm


# ===== RECEIVER INITIALIZATION =====

print("Initializing LLCC68 FSK Receiver (RadioLib config)...")

# Reset
hardware_reset()

# Set standby RC
write_opcode([0x80, 0x00])
time.sleep(0.01)

# Set packet type: GFSK
write_opcode([0x8A, 0x00])
time.sleep(0.01)

# Set RF frequency: 433 MHz
frq = freq_to_reg(433000000)
write_32bit_opcode(0x86, frq)
time.sleep(0.01)

# Calibrate for 433MHz
calibrate_image_433()
time.sleep(0.01)

# Set buffer base addresses
write_bufferbaseaddr()
time.sleep(0.01)

# Set modulation parameters (RadioLib defaults)
write_modparameters()
time.sleep(0.01)

# Set packet parameters (RadioLib defaults)
write_packetparameters()
time.sleep(0.01)

# Set CRC configuration (CCITT)
write_crc_config()
time.sleep(0.01)

# Configure IRQ
write_setdioirq()
time.sleep(0.01)

# Set sync word (RadioLib default)
write_syncword()
time.sleep(0.01)

# Clear any pending IRQs
write_clearirq()
time.sleep(0.01)

# Enter RX mode
print("Entering RX mode...")
RXEN.value(1)
time.sleep(0.01)
write_setrx()

# ===== WAIT FOR PACKET =====

print("Waiting for packet...")
print("Config: 48kbps, 50kHz dev, 156.2kHz BW, Gaussian 0.5")
print("Sync: 0x12AD, CRC: CCITT 2-byte inverted")
timeout_count = 0
packet_count = 0

while True:
    if DIO1.value() == 1:
        RXEN.value(0)
        
        irq_status, status = get_irq_status()
        
        if irq_status & (1 << 1):  # RxDone
            packet_count += 1
            if irq_status & (1 << 6):  # CRC error
                print(f"[{packet_count}] Packet received, CRC FAILED ✗")
            else:
                print(f"[{packet_count}] Packet received, CRC OK ✓")
                length, start_addr = get_rx_buffer_status()
                packet = read_rx_buffer(length, start_addr)
                
                # Get RSSI values
                rx_status, rssi_sync, rssi_avg = get_packet_status_fsk()
                
                print(f"  Length: {length}")
                #print(f"  RSSI (sync): {rssi_sync:.1f} dBm")
                print(f"  RSSI (avg):  {rssi_avg:.1f} dBm")
                print(f"Payload: {list(packet)}")
                try:
                    print(f"  ASCII: {bytes(packet).decode('utf-8')}")
                except:
                    print(f"  Raw: {list(packet)}")
                print()
        
        if irq_status & (1 << 9):  # Timeout
            timeout_count += 1
            print(f"[Timeout #{timeout_count}] No packet received in 10s")
            
        # Clear IRQs and restart RX
        write_clearirq()
        time.sleep(0.01)
        
        RXEN.value(1)
        time.sleep(0.01)
        write_setrx()
        
    time.sleep(0.001)  # Small delay to prevent busy-waiting

