from machine import SPI, Pin
import time

# Initialize SPI
spi = SPI(1, baudrate=1000000, polarity=0, phase=0, bits=8, firstbit=SPI.MSB, sck=Pin(4), mosi=Pin(6), miso=Pin(5))
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
    
# 0x07 SF7
# 0x04 LoRa_BW_125 (125kHz real)
# 0x01 LoRa_CR_4_5
# 0x00 LowDataRateOptimize OFF
def write_modparameters():
    write_opcode([0x8B,0x07,0x04,0x01,0x00])
    
# 0x0001 to 0xFFFF Preamble length (15:0)
# 0x01 Fixed length packet (implicit header)
# 0x00 to 0xFF Size of the payload (in bytes)
# 0x01 CRC ON
# 0x00 Standard IQ setup
def write_packetparameters():
    write_opcode([0x8C,0x00,0x0C,0x01,0x04,0x01,0x00])
    
# IrqMask[15:0], Dio1Mask[15:0], Dio2Mask[15:0], Dio3Mask[15:0]
# bit position : 0 TxDone, 1 RxDone, 6 CrcErr, 9 Timeout
def write_setdioirq():
    # Enable: RxDone (bit 1) + CrcErr (bit 6) + Timeout (bit 9)
    # 0x0242 = 0000 0010 0100 0010 in binary
    write_opcode([0x08,0x02,0x42,0x02,0x42,0x00,0x00,0x00,0x00])
    
def write_MSBsyncword():
    write_opcode([0x0D,0x07,0x40,0x14])

def write_LSBsyncword():
    write_opcode([0x0D,0x07,0x41,0x24])

# 0x000000 Timeout disable
def write_setrx():
    write_opcode([0x82,0x00,0x00,0x00])

# ClearIrqParam(15:0)  
def write_clearirqRxdone():
    write_opcode([0x02,0x02,0x02])

def freq_to_reg(freq_hz):
    return int((freq_hz / 32000000.0) * (1 << 25))

def calibrate_image_433():
    """Calibrate for 430-440MHz band"""
    write_opcode([0x98,0x6B,0x6F])
    
def calibrate_image_470():
    """Calibrate for 470-510MHz band"""
    write_opcode([0x98,0x75,0x81])
    
def calibrate_image_780():
    """Calibrate for 779-787MHz band"""
    write_opcode([0x98,0xC1,0xC5])
    
def calibrate_image_868():
    """Calibrate for 863-870MHz band"""
    write_opcode([0x98,0xD7,0xDB])
    
def calibrate_image_915():
    """Calibrate for 902-928MHz band"""
    write_opcode([0x98,0xE1,0xE9])

def get_irq_status():
    wait_if_busy()
    cs.value(0)

    tx = bytearray([0x12, 0x00, 0x00, 0x00])
    rx = bytearray(4)

    spi.write_readinto(tx, rx)

    cs.value(1)

    rfu     = rx[0]
    status  = rx[1]
    irq     = (rx[2] << 8) | rx[3]

    return irq, status

def get_rx_buffer_status():
    wait_if_busy()
    cs.value(0)

    # opcode + 3 NOP bytes
    tx = bytearray([0x13, 0x00, 0x00, 0x00])
    rx = bytearray(4)

    spi.write_readinto(tx, rx)
    cs.value(1)

    payload_len = rx[2]           # PayloadLengthRx
    start_addr  = rx[3]           # RxStartBufferPointer

    return payload_len, start_addr

def read_rx_buffer(length, start_address):
    wait_if_busy()
    cs.value(0)

    tx = bytearray([0x1E, start_address] + [0x00] * (1 + length))
    rx = bytearray(len(tx))

    spi.write_readinto(tx, rx)
    cs.value(1)

    return rx[3:3 + length]

def enable_rx_boosted_gain():
    # use this if you want improved sensitivity (higher power consumption) before setRx
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

# ------------------
# Start Main Routine
# ------------------

#reset
hardware_reset()

#set standby
write_opcode([0x80, 0x00])
time.sleep(0.01)

#set packet type Lora
write_opcode([0x8A, 0x01])
time.sleep(0.01)

#set RF frequency
frq = freq_to_reg(433000000)
write_32bit_opcode(0x86,frq)
time.sleep(0.01)

# Calibrate for 868MHz
calibrate_image_433()
time.sleep(0.01)

# set Buffer Base Addr (tx 0 , rx 0)
write_bufferbaseaddr()
time.sleep(0.01)

# write mod parameters sf7, 125khz, 4/5, low data rate opt off
write_modparameters()
time.sleep(0.01)

# write packet parameters preample #12, fixed length, lenght 4, crc on, standard iq
write_packetparameters()
time.sleep(0.01)

# write dioirq rxdone and timeout on dio1
write_setdioirq()
time.sleep(0.01)

# write sync word 0x1424
write_MSBsyncword()
write_LSBsyncword()
time.sleep(0.01)

def get_packet_status():
    """Get LoRa packet RSSI and SNR after RxDone (opcode 0x14)."""
    wait_if_busy()
    cs.value(0)

    tx = bytearray([0x14, 0x00, 0x00, 0x00, 0x00])
    rx = bytearray(5)
    spi.write_readinto(tx, rx)
    cs.value(1)

    rssi_pkt = rx[2]       # average RSSI over the packet
    snr_pkt  = rx[3]       # SNR (signed 2’s complement)
    # Optional: signal RSSI after despreading = rx[4]

    rssi_dbm = -rssi_pkt / 2

    if snr_pkt & 0x80:      # convert signed 8-bit
        snr_pkt = -((~snr_pkt + 1) & 0xFF)
    snr_db = snr_pkt / 4

    return rssi_dbm, snr_db


while True:
    RXEN.value(1)  # Enable RX pin
    time.sleep(0.001)
    # Set RX in single mode
    write_setrx()

    # Wait for RxDone
    while DIO1.value() == 0:
        time.sleep(0.001)

    RXEN.value(0)  # Disable RX while reading

    irq_status, status = get_irq_status()

    if irq_status & (1 << 1):  # RxDone
        if irq_status & (1 << 6):  # CRC error
            print("Packet received, but CRC failed!")
        else:
            # Read payload
            length, start_addr = get_rx_buffer_status()
            packet = read_rx_buffer(length, start_addr)

            # Read packet RSSI & SNR
            rssi, snr = get_packet_status()

            print(f"Packet received: {list(packet)}")
            print(f"RSSI: {rssi:.1f} dBm, SNR: {snr:.1f} dB")

    write_clearirqRxdone()


