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

# +22dBm 0x04 0x07 0x00 0x01
# +20dBm 0x03 0x05 0x00 0x01
# +17dBm 0x02 0x03 0x00 0x01
# +14dBm 0x02 0x02 0x00	0x01
def write_paconfig():
    write_opcode([0x95,0x02,0x02,0x00,0x01])  

# -9 (0xF7) to +22 (0x16)dBm by steps of 1dB
# SET_RAMP_200U 0x04 200 ms
def write_txparam():
    write_opcode([0x8E,0x02,0x04])

def write_bufferbaseaddr():
    write_opcode([0x8F,0x00,0x00])

def write_writebuffer():
    write_opcode([0x0E,0x00,0x05,0x06,0x08,0x06])

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

# IrqMask[15:0], Dio1Mask[15:0], Dio2Mask[15:0], Dio3Mask[15:0],
# bit position : 0 TxDone, 1 RxDone, 6 CrcErr Wrong, 9 Timeout
def write_setdioirq():
    write_opcode([0x08,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00])

def write_MSBsyncword():
    write_opcode([0x0D,0x07,0x40,0x14])

def write_LSBsyncword():
    write_opcode([0x0D,0x07,0x41,0x24])

# 0x000000 Timeout disable
def write_settx():
    write_opcode([0x83,0x00,0x00,0x00])

# ClearIrqParam(15:0)
def write_clearirqTxdone():
    write_opcode([0x02,0x00,0x01])

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

# Calibrate for 433MHz
calibrate_image_433()
time.sleep(0.01)

# set PA config
write_paconfig()
time.sleep(0.01)

# set TX param (2dbm, 200us ramp)
write_txparam()
time.sleep(0.01)

# set Buffer Base Addr (tx 0 , rx 0)
write_bufferbaseaddr()
time.sleep(0.01)

# write buffer (offset 0, data 5,6,8,5)
write_writebuffer()
time.sleep(0.01)

# write mod parameters sf7, 125khz, 4/5, low data rate opt off
write_modparameters()
time.sleep(0.01)

# write packet parameters preample #12, fixed length, lenght 4, crc on, standard iq
write_packetparameters()
time.sleep(0.01)

# write dioirq txdone on dio1
write_setdioirq()
time.sleep(0.01)

# write sync word 0x1424
write_MSBsyncword()
write_LSBsyncword()
time.sleep(0.01)

counter = 0  # Initialize packet counter

while True:
    # Create payload: first byte is the counter, rest can be static
    payload = bytearray([counter & 0xFF, 0x06, 0x08, 0x06])

    # Write payload to TX buffer
    wait_if_busy()
    cs.value(0)
    cs_data = bytearray([0x0E, 0x00]) + payload  # WriteBuffer opcode + offset + data
    spi.write(cs_data)
    cs.value(1)

    # Set TX mode
    TXEN.value(1)
    time.sleep(0.001)
    write_settx()

    # Wait for TX Done
    timeout = 3000  # 3 seconds
    start_time = time.ticks_ms()
    while DIO1.value() == 0:
        if time.ticks_diff(time.ticks_ms(), start_time) > timeout:
            print("TX Timeout!")
            break
        time.sleep(0.001)

    TXEN.value(0)
    write_clearirqTxdone()
    print(f"TxDone! Payload sent: {list(payload)}")

    # Increment counter
    counter = (counter + 1)

    # Wait 5 seconds before next packet
    time.sleep(5)

