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
    
def write_paconfig():
    write_opcode([0x95,0x02,0x02,0x00,0x01])  

def write_txparam():
    write_opcode([0x8E,0x02,0x04])  

def write_bufferbaseaddr():
    write_opcode([0x8F,0x00,0x00])

def write_writebuffer(payload):
    """Write variable-length payload"""
    data = bytearray([0x0E, 0x00]) + payload
    write_opcode(data)

def write_modparameters():
    """RadioLib FSK defaults"""
    br = 21333      # 48 kbps
    fdev = 52428    # 50 kHz
    data = bytearray([
        0x8B,
        (br >> 16) & 0xFF,
        (br >> 8) & 0xFF,
        br & 0xFF,
        0x09,                # Gaussian BT 0.5
        0x1A,                # RX_BW: 156.2 kHz
        (fdev >> 16) & 0xFF,
        (fdev >> 8) & 0xFF,
        fdev & 0xFF
    ])
    write_opcode(data)

def write_packetparameters(payload_len):
    """RadioLib FSK packet params"""
    data = bytearray([
        0x8C,
        0x00, 0x10,  # 16 bytes preamble
        0x05,        # 16-bit preamble detector
        0x10,        # 16-bit sync word (2 bytes)
        0x00,        # No address filtering
        0x01,        # Variable length
        payload_len,  # +1 for length byte
        0x06,        # CRC 2-byte inverted
        0x00         # No whitening
    ])
    write_opcode(data)

def write_setdioirq():
    data = bytearray([
        0x08,
        0x00, 0x01,  # IrqMask: TxDone
        0x00, 0x01,  # DIO1Mask: TxDone
        0x00, 0x00,
        0x00, 0x00
    ])
    write_opcode(data)

def write_syncword():
    """RadioLib default: 0x12AD"""
    data = bytearray([
        0x0D,       # WriteRegister
        0x06, 0xC0, # Address
        0x12, 0xAD  # Sync word
    ])
    write_opcode(data)
    
def write_crc_config():
    """CCITT CRC configuration"""
    data = bytearray([
        0x0D,
        0x06, 0xBC,
        0x1D, 0x0F  # CCITT initial value
    ])
    write_opcode(data)
    time.sleep_us(100)
    data = bytearray([
        0x0D,
        0x06, 0xBE,
        0x10, 0x21  # CCITT polynomial
    ])
    write_opcode(data)

def write_settx():
    data = bytearray([
        0x83,
        0x00, 0x00, 0x00  # No timeout
    ])
    write_opcode(data)

def write_clearirq():
    data = bytearray([
        0x02,
        0xFF, 0xFF
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
        
# ===== TRANSMITTER =====

print("Initializing LLCC68 FSK Transmitter (RadioLib config)...")

# Reset
hardware_reset()

# Set standby
write_opcode([0x80, 0x00])
time.sleep(0.01)

# Set packet type GFSK
write_opcode([0x8A, 0x00])
time.sleep(0.01)

# Set RF frequency 433 MHz
frq = freq_to_reg(433000000)
write_32bit_opcode(0x86, frq)
time.sleep(0.01)

# Calibrate for 433MHz
calibrate_image_433()
time.sleep(0.01)

# Set PA config
write_paconfig()
time.sleep(0.01)

# Set TX param
write_txparam()
time.sleep(0.01)

# Set Buffer Base Addr
write_bufferbaseaddr()
time.sleep(0.01)

# Prepare payload
payload = bytearray(b"Hello FSK!")
print(f"Payload: {payload}")

# Write buffer with variable-length format
write_writebuffer(payload)
time.sleep(0.01)

# Set modulation parameters
write_modparameters()
time.sleep(0.01)

# Set packet parameters
write_packetparameters(len(payload))
time.sleep(0.01)

# Set CRC config
write_crc_config()
time.sleep(0.01)

# Set DIO IRQ
write_setdioirq()
time.sleep(0.01)

# Set sync word
write_syncword()
time.sleep(0.01)

# Clear IRQs
write_clearirq()
time.sleep(0.01)

# Transmit
print("Transmitting...")
TXEN.value(1)
time.sleep(0.01)
write_settx()

# Wait for TX Done
timeout = 3000
start_time = time.ticks_ms()
while DIO1.value() == 0:
    if time.ticks_diff(time.ticks_ms(), start_time) > timeout:
        print("TX Timeout!")
        break
    time.sleep(0.001)

TXEN.value(0)
write_clearirq()
print("✓ TxDone!")
print("\nConfig used:")
print("- Bit rate: 48 kbps")
print("- Freq dev: 50 kHz")
print("- RX BW: 156.2 kHz")
print("- Pulse shape: Gaussian 0.5")
print("- Sync word: 0x12AD")
print("- CRC: CCITT 2-byte inverted")

