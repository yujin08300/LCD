import os
import time

# GPIO path
GPIO_BASE_PATH = "/sys/class/gpio"
GPIO_EXPORT_PATH = os.path.join(GPIO_BASE_PATH, "export")
GPIO_UNEXPORT_PATH = os.path.join(GPIO_BASE_PATH, "unexport")

# GPIO pin configuration
LCD_RS = 117
LCD_E = 121
LCD_D4 = 114
LCD_D5 = 113
LCD_D6 = 112
LCD_D7 = 61

LCD_WIDTH = 16
LCD_CHR = "1"
LCD_CMD = "0"

LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0

E_PULSE = 0.0005
E_DELAY = 0.0005

# GPIO control functions
def gpio_export(pin):
    if not os.path.exists(os.path.join(GPIO_BASE_PATH, f"gpio{pin}")):
        with open(GPIO_EXPORT_PATH, 'w') as f:
            f.write(str(pin))

def gpio_unexport(pin):
    with open(GPIO_UNEXPORT_PATH, 'w') as f:
        f.write(str(pin))

def gpio_set_direction(pin, direction):
    direction_path = os.path.join(GPIO_BASE_PATH, f"gpio{pin}", "direction")
    with open(direction_path, 'w') as f:
        f.write(direction)

def gpio_write(pin, value):
    value_path = os.path.join(GPIO_BASE_PATH, f"gpio{pin}", "value")
    with open(value_path, 'w') as f:
        f.write(str(value))

# LCD initialization
def lcd_init():
    gpio_export(LCD_E)
    gpio_export(LCD_RS)
    gpio_export(LCD_D4)
    gpio_export(LCD_D5)
    gpio_export(LCD_D6)
    gpio_export(LCD_D7)

    gpio_set_direction(LCD_E, "out")
    gpio_set_direction(LCD_RS, "out")
    gpio_set_direction(LCD_D4, "out")
    gpio_set_direction(LCD_D5, "out")
    gpio_set_direction(LCD_D6, "out")
    gpio_set_direction(LCD_D7, "out")

    lcd_byte(0x33, LCD_CMD)
    lcd_byte(0x32, LCD_CMD)
    lcd_byte(0x28, LCD_CMD)
    lcd_byte(0x0C, LCD_CMD)
    lcd_byte(0x06, LCD_CMD)
    lcd_byte(0x01, LCD_CMD)
    time.sleep(E_DELAY)

def lcd_byte(bits, mode):
    gpio_write(LCD_RS, mode)
    gpio_write(LCD_D4, "0")
    gpio_write(LCD_D5, "0")
    gpio_write(LCD_D6, "0")
    gpio_write(LCD_D7, "0")

    if bits & 0x10:
        gpio_write(LCD_D4, "1")
    if bits & 0x20:
        gpio_write(LCD_D5, "1")
    if bits & 0x40:
        gpio_write(LCD_D6, "1")
    if bits & 0x80:
        gpio_write(LCD_D7, "1")

    lcd_toggle_enable()

    gpio_write(LCD_D4, "0")
    gpio_write(LCD_D5, "0")
    gpio_write(LCD_D6, "0")
    gpio_write(LCD_D7, "0")

    if bits & 0x01:
        gpio_write(LCD_D4, "1")
    if bits & 0x02:
        gpio_write(LCD_D5, "1")
    if bits & 0x04:
        gpio_write(LCD_D6, "1")
    if bits & 0x08:
        gpio_write(LCD_D7, "1")

    lcd_toggle_enable()

def lcd_toggle_enable():
    time.sleep(E_DELAY)
    gpio_write(LCD_E, "1")
    time.sleep(E_PULSE)
    gpio_write(LCD_E, "0")
    time.sleep(E_DELAY)

# Function to display message on LCD
def lcd_string(message, line):
    message = message.ljust(LCD_WIDTH, " ")
    lcd_byte(line, LCD_CMD)
    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]), LCD_CHR)

# Mock function to simulate CAN message reception
def receive_can_message():
    # Simulate CAN messages with a delay
    time.sleep(3)  # Simulate delay for message reception
    return {"canId": 1, "data": "Face"}

# Main program
if __name__ == "__main__":
    try:
        lcd_init()  # Initialize the LCD
        while True:
            # Simulate receiving a CAN message
            message = receive_can_message()
            can_id = message["canId"]
            data = message["data"]

            # Update LCD based on CAN message
            if can_id == 1:
                lcd_string("Face Detected", LCD_LINE_1)
                lcd_string(data, LCD_LINE_2)
            elif can_id == 2:
                lcd_string("Eyes Detected", LCD_LINE_1)
                lcd_string(data, LCD_LINE_2)
            elif can_id == 3:
                lcd_string("Smile Detected", LCD_LINE_1)
                lcd_string(data, LCD_LINE_2)
            elif can_id == 4:
                lcd_string("Cat Detected", LCD_LINE_1)
                lcd_string(data, LCD_LINE_2)
            else:
                lcd_string("Unknown ID", LCD_LINE_1)
                lcd_string(str(can_id), LCD_LINE_2)

    except KeyboardInterrupt:
        print("\nProgram stopped by User")
    finally:
        gpio_unexport(LCD_E)
        gpio_unexport(LCD_RS)
        gpio_unexport(LCD_D4)
        gpio_unexport(LCD_D5)
        gpio_unexport(LCD_D6)
        gpio_unexport(LCD_D7)
