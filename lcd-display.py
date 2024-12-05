import os
import time
import cv2

# GPIO Paths
GPIO_BASE_PATH = "/sys/class/gpio"
GPIO_EXPORT_PATH = os.path.join(GPIO_BASE_PATH, "export")
GPIO_UNEXPORT_PATH = os.path.join(GPIO_BASE_PATH, "unexport")

# LCD Pin Configurations
LCD_RS = 117
LCD_E = 121
LCD_D4 = 114
LCD_D5 = 113
LCD_D6 = 112
LCD_D7 = 61

LCD_WIDTH = 16
LCD_CMD = 0
LCD_CHR = 1

LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0

E_PULSE = 0.0005
E_DELAY = 0.0005

# GPIO Helper Functions
def gpio_export(pin):
    if not os.path.exists(os.path.join(GPIO_BASE_PATH, f"gpio{pin}")):
        with open(GPIO_EXPORT_PATH, 'w') as f:
            f.write(str(pin))


def gpio_unexport(pin):
    if os.path.exists(os.path.join(GPIO_BASE_PATH, f"gpio{pin}")):
        with open(GPIO_UNEXPORT_PATH, 'w') as f:
            f.write(str(pin))


def gpio_set_direction(pin, direction):
    direction_path = os.path.join(GPIO_BASE_PATH, f"gpio{pin}/direction")
    with open(direction_path, 'w') as f:
        f.write(direction)


def gpio_write(pin, value):
    value_path = os.path.join(GPIO_BASE_PATH, f"gpio{pin}/value")
    with open(value_path, 'w') as f:
        f.write(str(value))


# LCD Functions
def lcd_init():
    """Initialize LCD Display."""
    gpio_export(LCD_RS)
    gpio_export(LCD_E)
    gpio_export(LCD_D4)
    gpio_export(LCD_D5)
    gpio_export(LCD_D6)
    gpio_export(LCD_D7)

    gpio_set_direction(LCD_RS, "out")
    gpio_set_direction(LCD_E, "out")
    gpio_set_direction(LCD_D4, "out")
    gpio_set_direction(LCD_D5, "out")
    gpio_set_direction(LCD_D6, "out")
    gpio_set_direction(LCD_D7, "out")

    lcd_byte(0x33, LCD_CMD)  # Initialize
    lcd_byte(0x32, LCD_CMD)  # Set to 4-bit mode
    lcd_byte(0x28, LCD_CMD)  # 2 lines, 5x7 matrix
    lcd_byte(0x0C, LCD_CMD)  # Display ON, Cursor OFF
    lcd_byte(0x06, LCD_CMD)  # Entry mode
    lcd_byte(0x01, LCD_CMD)  # Clear display
    time.sleep(E_DELAY)


def lcd_byte(bits, mode):
    """Send a byte to the LCD."""
    gpio_write(LCD_RS, mode)

    # Send high nibble
    gpio_write(LCD_D4, (bits & 0x10) >> 4)
    gpio_write(LCD_D5, (bits & 0x20) >> 5)
    gpio_write(LCD_D6, (bits & 0x40) >> 6)
    gpio_write(LCD_D7, (bits & 0x80) >> 7)
    lcd_toggle_enable()

    # Send low nibble
    gpio_write(LCD_D4, bits & 0x01)
    gpio_write(LCD_D5, (bits & 0x02) >> 1)
    gpio_write(LCD_D6, (bits & 0x04) >> 2)
    gpio_write(LCD_D7, (bits & 0x08) >> 3)
    lcd_toggle_enable()


def lcd_toggle_enable():
    """Toggle the Enable pin."""
    time.sleep(E_DELAY)
    gpio_write(LCD_E, 1)
    time.sleep(E_PULSE)
    gpio_write(LCD_E, 0)
    time.sleep(E_DELAY)


def lcd_string(message, line):
    """Display a string on the LCD."""
    message = message.ljust(LCD_WIDTH, " ")
    lcd_byte(line, LCD_CMD)
    for char in message:
        lcd_byte(ord(char), LCD_CHR)


def lcd_clear():
    """Clear the LCD display."""
    lcd_byte(0x01, LCD_CMD)
    time.sleep(E_DELAY)


# Feature Detection
def detect_features(image):
    # Load Haar cascades
    cascade_face_detector = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    cascade_eye_detector = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')
    cascade_smile_detector = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_smile.xml')
    cascade_cat_face_detector = cv2.CascadeClassifier('haarcascades/haarcascade_frontalcatface.xml')

    # Resize image
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect faces
    face_detections = cascade_face_detector.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=4)
    if len(face_detections) > 0:
        return 1, "Face Detected"  # ID: 1

    # Detect cats
    cat_detections = cascade_cat_face_detector.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
    if len(cat_detections) > 0:
        return 4, "Cat Detected"  # ID: 4

    return 0, "No Detection"


# Main Program
if __name__ == "__main__":
    try:
        lcd_init()
        cap = cv2.VideoCapture(1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Feature detection
            can_id, message = detect_features(frame)

            # Display CAN data on LCD
            if can_id == 1:
                lcd_string("Face Detected", LCD_LINE_1)
                lcd_string(message, LCD_LINE_2)
            elif can_id == 4:
                lcd_string("Cat Detected", LCD_LINE_1)
                lcd_string(message, LCD_LINE_2)
            else:
                lcd_string("No Detection", LCD_LINE_1)
                lcd_clear()

            # Show detection result
            cv2.imshow('Feature Detection', frame)

            # Exit on pressing 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        gpio_unexport(LCD_RS)
        gpio_unexport(LCD_E)
        gpio_unexport(LCD_D4)
        gpio_unexport(LCD_D5)
        gpio_unexport(LCD_D6)
        gpio_unexport(LCD_D7)
