import time
import board
import neopixel_spi as neopixel

# Configuration
LED_COUNT = 60
LED_PIN = board.SPI()

# Colors
off = (0, 0, 0, 0)  # Black
colors = {
    "pink": (245, 0, 87, 0), "red": (255, 0, 0, 0), "green": (0, 255, 0, 0), "blue": (0, 0, 255, 0),
    "white": (0, 0, 0, 255), "purple": (106, 27, 154, 0), "lavender": (234, 128, 252, 0),
    "light_blue": (0, 176, 255, 0), "cyan": (0, 172, 193, 0), "mint": (100, 255, 218, 0),
    "lime": (118, 255, 3, 0), "yellow": (255, 255, 0, 0), "orange": (230, 81, 0, 0)
}

# Initialize LED strip
pixels = neopixel.NeoPixel_SPI(LED_PIN, LED_COUNT, pixel_order=neopixel.GRBW, auto_write=True)

def clear_pixels():
    pixels.fill(off)

def loop(color=colors["yellow"], width=1, delay=0.005, reverse=False):
    for _ in range(3):
        for pixel in range(LED_COUNT):
            time.sleep(delay)
            index = LED_COUNT - 1 - pixel if reverse else pixel
            off_index = index + 1 if reverse else index - 1
            pixels[off_index % LED_COUNT] = off
            for i in range(width):
                pixels[(index - i) % LED_COUNT if reverse else (index + i) % LED_COUNT] = color
    clear_pixels()

def windmill(color_1=colors["orange"], color_2=colors["purple"], delay=0.05, sections=4, reverse=False):
    for _ in range(2):
        for pixel in range(LED_COUNT):
            time.sleep(delay)
            index = LED_COUNT - 1 - pixel if reverse else pixel
            for section in range(sections):
                color = color_1 if section % 2 == 0 else color_2
                pixels[(index + section * (LED_COUNT // sections)) % LED_COUNT] = color
    clear_pixels()

def pattern(color=colors["red"], width=1, delay=0.2):
    pixel_group = [i for i in range(0, LED_COUNT, 2 * width)]
    for _ in range(4):
        time.sleep(delay)
        pixels.fill(color)
        for pixel in pixel_group:
            pixels[pixel] = off
        time.sleep(delay)
        pixels.fill(color)
        for pixel in range(LED_COUNT):
            if pixel not in pixel_group:
                pixels[pixel] = off
    clear_pixels()

def main():
    while True:
        clear_pixels()
        windmill(color_1=colors["mint"], color_2=colors["red"])
        loop()
        loop(color=colors["pink"], delay=0.02, width=5)
        pattern()
        windmill(sections=6, reverse=True)
        pattern(color=colors["lavender"], delay=0.6, width=3)
        loop(color=colors["cyan"], width=8, reverse=True)
        loop(color=colors["yellow"], width=2)
        windmill(sections=10, reverse=True, delay=0.05, color_1=colors["blue"], color_2=colors["pink"])

if __name__ == "__main__":
    main()
