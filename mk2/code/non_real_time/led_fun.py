
import time
import board
import neopixel_spi as neopixel

LED_COUNT = 60
off = (0, 0, 0, 0) #black
pink = (245, 0, 87, 0)
red = (255, 0, 0, 0)
green = (0, 255, 0, 0)
blue = (0, 0, 255, 0)
white = (0, 0, 0, 255)
purple = (106, 27, 154, 0)
lavender = (234, 128, 252, 0)
light_blue = (0, 176, 255, 0)
cyan = (0, 172, 193, 0)
mint = (100, 255, 218, 0)
lime = (118, 255, 3, 0)
yellow = (255, 255, 0, 0)
orange = (230, 81, 0, 0)

def main():

    LED_PIN = board.SPI()
    pixels = neopixel.NeoPixel_SPI(LED_PIN, LED_COUNT, pixel_order = neopixel.GRBW, auto_write = True)
    
    def clear_pixels():
        pixels.fill(off)

    def loop(color = yellow, width = 1, delay = 0.005, reverse = False):

        for repeat in range(3):
            for pixel in range(LED_COUNT):
                time.sleep(delay)
                index = LED_COUNT - 1 - pixel if reverse else pixel
                off_index = index + 1 if reverse else index - 1
                pixels[off_index % LED_COUNT] = off 
                for i in range(width):
                    if reverse:
                        pixels[(index - i) % LED_COUNT] = color 
                    else:
                        pixels[(index + i) % LED_COUNT] = color 
         
        clear_pixels()        
   
    def windmill(color_1 = orange, color_2 = purple, delay = 0.05, sections = 4, reverse = False):
            
        for repeat in range(2):
            for pixel in range(int(LED_COUNT)):
                time.sleep(delay)
                index = LED_COUNT - 1 - pixel if reverse else pixel
                for section in range(sections):
                    if section % 2 == 0:
                        pixels[(index + (section*int((LED_COUNT/sections)))) % LED_COUNT] = color_1
                    else:
                        pixels[(index + (section*int((LED_COUNT/sections)))) % LED_COUNT] = color_2
        clear_pixels()        
                
    def pattern(color = red, width = 1, delay = 0.2):

        first_group = [i for i in range(width)]
        pixel_group = [i for i in range(width)]

        for x in range(int(LED_COUNT / width)):
            if x != 0 and x % 2 == 0:
                for i in first_group:
                    pixel_group.append(i + width*x)        
        
        for repeat in range(4):
               
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
    
 
    clear_pixels()

    windmill(color_1 = mint, color_2 = red)
    loop()
    loop(color = pink, delay = 0.02, width = 5)
    pattern()
    windmill(sections = 6, reverse = True)
    pattern(color = lavender, delay = 0.6, width = 3)
    loop(color = cyan, width = 8, reverse = True)
    loop(color = yellow, width = 2)
    windmill(sections = 10, reverse = True, delay = 0.05, color_1 = blue, color_2 = pink)
 
if __name__ == "__main__":
    main()

