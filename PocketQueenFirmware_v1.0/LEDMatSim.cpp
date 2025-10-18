// LED Matrix Simulator
// Simulates an LED dot matrix on 
// GFX compatible OLED displays
// Dot bitmaps are configurable
// and some functions are API compatible
// with the MAX72xx LED matrix library

#ifndef LEDMATSIM_H
#define LEDMATSIM_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define BOUNDS_CHECKING
#define BEGIN_ARGS SSD1306_SWITCHCAPVCC, 0x3C

typedef Adafruit_SSD1306 display_t;
typedef uint8_t bits_t;
constexpr bits_t LOG2(bits_t i) { return i <= 1 ? 0 : 1 + LOG2(i >> 1); }
#define BITS_T_BITS sizeof(bits_t) << 3
#define BITS_T_MULT(i) i << LOG2(BITS_T_BITS)
#define BITS_T_DIV(i) i >> LOG2(BITS_T_BITS)

class LEDMatSim
{
    private:
    display_t* display;
    bits_t mat_width;
    bits_t mat_height;
    bits_t* matrix;

    #define DOT_ON true
    #define DOT_OFF false
    #define DOT_WIDTH 4
    #define DOT_HEIGHT 4
    bits_t dot_width = DOT_WIDTH;
    bits_t dot_height = DOT_HEIGHT;
    const bits_t* dot = DEFAULT_DOT;

    #define DEFAULT_FONT_WIDTH 6
    #define DEFAULT_FONT_HEIGHT 10
    #define DEFAULT_FONT_LENGTH 10
    #define SMALL_FONT_WIDTH 4
    #define SMALL_FONT_HEIGHT 6
    #define SMALL_FONT_LENGTH 10
    bits_t font_width = DEFAULT_FONT_WIDTH;
    bits_t font_height = DEFAULT_FONT_HEIGHT;
    bits_t font_length = DEFAULT_FONT_LENGTH;
    const bits_t* font = DEFAULT_FONT;

    bits_t byteIndex(bits_t x, bits_t y)
    {
        return BITS_T_DIV((y * mat_width) + x);
    }
    
    public:
    LEDMatSim(display_t& display, bits_t mat_width, bits_t mat_height)
    {
        this->display = &display;
        this->mat_width = mat_width;
        this->mat_height = mat_height;
        matrix = new bits_t[BITS_T_DIV(mat_width * mat_height)];
        for(bits_t i = 0; i < BITS_T_DIV(mat_width * mat_height); i++)
            matrix[i] = 0;
    }

    ~LEDMatSim()
    {
        delete[] matrix;
    }

    bool begin()
    {
        if(display->begin(BEGIN_ARGS))
        {
            clear();
            return true;
          }
        return false;
    }

    void update(bool display = true)
    {
        this->display->clearDisplay();
        for(bits_t y = 0; y < mat_height; y++)
            for(bits_t x = 0; x < mat_width; x++)
                this->display->drawBitmap(x * dot_width, y * dot_height, 
                    dot, dot_width, dot_height,
                    matrix[byteIndex(x, y)] >> (x & 7U) & 1U ? WHITE : BLACK);
        if(display)
            this->display->display();
    }

    void clear()
    {
        display->clearDisplay();
        display->display();
        memset(matrix, 0, mat_width * mat_height);
    }

    void configDotBitmap(const bits_t* bitmap, bits_t width, bits_t height)
    {
        dot = bitmap;
        dot_width = width;
        dot_height = height;
    }

    void configFont(const bits_t* bitmap, bits_t width, bits_t height, bits_t length)
    {
        font = bitmap;
        font_width = width;
        font_height = height;
        font_length = length;
    }

    void setPixel(bits_t x, bits_t y, bool on = true)
    {
        #ifdef BOUNDS_CHECKING
        if(x < mat_width && y < mat_height)
        #endif
        {
            bits_t index = byteIndex(x, y);
            bits_t mod = x & 7U;
            matrix[index] = (matrix[index] & ~(1U << mod) | (on << mod));
        }
    }

    bool getPixel(bits_t x, bits_t y)
    {
        #ifdef BOUNDS_CHECKING
        if(x < mat_width && y < mat_height)
        #endif
        {
            bits_t index = byteIndex(x, y);
            bits_t mod = x & 7U;
            return (matrix[index] >> mod) & 1U;
        }
        return false;
    }

    void drawRect(bits_t x, bits_t y, bits_t w, bits_t h, bool on = true)
    {
        for(bits_t dy = 0; dy < h; dy++)
            for(bits_t dx = 0; dx < w; dx++)
                setPixel(x + dx, y + dy, on);
    }

    void drawChar(char character, bits_t x, bits_t y)
    {
        bits_t font_index = character;
        bits_t font_width_div = (font_width >> LOG2(BITS_T_BITS)) || 1;
        #ifdef BOUNDS_CHECKING
        if(font_index < font_length)
        #endif
        for(bits_t dy = 0; dy < font_height; dy ++)
            for(bits_t dx = 0; dx < font_width; dx ++)
                setPixel(x + dx, y + dy, (font[(font_index * font_width_div * font_height) + (dy * font_width_div) + (dx >> LOG2(BITS_T_BITS))] >> (dx & 7U)) & 1U);
    }

    const PROGMEM bits_t DEFAULT_DOT[DOT_HEIGHT] =
    {
        0b01000000,
        0b11100000,
        0b01000000,
        0b00000000,
    };

    const PROGMEM bits_t DEFAULT_FONT[DEFAULT_FONT_LENGTH * DEFAULT_FONT_HEIGHT] =
    {
        // 0
        0b00011111,
        0b00010001,
        0b00010001,
        0b00010001,
        0b00010001,
        0b00010001,
        0b00010001,
        0b00010001,
        0b00011111,
        0b00000000,
        
        // 1
        0b00000100,
        0b00000100,
        0b00000100,
        0b00000100,
        0b00000100,
        0b00000100,
        0b00000100,
        0b00000100,
        0b00000100,
        0b00000000,

        // 2
        0b00011111,
        0b00010000,
        0b00010000,
        0b00010000,
        0b00011111,
        0b00000001,
        0b00000001,
        0b00000001,
        0b00011111,
        0b00000000,

        // 3
        0b00011111,
        0b00010000,
        0b00010000,
        0b00010000,
        0b00011111,
        0b00010000,
        0b00010000,
        0b00010000,
        0b00011111,
        0b00000000,

        // 4
        0b00010001,
        0b00010001,
        0b00010001,
        0b00010001,
        0b00011111,
        0b00010000,
        0b00010000,
        0b00010000,
        0b00010000,
        0b00000000,

        // 5
        0b00011111,
        0b00000001,
        0b00000001,
        0b00000001,
        0b00011111,
        0b00010000,
        0b00010000,
        0b00010000,
        0b00011111,
        0b00000000,

        // 6
        0b00011111,
        0b00000001,
        0b00000001,
        0b00000001,
        0b00011111,
        0b00010001,
        0b00010001,
        0b00010001,
        0b00011111,
        0b00000000,

        // 7
        0b00011111,
        0b00010000,
        0b00010000,
        0b00010000,
        0b00010000,
        0b00010000,
        0b00010000,
        0b00010000,
        0b00010000,
        0b00000000,

        // 8
        0b00011111,
        0b00010001,
        0b00010001,
        0b00010001,
        0b00011111,
        0b00010001,
        0b00010001,
        0b00010001,
        0b00011111,
        0b00000000,

        // 9
        0b00011111,
        0b00010001,
        0b00010001,
        0b00010001,
        0b00011111,
        0b00010000,
        0b00010000,
        0b00010000,
        0b00011111,
        0b00000000,
    };

    const PROGMEM bits_t SMALL_FONT[SMALL_FONT_LENGTH * SMALL_FONT_HEIGHT] =
    {
        // 0
        0b111,
        0b101,
        0b101,
        0b101,
        0b111,
        0b000,
        
        // 1
        0b010,
        0b010,
        0b010,
        0b010,
        0b010,
        0b000,

        // 2
        0b111,
        0b100,
        0b111,
        0b001,
        0b111,
        0b000,

        // 3
        0b111,
        0b100,
        0b111,
        0b100,
        0b111,
        0b000,

        // 4
        0b101,
        0b101,
        0b111,
        0b100,
        0b100,
        0b000,

        // 5
        0b111,
        0b001,
        0b111,
        0b100,
        0b111,
        0b000,

        // 6
        0b111,
        0b001,
        0b111,
        0b101,
        0b111,
        0b000,

        // 7
        0b111,
        0b100,
        0b100,
        0b100,
        0b100,
        0b000,

        // 8
        0b111,
        0b101,
        0b111,
        0b101,
        0b111,
        0b000,

        // 9
        0b111,
        0b101,
        0b111,
        0b100,
        0b111,
        0b000,

        
    };
};

#endif // LEDMATSIM_H