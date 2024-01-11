#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#define SIMULATION 1


// CONSTANTS
#define SCREEN_WIDTH  320        
#define SCREEN_HEIGHT 240
#define ROAD_WIDTH 160
#define LANE_NUMBER 5
#define ROAD_STARTING_X ((SCREEN_WIDTH - ROAD_WIDTH) / 2 + 3)
#define ROAD_ENDING_X ((SCREEN_WIDTH + ROAD_WIDTH) / 2 - 3)   
#define CAR_WIDTH 14
#define CAR_HEIGHT 35
#define NUM_OBSTACLES 12
#define X_ACCELERATION 0.1
#define Y_ACCELERATION 2
#define TIMER_VALUE 100 //ms



// COLOR PALETTE
#define WHITE 0xFFFF
#define YELLOW 0xFFE0
#define RED 0xF800
#define DARK_RED 0x700c0c
#define GREEN 0x07E0
#define DARK_GREEN 0x03E0
#define BLUE 0x211b
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define GREY 0xC618
#define PINK 0xFC18
#define ORANGE 0xFC00
#define BLACK 0

// REGISTERS
#define GIC_ICCPMR 0xFFFEC104
#define GIC_ICDDCR 0xFFFED000
#define GIC_ICCICR 0xFFFEC100
#define GIC_ICCIAR 0xFFFEC10C
#define SYSMGR_GENERALIO7 ((volatile unsigned int * ) 0xFFD0849C)
#define SYSMGR_GENERALIO8 ((volatile unsigned int * ) 0xFFD084A0)
#define SYSMGR_I2C0USEFPGA ((volatile unsigned int * ) 0xFFD08704)
#define I2C0_ENABLE ((volatile unsigned int * ) 0xFFC0406C)
#define I2C0_ENABLE_STATUS ((volatile unsigned int * ) 0xFFC0409C)
#define I2C0_CON ((volatile unsigned int * ) 0xFFC04000)
#define I2C0_TAR ((volatile unsigned int * ) 0xFFC04004)
#define I2C0_FS_SCL_HCNT ((volatile unsigned int * ) 0xFFC0401C)
#define I2C0_FS_SCL_LCNT ((volatile unsigned int * ) 0xFFC04020)
#define I2C0_DATA_CMD ((volatile unsigned int * ) 0xFFC04010)
#define I2C0_RXFLR ((volatile unsigned int * ) 0xFFC04078)
#define TIMER_STATUS (TIMER_BASE + 0x00)
#define TIMER_CONTROL (TIMER_BASE + 0x04)
#define TIMER_STARTLOW 0xFF202008
#define TIMER_STARTHIGH 0xFF20200C

// COMMANDS
#define ADXL345_REG_DEVID 0x00
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_POWER_CTL 0x2D
#define ADXL345_REG_BW_RATE 0x2C
#define ADXL345_REG_INT_SOURCE 0x30
#define XL345_RANGE_2G 0x00
#define XL345_FULL_RESOLUTION 0x08
#define XL345_RATE_100 0x0A
#define XL345_STANDBY 0x00
#define XL345_MEASURE 0x08
#define XL345_DATAREADY 0x80
#define XL345_RANGE_16G 0x03
#define XL345_RANGE_2G 0x00

// MEMORY ADDRESSES
#define PS2_BASE 0xFF200100
#define VGA_BASE_ADDR 0xC8000000 
#define VIDEO_TEXT_BASE 0xC9000000
#define TIMER_BASE 0xFF202000
#define HEX0_3 ((volatile unsigned int * ) 0xFF200020)
#define HEX4_5 ((volatile unsigned int * ) 0xFF200030)
#define LEDS ((volatile unsigned int * ) 0xFF200000)
#define PIXEL_CTRL_ADDR 0xFF203020

/**********************
*       STRUCTS       *
***********************/

typedef struct {
    int x; // X position
    int y; // Y position
    int width; // Width of the obstacle
    int height; // Height of the obstacle
    int speed; // Speed at which the obstacle moves
    short int color; // Color of the obstacle
} Obstacle;

/**********************
* FUNCTION PROTOTYPES *
***********************/
void plot_pixel(int x, int y, short int line_color);
void draw_white_rectangle(int x, int y);
void draw_road_lines(short int line_color, int offset);
void draw_car(int x, int y, short int line_color);
void clear_screen();
void draw_environment();
void draw_line(int x0, int y0, int x1, int y1, short int line_color);
void swap(int *first, int *second);
void erase_car(int x, int y);
void write_text(int x, int y, char * text_ptr);
void delete_text(int x, int y, char * text_ptr);
void draw_pixel_map();
void redraw_dashed_lines();
void start_screen();
void start_game();
void clear_old_lines(int x, int y_start, int y_end, int dash_length, int gap_length);
void clear_road_lines(int offset);
void draw_obstacles(int lane_num, double speed, short int color);
bool check_collision(Obstacle rect2);
void init_obstacles();
bool draw_obstacle(Obstacle obstacle);
void setup_timer(uint32_t load_value);
void game_over();
bool start_acc();

void ADXL345_XYZ_Read();
bool ADXL345_IsDataReady();
void ADXL345_Init(); 

void keyboard_ISR(void);
void config_interrupt(int N, int CPU_target);
void config_GIC(void);
void enable_A9_interrupts(void);
void disable_A9_interrupts(void);
void set_A9_IRQ_stack(void);
void config_KEYs(void);


void acc_control();




/**********************
*   GLOBAL VARIABLES  *
***********************/
int car_x = 154; // Starting position of the car
int car_y = 200; // Starting position of the car
bool keyboard_control = true; // Flag for keyboard control
bool accelerometer_control = false; // Flag for accelerometer control
bool leftArrowPressed = false; // Flag for left arrow key
bool rightArrowPressed = false; // Flag for right arrow key
bool upArrowPressed = false;    // Flag for up arrow key
bool downArrowPressed = false; // Flag for down arrow key
volatile bool timer_end = false; // Flag for timer interrupt
volatile bool is_game_started = false; // Flag for game start
double car_vel_x = 0.0;  // Velocity of the car in x direction
double car_vel_y = 0.0; // Velocity of the car in y direction
Obstacle obstacles[NUM_OBSTACLES];
int level = 0; // Level of the game
int16_t acc_value[3];
int second = 0;
int score = 0;

volatile int pixel_buffer_start;




/*************************
*       PIXEL MAPS       *
**************************/
short int car[14][35]= 
{
    {0x0000, 0xEF7D, 0xDEFB, 0xEF9E, 0xE73C, 0xDEFB, 0xD4D3, 0xD492, 0xC659, 0xBDF7, 0xBDD7, 0xD6BA, 0xEF7D, 0x0000},
    {0x0000, 0xEF7D, 0xD69A, 0xEF5D, 0xC618, 0xB596, 0xD126, 0xD926, 0xD6BA, 0xD69A, 0xB5D7, 0xCE79, 0xEF7D, 0x0000},
    {0x0000, 0xEF7D, 0xCE79, 0xD69A, 0xB5B6, 0xAD75, 0xD0C5, 0xD8A4, 0xDEDB, 0xDEDB, 0xBDD7, 0xCE79, 0xEF7D, 0x0000},
    {0x0000, 0xEF7D, 0xCE59, 0xD69A, 0xC638, 0xB5B6, 0xD8A4, 0xD883, 0xDEFB, 0xD6BA, 0xBE18, 0xCE79, 0xEF7D, 0x0000},
    {0x2965, 0xAD54, 0xBDD7, 0xC638, 0xD69A, 0xCE79, 0xF4C6, 0xF4C5, 0xDEDB, 0xDEDB, 0xC638, 0xB596, 0xA514, 0x2945},
    {0x31A6, 0x41E7, 0x4A49, 0x9492, 0xFFFF, 0xECD3, 0xE223, 0xEA23, 0xC800, 0x0000, 0x738E, 0x4208, 0x39E7, 0x31A6},
    {0x4A49, 0x528A, 0x4A28, 0x632C, 0x6B6D, 0x9AAB, 0xD802, 0xD802, 0x7166, 0x6B6D, 0x630C, 0x4A28, 0x528A, 0x4A49},
    {0x52AA, 0x5AAA, 0x5269, 0x3186, 0x630C, 0x6125, 0xD802, 0xD802, 0x6105, 0x31A6, 0x3165, 0x5269, 0x5AAA, 0x52AA},
    {0x4208, 0x4A49, 0x4228, 0x4A48, 0x6B4C, 0xA2EC, 0xD802, 0xD802, 0x90A3, 0x2104, 0x31A6, 0x4208, 0x4A49, 0x41E7},
    {0x3165, 0x39E7, 0x3186, 0x0000, 0x4228, 0xBA2A, 0xD802, 0xD802, 0xA883, 0x3186, 0x0000, 0x3185, 0x39C7, 0x3165},
    {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xA883, 0xD802, 0xD802, 0xA883, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
    {0x0000, 0x0000, 0x0000, 0x0000, 0x4208, 0xA125, 0xD802, 0xD802, 0xA125, 0x4228, 0x0000, 0x0000, 0x0000, 0x0000},
    {0x0000, 0x0000, 0x0000, 0x0000, 0x2965, 0x9883, 0xD802, 0xD802, 0xA083, 0x2965, 0x0000, 0x0000, 0x0000, 0x0000},
    {0x0000, 0x0000, 0x0000, 0x39C7, 0x4208, 0xA863, 0xD002, 0xD002, 0xB043, 0x4208, 0x4207, 0x0000, 0x0000, 0x0000},
    {0x0000, 0x38C3, 0x8863, 0x6B4D, 0x3186, 0xA842, 0x50A3, 0x50A3, 0xA842, 0x3186, 0x528A, 0x8883, 0x40A3, 0x0000},
    {0x0000, 0x8883, 0x71A7, 0x52AA, 0x3965, 0xA022, 0x2124, 0x2124, 0xA022, 0x4165, 0x52AA, 0x7A08, 0x8883, 0x0000},
    {0x0000, 0x3166, 0xA1E8, 0xC106, 0xC843, 0xA022, 0x630C, 0x630C, 0xA022, 0xC967, 0xC967, 0x9966, 0x3165, 0x0000},
    {0x0000, 0xD802, 0xD802, 0xD802, 0xD863, 0xA925, 0xDEDB, 0xDEDB, 0xA925, 0xD863, 0xD802, 0xD802, 0xD802, 0x0000},
    {0x0000, 0xD802, 0xD802, 0xD802, 0xDB6E, 0xACF3, 0x6B4D, 0x6B4D, 0xA4F3, 0xDB6E, 0xD802, 0xD802, 0xD802, 0x0000},
    {0x0000, 0xD802, 0xD802, 0xD802, 0xDCF4, 0xBD96, 0x28E3, 0x2903, 0xBDD7, 0xDD14, 0xD802, 0xD802, 0xD802, 0x0000},
    {0x0000, 0xD802, 0xD802, 0xD802, 0xDD14, 0xDDD7, 0xD965, 0xDA69, 0xDEDB, 0xDD35, 0xD802, 0xD802, 0xD802, 0x0000},
    {0x0000, 0xB842, 0xD802, 0xD802, 0xDB8F, 0xDDB6, 0xC186, 0xCB4D, 0xDEDB, 0xDB8F, 0xD802, 0xD802, 0xB843, 0x0000},
    {0x0000, 0x3186, 0x6904, 0xD802, 0xD947, 0xDB8F, 0x5AAA, 0x5ACB, 0xDCD3, 0xD947, 0xD802, 0x68E4, 0x3186, 0x0000},
    {0x0000, 0x3186, 0x3186, 0xB842, 0x7A08, 0xB431, 0x9BCF, 0x9BCF, 0xB451, 0x8A6A, 0xC022, 0x3186, 0x3186, 0x0000},
    {0x0000, 0x3186, 0x3186, 0x78E4, 0x4228, 0xB209, 0xC987, 0xC987, 0xB209, 0x5AAA, 0x78E4, 0x3186, 0x3186, 0x0000},
    {0x0000, 0x3186, 0x3186, 0x4166, 0x41E7, 0xB0E5, 0xD802, 0xD802, 0xA883, 0x4A69, 0x4165, 0x3186, 0x3186, 0x0000},
    {0x0000, 0x3186, 0x3186, 0x3186, 0x4A49, 0x9905, 0xD802, 0xD802, 0x9905, 0x4A49, 0x3186, 0x3186, 0x3186, 0x0000},
    {0x0000, 0x3186, 0x3186, 0x3186, 0x4208, 0x79A7, 0xD802, 0xD802, 0x7987, 0x4208, 0x3186, 0x3186, 0x3165, 0x0000},
    {0x0000, 0x3186, 0x3165, 0x3186, 0x2965, 0x51E7, 0xD802, 0xD802, 0x51C7, 0x2965, 0x3186, 0x3186, 0x3186, 0x2104},
    {0x31A6, 0x41E7, 0x31A6, 0x39C7, 0x5AEB, 0x8410, 0xC3D0, 0xBBAF, 0x7BAE, 0x5AEB, 0x39E7, 0x31A6, 0x39E7, 0x31A6},
    {0x4A49, 0x4A69, 0x4A28, 0x3186, 0x4A49, 0x41E7, 0xB0C4, 0xB8C4, 0x39E7, 0x4A28, 0x3186, 0x4A28, 0x4A69, 0x4A49},
    {0x5AAA, 0x5AAA, 0x5AAA, 0x630C, 0x6B2C, 0x632C, 0x92EC, 0x92CB, 0x632C, 0x6B2C, 0x630C, 0x52AA, 0x5ACB, 0x52AA},
    {0x4228, 0x4A49, 0x4228, 0xA534, 0xB596, 0xB596, 0xB5B6, 0xAD75, 0xB596, 0xB596, 0xA535, 0x4208, 0x4A49, 0x4208},
    {0x31A6, 0x41E7, 0x3186, 0xD6BA, 0xDEFB, 0xD69A, 0xCE79, 0xC618, 0xD69A, 0xDEFB, 0xE71C, 0x3186, 0x39E7, 0x31A6},
    {0x0000, 0x0000, 0x0000, 0x4228, 0x52AA, 0x8430, 0x4A28, 0x4228, 0x8C51, 0x52AA, 0x4228, 0x0000, 0x0000, 0x0000},
    {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x2965, 0x3165, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}
};

short int car2[20][30] = {
{0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF5AB,0xCA41,0xB0A0,0xBAE0,0xC3C0,0xB140,0xC140,0xE3C0,0xF717,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF},
{0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xEF9D,0xE62F,0xED24,0xD1E1,0xC880,0xDBE2,0xE502,0xC9A1,0xC8C1,0xE401,0xE568,0xD613,0xEF7C,0xFFFF,0xFFFF,0xFFFF},
{0xFFFF,0xFFFF,0xF7BD,0xBD71,0xDDAB,0xE564,0x9385,0xB942,0xD0A1,0xECA3,0xF5E4,0xD9E2,0xD0A1,0x8A85,0xCCA3,0xE566,0xC5D2,0xEF5C,0xFFFF,0xFFFF},
{0xFFFF,0xFFFF,0xF68F,0xD545,0xE564,0x9BE4,0xAC85,0xD1C3,0xD8C2,0xED04,0xF645,0xDA22,0xD8C2,0xC3E5,0x9C49,0xD545,0xDD87,0xEE0A,0xF77A,0xFFFF},
{0xFFFF,0xF7BC,0xF629,0xF5E4,0xF604,0xC4C3,0xE5E5,0xE243,0xD8E2,0xED45,0xF6A6,0xE243,0xD8E2,0xED25,0xC506,0xD564,0xEDE4,0xF5C4,0xF778,0xFFFF},
{0xFFFF,0xF6AF,0xF626,0xF645,0xF645,0xDD63,0xF645,0xE263,0xD903,0xF566,0xF6C7,0xE264,0xD903,0xF545,0xDD63,0xE5E4,0xF645,0xF625,0xF6D2,0xF7BC},
{0xFFFF,0xF62A,0xF625,0xF686,0xEE25,0xDD63,0xEE86,0xE284,0xE123,0xF586,0xFEE7,0xE284,0xE123,0xF586,0xDD63,0xEE05,0xF686,0xF665,0xED66,0xF739},
{0xFFFF,0xED88,0xF625,0xF686,0xE5E4,0xDD83,0xF6A6,0xEAA4,0xE144,0xF5A6,0xFF07,0xEAA4,0xE144,0xF586,0xDD63,0xE604,0xE626,0xF686,0xED44,0xF738},
{0xFFFF,0xED47,0xF645,0xF6C7,0xDD84,0x7B64,0x6B26,0x61C5,0x6165,0x7B68,0x7BE9,0x7A88,0x7228,0x7B68,0x7347,0xC545,0xEE66,0xF6C7,0xF6D2,0xF7BD},
{0xFFFF,0xF651,0xF6A8,0x8C27,0x62C5,0x39E6,0x2986,0x2985,0x2965,0x31A6,0x4228,0x4248,0x4248,0x4228,0x4228,0x62E6,0x7347,0xD607,0xFF98,0xFFFE},
{0xFFFF,0xC5F6,0x840A,0x4226,0x31A6,0x29A6,0x2986,0x2985,0x2965,0x2965,0x2985,0x31C6,0x31C7,0x31C6,0x31A6,0x31A6,0x31A6,0x62E6,0xAD53,0xEF7D},
{0xFFFF,0xE71B,0x5B0B,0x31C6,0x31A6,0x29A6,0x2986,0x2985,0x2965,0x2965,0x2145,0x2965,0x2965,0x2965,0x2145,0x2144,0x2124,0x2124,0xB5B6,0xF7BE},
{0xFFFF,0xF7BD,0x5B0B,0x4228,0x31E7,0x29A6,0x2986,0x2985,0x2965,0x2965,0x2145,0x2144,0x2124,0x2124,0x2104,0x1904,0x2986,0x39C4,0xCE78,0xFFFF},
{0xFFFF,0xF7BC,0x5AEB,0x39E7,0x31C6,0x4246,0xA4A7,0xBAA6,0xB1C6,0xBC66,0xBD46,0xB2A6,0xB1C6,0xB466,0x4224,0x1904,0x2124,0x62C4,0xDED8,0xFFFF},
{0xFFFF,0xF75A,0x5AEA,0x31A6,0x9C87,0xC5A7,0xEEC7,0xF347,0xF207,0xFDC7,0xFF07,0xF347,0xF207,0xFDC7,0xC586,0xB546,0x4204,0x9425,0xEF59,0xFFFF},
{0xFFFF,0xF739,0x8C4A,0x4226,0xCDC7,0xFF07,0xFF07,0xF347,0xF207,0xF5C7,0xFF07,0xF347,0xF207,0xF5C7,0xFF07,0xFF07,0x5264,0xBD46,0xF799,0xFFFF},
{0xFFFF,0xF651,0xDDE8,0x5AC6,0xCDC7,0xFF07,0xFF07,0xF327,0xF206,0xF5C7,0xFF07,0xF327,0xF206,0xF5C7,0xFF07,0xFF07,0x5264,0xC566,0xF7B9,0xFFFF},
{0xFFFF,0xED69,0xF625,0x62E6,0xCDC7,0xFF07,0xFF07,0xF306,0xF1E6,0xF5C7,0xFF07,0xF306,0xF1E6,0xF5C7,0xFF07,0xFF07,0x5264,0xBD46,0xF757,0xF7DE},
{0xFFFF,0xED47,0xEDE4,0x5AC6,0xCDC7,0xFF07,0xFF07,0xF306,0xE9C5,0xF5C7,0xFF07,0xF306,0xE9C5,0xF5C7,0xFF07,0xDE27,0x4224,0xBD05,0xF60C,0xF77A},
{0xFFFF,0xED47,0xE583,0x7B86,0xC587,0xF6C7,0xFF07,0xEAE6,0xE9A5,0xF5A7,0xFF07,0xEAE6,0xE9A5,0xF5A7,0xFF07,0x7365,0x9425,0xDDA4,0xED03,0xF718},
{0xFFFF,0xF64A,0xD523,0xCD45,0xE667,0x8C27,0x6B46,0x61E5,0x6185,0x62A6,0x6305,0x61A4,0x5944,0x6284,0x5AA4,0x9C86,0xE626,0xDD63,0xE4A1,0xF717},
{0xFFFF,0xF68B,0xD522,0xEE05,0xF6E7,0x73A9,0x4A89,0x4249,0x31A6,0x2965,0x2145,0x2124,0x2124,0x2104,0x18E3,0xC586,0xF686,0xDD43,0xE563,0xF778},
{0xF77B,0xEE4A,0xDCE2,0xEDE4,0xF6C7,0x7C0A,0x52EB,0x4AAA,0x4248,0x2986,0x2145,0x2124,0x2124,0x2104,0x18E3,0xC566,0xF666,0xDD02,0xE5A4,0xF778},
{0xED27,0xEDC5,0xDCC2,0xE5A4,0xF6A6,0x7BEA,0x52EB,0x4AAA,0x4269,0x31A6,0x2145,0x2124,0x2124,0x2104,0x18E3,0xC566,0xE5E5,0xD4C2,0xE5A4,0xF735},
{0xEC60,0xED83,0xDD02,0xD503,0xEE45,0x6B27,0x4AAA,0x4AAA,0x4269,0x3A08,0x2965,0x2124,0x2124,0x2104,0x18E3,0xC545,0xE584,0xDCC2,0xEDA4,0xF6F4},
{0xF717,0xF608,0xE543,0xDCE2,0xEDC4,0xC545,0x62E8,0x4A89,0x4269,0x4228,0x31C6,0x2145,0x2124,0x2104,0x8BC4,0xDDA4,0xDD23,0xE522,0xED83,0xF737},
{0xFFFF,0xF5C9,0xED63,0xE522,0xE543,0xE5A4,0xC525,0x52A6,0x4248,0x4228,0x39E7,0x2945,0x2124,0x8BC4,0xD564,0xE563,0xE522,0xED43,0xECE1,0xF738},
{0xFFFF,0xF738,0xED87,0xED22,0xE522,0xE563,0xEDC4,0xB203,0xA903,0xB3E5,0xBCC5,0xA1C3,0x98A2,0xDCA3,0xE583,0xE522,0xED22,0xD481,0xEE30,0xF7BD},
{0xFFFF,0xFFFF,0xF738,0xD4C6,0x7A21,0xCC42,0xED42,0xD9A1,0xD060,0xEC63,0xF5A3,0xD9A1,0xD060,0xEC42,0xDCC2,0x8241,0xC3A1,0xE610,0xF7BD,0xFFFF},
{0xFFFF,0xFFFF,0xFFFF,0xEEF7,0xC5D6,0xE6B6,0xD4C6,0xB960,0xB060,0xC361,0xCC61,0xB940,0xB0C0,0xC340,0xE610,0xC5D6,0xE695,0xF7BD,0xFFFF,0xFFFF},

};

/**********************
*   MAIN FUNCTION     *
***********************/
int main() {

    volatile int *pixel_ctrl_ptr = (int *)PIXEL_CTRL_ADDR;
	pixel_buffer_start = *pixel_ctrl_ptr; /* Read location of the pixel buffer from the pixel buffer controller */
    
    clear_screen();
    start_screen();
    setup_timer(TIMER_VALUE); //
    disable_A9_interrupts();
	set_A9_IRQ_stack(); 
	config_GIC(); 
	config_KEYs(); 
	enable_A9_interrupts();

    
    int y_offset = 0;
    int passive_obstacle = 0;
    int time_loop = 0;
    while(true){
       
        if(is_game_started) { // Animation loop

            y_offset++;
            if(!SIMULATION) 
            {
                acc_control();
                if(accelerometer_control){
                    if (ADXL345_IsDataReady()) {
                        ADXL345_XYZ_Read(acc_value);
                    }
                }
            }

            if(timer_end){
                time_loop++;
                if (time_loop == (int)1000/TIMER_VALUE){
                    second++;
                    delete_text(12, 10,"");
                    char str[10];
                    score += second * level;
                    sprintf(str, "%d", score);
                    write_text(12, 10, str);
                    time_loop = 0;
                }
                    draw_road_lines(WHITE, y_offset);
                    timer_end = false;
            }
            
            if (y_offset >= 10) {
                y_offset = 0; // Reset the offset after a complete cycle
            }

            for (int i = 0; i < level + 3; i++) {
                if (obstacles[i].y >= SCREEN_HEIGHT) {
                    passive_obstacle++;
                    continue;
                }
                    obstacles[i].y += obstacles[i].speed; // Move obstacle down
            }

            draw_car(car_x, car_y, BLUE);

            // Drawing obstacles
            for (int i = 0; i < level + 3; i++) {
                if(obstacles[i].y < SCREEN_HEIGHT)
                    draw_obstacle(obstacles[i]);

                
                // if(check_collision(obstacles[i])){
                //     //game over
                //     printf("game over\n");
                //     // game_over();
                    
                // }
            }
            if (passive_obstacle >= level + 3) {
                passive_obstacle = 0;
                // for (int i = 0; i < level + 3; i++) {
                //     if(obstacles[i].y < SCREEN_HEIGHT)
                //         obstacles[i].y = 0;
                // }
                if(level < 4) level++;
                printf("level: %d\n", level);
                for(int i = 0; i < level + 3; i++){
                    obstacles[i].y = 0;
                    obstacles[i].speed += 1;
                }
            }

            
            
        }
        
    }
    return 0;
}

/*****************************
*    FUNCTION DEFINITIONS    *
******************************/

void start_game(){

    is_game_started = true;
    printf("game is started %d\n",is_game_started);
    clear_screen();
    draw_environment();
    draw_car(car_x, car_y, BLUE);
    write_text(5,10,"SCORE:");
    write_text(12,10,"0");
    init_obstacles();
    
    // animate_dashed_lines();

}
void plot_pixel(int x, int y, short int line_color)
{
    *(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = line_color;
}   

void draw_white_rectangle(int x, int y) {
    for (int i = 0; i < 40; ++i) {
        for (int j = 0; j < 40; ++j) {
            plot_pixel(x + i, y + j, WHITE);
        }
    }
}
void draw_environment(){
    for(int i = 0; i < SCREEN_WIDTH; i++){
        for(int j = 0; j < SCREEN_HEIGHT; j++){
            if(i < ROAD_STARTING_X || i > ROAD_ENDING_X){
                plot_pixel(i, j, DARK_GREEN);
            }
            if((i < ROAD_STARTING_X && i > ROAD_STARTING_X- 6) || 
                (i < ROAD_ENDING_X + 6 && i > ROAD_ENDING_X))
				if(j % 10 > 1 ) plot_pixel(i, j, RED);
                else plot_pixel(i, j, WHITE);
        }
    }
}

void start_screen(){

    for(int i = 0; i < SCREEN_WIDTH; i++){
        for(int j = 0; j < SCREEN_HEIGHT; j++){
                plot_pixel(i, j, 0x700c0c);
            
        }
    }
    
    //write start screen text and mode choosing text
    int offset = 100, offset2 = 20;

    write_text(25, 10, "Eyup Sahin");
    write_text(40, 10, "Efdal Ayas"); 
    write_text(25,52, "Press ENTER for keyboard control");
    write_text(25, 55, "Press SPACE for accelerometer control");


    draw_line(offset, 90, offset + 12, 90, 0xFFFF);
    draw_line(offset + 6, 90, offset + 6, 120, 0xFFFF);
    offset += 16; 

    // R
    draw_line(offset, 90, offset + 15, 90, 0xFFFF);
    draw_line(offset, 90, offset, 120, 0xFFFF);
    draw_line(offset, 102, offset + 15, 102, 0xFFFF);
    draw_line(offset + 15, 90, offset + 15, 102, 0xFFFF);
    draw_line(offset, 102, offset + 15, 120, 0xFFFF);
    offset += 19; 

    // A 
    draw_line(offset, 90, offset + 12, 90, 0xFFFF);
    draw_line(offset, 105, offset + 12, 105, 0xFFFF);
    draw_line(offset, 90, offset, 120, 0xFFFF);
    draw_line(offset + 12, 90, offset + 12, 120, 0xFFFF);
    offset += 16; 

    // F
    draw_line(offset, 90, offset, 120, 0xFFFF);
    draw_line(offset, 90, offset + 12, 90, 0xFFFF);
    draw_line(offset, 105, offset + 6, 105, 0xFFFF);
    offset += 16; 

    // F 
    draw_line(offset, 90, offset, 120, 0xFFFF);
    draw_line(offset, 90, offset + 12, 90, 0xFFFF);
    draw_line(offset, 105, offset + 6, 105, 0xFFFF);
    offset += 16; 

    // I
    draw_line(offset + 6, 90, offset + 6, 120, 0xFFFF);
    offset += 16; 

    // C 
    draw_line(offset, 90, offset + 12, 90, 0xFFFF);
    draw_line(offset, 120, offset + 12, 120, 0xFFFF);
    draw_line(offset, 90, offset, 120, 0xFFFF);
    offset = 132; 

    // R 
    draw_line(offset, 150, offset + 15, 150, 0xFFFF);
    draw_line(offset, 150, offset, 180, 0xFFFF);
    draw_line(offset, 162, offset + 15, 162, 0xFFFF);
    draw_line(offset + 15, 150, offset + 15, 162, 0xFFFF);
    draw_line(offset, 162, offset + 15, 180, 0xFFFF);
    offset += 19; 

    // A 
    draw_line(offset, 150, offset + 12, 150, 0xFFFF);
    draw_line(offset, 162, offset + 12, 162, 0xFFFF);
    draw_line(offset, 150, offset, 180, 0xFFFF);
    draw_line(offset + 12, 150, offset + 12, 180, 0xFFFF);
    offset += 16; 

    // C 
    draw_line(offset, 150, offset + 12, 150, 0xFFFF);
    draw_line(offset, 180, offset + 12, 180, 0xFFFF);
    draw_line(offset, 150, offset, 180, 0xFFFF);
    offset += 16; 

    // E 
    draw_line(offset, 150, offset + 12, 150, 0xFFFF);
    draw_line(offset, 180, offset + 12, 180, 0xFFFF);
    draw_line(offset, 162, offset + 6, 162, 0xFFFF);
    draw_line(offset, 150, offset, 180, 0xFFFF);
}

void clear_road_lines(int offset){
    int lane_width = ROAD_WIDTH / LANE_NUMBER;
    
	for (int i = ROAD_STARTING_X + lane_width - 2; i <ROAD_STARTING_X + 4 * lane_width + 2; ++i){
		for (int j = offset; j < SCREEN_HEIGHT; ++j) {

            if((i < ROAD_STARTING_X + lane_width +1 && i > ROAD_STARTING_X + lane_width - 1) || 
                (i < ROAD_STARTING_X + 2 * lane_width +1 && i > ROAD_STARTING_X + 2 * lane_width - 1) ||
                (i < ROAD_STARTING_X + 3 * lane_width +1 && i > ROAD_STARTING_X + 3 * lane_width - 1) || 
                (i < ROAD_STARTING_X + 4 * lane_width +1 && i > ROAD_STARTING_X + 4 * lane_width - 1)) 
                {
                    if(offset + 6 > j % 16 && j % 16 > offset)
                        plot_pixel(i, j, BLACK);
                }
        }
	}
}
// void draw_road_lines(short int line_color, int offset){
// 	int lane_width = ROAD_WIDTH / LANE_NUMBER;
    
// 	for (int i = ROAD_STARTING_X + lane_width - 2; i <ROAD_STARTING_X + 4 * lane_width + 2; ++i){
// 		for (int j = offset; j < SCREEN_HEIGHT; ++j) {
            
//             if((i < ROAD_STARTING_X + lane_width +1 && i > ROAD_STARTING_X + lane_width - 1) || 
//                 (i < ROAD_STARTING_X + 2 * lane_width +1 && i > ROAD_STARTING_X + 2 * lane_width - 1) ||
//                 (i < ROAD_STARTING_X + 3 * lane_width +1 && i > ROAD_STARTING_X + 3 * lane_width - 1) || 
//                 (i < ROAD_STARTING_X + 4 * lane_width +1 && i > ROAD_STARTING_X + 4 * lane_width - 1)) 
//                 {
//                     if(offset + 6 > j % 16 && j % 16  > offset)
//                         plot_pixel(i, j, line_color);
//                 }
//         }
// 	}
// }
void draw_road_lines(short int line_color, int offset){
	int lane_width = ROAD_WIDTH / LANE_NUMBER;
    int len = 7, gap = 5;
    int y = offset + len;
		for (int j = offset; j < SCREEN_HEIGHT; j++) {
            for (int i = ROAD_STARTING_X + lane_width - 2; i <ROAD_STARTING_X + 4 * lane_width + 2; i++){
            
            if((i < ROAD_STARTING_X + lane_width +1 && i > ROAD_STARTING_X + lane_width - 1) || 
                (i < ROAD_STARTING_X + 2 * lane_width +1 && i > ROAD_STARTING_X + 2 * lane_width - 1) ||
                (i < ROAD_STARTING_X + 3 * lane_width +1 && i > ROAD_STARTING_X + 3 * lane_width - 1) || 
                (i < ROAD_STARTING_X + 4 * lane_width +1 && i > ROAD_STARTING_X + 4 * lane_width - 1)) 
                {   
                    if(j < y){

                        plot_pixel(i, j, line_color);
                    }
                    else if ((j > y) && (j < (y + gap)))
                    {
                        plot_pixel(i, j, BLACK);
                    }
                    else if (j > y + gap){
                        y += len + gap;
                        if(y > SCREEN_HEIGHT){
                            y = offset + len;
                        }
                    }
                }
        }
	}
}



//redraw the dashed lines in the same region with the car
// void redraw_dashed_lines(){
//     int lane_width = ROAD_WIDTH / LANE_NUMBER;

//     for(int i= car_x - 30; i< car_x + 30; i++){
//         for(int j= car_y - 30; j< car_y + 30; j++){
//             if((i < ROAD_STARTING_X + lane_width +1 && i > ROAD_STARTING_X + lane_width - 1) ||
//                 (i < ROAD_STARTING_X + 2 * lane_width +1 && i > ROAD_STARTING_X + 2 * lane_width - 1) ||
//                 (i < ROAD_STARTING_X + 3 * lane_width +1 && i > ROAD_STARTING_X + 3 * lane_width - 1) ||
//                 (i < ROAD_STARTING_X + 4 * lane_width +1 && i > ROAD_STARTING_X + 4 * lane_width - 1))
//                 {
//                     if(j % 10 > 6)
//                         plot_pixel(i, j, WHITE);
//                 }
//         }
//     }

// }

// void clear_old_lines(int x, int y_start, int y_end, int dash_length, int gap_length){
//     int y = y_start;
//     while (y < y_end) {
//         for (int i = 0; i < dash_length && y < y_end; i++, y++) {
//             plot_pixel(x, y, BLACK); 
//         }
//         y += gap_length;
//     }

// }


void draw_dashed_line(int x, int y_start, int y_end, int dash_length, int gap_length, int color) {
    int y = y_start;
    while (y < y_end) {
        for (int i = 0; i < dash_length && y < y_end; i++, y++) {
            plot_pixel(x, y, color); 
        }
        y += gap_length;
    }
}

void animate_dashed_lines() {
    int dash_length = 5; // Length of each dash
    int gap_length = 3;  // Length of the gap between dashes
    int line_color = 0xFFFF; // Example color code
    int x_positions[] = {10, 30, 50}; // X positions of the lines
    int num_lines = sizeof(x_positions) / sizeof(x_positions[0]);
    int y_offset = 0; // Vertical offset for the animation

    while (!timer_end) { // Animation loop
        clear_screen(); // Clear the screen

        // Draw dashed lines at different x positions
        for (int i = 0; i < num_lines; i++) {
            draw_dashed_line(x_positions[i], y_offset, SCREEN_HEIGHT + dash_length, dash_length, gap_length, line_color);
        }

        y_offset++;
        if (y_offset >= dash_length + gap_length) {
            y_offset = 0; // Reset the offset after a complete cycle
        }

    }

}


void draw_car(int x, int y, short int line_color){
    for (int j = 0; j < CAR_HEIGHT; ++j) {
        for (int i = 0; i < CAR_WIDTH; ++i) {
            if (x + i < ROAD_ENDING_X && x+ i > ROAD_STARTING_X && y +j < SCREEN_HEIGHT && y + j > 0)
                plot_pixel(x + i, y + j, BLUE);            
        }
    }
}

void draw_pixel_map(char** pixel_map){
    //get dimension size of 2d char array
    int row = sizeof(pixel_map) / sizeof(pixel_map[0]);
    int col = sizeof(pixel_map[0]) / sizeof(pixel_map[0][0]);
    row = 30;
    col = 20;
    printf("row: %d, col: %d\n", row, col);
    for(int i = 0; i < col; i++){
        for(int j = 0; j < row; j++){
            // plot_pixel(250 + i, 20+ j, car2[j][i]);
        }
    }
}


void init_obstacles() {
    for (int i = 0; i < NUM_OBSTACLES; i++) {
        // Initialize obstacle properties (position, size, color)
        obstacles[i].height = 20 + rand() % 10; //  height
        obstacles[i].width = 20; //  width
        obstacles[i].x = ROAD_STARTING_X + (rand() % 5) * ROAD_WIDTH/LANE_NUMBER + (ROAD_WIDTH/LANE_NUMBER - obstacles[i].width)/2;
        obstacles[i].y =  0; // Start off-screen
        obstacles[i].speed = 2; // initial speed
        obstacles[i].color = RED; //  color
    }
}

bool draw_obstacle(Obstacle obstacle) {
    for (int j = 0; j < obstacle.height; j++) {
        for (int i = 0; i < obstacle.width; i++) {
            if(j < obstacle.speed + 1 && obstacle.y - j > 0) 
                plot_pixel(obstacle.x + i, obstacle.y - j, BLACK);
            else if(obstacle.y + j >= SCREEN_HEIGHT)
                continue;
            else
                plot_pixel(obstacle.x + i, obstacle.y + j, obstacle.color);
        }
    }
    return true;
}

bool check_collision(Obstacle rect2) {
    
    if ((car_x > rect2.x + rect2.width || rect2.x > car_x + CAR_WIDTH) && 
      (car_y > rect2.y + rect2.height || rect2.y > car_y + CAR_HEIGHT)) // Check if there is no overlap on x and y-axis
        return false;
    else
        return true; // Collision detected
    
      
}

void draw_line(int x0, int y0, int x1, int y1, short int line_color) {
    bool is_steep = ( abs(y1 - y0) > abs(x1 - x0) );
	
    if (is_steep) {
        swap(&x0, &y0);
        swap(&x1, &y1);
    }
   
    if (x0 > x1) {
        swap(&x0, &x1);
        swap(&y0, &y1);
    }
    
    int delta_x = x1 - x0;
    int delta_y = abs(y1 - y0);
    int error = -(delta_x / 2);
    
    int y = y0;
    int y_step;
    if (y0 < y1) 
        y_step =1;
    else 
        y_step = -1;
    
    for(int x = x0; x <= x1; x++) {
        if (is_steep) 
            plot_pixel(y, x, line_color);
        else 
            plot_pixel(x, y, line_color);
        
        error += delta_y;
        
        if (error >= 0) {
            y +=y_step;
            error -= delta_x;
        }
    } 
}

void swap(int *first, int *second){
	int temp = *first;
    *first = *second;
    *second = temp;   
}

void erase_car(int x, int y){
    
    
    if((leftArrowPressed + rightArrowPressed + upArrowPressed + downArrowPressed) > 1){
        x += leftArrowPressed * CAR_WIDTH - rightArrowPressed * car_vel_x;
        y += upArrowPressed * CAR_HEIGHT - downArrowPressed * car_vel_y;
        for (int i = 0; i < CAR_WIDTH; ++i) {
            for (int j = 0; j < CAR_HEIGHT; ++j) {
                plot_pixel(x + i, y + j, BLACK);
            }
        }
    }
    else {

    if(car_vel_x > 0){ // right arrow pressed
        x -= car_vel_x;
        for (int i = 0; i < (int)car_vel_x + 1; ++i) {
            for (int j = 0; j < CAR_HEIGHT; ++j) {
                plot_pixel(x + i, y + j, BLACK);
            }
        }
    }
    else if (car_vel_x < 0)
    {
        x += CAR_WIDTH;
        for (int i = 0; i < abs((int)car_vel_x); ++i) {
            for (int j = 0; j < CAR_HEIGHT; ++j) {
                plot_pixel(x + i, y + j, BLACK);
            }
        }   
    }
    if (car_vel_y > 0) // down arrow pressed
    {
        y -= car_vel_y;
        for (int i = 0; i < CAR_WIDTH; ++i) {
            for (int j = 0; j < (int)car_vel_y; ++j) {
                plot_pixel(x + i, y + j, BLACK);
            }
        }
    }
    else if (car_vel_y < 0) // up arrow pressed
    {   
        y += CAR_HEIGHT;
        for (int i = 0; i < CAR_WIDTH; ++i) {
            for (int j = 0; j < abs((int)car_vel_y); ++j) {
                plot_pixel(x + i, y  + j, BLACK);
            }
        }   
    }
    }
    
}


void clear_screen()
{   
    volatile char * character_buffer = (char *)VIDEO_TEXT_BASE;
    int offset;

    for (int y = 0; y < SCREEN_HEIGHT; y++)
    {
        for (int x = 0; x < SCREEN_WIDTH; x++)
        {
            plot_pixel(x, y, BLACK);
            
        }
    }
    for (int y = 0; y < 60; y++)
    {
        for (int x = 0; x < 80; x++)
        {
            offset = (y << 7) + x;
            *(character_buffer + offset) = ' ';
        }
    }
}

void write_text(int x, int y, char * text_ptr) {
	int offset;
	volatile char * character_buffer = (char *)VIDEO_TEXT_BASE;
	
	offset = (y << 7) + x;
	
	while (*(text_ptr)) {
		*(character_buffer + offset) = *(text_ptr); // write to the character buffer
		++text_ptr;
		++offset;
	}
}

void delete_text(int x, int y, char * text_ptr) {
    int offset;
    volatile char * character_buffer = (char *)VIDEO_TEXT_BASE;
    
    offset = (y << 7) + x;
    
    while (*(text_ptr)) {
        *(character_buffer + offset) = ' '; // write to the character buffer
        ++text_ptr;
        ++offset;
    }
}

void game_over(){

    clear_screen();
    write_text(15, 10, "GAME OVER");
    // is_game_started = false;

}

void acc_control(){

    int16_t first = acc_value[0] / 2.5;
    int16_t second = acc_value[1] / 2.5;
    int16_t third = acc_value[2] / 2.5;
	
	if(first > 99) {
		first = 99;
	}
	else if(first < 0){
		if (first < -99){
			first = 99;
		}
		else{
			first *= -1;

		}
	}
	
	if(second > 99) {
		second = 99;
	}
	else if(second < 0){
		if (second < -99){
			second = 99;
		}
		else{
			second *= -1;
		}
	}
	
	
	if(third > 99) {
		third = 99;
	}
	else if(third < 0){
		if (third < -99){
			third = 99;
		}
		else
		{
			third *= -1;
		}
	}

    first *= 0.002;
    second *= 0.002;
    // third *= 0.002;
    car_vel_x += first;
    car_vel_y += second;

}

void setup_timer(uint32_t load_value) {
    load_value = load_value * 100000;
    uint16_t counter_low = (load_value) & 0xFFFF;
    uint16_t counter_high = (load_value >> 16) & 0xFFFF;
    // uint16_t counter_low = 0xD784;
    // uint16_t counter_high = 0x017;
    printf("counter_low: %x, counter_high: %x\n", counter_low, counter_high);
    
    *(volatile uint32_t *)TIMER_STARTLOW = counter_low;
    *(volatile uint32_t *)TIMER_STARTHIGH = counter_high;

    // *(volatile uint32_t *)TIMER_CONTROL = (1 << 0) | (1 << 1); // 0th bit is enable, 1st bit is auto-reload
    *(volatile uint32_t *)TIMER_CONTROL = 0x07;
}


/* setup the PS/2 interrupts in the FPGA */
void config_KEYs() {
	volatile int * PS2_ptr = (int *) PS2_BASE; // PS/2 base address
	*(PS2_ptr + 1) = 0x00000001; // set RE to 1 to enable interrupts
}

void keyboard_ISR(void) {

	volatile int * PS2_base = (int *)PS2_BASE; // Points to PS2 Base
	unsigned char byte0 = 0, byte1 =0;
    
	int PS2_data = *(PS2_base);
	int RVALID = PS2_data & 0x8000;
	
	//Read Interrupt Register
	int readInterruptReg;
	readInterruptReg = *(PS2_base + 1 ); 
	  
	//Clear Interrupt 
	*(PS2_base+1) = readInterruptReg; 

	// when RVALID is 1, there is data 
	if (RVALID != 0){
               
		byte0 = (PS2_data & 0xFF); //data in LSB	
        if (byte0 == 0xF0) { // Key release detected
            byte1 = *(PS2_base) & 0xFF; // Read next byte for the released key
            if (byte1 == 0x6B) leftArrowPressed = false;
            if (byte1 == 0x74) rightArrowPressed = false;
            if (byte1 == 0x75) upArrowPressed = false;
            if (byte1 == 0x72) downArrowPressed = false;
        } else { // Key press detected
            if (byte0 == 0x6B) leftArrowPressed = true;
            if (byte0 == 0x74) rightArrowPressed = true;
            if (byte0 == 0x75) upArrowPressed = true;
            if (byte0 == 0x72) downArrowPressed = true;
        }
        if (byte0 == 0x29 && !is_game_started) { // Space key
            if(!SIMULATION) {

                if(!start_acc()){
                    printf("Error in starting accelerometer\n");
                    return;
                }
            }
            keyboard_control = false;
            accelerometer_control = true;
            start_game();
        }
        

        if(byte0 == 0x5A && !is_game_started){ //enter key
            
            keyboard_control = true;
            accelerometer_control = false;
            start_game();

        }
        if(keyboard_control &&is_game_started){


            if(leftArrowPressed){  //left arrow
                if(car_x > ROAD_STARTING_X + CAR_WIDTH){
                car_vel_x -= X_ACCELERATION;    
                }
            }
            if(rightArrowPressed){  //right arrow
                if(car_x < ROAD_ENDING_X - CAR_WIDTH ){
                    car_vel_x += X_ACCELERATION;
                }
            }
            if(upArrowPressed){ //up arrow
                if(car_y > 0){
                    car_vel_y = - Y_ACCELERATION;
                }
                }
            if (downArrowPressed) // down arrow
            {
                if(car_y < SCREEN_HEIGHT - CAR_HEIGHT){
                    car_vel_y = Y_ACCELERATION;
                }
            }
            if(car_vel_x > 4)
                car_vel_x = 4;
            if(car_vel_y > 4)
                car_vel_y = 4;

            car_x += (int)car_vel_x;
            car_y += (int)car_vel_y;

            if(car_x < ROAD_STARTING_X){
                // car_x = ROAD_STARTING_X + 6;
                car_x -= (int)car_vel_x;
                car_vel_x = 0;
            }
            if(car_x > ROAD_ENDING_X - CAR_WIDTH){
                // car_x = ROAD_ENDING_X - CAR_WIDTH - 6;
                car_x -= (int)car_vel_x;
                car_vel_x = 0;
            }
            if(car_y < 0){
                // car_y = 6;
                car_y -= (int)car_vel_y;
                car_vel_y = 0;
            }
            if(car_y > SCREEN_HEIGHT - CAR_HEIGHT){
                // car_y = SCREEN_HEIGHT - CAR_HEIGHT - 6;
                car_y -= (int)car_vel_y;
                car_vel_y = 0;
            }

            printf("keyboard isr: car_x: %d, car_y: %d\n", car_x, car_y);
            printf("car_vel_x: %f, car_vel_y: %f\n", car_vel_x, car_vel_y);
            erase_car(car_x, car_y);

            

            // redraw_dashed_lines();
        }

    }
    return;
}

static void ADXL345_REG_WRITE(uint8_t address, uint8_t value) {

  *(I2C0_DATA_CMD) = address + 0x400;
  *(I2C0_DATA_CMD) = value;
}
void ADXL345_Init() {

    ADXL345_REG_WRITE(ADXL345_REG_DATA_FORMAT, XL345_RANGE_2G | XL345_FULL_RESOLUTION);
    // Output Data Rate: 100Hz
    ADXL345_REG_WRITE(ADXL345_REG_BW_RATE, XL345_RATE_100);
    // stop measure
    ADXL345_REG_WRITE(ADXL345_REG_POWER_CTL, XL345_STANDBY);
    // start measure
    ADXL345_REG_WRITE(ADXL345_REG_POWER_CTL, XL345_MEASURE);
	
}
// Read value from internal register at address
void ADXL345_REG_READ(uint8_t address, uint8_t * value) {

    // Send reg address (+0x400 to send START signal)
    * I2C0_DATA_CMD = address + 0x400;
    // Send read signal
    * I2C0_DATA_CMD = 0x100;
    // Read the response (first wait until RX buffer contains data)
    while ( * I2C0_RXFLR == 0) {}
    * value = * I2C0_DATA_CMD;
}
// Return true if there is new data
bool ADXL345_IsDataReady() {
    bool bReady = false;
    uint8_t data8;

    ADXL345_REG_READ(ADXL345_REG_INT_SOURCE, & data8);
    if (data8 & XL345_DATAREADY)
        bReady = true;

    return bReady;
}

/* Multiple Byte Write */
void ADXL345_REG_MULTI_READ(uint8_t address, uint8_t values[], uint8_t len) {

  int i = 0;
  int nth_byte = 0;
  *(I2C0_DATA_CMD) = address + 0x400;

  //send read signal multiple times to prevent overwritten data at 
  //inconsistent times

  for (i = 0; i < len; i++)
    *
    (I2C0_DATA_CMD) = 0x100;

  while (len) {
    if ( * (I2C0_RXFLR) > 0) {
      values[nth_byte] = * (I2C0_DATA_CMD) & 0xFF;
      nth_byte++;
      len--;
    }
  }
}


// Read acceleration data of all three axes
void ADXL345_XYZ_Read(int16_t szData16[3]) {

  uint8_t szData8[6];
  ADXL345_REG_MULTI_READ(0x32, (uint8_t * ) & szData8, sizeof(szData8));

  szData16[0] = (szData8[1] << 8) | szData8[0];
  szData16[1] = (szData8[3] << 8) | szData8[2];
  szData16[2] = (szData8[5] << 8) | szData8[4];
}

void I2C0_Init() {

  // Abort any ongoing transmits and disable I2C0.
  * I2C0_ENABLE = 2;

  // Wait until I2C0 is disabled
  while ((( * I2C0_ENABLE_STATUS) & 0x1) == 1) {}
  // Configure the config reg with the desired setting (act as
  // a master, use 7bit addressing, fast mode (400kb/s)).
  * I2C0_CON = 0x65;

  // Set target address (disable special commands, use 7bit addressing)
  * I2C0_TAR = 0x53;

  // Set SCL high/low counts (Assuming default 100MHZ clock input to
  //I2C0 Controller).
  // The minimum SCL high period is 0.6us, and the minimum SCL low
  //period is 1.3 us,
  // However, the combined period must be 2.5us or greater, so add 0.3us
  //to each.
  * I2C0_FS_SCL_HCNT = 60 + 30; // 0.6us + 0.3us
  * I2C0_FS_SCL_LCNT = 130 + 30; // 1.3us + 0.3us

  // Enable the controller
  * I2C0_ENABLE = 1;

  // Wait until controller is powered on
  while ((( * I2C0_ENABLE_STATUS) & 0x1) == 0) {}
}

void Pinmux_Config() {
  * SYSMGR_I2C0USEFPGA = 0;
  * SYSMGR_GENERALIO7 = 1;
  * SYSMGR_GENERALIO8 = 1;
}

bool start_acc(){
    uint8_t device_id;
    
    Pinmux_Config();
    I2C0_Init();
    ADXL345_REG_READ(0x00, & device_id);

    if (device_id == 0xE5)
    {
        ADXL345_Init();
    } 
    else
    {
        printf("Device id is not correct\n");
        return false;
    }
    return true;

}

    
void timer_ISR(){
    *(volatile uint32_t *)TIMER_STATUS = 0;
    timer_end = true;
}

// Define the IRQ exception handler
void __attribute__((interrupt)) __cs3_isr_irq(void) {
	// Read the ICCIAR from the CPU Interface in the GIC
	int interrupt_ID = *((int *)GIC_ICCIAR);
	if (interrupt_ID == 79) // check if interrupt is from the KEYs
	{   
        if(keyboard_control == true)
            keyboard_ISR();
    }
    else if (interrupt_ID == 72){
        timer_ISR();
    }
    else{

	while (1); // if unexpected, then stay here
    }
	// Write to the End of Interrupt Register (ICCEOIR)
	*((int *)0xFFFEC110) = interrupt_ID;
}

// Define the remaining exception handlers
void __attribute__((interrupt)) __cs3_reset(void) {
	while (1);
}

void __attribute__((interrupt)) __cs3_isr_undef(void) {
	while (1);
}

void __attribute__((interrupt)) __cs3_isr_swi(void) {
	while (1);
}

void __attribute__((interrupt)) __cs3_isr_pabort(void) {
	while (1);
}

void __attribute__((interrupt)) __cs3_isr_dabort(void) {
	while (1);
}

void __attribute__((interrupt)) __cs3_isr_fiq(void) {
	while (1);
}

void config_interrupt(int N, int CPU_target) {
	int reg_offset, index, value, address;
	/* Configure the Interrupt Set-Enable Registers (ICDISERn).
	* reg_offset = (integer_div(N / 32) * 4
	* value = 1 << (N mod 32) */
	reg_offset = (N >> 3) & 0xFFFFFFFC;
	index = N & 0x1F;
	value = 0x1 << index;
	address = 0xFFFED100 + reg_offset;
	/* Now that we know the register address and value, set the appropriate bit */
	*(int *)address |= value;

	/* Configure the Interrupt Processor Targets Register (ICDIPTRn)
	* reg_offset = integer_div(N / 4) * 4
	* index = N mod 4 */
	reg_offset = (N & 0xFFFFFFFC);
	index = N & 0x3;
	address = 0xFFFED800 + reg_offset + index;
	/* Now that we know the register address and value, write to (only) the
	* appropriate byte */
	*(char *)address = (char)CPU_target;
}

void config_GIC(void) {
	config_interrupt (79, 1); // configure the FPGA KEYs interrupt 
    config_interrupt (72, 1); // configure the Timer interrupt
	// Set Interrupt Priority Mask Register (ICCPMR). Enable interrupts of all
	// priorities
	*((int *) GIC_ICCPMR) = 0xFFFF;
	// Set CPU Interface Control Register (ICCICR). Enable signaling of
	// interrupts
	*((int *) GIC_ICCICR) = 1;
	// Configure the Distributor Control Register (ICDDCR) to send pending
	// interrupts to CPUs
	*((int *) GIC_ICDDCR) = 1;
}

void enable_A9_interrupts(void) {
	int status = 0b01010011;
	__asm__("msr cpsr, %[ps]" : : [ps] "r"(status));
}

// Turn off interrupts in the ARM processor
void disable_A9_interrupts(void) {
	int status = 0b11010011;
	__asm__("msr cpsr, %[ps]" : : [ps] "r"(status));
}

//Initialize the banked stack pointer register for IRQ mode
void set_A9_IRQ_stack(void) {
	int stack, mode;
	stack = 0xFFFFFFFF - 7;
	/* change processor to IRQ mode with interrupts disabled */
	mode = 0b11010010;
	__asm__("msr cpsr, %[ps]" : : [ps] "r"(mode));
	/* set banked stack pointer */
	__asm__("mov sp, %[ps]" : : [ps] "r"(stack));
	/* go back to SVC mode before executing subroutine return! */
	mode = 0b11010011;
	__asm__("msr cpsr, %[ps]" : : [ps] "r"(mode));
}