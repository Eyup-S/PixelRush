#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

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

#define TIMER_STATUS (TIMER_BASE + 0x00)
#define TIMER_CONTROL (TIMER_BASE + 0x04)
#define TIMER_STARTLOW 0xFF202008
#define TIMER_STARTHIGH 0xFF20200C

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

// INTERRUPT REGISTERS
#define GIC_ICCPMR 0xFFFEC104
#define GIC_ICDDCR 0xFFFED000
#define GIC_ICCICR 0xFFFEC100
#define GIC_ICCIAR 0xFFFEC10C

// MEMORY ADDRESSES
#define PS2_BASE 0xFF200100
#define VGA_BASE_ADDR 0xC8000000 
#define VIDEO_TEXT_BASE 0xC9000000
#define TIMER_BASE 0xFF202000

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
void draw_obstacle(Obstacle obstacle);


void keyboard_ISR(void);
void config_interrupt(int N, int CPU_target);
void config_GIC(void);
void enable_A9_interrupts(void);
void disable_A9_interrupts(void);
void set_A9_IRQ_stack(void);
void config_KEYs(void);

keyboard_ISR_acc();




/**********************
*   GLOBAL VARIABLES  *
***********************/
int car_x = 154; // Starting position of the car
int car_y = 220; // Starting position of the car
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

/**********************
*   MAIN FUNCTION     *
***********************/
int main() {

	clear_screen();
    start_screen();
    setup_timer(1000000); //
    disable_A9_interrupts();
	set_A9_IRQ_stack(); 
	config_GIC(); 
	config_KEYs(); 
	enable_A9_interrupts(); 

    int y_offset = 0;
    
    while(true){
       
        if(timer_end && is_game_started) { // Animation loop
            clear_road_lines(y_offset);
            y_offset++;

            draw_road_lines(WHITE, y_offset);
            if (y_offset >= 10) {
                y_offset = 0; // Reset the offset after a complete cycle
            }
            printf("y offset : %d\n", y_offset);

            for (int i = 0; i < level + 3; i++) {
                obstacles[i].y += obstacles[i].speed; // Move obstacle down
                if (obstacles[0].y > SCREEN_HEIGHT) {
                    // finish this level and increse level
                    level++;
                }
            }

            // Drawing obstacles
            for (int i = 0; i < level + 3; i++) {
                draw_obstacle(obstacles[i]);
                if(check_collision(obstacles[i])){
                    //game over
                    printf("game over\n");
                    
                }
            }
            
            
            timer_end = false;
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
    write_text(6,10,"0");
    init_obstacles();
    
    // animate_dashed_lines();

}
void plot_pixel(int x, int y, short int line_color)
{
	volatile int *pixel_ctrl_ptr = (int *)0xFF203020;
	/* Read location of the pixel buffer from the pixel buffer controller */
	volatile int pixel_buffer_start = *pixel_ctrl_ptr;
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
    int start_point = 44;

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
void draw_road_lines(short int line_color, int offset){
	int lane_width = ROAD_WIDTH / LANE_NUMBER;
    
	for (int i = ROAD_STARTING_X + lane_width - 2; i <ROAD_STARTING_X + 4 * lane_width + 2; ++i){
		for (int j = offset; j < SCREEN_HEIGHT; ++j) {
            
            if((i < ROAD_STARTING_X + lane_width +1 && i > ROAD_STARTING_X + lane_width - 1) || 
                (i < ROAD_STARTING_X + 2 * lane_width +1 && i > ROAD_STARTING_X + 2 * lane_width - 1) ||
                (i < ROAD_STARTING_X + 3 * lane_width +1 && i > ROAD_STARTING_X + 3 * lane_width - 1) || 
                (i < ROAD_STARTING_X + 4 * lane_width +1 && i > ROAD_STARTING_X + 4 * lane_width - 1)) 
                {
                    if(offset + 6 > j % 16 && j % 16  > offset)
                        plot_pixel(i, j, line_color);
                }
        }
	}
}

//redraw the dashed lines in the same region with the car
void redraw_dashed_lines(){
    int lane_width = ROAD_WIDTH / LANE_NUMBER;

    for(int i= car_x - 30; i< car_x + 30; i++){
        for(int j= car_y - 30; j< car_y + 30; j++){
            if((i < ROAD_STARTING_X + lane_width +1 && i > ROAD_STARTING_X + lane_width - 1) ||
                (i < ROAD_STARTING_X + 2 * lane_width +1 && i > ROAD_STARTING_X + 2 * lane_width - 1) ||
                (i < ROAD_STARTING_X + 3 * lane_width +1 && i > ROAD_STARTING_X + 3 * lane_width - 1) ||
                (i < ROAD_STARTING_X + 4 * lane_width +1 && i > ROAD_STARTING_X + 4 * lane_width - 1))
                {
                    if(j % 10 > 6)
                        plot_pixel(i, j, WHITE);
                }
        }
    }

}
void clear_old_lines(int x, int y_start, int y_end, int dash_length, int gap_length){
    int y = y_start;
    while (y < y_end) {
        for (int i = 0; i < dash_length && y < y_end; i++, y++) {
            plot_pixel(x, y, BLACK); 
        }
        y += gap_length;
    }

}


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
    for (int i = 0; i < CAR_WIDTH; ++i) {
        for (int j = 0; j < CAR_HEIGHT; ++j) {
            plot_pixel(x + i, y + j, BLUE);
        }
    }
}

void draw_pixel_map(char** pixel_map){
    //get dimension size of 2d char array
    int row = sizeof(pixel_map) / sizeof(pixel_map[0]);
    int col = sizeof(pixel_map[0]) / sizeof(pixel_map[0][0]);

    for(int i = 0; i < col; i++){
        for(int j = 0; j < row; j++){
            plot_pixel(280 + i, 20+ j, pixel_map[i][j]);
        }
    }
}


void init_obstacles() {
    for (int i = 0; i < NUM_OBSTACLES; i++) {
        // Initialize obstacle properties (position, size, color)
        obstacles[i].height = 20 + rand() % 10; //  height
        obstacles[i].width = 20; //  width
        obstacles[i].x = ROAD_STARTING_X + (rand() % 4) * ROAD_WIDTH/LANE_NUMBER + (ROAD_WIDTH/LANE_NUMBER - obstacles[i].width)/2;
        obstacles[i].y =  0; // Start off-screen
        obstacles[i].speed = 1 + rand() % 3; // Example speed
        obstacles[i].color = RED; // Example color
    }
}

void draw_obstacle(Obstacle obstacle) {
    for (int i = 0; i < obstacle.width; i++) {
        for (int j = 0; j < obstacle.height; j++) {
            if(j < obstacle.speed + 1) plot_pixel(obstacle.x + i, obstacle.y - j, BLACK);
            else if(obstacle.y + j > SCREEN_HEIGHT) continue;
            else
            plot_pixel(obstacle.x + i, obstacle.y + j, obstacle.color);
        }
    }
}

bool check_collision(Obstacle rect2) {
    // Check if there is no overlap on x-axis
    if (car_x > rect2.x + rect2.width || rect2.x > car_x + CAR_WIDTH) {
        return false; // No collision
    }
    // Check if there is no overlap on y-axis
    if (car_y > rect2.y + rect2.height || rect2.y > car_y + CAR_HEIGHT) {
        return false; // No collision
    }
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
        for (int i = 0; i < CAR_WIDTH; ++i) {
            for (int j = 0; j < CAR_HEIGHT; ++j) {
                plot_pixel(x + i, y + j, BLACK);
            }
        }
    }
    else if(leftArrowPressed){
        x += CAR_WIDTH - 2;
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < CAR_HEIGHT; ++j) {
                plot_pixel(x + i, y + j, BLACK);
            }
        }
    }
    else if (rightArrowPressed)
    {
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < CAR_HEIGHT; ++j) {
                plot_pixel(x + i, y + j, BLACK);
            }
        }
    }
    
    else if (upArrowPressed)
    {
        y += CAR_HEIGHT - 2;
        for (int i = 0; i < CAR_WIDTH; ++i) {
            for (int j = 0; j < 2; ++j) {
                plot_pixel(x + i, y + j, BLACK);
            }
        }
    }
    else if (downArrowPressed)
    {
        for (int i = 0; i < CAR_WIDTH; ++i) {
            for (int j = 0; j < 2; ++j) {
                plot_pixel(x + i, y + j, BLACK);
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

void setup_timer(uint32_t load_value) {
    // uint16_t counter_low = load_value & 0xFFFF;
    // uint16_t counter_high = (load_value >> 16) & 0xFFFF;
    uint16_t counter_low = 0xD784;
    uint16_t counter_high = 0x017;
    
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
        if(byte0 == 0x5A && !is_game_started){ //enter key
            keyboard_control = true;
            accelerometer_control = false;
            start_game();

        }
        if(keyboard_control){

            erase_car(car_x, car_y);

            if(leftArrowPressed){  //left arrow
                if(car_x > ROAD_STARTING_X + 2){
                car_x -= 2;    
                }
            }
            if(rightArrowPressed){  //right arrow
                if(car_x < ROAD_ENDING_X - 20 ){
                    car_x += 2;
                }
            }
            if(upArrowPressed){ //up arrow
                if(car_y > 0){
                    car_y -= 2;
                }
                }
            if (downArrowPressed) // down arrow
            {
                if(car_y < SCREEN_HEIGHT - 20){
                    car_y += 2;
                }
            }

            // redraw_dashed_lines();
            draw_car(car_x, car_y, BLUE);
        }

    }
    return;
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
        if(keyboard_control = true)
            keyboard_ISR();
        else if (accelerometer_control = true)
        {
            keyboard_ISR_acc();
        }
        
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
	asm("msr cpsr, %[ps]" : : [ps] "r"(status));
}

// Turn off interrupts in the ARM processor
void disable_A9_interrupts(void) {
	int status = 0b11010011;
	asm("msr cpsr, %[ps]" : : [ps] "r"(status));
}

//Initialize the banked stack pointer register for IRQ mode
void set_A9_IRQ_stack(void) {
	int stack, mode;
	stack = 0xFFFFFFFF - 7;
	/* change processor to IRQ mode with interrupts disabled */
	mode = 0b11010010;
	asm("msr cpsr, %[ps]" : : [ps] "r"(mode));
	/* set banked stack pointer */
	asm("mov sp, %[ps]" : : [ps] "r"(stack));
	/* go back to SVC mode before executing subroutine return! */
	mode = 0b11010011;
	asm("msr cpsr, %[ps]" : : [ps] "r"(mode));
}