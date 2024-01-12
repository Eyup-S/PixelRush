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
#define NUM_OBSTACLES 4
#define X_ACCELERATION 0.1
#define Y_ACCELERATION 0.1
#define TIMER_VALUE 1 //ms
#define CAR_START_X 154
#define CAR_START_Y 180
#define MAX_X_VELOCITY 3
#define MAX_Y_VELOCITY 3



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
    bool passive;
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
void game_over_screen();
bool start_acc();
void setup_timer(uint32_t load_value);

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


void acc_control(int16_t acc_data[3]);




/**********************
*   GLOBAL VARIABLES  *
***********************/
int car_x = CAR_START_X; // Starting position of the car
int car_y = CAR_START_Y; // Starting position of the car
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
int acc_filter = 0;
int acc_queue[5];


volatile int pixel_buffer_start;
volatile int * led_ptr = (int *) LEDS;

short int game_over_buffer[114][160];
short int car[35][14];



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

    if(!SIMULATION){

    if(!start_acc()){
        printf("Error in starting accelerometer\n");
        return;
        }
        printf("acc is started ...");
    }

    int y_offset = 0;
    int passive_obstacle = 0;
    int time_loop = 0;
    while(true){
       
        if(is_game_started) { // Animation loop
            car_x += (int)car_vel_x;
            car_y += (int)car_vel_y;

            if(car_x < ROAD_STARTING_X + 2){
                // car_x = ROAD_STARTING_X + 6;
                car_x -= (int)car_vel_x;
                car_vel_x = 0;
            }
            if(car_x > ROAD_ENDING_X - CAR_WIDTH){
                // car_x = ROAD_ENDING_X - CAR_WIDTH - 6;
                car_x -= (int)car_vel_x;
                car_vel_x = 0;
            }
            // if(car_y < 0){
            //     // car_y = 6;
            //     car_y -= (int)car_vel_y;
            //     car_vel_y = 0;
            // }
            // if(car_y > SCREEN_HEIGHT - CAR_HEIGHT){
            //     // car_y = SCREEN_HEIGHT - CAR_HEIGHT - 6;
            //     car_y -= (int)car_vel_y;
            //     car_vel_y = 0;
            // }

           

            y_offset++;
            if(!SIMULATION) 
            {
                acc_control(acc_value);
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
                    score +=  level;
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

            for (int i = 0; i < NUM_OBSTACLES; i++) {
                if (obstacles[i].y >= SCREEN_HEIGHT) {
                    obstacles[i].passive = true;
                    continue;
                }
                    passive_obstacle += i;
                    obstacles[i].y += obstacles[i].speed; // Move obstacle down
            }
            erase_car(car_x, car_y);

            draw_car(car_x, car_y, BLUE);

            // Drawing obstacles
            if(level < 4){
                for (int i = 0; i < NUM_OBSTACLES; i++) {
                    if(obstacles[i].y < SCREEN_HEIGHT)
                        draw_obstacle(obstacles[i]);   

                if(check_collision(obstacles[i])){
                    //game over
                    printf("game over\n");
                    game_over();
                    break;
                }
                    
            } 
            }
            if (passive_obstacle >=  NUM_OBSTACLES) {
                passive_obstacle = 0;
                if(level < 4) level++;
                for(int i = 0; i < 4; i++){
                    obstacles[i].y = 0;
                    obstacles[i].speed = level + 1;
                    obstacles[i].x = ROAD_STARTING_X + (rand() % LANE_NUMBER) * ROAD_WIDTH/LANE_NUMBER + (ROAD_WIDTH/LANE_NUMBER - obstacles[i].width)/2;
                }
            }
            passive_obstacle = 0;

        }
        
    }
    return 0;
}

/*****************************
*    FUNCTION DEFINITIONS    *
******************************/

void start_game(){

    is_game_started = true;
    // printf("game is started %d\n",is_game_started);
    clear_screen();
    draw_environment();
    draw_car(car_x, car_y, BLUE);
    write_text(5,10,"SCORE:");
    write_text(12,10,"0");
    init_obstacles();
    draw_road_lines(WHITE, 0);
    

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

void draw_car(int x, int y, short int line_color){
    for (int i = 0; i < CAR_WIDTH; ++i) {
        for (int j = 0; j < CAR_HEIGHT; ++j) {
            if (x + i < ROAD_ENDING_X && x + i > ROAD_STARTING_X && y + j < SCREEN_HEIGHT){
                plot_pixel( x + i,y + j,  car[j][i]);            
            }
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
        bool passive = false;
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
    
    // if ((car_x > rect2.x + rect2.width || rect2.x > car_x + CAR_WIDTH) && 
    //   (car_y > rect2.y + rect2.height || rect2.y > car_y + CAR_HEIGHT)) // Check if there is no overlap on x and y-axis
    //     return false;
    // else
    //     return true; // Collision detected
    if (car_x + CAR_WIDTH < rect2.x || rect2.x + rect2.width < car_x)
        return false;

    // Check if one rectangle is above the other
    if (car_y + CAR_HEIGHT < rect2.y || rect2.y + rect2.height < car_y)
        return false;

    return true; // Rectangles overlap
    
      
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
    
    
    // if((car_vel_x) ){
    //     x += leftArrowPressed * CAR_WIDTH - rightArrowPressed * car_vel_x;
    //     y += upArrowPressed * CAR_HEIGHT - downArrowPressed * car_vel_y;
    //     for (int i = 0; i < CAR_WIDTH; ++i) {
    //         for (int j = 0; j < CAR_HEIGHT; ++j) {
    //             plot_pixel(x + i, y + j, BLACK);
    //         }
    //     }
    // }
    // else {

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
    // if(downArrowPressed)
    {
        y -= car_vel_y;
        for (int i = 0; i < CAR_WIDTH; ++i) {
            for (int j = 0; j < (int)car_vel_y; ++j) {
                plot_pixel(x + i, y + j, BLACK);
            }
        }
    }
    else if (car_vel_y < 0) // up arrow pressed
    // else if(upArrowPressed )
    {   
        y += CAR_HEIGHT;
        for (int i = 0; i < CAR_WIDTH; ++i) {
            for (int j = 0; j < abs((int)car_vel_y); ++j) {
                plot_pixel(x + i, y  + j, BLACK);
            }
        }   
    }
    // }
    
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
    is_game_started = false;
    level = 0;
    score = 0;
    second = 0;
    car_vel_x = 0;
    car_vel_y = 0;
    car_x = CAR_START_X;
    car_y = CAR_START_Y;
    clear_screen();
    game_over_screen();
    write_text(20, 55, "Press ENTER to restart for playing using keyboard");
    write_text(20, 58, "Press SPACE to restart for playing using accelerometer");
    // is_game_started = false;

}

void game_over_screen(){
    for(int row = 0; row < 160; row++)
        for(int col = 0; col < 114; col++)
            plot_pixel(row + 80, col + 60, game_over_buffer[col][row]);
}

void acc_control(int16_t acc_data[3]){

    //int16_t first = acc_data[0] / 2.5;
    double second = - acc_data[1] / 2.5;
    //int16_t third = acc_value[2] / 2.5;

	
    //first *= 0.002;
    if(second > 99) {
        *led_ptr = 0x01;
		second = 99;
	}
	else if(second < 0){
		*led_ptr = 0x0200;
		if (second < -99){
			second = -99;
		}	
	}

    if(second < 8 || second > -8){
        second = 0;
    }

    acc_filter += 1;
    acc_queue[acc_filter] = second;

    if(acc_filter % 5 == 4){
        double mean = 0;
        for(int i = 0; i<5;i++){
            mean += acc_queue[i];
        }
        mean = mean / 5;
        printf("second: %f, mean: %f\n",second,mean); 
        acc_filter = 0;

        double vel = mean * 0.004;
    // third *= 0.002;
        car_vel_x += vel;
        //printf("girdi: %f\n", second);

        if(car_vel_x > 3)
                car_vel_x = 3;
        if(car_vel_x < -3)
                car_vel_x = 3;

           // if(car_vel_y > 4)
             //   car_vel_y = 4;

            car_x += (int)car_vel_x;
           // car_y += (int)car_vel_y;

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
            /*
            if(car_y < 0){
                // car_y = 6;
                car_y -= (int)car_vel_y;
                car_vel_y = 0;
            }
            if(car_y > SCREEN_HEIGHT - CAR_HEIGHT){
                // car_y = SCREEN_HEIGHT - CAR_HEIGHT - 6;
                car_y -= (int)car_vel_y;
                car_vel_y = 0;
            } */       
        
        erase_car(car_x, car_y);
        draw_car(car_x, car_y, WHITE);
        *HEX4_5 = car_x % 10;
//    printf("vel: %f, car_vel_x: %f, x: %d \n", vel, car_vel_x, car_x);
}}

uint16_t getSevenSegmentDecoding(uint16_t number){
    switch(number){
        case 0:
            return 0x3F;
        case 1:
            return 0x06;
        case 2:
            return 0x5B;
        case 3:
            return 0x4F;
        case 4:
            return 0x66;
        case 5:
            return 0x6D;
        case 6:
            return 0x7D;
        case 7:
            return 0x07;
        case 8:
            return 0x7F;
        case 9:
            return 0x6F;
        default:
            return 0x03;
    }
}

void displayInBoard(int value){
    uint16_t firstDigit = value / 10;
    uint16_t secondDigit = value % 10;
    uint16_t firstDigitDecoding = getSevenSegmentDecoding(firstDigit);
    uint16_t secondDigitDecoding = getSevenSegmentDecoding(secondDigit);
    *(uint32_t *)HEX0_3 = (firstDigitDecoding << 16) | secondDigitDecoding;


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
                printf("start acc\n");
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
                    car_vel_y = Y_ACCELERATION;
                }
                }
            if (downArrowPressed) // down arrow
            {
                if(car_y < SCREEN_HEIGHT - CAR_HEIGHT){
                    car_vel_y = -Y_ACCELERATION;
                }
            }
            if(car_vel_x > MAX_X_VELOCITY)
                car_vel_x = MAX_X_VELOCITY;
            else if (car_vel_x < -MAX_X_VELOCITY)
                car_vel_x = -MAX_X_VELOCITY;
            if(car_vel_y > MAX_Y_VELOCITY)
                car_vel_y = MAX_Y_VELOCITY;
            else if (car_vel_y < -MAX_Y_VELOCITY)
                car_vel_y = -MAX_Y_VELOCITY;
            
            

            // car_x += (int)car_vel_x;
            // car_y += (int)car_vel_y;

            // if(car_x < ROAD_STARTING_X){
            //     // car_x = ROAD_STARTING_X + 6;
            //     car_x -= (int)car_vel_x;
            //     car_vel_x = 0;
            // }
            // if(car_x > ROAD_ENDING_X - CAR_WIDTH){
            //     // car_x = ROAD_ENDING_X - CAR_WIDTH - 6;
            //     car_x -= (int)car_vel_x;
            //     car_vel_x = 0;
            // }
            // if(car_y < 0){
            //     // car_y = 6;
            //     car_y -= (int)car_vel_y;
            //     car_vel_y = 0;
            // }
            // if(car_y > SCREEN_HEIGHT - CAR_HEIGHT){
            //     // car_y = SCREEN_HEIGHT - CAR_HEIGHT - 6;
            //     car_y -= (int)car_vel_y;
            //     car_vel_y = 0;
            // }

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

/*************************
*       PIXEL MAPS       *
**************************/


short int car[35][14]= 
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

short int game_over_buffer[114][160] =
{
    {2, 4, 137, 334, 466, 4724, 8915, 17301, 19414, 19413, 19381, 19381, 19381, 19349, 17301, 17301, 17269, 17268, 15220, 15188, 15188, 15188, 15156, 15156, 15156, 13107, 13075, 13075, 13043, 13043, 13043, 13043, 10995, 10995, 10995, 10963, 10963, 8915, 10963, 10963, 8915, 8883, 8883, 8883, 8915, 8883, 8883, 8883, 8883, 8883, 8883, 8883, 8883, 8883, 8883, 8883, 8883, 8883, 6803, 6835, 6835, 6835, 6835, 6803, 6803, 6803, 6803, 6803, 6803, 6803, 6803, 6803, 6803, 6803, 4755, 4755, 4723, 4723, 4724, 4724, 4723, 4756, 4723, 4723, 4723, 4723, 4723, 4723, 4724, 4724, 4724, 2675, 2675, 2675, 2644, 2644, 2644, 2644, 2644, 2644, 2643, 2644, 2644, 2644, 2644, 2643, 563, 2644, 596, 596, 564, 564, 564, 564, 564, 564, 564, 564, 564, 564, 564, 564, 564, 532, 532, 532, 532, 532, 532, 532, 532, 532, 532, 532, 532, 532, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 468, 467, 467, 435, 402, 337, 270, 203, 136, 70, 3, 1, 0},
    {4, 167, 2413, 6770, 17337, 38495, 51167, 53247, 53247, 53247, 53247, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 53247, 53247, 53247, 55295, 53247, 53247, 53247, 53247, 53247, 53247, 53247, 53247, 53247, 53247, 53247, 53247, 51199, 51167, 51167, 51167, 51167, 51135, 51135, 49087, 49055, 49055, 49055, 49023, 46975, 46975, 46975, 46943, 46943, 46943, 44863, 44863, 44863, 44831, 44831, 44831, 44831, 42751, 42751, 42751, 42719, 42719, 40671, 40639, 40639, 40639, 40639, 40607, 40607, 38559, 38527, 38527, 38527, 38495, 38495, 36447, 36447, 36415, 36415, 36415, 34335, 34335, 34335, 34335, 34303, 34303, 34303, 32223, 32223, 32223, 32191, 32191, 32191, 30111, 30111, 30111, 30111, 30079, 30079, 28031, 27999, 27999, 27999, 27967, 25919, 25919, 25887, 25887, 25887, 23838, 23806, 23806, 23774, 23774, 23774, 21693, 21693, 21693, 21661, 21661, 21661, 21661, 19581, 19580, 19580, 19548, 19548, 19548, 17500, 17468, 17468, 17435, 17435, 15387, 15387, 15354, 15354, 15354, 15322, 15322, 15322, 13242, 13209, 8950, 8885, 4658, 2446, 267, 169, 102, 35, 1},
    {170, 2446, 10931, 32125, 53247, 38527, 19580, 11096, 8983, 8983, 8983, 8983, 8983, 9015, 9015, 9016, 11096, 11096, 11096, 11128, 11128, 11128, 11129, 13177, 13209, 13209, 13241, 13241, 13242, 15322, 15322, 15322, 15322, 15354, 15354, 15355, 17435, 17435, 17467, 17467, 19515, 19516, 19548, 19548, 19580, 19580, 21628, 21660, 21661, 21661, 21693, 21693, 23741, 23773, 23774, 23774, 23806, 23806, 25854, 25854, 25886, 25886, 25887, 25919, 27967, 27967, 27967, 27999, 27999, 27999, 27999, 30079, 30079, 30079, 30111, 30111, 30111, 32191, 32191, 32191, 32223, 32223, 32223, 34303, 34303, 34303, 34335, 34335, 34335, 34367, 36415, 36415, 36415, 36415, 36447, 38495, 38495, 38527, 38527, 38527, 38559, 40607, 40607, 40639, 40639, 40639, 40639, 40671, 42719, 42719, 42719, 42751, 42751, 42751, 44831, 44831, 44831, 44863, 44863, 44863, 46911, 46943, 46943, 46943, 46975, 46975, 49023, 49023, 49055, 49055, 49087, 49087, 49087, 51167, 51135, 51167, 51167, 51167, 51199, 51199, 53247, 53247, 53247, 53247, 53247, 53247, 53247, 53247, 55295, 55295, 55295, 53215, 44767, 34173, 13141, 4657, 366, 234, 135, 68},
    {367, 8916, 36319, 51167, 23641, 8884, 530, 466, 434, 433, 401, 401, 401, 400, 401, 401, 400, 400, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433, 465, 465, 465, 465, 465, 465, 465, 465, 465, 465, 465, 465, 465, 465, 465, 465, 497, 497, 497, 497, 465, 2545, 2545, 2545, 2545, 2578, 2545, 2578, 2577, 2577, 2578, 2578, 2578, 2578, 2578, 2578, 2578, 2578, 4658, 2610, 2610, 4658, 4658, 4658, 4658, 4658, 4690, 4690, 4690, 4690, 4690, 4690, 4690, 4690, 4690, 4690, 4690, 4722, 4690, 6770, 6770, 6770, 6770, 6771, 6771, 6771, 6771, 6771, 6771, 6771, 6771, 6771, 6771, 6771, 6771, 6771, 8851, 6803, 8851, 8851, 8850, 8850, 8850, 8850, 8850, 8850, 8850, 8850, 8850, 8851, 8851, 8883, 8850, 8850, 8883, 8883, 8883, 8882, 8882, 8883, 8883, 8882, 10931, 8883, 10963, 10963, 10931, 10963, 10962, 10962, 10963, 10963, 10995, 10995, 10995, 13043, 13043, 13075, 15156, 23608, 32092, 46879, 51167, 25853, 6804, 398, 234, 135},
    {6771, 27966, 51167, 17368, 4658, 432, 334, 301, 268, 267, 235, 235, 202, 202, 201, 201, 201, 201, 201, 201, 202, 202, 202, 202, 202, 202, 202, 234, 234, 234, 234, 234, 235, 235, 235, 235, 235, 235, 235, 235, 235, 234, 234, 234, 234, 234, 234, 233, 201, 201, 201, 201, 201, 201, 201, 201, 234, 234, 234, 234, 234, 234, 234, 234, 234, 267, 235, 235, 235, 266, 235, 267, 267, 267, 267, 267, 267, 267, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 332, 332, 333, 333, 333, 333, 333, 333, 333, 333, 333, 333, 333, 333, 333, 333, 333, 333, 332, 332, 333, 333, 333, 333, 365, 365, 365, 333, 365, 365, 365, 365, 365, 365, 365, 365, 365, 365, 365, 365, 365, 365, 2414, 365, 365, 398, 398, 398, 398, 398, 398, 398, 2479, 2479, 2545, 2643, 6869, 25821, 51199, 32126, 8883, 398, 203},
    {17336, 53247, 23708, 2610, 399, 301, 268, 234, 233, 168, 135, 102, 101, 69, 68, 68, 68, 68, 69, 69, 101, 101, 101, 101, 101, 101, 101, 101, 101, 101, 102, 102, 134, 134, 134, 134, 134, 134, 134, 134, 134, 134, 102, 102, 102, 101, 101, 101, 69, 68, 68, 68, 68, 68, 69, 69, 101, 101, 101, 101, 101, 102, 134, 134, 134, 134, 134, 134, 134, 134, 102, 134, 134, 134, 135, 135, 135, 135, 167, 135, 167, 168, 168, 168, 168, 200, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 200, 201, 201, 201, 201, 201, 201, 201, 201, 201, 201, 201, 201, 201, 201, 200, 200, 200, 200, 168, 200, 168, 200, 201, 201, 201, 201, 201, 201, 201, 201, 201, 201, 201, 201, 201, 201, 201, 201, 202, 234, 234, 234, 234, 234, 234, 234, 235, 235, 267, 235, 267, 268, 268, 301, 368, 465, 4723, 21562, 51199, 27932, 6771, 335},
    {29947, 46911, 6804, 433, 301, 268, 234, 200, 134, 68, 35, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 34, 34, 34, 34, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 34, 34, 2, 2, 2, 2, 2, 34, 34, 34, 35, 35, 35, 35, 35, 35, 67, 67, 67, 67, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 101, 101, 101, 101, 101, 101, 101, 101, 101, 101, 101, 101, 68, 100, 68, 68, 68, 68, 68, 68, 68, 68, 68, 100, 69, 101, 69, 101, 69, 101, 101, 101, 101, 101, 101, 101, 101, 102, 102, 134, 134, 135, 134, 135, 167, 135, 135, 135, 167, 168, 168, 200, 201, 234, 267, 301, 400, 4690, 25787, 53247, 17402, 498},
    {38365, 40543, 4691, 368, 301, 267, 201, 134, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 35, 35, 35, 35, 35, 36, 35, 35, 68, 68, 68, 69, 134, 168, 234, 269, 432, 6803, 40607, 34335, 4757},
    {42655, 34205, 4659, 368, 300, 234, 168, 68, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 4096, 6144, 10240, 12288, 14336, 14336, 12288, 10240, 6144, 4096, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 68, 167, 234, 333, 2578, 21661, 49087, 8948},
    {44703, 34205, 4658, 367, 268, 234, 135, 35, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 2048, 4096, 8192, 10240, 16417, 18498, 22626, 24674, 26723, 26723, 24674, 22626, 18498, 14337, 10240, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 4096, 8192, 10240, 10240, 10240, 8192, 4096, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 4096, 6144, 8192, 8192, 6144, 4096, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 4096, 4096, 6144, 6144, 4096, 4096, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 2048, 2048, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 2048, 2048, 0, 0, 0, 0, 0, 0, 0, 2, 69, 168, 300, 498, 15322, 53247, 11028},
    {42623, 34205, 4658, 367, 268, 233, 134, 35, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 4096, 12288, 20480, 26625, 30851, 32997, 35175, 41546, 49966, 52079, 52111, 47885, 41514, 35110, 32964, 28706, 22528, 12288, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 12288, 20480, 26625, 32802, 32834, 30754, 28673, 20480, 10240, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 10240, 18432, 24576, 28672, 28672, 24576, 18432, 10240, 4096, 0, 0, 0, 0, 0, 0, 0, 4096, 8192, 16384, 22528, 26624, 24576, 20480, 14336, 8192, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 6144, 8192, 10240, 12288, 12288, 12288, 12288, 12288, 12288, 12288, 12288, 12288, 12288, 12288, 12288, 12288, 12288, 10240, 8192, 6144, 2048, 0, 0, 0, 0, 0, 1, 67, 167, 268, 466, 13242, 53247, 13075},
    {42623, 34205, 4658, 335, 268, 233, 102, 3, 0, 0, 0, 0, 0, 0, 0, 4096, 8192, 16384, 24642, 32964, 45480, 64400, 64822, 65147, 65180, 65049, 64984, 64952, 65082, 65180, 65050, 64627, 51754, 37060, 24642, 14336, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 12288, 22594, 35012, 45382, 53802, 60109, 60141, 47528, 35012, 22593, 10240, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 12288, 22594, 35012, 41221, 45382, 45350, 41221, 35012, 22594, 12288, 4096, 0, 0, 0, 0, 0, 4096, 12288, 22561, 30851, 37060, 41189, 39109, 35012, 30851, 22561, 12288, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 8192, 18530, 28966, 35239, 37287, 37255, 37255, 37288, 37288, 37320, 37320, 37320, 37320, 37288, 37320, 39368, 39368, 37320, 35272, 26917, 16450, 6144, 2048, 0, 0, 0, 0, 0, 34, 135, 267, 466, 13242, 53247, 13108},
    {42623, 34238, 4658, 335, 268, 233, 102, 2, 0, 0, 0, 0, 0, 2048, 4096, 12288, 24577, 32932, 43562, 64724, 65179, 64887, 64432, 53802, 45382, 41157, 41157, 41157, 41189, 45382, 58028, 64692, 65212, 64757, 43432, 32932, 22528, 12288, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 14336, 24642, 37158, 58223, 65179, 65147, 64984, 65017, 65244, 64724, 39206, 24577, 8192, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 14336, 24642, 37190, 58287, 65082, 65212, 65179, 65017, 60401, 39303, 28771, 14336, 4096, 0, 0, 0, 2048, 12288, 28705, 41189, 47690, 60433, 64627, 62546, 58321, 45610, 41221, 28706, 12288, 2048, 0, 0, 0, 0, 0, 0, 0, 4096, 16384, 35207, 56402, 65114, 65309, 65276, 65277, 65277, 65309, 65309, 65309, 65309, 65277, 65277, 65277, 65309, 65309, 65309, 65309, 65309, 65114, 54321, 28965, 12288, 2048, 0, 0, 0, 0, 2, 134, 235, 466, 13242, 55295, 15156},
    {42591, 36286, 4658, 367, 268, 233, 134, 2, 0, 0, 0, 0, 2048, 6144, 14336, 24642, 39173, 64529, 65179, 64789, 47755, 35077, 32899, 32801, 30720, 28672, 26624, 26624, 28672, 30721, 32834, 35045, 43561, 64887, 65050, 56012, 35077, 22594, 14336, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8192, 22529, 37060, 62449, 65147, 52014, 35142, 33029, 35077, 43594, 65082, 62352, 34980, 14336, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6144, 22528, 39141, 64627, 65114, 54127, 41514, 43627, 54159, 65017, 64822, 41286, 24577, 8192, 0, 0, 2048, 8192, 22561, 39108, 64367, 65179, 64887, 64724, 64757, 64887, 65147, 64303, 41189, 20481, 6144, 2048, 0, 0, 0, 0, 0, 2048, 10240, 37287, 64984, 65081, 48048, 33321, 31208, 31240, 31240, 31240, 31240, 31241, 31241, 31208, 31208, 31208, 29128, 29128, 31208, 31208, 35369, 50095, 65179, 58353, 24739, 4096, 0, 0, 0, 0, 2, 134, 235, 466, 13209, 53247, 15156},
    {40543, 36286, 4658, 367, 268, 233, 101, 2, 0, 0, 0, 0, 6144, 14336, 28738, 43399, 64887, 65017, 53867, 39108, 28770, 20545, 16384, 12288, 12288, 10240, 10240, 10240, 10240, 10240, 14336, 20513, 28770, 37093, 62351, 65147, 64724, 37287, 28770, 20480, 8192, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 12288, 28770, 53899, 65147, 49803, 28803, 22561, 18432, 22561, 34915, 58093, 65114, 39336, 22593, 10240, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10240, 30753, 49609, 65212, 45610, 30883, 26690, 26690, 28803, 41351, 65017, 62319, 30851, 12288, 4096, 0, 4096, 16384, 28770, 43497, 65179, 55980, 34947, 28770, 28770, 34947, 56013, 65114, 43432, 26722, 12288, 2048, 0, 0, 0, 0, 2048, 6144, 22691, 54257, 65114, 26950, 10240, 4096, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 6144, 16384, 47885, 65147, 33159, 4096, 0, 0, 0, 0, 2, 102, 235, 465, 13209, 53247, 15188},
    {40511, 36318, 4658, 367, 268, 201, 101, 2, 0, 0, 0, 4096, 14336, 26722, 41351, 64920, 64855, 41384, 34947, 28672, 18432, 12288, 12288, 12288, 14336, 16385, 16417, 16417, 14336, 12288, 10240, 12288, 20480, 26657, 32932, 45610, 64919, 65049, 47658, 30851, 18433, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 8192, 20513, 35045, 64854, 64562, 35044, 20480, 10240, 8192, 12288, 28673, 39108, 64984, 58353, 32964, 16384, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 14336, 34915, 62254, 64952, 35077, 22529, 14336, 14336, 20480, 34947, 62351, 64952, 35110, 20545, 8192, 2048, 6144, 24576, 37060, 62514, 64887, 37060, 24576, 16384, 16384, 28672, 37060, 65082, 51981, 32932, 16384, 4096, 0, 0, 0, 0, 2048, 10240, 31046, 65146, 48047, 10240, 4096, 0, 0, 0, 2048, 2048, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 6144, 12288, 39433, 65212, 35240, 4096, 0, 0, 0, 0, 2, 101, 234, 465, 11129, 55295, 15188},
    {40511, 36319, 4658, 367, 268, 201, 101, 2, 0, 0, 0, 8192, 24577, 39173, 64822, 64887, 41286, 30786, 18432, 12288, 10240, 16384, 26624, 30754, 34980, 35045, 35045, 35045, 32932, 30786, 24576, 16384, 10240, 12288, 20481, 30819, 41254, 64594, 65115, 45707, 28771, 18432, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 14336, 26722, 47852, 65179, 47496, 30754, 12288, 4096, 2048, 6144, 20480, 30851, 54094, 65114, 43367, 26625, 8192, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 16417, 37093, 64659, 64594, 30851, 14336, 6144, 4096, 10240, 28673, 45415, 65179, 47852, 28771, 16384, 6144, 12288, 30721, 45350, 65114, 49966, 24674, 12288, 6144, 6144, 18432, 30851, 64789, 62579, 39109, 24576, 6144, 0, 0, 0, 0, 2048, 10240, 33159, 65277, 37450, 6144, 2048, 0, 0, 2048, 6144, 10240, 14336, 14336, 14336, 14336, 14336, 14336, 14336, 14336, 14336, 14336, 14336, 20480, 54127, 65017, 31014, 4096, 0, 0, 0, 0, 1, 101, 234, 465, 11128, 55295, 15221},
    {40511, 36351, 4659, 367, 268, 201, 101, 2, 0, 0, 2048, 12288, 30851, 60238, 65082, 41416, 28738, 20480, 10240, 8192, 16384, 26722, 39141, 51754, 64562, 64757, 64855, 64757, 64594, 55980, 39173, 26690, 14336, 8192, 10240, 18432, 28673, 39141, 62514, 65114, 41286, 26657, 8192, 2048, 0, 0, 0, 0, 0, 0, 0, 6144, 22528, 37093, 64919, 62546, 37060, 22528, 6144, 0, 0, 2048, 10240, 24642, 35110, 64984, 64400, 32932, 14336, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 8192, 20545, 37158, 65049, 58060, 28738, 10240, 2048, 0, 6144, 22528, 34980, 64692, 64789, 37060, 24576, 10240, 18432, 36995, 62254, 65049, 33062, 16417, 6144, 2048, 2048, 12288, 24674, 52143, 64984, 43270, 28672, 8192, 0, 0, 0, 0, 2048, 10240, 33159, 65277, 37450, 4096, 0, 0, 2048, 6144, 22724, 33127, 35142, 35142, 37223, 37223, 35175, 35175, 37223, 37223, 37255, 37255, 37255, 43594, 65082, 58385, 20578, 4096, 0, 0, 0, 0, 1, 101, 202, 433, 11096, 55295, 15221},
    {40511, 36351, 4659, 367, 268, 200, 101, 1, 0, 2048, 6144, 18497, 35174, 65049, 62286, 34947, 18432, 8192, 10240, 18432, 30819, 43367, 64724, 65212, 64757, 64432, 62287, 64432, 64660, 65114, 64757, 39303, 26690, 16384, 6144, 6144, 14336, 26690, 41286, 64887, 64562, 28803, 12288, 2048, 0, 0, 0, 0, 0, 0, 2048, 12288, 28738, 53932, 65147, 43464, 26722, 12288, 2048, 0, 0, 0, 4096, 16384, 32867, 60174, 65082, 39303, 20513, 8192, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 12288, 24674, 43594, 65212, 45383, 26625, 8192, 0, 0, 2048, 12288, 26722, 45674, 65179, 47528, 32769, 18432, 24609, 39206, 64920, 62352, 28770, 10240, 2048, 0, 2048, 8192, 20578, 39433, 65212, 49511, 32802, 10240, 0, 0, 0, 0, 2048, 10240, 35207, 65277, 35402, 4096, 0, 0, 4096, 18433, 39433, 64854, 65179, 65212, 65244, 65244, 65243, 65244, 65244, 65276, 65276, 65276, 65277, 65276, 62773, 26950, 8192, 2048, 0, 0, 0, 0, 1, 69, 202, 433, 11064, 55295, 17301},
    {38430, 36351, 4691, 367, 268, 200, 101, 1, 0, 2048, 12288, 24706, 49966, 65114, 43270, 30721, 10240, 6144, 16384, 28771, 45513, 64952, 64952, 41514, 32996, 30851, 30819, 32899, 35012, 43496, 65017, 64725, 41253, 24577, 8192, 4096, 8192, 18432, 36995, 62190, 64952, 33029, 16417, 4096, 0, 0, 0, 0, 0, 2048, 8192, 18465, 35077, 64887, 64497, 35012, 18433, 6144, 2048, 2048, 2048, 2048, 2048, 10240, 28673, 41189, 65049, 56272, 30884, 16384, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 16384, 30883, 54192, 64984, 37060, 24576, 6144, 0, 0, 2048, 6144, 18465, 35077, 64854, 64465, 36963, 28672, 32867, 49868, 65147, 43366, 24577, 6144, 0, 0, 0, 4096, 16417, 30981, 65049, 60076, 36963, 14336, 2048, 0, 0, 0, 2048, 10240, 35207, 65277, 35369, 4096, 0, 0, 4096, 22529, 60434, 65049, 39660, 35499, 35499, 35499, 33418, 33418, 33386, 33386, 31305, 33353, 33353, 29160, 18563, 8192, 4096, 0, 0, 0, 0, 0, 1, 69, 202, 433, 11064, 55295, 17301},
    {38430, 36351, 4691, 367, 268, 200, 100, 1, 0, 4096, 16384, 30883, 62644, 64692, 37060, 24576, 6144, 6144, 24576, 39141, 64854, 64821, 39238, 28738, 18433, 12288, 10240, 12288, 20481, 28771, 43496, 65115, 58093, 34980, 18432, 10240, 8192, 16384, 34850, 57996, 65082, 35110, 16417, 4096, 0, 0, 0, 0, 0, 4096, 16384, 28771, 49933, 65147, 47496, 30753, 10240, 4096, 4096, 8192, 10240, 10240, 4096, 6144, 20480, 30883, 54127, 65082, 43334, 26625, 8192, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6144, 22528, 37060, 64757, 62611, 30851, 18432, 4096, 0, 0, 0, 2048, 12288, 30818, 58093, 65082, 41254, 36865, 43172, 64790, 62644, 30884, 18432, 4096, 0, 0, 0, 2048, 12288, 28803, 64692, 64562, 39141, 16384, 4096, 0, 0, 0, 2048, 10240, 35207, 65277, 35369, 4096, 0, 0, 4096, 20546, 62644, 62676, 18498, 4096, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 0, 0, 0, 0, 0, 0, 1, 68, 201, 433, 9015, 53247, 17334},
    {38430, 38431, 4691, 367, 268, 200, 100, 1, 0, 4096, 20480, 34980, 64822, 60466, 32900, 18432, 6144, 8192, 28672, 43302, 65212, 43659, 28738, 18432, 8192, 2048, 2048, 4096, 8192, 18432, 30851, 60303, 65017, 43399, 28771, 20480, 18432, 24642, 37060, 64562, 64790, 35077, 14337, 4096, 0, 0, 0, 0, 0, 8192, 22529, 37093, 64952, 60498, 37060, 22528, 8192, 6144, 14336, 22626, 26787, 22658, 14336, 10240, 14336, 24642, 37190, 65017, 62286, 32899, 12288, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8192, 28672, 43270, 65114, 47917, 24674, 12288, 2048, 0, 0, 0, 0, 8192, 26625, 41221, 65114, 56110, 41092, 51657, 65179, 43594, 22594, 10240, 2048, 0, 0, 0, 2048, 10240, 26690, 58158, 64952, 39238, 20545, 8192, 2048, 0, 0, 2048, 10240, 35240, 65277, 35369, 4096, 0, 0, 4096, 20546, 62676, 60563, 18465, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 68, 201, 433, 8983, 53247, 17334},
    {38398, 38463, 4691, 368, 268, 200, 68, 1, 0, 6144, 22528, 37028, 64855, 56272, 28803, 16384, 4096, 8192, 28673, 45350, 65212, 39465, 22594, 10240, 4096, 0, 0, 0, 2048, 10240, 24577, 37125, 65017, 62579, 39271, 37125, 35045, 37190, 56240, 65179, 53867, 30819, 10240, 2048, 0, 0, 0, 0, 2048, 12288, 30818, 56012, 65114, 41416, 24674, 14336, 8192, 12288, 28771, 45512, 54094, 39368, 28803, 18432, 10240, 16384, 32867, 62287, 65017, 37223, 20513, 8192, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10240, 30753, 51689, 65180, 37287, 20545, 8192, 2048, 0, 0, 0, 0, 4096, 18432, 30883, 60466, 64952, 39271, 64757, 64757, 32964, 16417, 6144, 2048, 0, 0, 0, 0, 8192, 24577, 45447, 65179, 43497, 26754, 12288, 2048, 0, 0, 2048, 10240, 35240, 65277, 35337, 4096, 0, 0, 4096, 20546, 64789, 60531, 20481, 6144, 4096, 2048, 2048, 2048, 2048, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 68, 201, 433, 8983, 55295, 19414},
    {38398, 38463, 4691, 368, 268, 168, 68, 1, 0, 6144, 24576, 39076, 64919, 54224, 28803, 14336, 4096, 8192, 30721, 47398, 65212, 39433, 20546, 8192, 2048, 0, 2048, 2048, 4096, 8192, 20480, 30754, 45642, 65082, 65017, 64790, 64757, 65050, 65114, 52014, 35012, 22528, 6144, 0, 0, 0, 0, 2048, 8192, 20513, 35077, 64919, 64497, 35012, 16384, 8192, 12288, 22561, 39174, 64887, 65115, 64952, 41221, 26624, 10240, 12288, 28673, 41221, 65082, 54159, 30884, 16384, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 12288, 34947, 64335, 64919, 32997, 16417, 4096, 0, 0, 0, 0, 0, 2048, 10240, 22594, 41416, 65082, 64985, 65082, 47593, 26690, 10240, 2048, 0, 0, 0, 0, 0, 6144, 22528, 35012, 65081, 52046, 32964, 16384, 4096, 0, 0, 2048, 10240, 35240, 65277, 35337, 4096, 0, 0, 4096, 20578, 64821, 60498, 24707, 16417, 14401, 12321, 12321, 12321, 10273, 10273, 8192, 4096, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 68, 169, 433, 8983, 55295, 19414},
    {38398, 38463, 4691, 368, 268, 200, 68, 1, 0, 6144, 24576, 39108, 64952, 54159, 28803, 14336, 4096, 8192, 30721, 47431, 65212, 37385, 18498, 8192, 2048, 2048, 6144, 12288, 16384, 20480, 22528, 28672, 36866, 45285, 60044, 64367, 64400, 57996, 43270, 32899, 20513, 10240, 2048, 0, 0, 0, 0, 4096, 16384, 28803, 52014, 65147, 45415, 28705, 10240, 8192, 20480, 32899, 52014, 65147, 45545, 65082, 58028, 32899, 14336, 10240, 22528, 32899, 56272, 65049, 41254, 26625, 8192, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 16385, 37093, 64724, 64530, 30851, 12288, 2048, 0, 0, 0, 0, 0, 0, 4096, 14336, 32899, 51689, 64335, 51657, 35012, 18432, 4096, 0, 0, 0, 0, 0, 0, 4096, 18432, 28803, 64724, 64692, 39141, 24576, 6144, 0, 0, 2048, 10240, 35207, 65277, 33289, 4096, 0, 0, 4096, 22594, 58385, 65082, 52176, 50063, 47982, 47950, 45869, 43756, 43724, 41643, 39498, 28933, 14336, 4096, 2048, 0, 0, 0, 0, 0, 0, 0, 1, 68, 169, 433, 6903, 53247, 19446},
    {38398, 38463, 4691, 368, 268, 200, 67, 1, 0, 6144, 26624, 41156, 64984, 52079, 28803, 14336, 4096, 8192, 30721, 49511, 65212, 37352, 18498, 8192, 6144, 10240, 18497, 24707, 28835, 30851, 32899, 34915, 36930, 41027, 41059, 41059, 39011, 36898, 36865, 28672, 14336, 6144, 2048, 0, 0, 0, 0, 8192, 24577, 39173, 64984, 60433, 34980, 22528, 8192, 10240, 28673, 41221, 64984, 60433, 39141, 64497, 64822, 39206, 20513, 12288, 14336, 24642, 37223, 65082, 60174, 32899, 14336, 2048, 0, 0, 0, 0, 0, 0, 0, 2048, 8192, 20545, 37191, 65082, 55948, 28738, 10240, 2048, 2048, 2048, 6144, 6144, 4096, 4096, 2048, 8192, 20480, 34947, 39108, 34947, 18432, 8192, 2048, 2048, 4096, 6144, 6144, 4096, 2048, 4096, 12288, 24674, 50030, 65049, 43269, 28672, 8192, 0, 0, 2048, 10240, 35207, 65277, 33256, 4096, 0, 0, 2048, 12288, 33192, 58580, 65016, 65049, 65049, 65114, 65147, 65179, 65212, 65212, 65244, 64984, 43659, 16450, 4096, 0, 0, 0, 0, 0, 0, 0, 1, 68, 201, 433, 6902, 53247, 19446},
    {38365, 40543, 4691, 368, 268, 200, 67, 1, 0, 6144, 24576, 39108, 65049, 49998, 26755, 14336, 4096, 10240, 30721, 49544, 65212, 37320, 18497, 12288, 16384, 26722, 35174, 52046, 60531, 62611, 62611, 62579, 62481, 60368, 58255, 56142, 53997, 45512, 43205, 34850, 20480, 10240, 2048, 0, 0, 0, 4096, 14336, 30819, 56013, 65082, 41383, 24642, 12288, 8192, 16385, 34947, 58093, 65082, 43367, 36866, 45415, 65179, 47788, 30851, 18432, 10240, 18432, 32899, 64400, 64952, 37190, 20513, 8192, 2048, 0, 0, 0, 0, 0, 0, 2048, 12288, 26723, 45675, 65180, 43302, 26625, 8192, 2048, 6144, 14336, 30754, 36963, 24577, 12288, 4096, 2048, 6144, 8192, 10240, 8192, 4096, 2048, 2048, 6144, 16384, 26625, 28705, 18432, 6144, 4096, 8192, 20578, 37320, 65212, 51624, 32802, 10240, 0, 0, 2048, 10240, 35239, 65277, 33256, 4096, 0, 0, 0, 2048, 6144, 14434, 16514, 18563, 18563, 20611, 20611, 20643, 22691, 24740, 33224, 58580, 65179, 35272, 10240, 2048, 0, 0, 0, 0, 0, 0, 1, 68, 201, 433, 6902, 53247, 21495},
    {36285, 40543, 4723, 400, 268, 200, 67, 1, 0, 6144, 24576, 39108, 65082, 49966, 26755, 12288, 4096, 10240, 30721, 49576, 65212, 37288, 18497, 14336, 28673, 43399, 64887, 65082, 64757, 64692, 64724, 64724, 64789, 64854, 64919, 64984, 65082, 65179, 64854, 49673, 30883, 18432, 4096, 0, 0, 2048, 8192, 20545, 35109, 64952, 64432, 35012, 16384, 8192, 12288, 24642, 39238, 64952, 64367, 38947, 34816, 38979, 64660, 64822, 39173, 26624, 10240, 10240, 28673, 43302, 65114, 52014, 30883, 16384, 4096, 0, 0, 0, 0, 0, 0, 4096, 18432, 32932, 56272, 64951, 37028, 22528, 6144, 4096, 12288, 30883, 45350, 47366, 43302, 24609, 10240, 2048, 0, 0, 0, 0, 0, 2048, 4096, 14336, 30883, 43334, 45383, 37093, 16384, 6144, 6144, 16449, 30949, 64984, 62189, 36995, 14336, 2048, 0, 2048, 10240, 37288, 65277, 33256, 4096, 0, 0, 0, 0, 0, 4096, 6144, 6144, 6144, 6144, 6144, 6144, 8192, 8192, 10240, 24869, 65212, 47820, 18432, 4096, 0, 0, 0, 0, 0, 0, 1, 36, 169, 400, 6902, 53247, 21527},
    {36285, 40543, 4723, 368, 268, 200, 67, 0, 0, 6144, 24576, 41157, 65082, 49966, 24707, 12288, 4096, 10240, 30721, 51624, 65212, 37255, 20513, 18432, 34980, 64464, 64920, 43367, 37028, 37028, 34980, 37028, 37028, 37060, 37060, 39108, 41156, 47528, 64659, 65114, 45512, 26690, 10240, 2048, 0, 4096, 18432, 30851, 52046, 65147, 45415, 28673, 10240, 6144, 18432, 30883, 56240, 65049, 47334, 43009, 38912, 38914, 47593, 65179, 51722, 30786, 12288, 8192, 20480, 32931, 58385, 64984, 41221, 24576, 8192, 0, 0, 0, 0, 0, 0, 6144, 24576, 39141, 64822, 60498, 30851, 18432, 6144, 8192, 24577, 41286, 64659, 65114, 60303, 35044, 16385, 6144, 2048, 0, 0, 0, 0, 2048, 14336, 26722, 43561, 65017, 65017, 41417, 24674, 14336, 6144, 12288, 28803, 64627, 64627, 39141, 16384, 4096, 0, 2048, 10240, 37320, 65277, 33256, 4096, 0, 0, 0, 0, 0, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 4096, 6144, 20676, 65211, 49868, 18432, 4096, 0, 0, 0, 0, 0, 0, 1, 36, 169, 400, 6870, 53247, 21527},
    {36285, 40575, 4723, 368, 268, 200, 35, 0, 0, 6144, 24576, 41157, 65050, 49966, 24707, 12288, 4096, 10240, 30721, 51657, 65179, 37223, 20481, 20480, 39141, 64854, 62319, 36931, 32768, 28672, 28672, 26624, 26624, 24576, 24576, 24576, 26624, 30753, 37125, 64757, 64757, 32964, 12288, 2048, 2048, 8192, 24577, 39141, 65017, 58353, 35012, 20480, 6144, 6144, 20480, 37125, 64854, 64627, 49511, 51495, 51494, 47431, 43399, 64789, 64594, 37060, 12288, 6144, 12288, 24642, 37255, 65115, 58061, 32867, 12288, 2048, 0, 0, 0, 0, 0, 8192, 28672, 45318, 65147, 45804, 24674, 12288, 6144, 10240, 30786, 58125, 65114, 56272, 65179, 41416, 24642, 10240, 2048, 0, 0, 0, 0, 6144, 22528, 37092, 64789, 64854, 64984, 58353, 32931, 20480, 6144, 10240, 28738, 56013, 64985, 39238, 20545, 8192, 2048, 2048, 10240, 37320, 65276, 33224, 4096, 0, 0, 0, 2048, 4096, 6144, 8192, 8192, 6144, 6144, 6144, 6144, 6144, 6144, 10240, 39596, 65244, 39401, 14336, 4096, 0, 0, 0, 0, 0, 0, 1, 36, 169, 400, 6870, 53247, 21527},
    {36285, 40575, 4724, 400, 268, 200, 35, 0, 0, 6144, 24576, 41157, 65049, 49966, 24707, 12288, 4096, 10240, 30721, 53737, 65179, 35174, 20481, 18432, 37093, 64920, 62254, 39011, 36865, 32768, 30720, 26624, 20480, 14336, 8192, 8192, 8192, 18432, 34915, 58028, 65017, 33029, 14369, 6144, 6144, 14336, 30819, 58125, 65082, 41351, 24642, 10240, 2048, 4096, 18432, 32964, 56142, 65147, 65212, 65180, 65212, 65212, 65244, 65212, 64432, 35012, 10240, 2048, 6144, 18433, 32900, 64465, 64919, 37190, 20513, 8192, 2048, 0, 0, 0, 0, 10240, 32802, 53770, 65179, 35207, 20546, 10240, 6144, 14337, 32964, 64692, 64530, 43302, 64952, 58320, 32931, 18432, 4096, 0, 0, 0, 2048, 10240, 28705, 49608, 65179, 45642, 60498, 64789, 39076, 24576, 8192, 8192, 26625, 43335, 65212, 43562, 26722, 12288, 4096, 2048, 10240, 37320, 65276, 33224, 4096, 0, 0, 2048, 10240, 28933, 37385, 43756, 41676, 41676, 41676, 41675, 41643, 39563, 39563, 41644, 64984, 62741, 22756, 6144, 2048, 0, 0, 0, 0, 0, 0, 1, 36, 169, 432, 6870, 53215, 23607},
    {34204, 40575, 4724, 368, 268, 168, 35, 0, 0, 6144, 24576, 41157, 65049, 49966, 26723, 12288, 4096, 10240, 30721, 53770, 65179, 35174, 20481, 18432, 32899, 64529, 64919, 47528, 41157, 39076, 39076, 35012, 28803, 18433, 6144, 2048, 2048, 12288, 32834, 57963, 65049, 33061, 14369, 6144, 10240, 22593, 37157, 64984, 64400, 34980, 16384, 4096, 0, 2048, 10240, 22561, 32964, 39271, 37288, 37320, 37352, 39433, 39465, 39433, 35044, 22529, 6144, 2048, 2048, 10240, 28706, 43335, 65147, 49933, 28835, 14336, 4096, 0, 0, 0, 2048, 12288, 34947, 64400, 64854, 30948, 16417, 8192, 8192, 18497, 35110, 65050, 62060, 43107, 58255, 65017, 41221, 26625, 8192, 0, 0, 0, 4096, 14336, 32931, 64497, 64822, 39173, 51949, 65082, 43237, 28673, 8192, 6144, 22528, 34980, 65017, 54159, 32964, 18432, 4096, 2048, 10240, 37352, 65277, 33224, 4096, 0, 0, 4096, 20514, 47788, 65147, 65147, 65179, 65211, 65212, 65212, 65244, 65244, 65277, 65244, 64886, 29128, 8192, 2048, 0, 0, 0, 0, 0, 0, 0, 1, 35, 169, 400, 6870, 53215, 23608},
    {34204, 40575, 4724, 368, 268, 200, 35, 1, 0, 6144, 24576, 39109, 65049, 49998, 26755, 14336, 4096, 10240, 30721, 53802, 65147, 35142, 22561, 16384, 26625, 41351, 64854, 65180, 65082, 64985, 64919, 64757, 54029, 30916, 12288, 4096, 4096, 12288, 32834, 60141, 64984, 35077, 16417, 10240, 20480, 30851, 54126, 65114, 43335, 26625, 10240, 2048, 0, 0, 4096, 10240, 16385, 20545, 20546, 22594, 22594, 20546, 20545, 18497, 16384, 8192, 2048, 0, 0, 6144, 20480, 32932, 60498, 64919, 39173, 24576, 6144, 0, 0, 0, 4096, 16417, 37125, 64757, 64465, 30851, 12288, 6144, 10240, 22626, 41546, 65180, 49447, 38913, 43335, 65147, 55915, 30786, 12288, 2048, 0, 2048, 10240, 22593, 37223, 65082, 60109, 38979, 43432, 65180, 49576, 30721, 10240, 6144, 18432, 28803, 62643, 64756, 39141, 24576, 6144, 2048, 10240, 37352, 65277, 33224, 4096, 0, 0, 4096, 22659, 64822, 64692, 30948, 26820, 26852, 26853, 26885, 28965, 28965, 28966, 28998, 18595, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 168, 400, 6838, 51167, 23640},
    {34172, 42655, 4724, 368, 268, 200, 35, 1, 0, 6144, 22528, 37060, 65049, 52046, 28803, 16384, 4096, 8192, 26625, 49641, 65212, 41448, 28770, 22528, 22528, 28673, 37093, 43465, 51884, 54062, 56240, 64724, 65050, 39206, 16417, 4096, 4096, 12288, 32867, 64400, 64790, 35045, 16384, 12288, 26625, 39173, 65049, 56272, 34980, 20480, 6144, 2048, 4096, 6144, 8192, 10240, 12288, 16384, 16384, 18432, 18432, 18432, 16384, 16384, 12288, 10240, 8192, 4096, 2048, 4096, 14336, 24642, 39368, 65147, 55948, 32867, 12288, 2048, 0, 2048, 8192, 20545, 39271, 65115, 53835, 28705, 10240, 6144, 16384, 28803, 54224, 64952, 43172, 34816, 39043, 64659, 64692, 37093, 20481, 10240, 8192, 10240, 18432, 30851, 52046, 65082, 43269, 34817, 39109, 64985, 60141, 32835, 14336, 6144, 14336, 24674, 47917, 65082, 45318, 28672, 8192, 4096, 12288, 39401, 65277, 31176, 2048, 0, 0, 2048, 20643, 64919, 56272, 20480, 10240, 8192, 8192, 8192, 8192, 8192, 8192, 8192, 4096, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 168, 400, 6837, 51167, 23640},
    {34172, 42687, 4724, 368, 268, 200, 67, 1, 0, 4096, 20480, 35012, 64919, 58353, 34947, 20480, 6144, 6144, 20480, 37125, 64984, 64724, 39141, 34817, 28672, 26624, 26624, 30720, 36865, 38979, 37126, 64595, 64822, 37093, 16385, 6144, 6144, 14336, 32932, 64627, 64594, 34980, 18432, 18432, 32867, 58158, 65049, 39271, 22594, 10240, 4096, 6144, 16384, 26625, 30786, 32834, 34850, 34818, 34818, 34817, 34817, 34817, 34817, 34817, 32769, 30721, 28673, 20480, 8192, 4096, 6144, 18465, 32931, 64530, 64854, 37158, 20513, 8192, 2048, 2048, 12288, 26755, 47788, 65179, 41221, 26625, 8192, 6144, 20480, 35012, 64789, 62579, 34979, 30720, 34817, 51786, 65179, 41449, 34915, 30720, 26624, 28672, 34818, 43270, 64984, 60433, 34947, 28672, 36995, 64692, 64530, 34980, 16384, 6144, 10240, 20546, 35239, 65212, 51657, 32802, 10240, 4096, 12288, 39401, 65277, 31176, 2048, 0, 0, 2048, 22691, 64951, 54192, 16384, 6144, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 168, 400, 6837, 51135, 25720},
    {34172, 42687, 4724, 368, 268, 200, 67, 1, 0, 4096, 14336, 28835, 58417, 64919, 41189, 26625, 10240, 6144, 14336, 24674, 43594, 65082, 64660, 47495, 37028, 32867, 30786, 32834, 39011, 45285, 64497, 65147, 53867, 30851, 12288, 6144, 10240, 20513, 35110, 64920, 62286, 34882, 22528, 24609, 37158, 64985, 62319, 34980, 14336, 6144, 6144, 12288, 28803, 45447, 53834, 57996, 57996, 55915, 55850, 55850, 53770, 53769, 53737, 53737, 51656, 49576, 45383, 37125, 18465, 8192, 4096, 10240, 28706, 45447, 65179, 47788, 28803, 14336, 4096, 4096, 18432, 32964, 58385, 64887, 34980, 22528, 6144, 6144, 24576, 39141, 65114, 49933, 28770, 20480, 26624, 35012, 64854, 65016, 51721, 43172, 39076, 41124, 47398, 64595, 64985, 39303, 24609, 20480, 32802, 62319, 64887, 35077, 18465, 8192, 8192, 16417, 30948, 64952, 64302, 37028, 12288, 6144, 12288, 39433, 65277, 31176, 2048, 0, 0, 4096, 22691, 64984, 54192, 16384, 6144, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 35, 168, 400, 6837, 51135, 25720},
    {32091, 42687, 4724, 368, 268, 200, 68, 1, 0, 2048, 8192, 20546, 41416, 65115, 58093, 34980, 18432, 8192, 8192, 14336, 28803, 47625, 64822, 65147, 64756, 54127, 49901, 54094, 60433, 65049, 64985, 51819, 35012, 20480, 6144, 6144, 18432, 30818, 47788, 65179, 47528, 32768, 28672, 32867, 54159, 65082, 43334, 26625, 8192, 4096, 12288, 22594, 41481, 65114, 65115, 65049, 65082, 65114, 65114, 65115, 65147, 65179, 65179, 65212, 65212, 65212, 65179, 54127, 32964, 20480, 6144, 6144, 20480, 32931, 62611, 64854, 39141, 24576, 8192, 8192, 24576, 39141, 64886, 58417, 28803, 16384, 6144, 8192, 28673, 49641, 65179, 39336, 22594, 14336, 14336, 24642, 39238, 64659, 65179, 65017, 64920, 64952, 65147, 64984, 47658, 28771, 14336, 12288, 30721, 51754, 65147, 37255, 22594, 10240, 6144, 12288, 28803, 64562, 64660, 37125, 16385, 8192, 14336, 39433, 65276, 31175, 2048, 0, 0, 4096, 24739, 64919, 56272, 24610, 18432, 18432, 18432, 18432, 18432, 16384, 16384, 16384, 16384, 16384, 12288, 8192, 2048, 0, 0, 0, 0, 0, 0, 0, 3, 136, 400, 6837, 51103, 25720},
    {34140, 42687, 4724, 401, 268, 201, 68, 1, 0, 0, 4096, 14336, 30883, 60303, 65049, 43432, 30851, 18432, 8192, 8192, 18432, 26690, 35077, 47788, 64724, 65017, 65114, 65017, 64854, 54159, 39238, 30851, 20481, 12288, 8192, 14336, 28706, 41286, 64952, 64627, 34980, 28672, 32768, 41221, 65049, 56240, 32932, 20480, 6144, 6144, 20480, 32964, 64692, 64854, 37222, 32997, 33029, 30981, 30981, 33061, 33061, 33094, 33094, 33126, 35174, 35207, 54159, 65179, 45447, 26657, 10240, 6144, 14336, 24674, 41449, 65179, 53835, 32834, 12288, 10240, 30720, 45382, 65179, 45772, 24674, 12288, 6144, 10240, 30818, 62287, 64887, 35077, 18497, 8192, 6144, 12288, 24609, 35012, 43562, 49998, 54191, 54191, 45772, 35142, 26690, 18432, 6144, 8192, 26624, 41254, 65179, 45740, 26723, 14336, 6144, 10240, 28706, 53900, 65049, 39271, 20545, 12288, 14336, 39433, 65276, 31176, 2048, 0, 0, 4096, 22659, 58353, 65114, 54224, 52079, 49966, 49966, 49933, 47820, 47788, 45707, 45675, 43562, 41514, 35240, 24772, 10240, 4096, 0, 0, 0, 0, 0, 0, 3, 136, 368, 6837, 49055, 25720},
    {32091, 44767, 4724, 401, 268, 201, 100, 1, 0, 0, 2048, 8192, 22529, 35077, 64887, 64854, 43334, 32834, 20480, 10240, 8192, 12288, 18465, 26755, 32964, 37093, 41189, 39141, 35012, 28803, 22594, 16384, 10240, 10240, 18432, 26657, 37158, 64594, 65017, 41286, 26657, 22528, 32834, 60239, 65017, 39271, 22593, 10240, 6144, 10240, 28705, 47528, 65179, 47755, 30851, 24642, 20545, 20545, 20578, 20546, 20578, 20578, 20578, 20545, 22593, 26722, 37125, 64886, 64529, 32964, 16384, 6144, 8192, 18433, 32932, 64627, 64757, 35077, 16384, 14336, 34850, 55915, 65179, 35175, 20546, 8192, 6144, 14336, 32964, 64724, 64530, 34980, 14336, 4096, 2048, 4096, 8192, 14336, 20546, 24707, 26755, 26755, 22626, 16449, 12288, 6144, 2048, 6144, 22528, 35012, 64920, 56272, 30851, 18432, 6144, 8192, 26625, 43302, 65180, 45642, 26755, 16384, 14336, 39465, 65277, 31143, 2048, 0, 0, 2048, 14336, 35207, 56402, 64984, 65049, 65082, 65114, 65179, 65211, 65212, 65244, 65277, 65277, 65309, 65244, 62773, 35272, 12288, 2048, 0, 0, 0, 0, 0, 3, 136, 367, 6837, 49023, 27801},
    {32059, 44767, 4757, 401, 268, 233, 100, 1, 0, 0, 0, 4096, 14336, 24642, 37287, 64952, 64822, 45447, 30884, 20481, 14336, 10240, 12288, 16384, 22528, 24576, 26624, 26624, 24576, 20480, 14336, 12288, 14336, 20481, 30818, 39206, 64595, 65114, 49738, 30786, 18432, 18432, 34980, 64725, 64497, 34980, 18432, 10240, 10240, 20480, 34980, 64530, 64854, 35077, 22529, 12288, 8192, 8192, 8192, 8192, 8192, 8192, 8192, 8192, 10240, 18432, 32866, 55979, 65147, 41416, 24642, 14336, 8192, 14336, 32802, 49576, 65180, 41481, 20513, 18432, 36995, 64465, 64790, 32964, 18432, 10240, 10240, 20513, 35110, 65082, 58028, 30786, 12288, 2048, 0, 0, 2048, 4096, 8192, 10240, 12288, 12288, 10240, 6144, 2048, 0, 0, 4096, 16384, 30883, 62579, 64757, 37060, 22528, 8192, 8192, 24576, 36995, 64952, 56272, 30916, 20480, 16384, 39465, 65277, 31143, 4096, 0, 0, 2048, 6144, 12288, 18530, 18563, 18563, 18595, 18595, 18627, 20708, 20708, 20741, 22821, 22821, 24902, 39628, 65016, 64724, 22659, 4096, 0, 0, 0, 0, 0, 3, 135, 367, 6837, 49023, 27833},
    {32059, 44799, 4757, 401, 300, 233, 101, 1, 0, 0, 0, 0, 4096, 14336, 26722, 41351, 64855, 65049, 51949, 35077, 32802, 26624, 22528, 18432, 16384, 16384, 16384, 16384, 16384, 18432, 22528, 26624, 32802, 37092, 43562, 64919, 65049, 47625, 30851, 20480, 10240, 16384, 35012, 64692, 64497, 36995, 24576, 20480, 26624, 32834, 43464, 65147, 55980, 30786, 14336, 4096, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 8192, 24577, 39141, 64919, 62546, 37027, 26624, 16384, 18432, 34817, 45285, 65179, 43626, 20513, 18432, 37028, 64660, 64530, 34947, 24576, 18432, 22528, 28738, 43594, 65212, 45383, 28673, 8192, 0, 0, 0, 0, 0, 2048, 2048, 2048, 2048, 2048, 0, 0, 0, 0, 2048, 12288, 24707, 49998, 65049, 43205, 30720, 12288, 10240, 24576, 34915, 62547, 64789, 35044, 22528, 16384, 39465, 65276, 31143, 4096, 0, 0, 0, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 6144, 14336, 47917, 65114, 26917, 4096, 0, 0, 0, 0, 0, 34, 135, 367, 6837, 48991, 27833},
    {32059, 44799, 4756, 401, 301, 233, 101, 1, 0, 0, 0, 0, 0, 6144, 16384, 26658, 35110, 60433, 65147, 64789, 53770, 41157, 32932, 28803, 24707, 22658, 22626, 22626, 24674, 28803, 34980, 41189, 51722, 64659, 65179, 64724, 41351, 30819, 18433, 10240, 6144, 12288, 30851, 64432, 64887, 41286, 32899, 30786, 36996, 45415, 64822, 64855, 37093, 24577, 6144, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 16384, 30851, 49900, 65179, 45448, 34915, 26690, 26690, 36995, 55948, 65147, 39335, 18465, 16384, 32867, 64497, 64887, 41286, 32932, 28738, 30786, 37093, 64692, 64887, 39140, 22528, 6144, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 8192, 20546, 39368, 65179, 53802, 34915, 24609, 22529, 28673, 34980, 62579, 64789, 32996, 20480, 14336, 33224, 65277, 37450, 12288, 6144, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 6144, 12288, 45772, 65179, 26917, 4096, 0, 0, 0, 0, 0, 34, 135, 367, 6805, 48991, 27833},
    {32058, 44799, 4756, 401, 301, 233, 133, 2, 0, 0, 0, 0, 0, 2048, 4096, 12288, 22561, 32964, 47528, 64659, 65179, 64919, 60498, 54159, 49998, 47853, 43659, 41514, 45739, 49966, 60498, 64984, 65179, 64724, 51786, 35012, 24642, 14336, 6144, 2048, 2048, 8192, 26657, 43399, 64984, 64984, 54127, 49933, 60498, 65082, 64919, 43432, 24642, 12288, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 8192, 20513, 37093, 64789, 64984, 54127, 43627, 43594, 52014, 65114, 64497, 32932, 14336, 10240, 24577, 41319, 65017, 64984, 56272, 47853, 52046, 64854, 65147, 47658, 28803, 14336, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 14336, 35012, 64562, 65049, 47787, 37190, 35077, 37158, 51916, 65147, 54062, 28835, 14336, 8192, 20676, 65016, 62741, 35207, 26755, 22529, 20513, 20481, 18433, 18433, 18433, 18433, 18433, 18433, 16384, 18432, 16384, 16384, 16384, 16384, 18432, 24707, 60563, 64887, 24772, 4096, 0, 0, 0, 0, 0, 2, 135, 367, 6805, 48991, 27866},
    {32026, 44831, 4756, 401, 301, 233, 101, 2, 0, 0, 0, 0, 0, 0, 0, 4096, 12288, 20480, 28706, 32932, 41481, 56305, 64789, 64952, 65017, 65114, 65179, 65212, 65179, 65049, 64854, 54192, 39401, 32996, 28738, 22528, 12288, 6144, 2048, 0, 0, 4096, 16384, 28771, 37255, 60498, 65017, 65114, 64854, 52046, 37126, 26690, 14336, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 12288, 26658, 39238, 60498, 65049, 65212, 65212, 65147, 58418, 35045, 22529, 8192, 6144, 16384, 26722, 37255, 56305, 64952, 65147, 65082, 62579, 45512, 32932, 18432, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 8192, 24609, 37125, 62611, 65212, 65049, 64887, 65050, 65147, 62449, 35110, 18465, 8192, 4096, 6144, 29225, 65146, 65049, 62514, 56272, 56272, 56272, 54192, 54192, 54192, 54192, 54159, 54159, 54159, 52079, 52079, 52079, 52078, 52046, 49998, 58482, 65244, 45804, 14337, 2048, 0, 0, 0, 0, 0, 2, 135, 367, 6804, 46911, 29914},
    {29946, 44831, 6805, 401, 300, 233, 101, 2, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 6144, 10240, 16385, 22594, 28803, 34980, 39108, 39108, 41156, 43237, 43302, 41189, 39108, 35012, 28803, 22626, 16417, 10240, 6144, 2048, 0, 0, 0, 0, 2048, 6144, 14336, 22594, 30883, 37060, 39141, 35012, 26755, 20513, 12288, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 14336, 22594, 32932, 39141, 41222, 43302, 39141, 30884, 20513, 10240, 2048, 2048, 6144, 14336, 22594, 30883, 37060, 39141, 39141, 35012, 26722, 16384, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 12288, 22593, 32932, 43367, 58061, 62254, 58060, 43367, 32964, 20513, 8192, 2048, 2048, 2048, 4096, 20741, 48080, 64854, 64951, 64984, 64984, 64984, 64984, 65016, 65016, 65016, 65016, 65049, 65017, 65049, 65049, 65049, 65049, 65049, 64919, 45869, 18563, 4096, 2048, 0, 0, 0, 0, 0, 2, 135, 367, 4756, 46911, 29914},
    {29946, 46879, 6805, 401, 268, 200, 101, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 4096, 10240, 16384, 20480, 24576, 26624, 28672, 28672, 28672, 26624, 24576, 20480, 16384, 10240, 4096, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 10240, 16384, 20480, 22528, 20480, 14336, 8192, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 10240, 18432, 24576, 26624, 26625, 24576, 18432, 8192, 2048, 0, 0, 0, 4096, 10240, 16384, 22528, 24576, 24576, 20480, 12288, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 4096, 10240, 20480, 26625, 28706, 30786, 30754, 26625, 18432, 10240, 4096, 0, 0, 2048, 2048, 6144, 10240, 14337, 18498, 18530, 18530, 18530, 18530, 18530, 18562, 18563, 18563, 18563, 18595, 18595, 20643, 20675, 20675, 20643, 18530, 12288, 4096, 2048, 0, 0, 0, 0, 0, 0, 2, 102, 334, 4756, 46911, 29946},
    {29946, 46911, 6805, 401, 268, 200, 68, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 4096, 4096, 6144, 8192, 10240, 10240, 10240, 10240, 8192, 6144, 4096, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 4096, 4096, 4096, 4096, 4096, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 4096, 6144, 6144, 6144, 6144, 4096, 2048, 0, 0, 0, 0, 0, 2048, 4096, 6144, 6144, 6144, 4096, 2048, 2048, 2048, 2048, 2048, 4096, 2048, 2048, 2048, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 4096, 4096, 6144, 8192, 10240, 8192, 6144, 4096, 2048, 2048, 2048, 4096, 4096, 4096, 6144, 6144, 6144, 8192, 8192, 6144, 6144, 6144, 6144, 6144, 4096, 4096, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 2, 102, 334, 4724, 46879, 29946},
    {29946, 46911, 6805, 401, 268, 168, 68, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 2048, 2048, 4096, 8192, 12321, 14369, 14401, 12321, 10273, 8192, 6144, 4096, 2048, 2048, 2048, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 2048, 4096, 4096, 2048, 2048, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 2048, 2048, 2048, 2048, 2048, 0, 0, 0, 0, 0, 0, 2048, 6144, 10240, 12288, 14336, 14337, 14336, 14336, 14336, 14336, 14336, 14336, 14336, 14337, 14337, 14337, 14336, 14336, 14336, 14336, 12288, 12288, 10240, 8192, 4096, 2048, 2048, 0, 0, 2048, 2048, 4096, 8192, 12288, 14336, 16385, 16385, 16385, 16417, 16385, 16417, 16417, 16417, 14336, 14336, 12288, 12288, 10240, 8192, 6144, 4096, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 102, 334, 4724, 46847, 29978},
    {27866, 46911, 4757, 400, 267, 167, 67, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 4096, 8192, 14369, 22756, 35370, 48015, 52241, 56402, 58515, 58547, 54322, 50096, 43821, 33288, 22756, 16417, 10240, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 6144, 14337, 22659, 22724, 20611, 14336, 8192, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 8192, 14336, 16417, 14337, 12288, 8192, 2048, 0, 0, 0, 0, 4096, 12288, 20480, 30754, 34947, 35012, 35012, 35012, 35012, 35012, 35012, 35012, 35012, 35012, 35012, 35012, 35012, 32964, 32964, 32964, 32964, 32964, 32932, 32867, 26625, 16384, 8192, 2048, 0, 0, 2048, 10240, 20480, 28673, 34914, 34947, 34980, 34980, 34980, 34980, 34980, 34980, 34980, 34980, 32900, 32899, 32867, 32834, 30722, 28673, 24576, 18432, 12288, 4096, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 102, 334, 4724, 44799, 32026},
    {27866, 46911, 4757, 400, 267, 167, 67, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 4096, 6144, 16514, 37482, 60596, 65147, 65277, 65082, 64951, 64854, 62741, 62709, 64887, 65017, 65146, 65277, 65049, 60563, 39531, 20643, 8192, 4096, 2048, 0, 0, 0, 0, 0, 2048, 8192, 26885, 49998, 64886, 65016, 64854, 52111, 26918, 10240, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 12321, 33256, 54289, 62708, 58547, 50063, 26982, 10240, 2048, 0, 0, 4096, 12288, 24610, 37060, 49609, 64400, 64627, 64659, 64627, 64627, 64627, 64627, 64627, 64627, 64627, 64627, 64627, 64627, 64627, 64627, 64627, 64627, 64595, 64562, 58093, 43367, 30883, 16384, 4096, 0, 2048, 8192, 22561, 34980, 45415, 60141, 64530, 64562, 64562, 64594, 64595, 64562, 64562, 64562, 64530, 64497, 64497, 64367, 60108, 53802, 47463, 39141, 32964, 24642, 16384, 10240, 4096, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 102, 334, 4724, 46847, 32026},
    {27866, 48991, 6837, 400, 267, 167, 67, 1, 0, 0, 0, 0, 0, 0, 0, 2048, 8192, 16417, 35402, 64984, 65244, 64822, 52014, 37320, 28901, 22659, 20610, 20611, 20611, 22691, 24772, 28933, 39433, 54159, 64822, 65244, 62774, 33321, 14369, 6144, 2048, 0, 0, 0, 0, 4096, 26885, 62611, 65179, 58482, 52111, 58450, 65147, 65049, 35305, 12288, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 14337, 45869, 65212, 64984, 62643, 64757, 65114, 65147, 37482, 10240, 2048, 2048, 12288, 24610, 37158, 60336, 65179, 64887, 64562, 64562, 64562, 64562, 64530, 64530, 64562, 64562, 64562, 64562, 64562, 64562, 64562, 64562, 64530, 64530, 64530, 64660, 65049, 65081, 47658, 26690, 8192, 2048, 6144, 16385, 35077, 56045, 65147, 65049, 64692, 64627, 64627, 64627, 64595, 64595, 64627, 64659, 64660, 64660, 64692, 64822, 64984, 65147, 65212, 64887, 56208, 41416, 37060, 28705, 18432, 10240, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 102, 334, 4724, 44799, 32059},
    {27865, 48991, 6837, 400, 267, 167, 35, 0, 0, 0, 0, 0, 0, 0, 2048, 6144, 22692, 50095, 65244, 62708, 39466, 26852, 20481, 12288, 10240, 8192, 8192, 8192, 8192, 8192, 8192, 10240, 12288, 16384, 24740, 39563, 64887, 65244, 54354, 22756, 6144, 2048, 0, 0, 2048, 10240, 43659, 65212, 41546, 20481, 16384, 20546, 31111, 64919, 64887, 24740, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 6144, 35304, 65179, 54257, 28933, 20546, 22659, 33094, 62676, 64984, 22691, 4096, 6144, 20480, 35012, 58255, 65147, 49868, 32997, 30851, 30851, 30851, 30851, 32899, 32899, 32867, 32867, 32867, 32899, 32899, 32899, 32899, 32899, 32899, 34947, 34947, 34980, 37158, 64724, 64790, 35012, 12288, 4096, 10240, 22626, 43465, 65179, 58223, 35077, 32964, 30884, 30883, 30883, 32932, 32899, 32931, 32932, 32932, 34980, 35012, 35045, 35077, 35142, 41546, 58418, 65017, 65114, 64400, 45382, 32899, 20513, 10240, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 2, 102, 334, 4723, 44767, 32059},
    {27833, 49023, 6837, 400, 267, 167, 35, 0, 0, 0, 0, 0, 0, 2048, 10240, 26852, 58515, 65244, 50030, 26852, 12288, 4096, 4096, 4096, 2048, 4096, 6144, 6144, 6144, 4096, 4096, 4096, 4096, 4096, 4096, 10240, 26852, 49933, 65049, 64984, 24869, 8192, 2048, 0, 2048, 14434, 52176, 64952, 20578, 8192, 6144, 6144, 12288, 39466, 65244, 39563, 8192, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 8192, 20611, 58547, 64952, 26917, 8192, 6144, 6144, 14336, 41481, 65276, 35337, 4096, 8192, 28705, 47496, 65179, 51981, 30851, 22529, 14336, 12288, 12288, 14336, 14336, 16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384, 18432, 18432, 18432, 18432, 24576, 34915, 58093, 64920, 37158, 12288, 6144, 14336, 26787, 51982, 65050, 39140, 28673, 16384, 12288, 12288, 12288, 14336, 14336, 14336, 16384, 16384, 16384, 16384, 16384, 18433, 20513, 22626, 28803, 37061, 56012, 64984, 65082, 54030, 32996, 24609, 12288, 4096, 0, 0, 0, 0, 0, 0, 0, 2, 102, 334, 4723, 44767, 32059},
    {27801, 49023, 6837, 400, 235, 167, 35, 0, 0, 0, 0, 0, 2048, 6144, 26885, 62676, 65179, 39595, 20514, 10240, 4096, 2048, 2048, 4096, 4096, 6144, 12321, 14402, 12353, 10273, 6144, 4096, 2048, 2048, 2048, 4096, 10240, 22594, 37353, 64984, 65016, 29063, 6144, 2048, 4096, 12353, 50096, 65049, 22659, 8192, 2048, 2048, 8192, 22691, 64854, 60693, 20643, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 12288, 35272, 65244, 43821, 10240, 4096, 0, 4096, 14336, 39368, 65277, 37450, 6144, 10240, 30786, 60141, 65017, 35109, 22561, 12288, 6144, 4096, 8192, 10240, 16384, 18432, 20480, 20480, 20480, 20480, 20480, 20480, 20480, 20480, 20480, 20480, 22528, 26625, 34915, 62287, 64854, 35109, 12288, 6144, 16384, 30883, 56305, 64854, 39076, 24576, 8192, 4096, 6144, 10240, 14336, 16384, 18432, 18432, 18432, 18432, 18432, 16384, 16384, 16384, 16384, 20480, 26624, 30786, 35110, 56207, 65147, 56110, 37060, 24609, 8192, 2048, 0, 0, 0, 0, 0, 0, 2, 102, 334, 4691, 44735, 32092},
    {27801, 49023, 6837, 401, 235, 167, 35, 0, 0, 0, 0, 2048, 6144, 18530, 54289, 65179, 39498, 14337, 6144, 2048, 2048, 4096, 10240, 18433, 28966, 41676, 52241, 56467, 56467, 52241, 45902, 31176, 20578, 8192, 4096, 2048, 2048, 6144, 18433, 35240, 64887, 65081, 26982, 6144, 4096, 8192, 41644, 65244, 33126, 10240, 2048, 0, 2048, 8192, 41676, 65244, 37352, 10240, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 20546, 58418, 65016, 24772, 4096, 0, 0, 4096, 24675, 60466, 64984, 22724, 6144, 12288, 30851, 64530, 64660, 37093, 18433, 6144, 2048, 6144, 16384, 28673, 36995, 39076, 39076, 39076, 39076, 39076, 39076, 37028, 39076, 39076, 39044, 39044, 39044, 39076, 39238, 64887, 64562, 32964, 10240, 6144, 16384, 30883, 58385, 64854, 37028, 22528, 6144, 4096, 12288, 22561, 28803, 32964, 32964, 32964, 32964, 32932, 34948, 34915, 34850, 30720, 22528, 16384, 12288, 14336, 22561, 32964, 54061, 65147, 58093, 34980, 18432, 6144, 2048, 0, 0, 0, 0, 0, 1, 102, 301, 4723, 44735, 34172},
    {25752, 49055, 6837, 401, 235, 167, 35, 0, 0, 0, 0, 2048, 14336, 41643, 65244, 45869, 16385, 6144, 2048, 0, 4096, 10240, 26885, 52078, 65081, 65212, 64952, 64789, 64789, 64919, 65147, 65244, 64789, 35369, 12289, 4096, 0, 2048, 8192, 16385, 35240, 64919, 62774, 16482, 4096, 6144, 28966, 65082, 54224, 18433, 4096, 0, 0, 4096, 24804, 64984, 58418, 20546, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 8192, 33191, 65179, 47982, 12288, 2048, 0, 2048, 8192, 35272, 65179, 47950, 12288, 6144, 12288, 28803, 64594, 64595, 37061, 16385, 4096, 2048, 8192, 28738, 43302, 57996, 64497, 64497, 64497, 64497, 64497, 64465, 64465, 64433, 64432, 64432, 64400, 64400, 64432, 64854, 65114, 41351, 24609, 6144, 6144, 16384, 30883, 58417, 64854, 34980, 20480, 6144, 6144, 20480, 35012, 64594, 64757, 64789, 64790, 64757, 64692, 64595, 64367, 55915, 43270, 32931, 22561, 12288, 8192, 12288, 20481, 32932, 60303, 65082, 43399, 28770, 14336, 4096, 0, 0, 0, 0, 0, 1, 102, 301, 4723, 42655, 34172},
    {25753, 51103, 6837, 401, 235, 167, 35, 0, 0, 0, 0, 4096, 31111, 65082, 60531, 22659, 6144, 2048, 2048, 4096, 16384, 35239, 64822, 65211, 56337, 33126, 22659, 20578, 20611, 22691, 28901, 45707, 64790, 65212, 50063, 16450, 4096, 2048, 2048, 6144, 20481, 43594, 65244, 41741, 8192, 6144, 18433, 54224, 65081, 26950, 4096, 0, 0, 2048, 14336, 49998, 65147, 31111, 6144, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 6144, 18498, 54289, 65082, 24837, 6144, 2048, 2048, 6144, 20579, 54321, 65049, 22756, 8192, 4096, 12288, 28803, 64594, 64562, 37060, 14337, 4096, 4096, 14336, 32964, 64432, 65115, 64757, 64692, 64724, 64724, 64724, 64756, 64757, 64757, 64757, 64789, 64789, 64790, 64822, 64594, 41416, 24674, 12288, 2048, 4096, 16384, 30883, 58418, 64822, 34980, 20480, 6144, 8192, 26625, 45447, 65244, 62579, 64432, 64367, 64432, 64465, 64595, 64854, 65147, 65082, 56142, 35077, 24610, 14336, 6144, 10240, 24577, 35045, 64984, 62546, 39141, 24576, 6144, 0, 0, 0, 0, 0, 2, 102, 302, 4691, 42655, 34172},
    {25720, 51103, 6805, 401, 235, 167, 35, 0, 0, 0, 2048, 12321, 48015, 65114, 31046, 10240, 2048, 0, 4096, 14337, 39401, 65017, 65016, 35337, 16385, 10240, 8192, 6144, 6144, 8192, 10240, 16384, 28933, 50030, 65244, 45902, 12321, 2048, 0, 2048, 10240, 24739, 60628, 60693, 16514, 8192, 12288, 33159, 65179, 47983, 12289, 4096, 0, 2048, 8192, 26917, 65114, 50128, 14401, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 10240, 33127, 65146, 50128, 10240, 4096, 0, 2048, 12288, 33159, 65179, 48015, 10240, 4096, 4096, 12288, 30851, 64594, 64562, 37060, 14336, 4096, 6144, 16417, 35045, 64855, 60336, 35012, 28803, 26787, 26787, 26787, 26787, 28835, 28835, 28836, 28836, 28836, 28836, 28835, 26723, 20481, 10240, 4096, 0, 4096, 16384, 30883, 58418, 64822, 34980, 20480, 6144, 8192, 30721, 47496, 65244, 39368, 34980, 34915, 32899, 32932, 32964, 32997, 35175, 54159, 65147, 64562, 34980, 22529, 6144, 4096, 16384, 26722, 45772, 65179, 49576, 32802, 10240, 0, 0, 0, 0, 0, 1, 102, 301, 4691, 42655, 34204},
    {25720, 51103, 6837, 401, 235, 135, 35, 0, 0, 2048, 6144, 20643, 60596, 62741, 16450, 6144, 2048, 2048, 10240, 33126, 64951, 64919, 26982, 8192, 4096, 2048, 2048, 2048, 2048, 2048, 2048, 4096, 8192, 20676, 52176, 65211, 31143, 8192, 2048, 0, 4096, 10240, 52241, 64951, 26853, 10240, 8192, 16482, 56434, 65016, 24804, 8192, 2048, 0, 4096, 12321, 52273, 65049, 28965, 10240, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 20513, 54159, 65114, 26918, 4096, 0, 0, 4096, 20546, 56272, 65082, 24836, 4096, 2048, 4096, 12288, 30852, 64595, 64562, 37060, 14336, 4096, 6144, 18497, 33029, 64984, 58093, 32834, 16384, 10240, 10240, 8192, 8192, 10240, 8192, 8192, 10240, 10240, 10240, 8192, 8192, 6144, 2048, 0, 0, 4096, 16384, 30851, 60498, 64821, 34980, 20480, 6144, 10240, 30721, 49576, 65212, 37288, 22561, 16384, 12288, 10240, 12288, 16384, 22561, 30916, 49803, 65179, 49641, 28706, 10240, 4096, 8192, 18465, 30948, 64951, 62287, 34980, 12288, 2048, 0, 0, 0, 0, 1, 69, 301, 4691, 42655, 36253},
    {25720, 51135, 6837, 401, 235, 167, 35, 0, 0, 2048, 8192, 24772, 64887, 54354, 8192, 2048, 0, 4096, 18432, 49933, 65211, 29063, 8192, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 6144, 28998, 65147, 47917, 12288, 2048, 0, 0, 4096, 48015, 65049, 28933, 12288, 6144, 6144, 33257, 65212, 45805, 14336, 4096, 0, 2048, 4096, 31143, 65212, 49933, 16384, 4096, 0, 0, 0, 0, 0, 0, 0, 2048, 6144, 31046, 65049, 54257, 14336, 2048, 0, 2048, 6144, 33159, 65114, 52143, 12288, 2048, 0, 2048, 12288, 30851, 64627, 64530, 37060, 14336, 4096, 6144, 18498, 35110, 65017, 60076, 30754, 10240, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 0, 0, 0, 0, 0, 4096, 18432, 30883, 60498, 64789, 34980, 20480, 6144, 10240, 30721, 51624, 65179, 37255, 18497, 8192, 4096, 2048, 2048, 6144, 12288, 22528, 37125, 64789, 64529, 30819, 12288, 4096, 4096, 14336, 30851, 64562, 64660, 37093, 14337, 2048, 0, 0, 0, 0, 1, 69, 301, 4690, 42623, 36253},
    {23672, 51167, 6837, 433, 235, 167, 35, 0, 0, 2048, 10240, 26852, 65049, 48047, 6144, 0, 0, 4096, 20546, 58450, 64822, 16450, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 24772, 65049, 52111, 14336, 2048, 0, 0, 4096, 43821, 65146, 30981, 10240, 4096, 4096, 20578, 62643, 64854, 22692, 4096, 0, 0, 4096, 18498, 60563, 64887, 26853, 4096, 0, 0, 0, 0, 0, 0, 2048, 6144, 16417, 47982, 65211, 29030, 8192, 2048, 2048, 6144, 16450, 52176, 65146, 26917, 6144, 2048, 0, 2048, 12288, 30851, 64627, 64562, 35012, 14336, 6144, 8192, 18498, 35110, 65050, 57996, 30754, 10240, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 18432, 30883, 60498, 64789, 34979, 20480, 6144, 10240, 30721, 51657, 65179, 35207, 18497, 6144, 2048, 0, 0, 2048, 6144, 14336, 32932, 64529, 64659, 32932, 14336, 4096, 4096, 12288, 30786, 58061, 65017, 35077, 16417, 4096, 0, 0, 0, 0, 1, 69, 269, 4658, 40543, 36252},
    {23639, 51167, 6869, 401, 235, 167, 35, 0, 0, 2048, 10240, 28900, 65114, 45934, 6144, 0, 0, 4096, 20546, 62644, 60628, 16449, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 24772, 65017, 52143, 14336, 2048, 0, 0, 4096, 41741, 65179, 30981, 10240, 2048, 2048, 12288, 39466, 65244, 39563, 8192, 2048, 0, 2048, 8192, 37385, 65244, 41611, 10240, 2048, 0, 0, 0, 0, 0, 2048, 8192, 28966, 65016, 54386, 14369, 4096, 2048, 2048, 10240, 31078, 65114, 52241, 12288, 4096, 0, 0, 2048, 12288, 30851, 64627, 64562, 35012, 14336, 6144, 8192, 18498, 35110, 65082, 57963, 32769, 14336, 6144, 6144, 6144, 6144, 6144, 6144, 6144, 4096, 4096, 2048, 2048, 0, 0, 0, 0, 0, 4096, 18432, 30883, 60498, 64789, 34980, 20480, 6144, 10240, 30721, 51657, 65179, 35175, 18497, 6144, 2048, 0, 0, 2048, 6144, 16384, 32932, 64562, 64627, 32932, 14336, 4096, 2048, 10240, 30786, 60141, 64952, 35109, 16417, 4096, 0, 0, 0, 0, 1, 69, 269, 4658, 40543, 36285},
    {23607, 51167, 6869, 401, 235, 167, 35, 1, 0, 2048, 10240, 28901, 65114, 43821, 6144, 0, 0, 2048, 18530, 64757, 60531, 18465, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 24772, 64984, 52143, 12288, 2048, 0, 0, 4096, 41708, 65211, 31013, 10240, 2048, 2048, 6144, 20675, 62774, 62741, 20611, 6144, 2048, 2048, 4096, 16515, 62806, 60660, 24772, 8192, 2048, 0, 0, 0, 0, 4096, 18432, 47853, 65244, 31176, 4096, 0, 0, 4096, 18433, 52079, 65147, 28998, 4096, 0, 0, 0, 2048, 12288, 30883, 64627, 64562, 35012, 14336, 6144, 8192, 18498, 35110, 65082, 55883, 36865, 28672, 24576, 24576, 24576, 24576, 22528, 22528, 22528, 20480, 18432, 12288, 6144, 2048, 0, 0, 0, 0, 4096, 18432, 32932, 60531, 64757, 34980, 20480, 6144, 10240, 30721, 51657, 65147, 35175, 20513, 10240, 6144, 4096, 4096, 8192, 14336, 24577, 35077, 64887, 64400, 32867, 12288, 4096, 4096, 12288, 30819, 64432, 64789, 35077, 16417, 4096, 0, 0, 0, 0, 1, 69, 300, 4690, 40543, 38365},
    {23639, 53247, 6869, 401, 235, 135, 35, 0, 0, 2048, 10240, 28933, 65179, 43789, 4096, 0, 0, 2048, 18530, 64822, 58450, 18465, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 24772, 64952, 52143, 12288, 2048, 0, 0, 4096, 39628, 65212, 31014, 10240, 2048, 0, 2048, 10240, 41644, 65244, 37385, 10240, 2048, 0, 2048, 6144, 39628, 65244, 39401, 14336, 4096, 0, 0, 0, 0, 6144, 28933, 64887, 60563, 16417, 2048, 0, 2048, 6144, 28966, 65017, 56337, 16385, 2048, 0, 0, 0, 2048, 12288, 30883, 64659, 64530, 35012, 14336, 4096, 6144, 16417, 35077, 64920, 64432, 43237, 43205, 41189, 41189, 41189, 41189, 39141, 39141, 39141, 37093, 35012, 26722, 18465, 8192, 2048, 0, 0, 0, 4096, 18432, 32932, 62579, 64757, 34980, 20480, 6144, 10240, 30721, 51689, 65179, 35175, 22561, 18432, 14336, 14336, 14336, 18433, 26690, 32964, 58255, 65114, 47560, 28706, 10240, 2048, 4096, 14336, 30884, 64627, 64595, 35012, 14336, 2048, 0, 0, 0, 0, 1, 69, 300, 4690, 40511, 38365},
    {23607, 53247, 6870, 401, 235, 135, 35, 1, 0, 2048, 10240, 28933, 65211, 41741, 4096, 0, 0, 2048, 18530, 64854, 58418, 18465, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 24772, 64952, 52176, 12288, 2048, 0, 0, 4096, 37580, 65212, 31046, 10240, 2048, 0, 0, 4096, 24772, 64887, 60563, 18498, 4096, 0, 0, 4096, 20643, 64984, 60466, 24707, 4096, 0, 0, 0, 4096, 12289, 43724, 65244, 33256, 8192, 2048, 0, 4096, 14369, 47983, 65211, 31111, 8192, 2048, 0, 0, 0, 2048, 12288, 30883, 64659, 64562, 35012, 12288, 4096, 4096, 12288, 32899, 55980, 65147, 65049, 65017, 64984, 64952, 64919, 64887, 64855, 64822, 64789, 64757, 62579, 49835, 30916, 18465, 6144, 2048, 0, 0, 4096, 18432, 32932, 62579, 64757, 32932, 20480, 6144, 10240, 30721, 51689, 65212, 35239, 32996, 32964, 32964, 32964, 32964, 35077, 41481, 64692, 65147, 56045, 35012, 20480, 4096, 2048, 8192, 20545, 33062, 65082, 60109, 34915, 12288, 2048, 0, 0, 0, 0, 1, 69, 268, 4658, 38463, 38366},
    {21527, 53247, 6870, 401, 235, 135, 35, 0, 0, 2048, 10240, 28933, 65212, 41676, 4096, 0, 0, 2048, 18530, 64887, 58417, 18465, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 24772, 64952, 54224, 12288, 2048, 0, 0, 4096, 37547, 65244, 33094, 10240, 2048, 0, 0, 2048, 14336, 47885, 65211, 33192, 6144, 2048, 0, 2048, 10240, 47982, 65147, 35239, 8192, 2048, 0, 2048, 8192, 24837, 64854, 60693, 16482, 6144, 2048, 2048, 8192, 26917, 65017, 54386, 14401, 4096, 2048, 0, 0, 0, 2048, 12288, 30883, 64659, 64530, 34980, 12288, 2048, 2048, 8192, 22528, 37093, 41448, 49965, 49998, 52079, 52111, 54192, 54224, 56305, 56337, 58417, 60466, 64756, 65147, 47723, 26722, 12288, 2048, 0, 0, 4096, 18432, 32932, 62611, 64757, 32932, 20480, 6144, 8192, 28673, 49706, 65244, 64854, 64789, 64757, 64724, 64660, 64757, 64952, 65212, 64854, 47658, 35012, 22561, 10240, 4096, 4096, 16384, 28803, 49965, 65147, 45382, 28673, 8192, 0, 0, 0, 0, 0, 1, 69, 268, 4658, 38431, 38398},
    {21527, 53247, 6870, 401, 235, 135, 35, 0, 0, 2048, 10240, 31013, 65212, 39595, 4096, 0, 0, 2048, 18530, 64887, 56337, 18465, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 24772, 64952, 54256, 12288, 2048, 0, 0, 4096, 37515, 65244, 33126, 12288, 2048, 0, 0, 2048, 8192, 26917, 65049, 54322, 14401, 4096, 2048, 2048, 6144, 22789, 65146, 52144, 20578, 6144, 2048, 4096, 16384, 43627, 65244, 35402, 6144, 2048, 0, 4096, 16384, 49965, 65211, 31143, 6144, 2048, 0, 0, 0, 2048, 4096, 12288, 30883, 64660, 64530, 35012, 12288, 2048, 0, 2048, 8192, 16384, 20546, 22626, 24674, 24674, 26755, 26755, 26723, 26755, 26755, 28770, 34882, 39141, 64627, 64887, 35045, 18432, 4096, 0, 0, 4096, 18432, 32931, 62611, 64724, 32931, 18432, 4096, 6144, 22528, 39206, 60303, 64432, 64464, 64464, 64465, 64497, 64464, 60173, 49609, 39141, 28803, 18465, 8192, 4096, 4096, 12288, 30721, 43269, 64952, 60466, 35012, 20480, 4096, 0, 0, 0, 0, 0, 1, 69, 268, 4657, 38431, 38398},
    {21527, 53247, 6870, 401, 235, 135, 35, 0, 0, 2048, 10240, 31014, 65212, 39595, 4096, 0, 0, 2048, 18530, 64919, 56337, 18433, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 24772, 64919, 54224, 12288, 2048, 0, 0, 4096, 37482, 65244, 33126, 12288, 2048, 0, 0, 0, 4096, 14369, 50095, 65179, 28998, 8192, 2048, 0, 2048, 10240, 52273, 65049, 33094, 12288, 4096, 6144, 26788, 64724, 64822, 18530, 2048, 2080, 2048, 6144, 26885, 64951, 60563, 18530, 4096, 0, 32, 0, 0, 2048, 4096, 14369, 30916, 64692, 64530, 35012, 14336, 4096, 2048, 2048, 6144, 8192, 14336, 16384, 18432, 20480, 20480, 20480, 20480, 20480, 20480, 22528, 30720, 36898, 49901, 65114, 39206, 20480, 4096, 0, 0, 4096, 18432, 32931, 62611, 64724, 32931, 18432, 4096, 2048, 12288, 22529, 30819, 32899, 32899, 32899, 32899, 34947, 32899, 32834, 30721, 24576, 14336, 8192, 2048, 2048, 10240, 22561, 39109, 62254, 65114, 39336, 24642, 12288, 2048, 0, 0, 0, 0, 0, 1, 69, 268, 4625, 38399, 38398},
    {21527, 53247, 6902, 401, 235, 135, 35, 1, 0, 2048, 10240, 31014, 65244, 39563, 4096, 0, 0, 2048, 18530, 64919, 58385, 18433, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 24772, 64919, 54257, 12288, 2048, 0, 0, 4096, 35434, 65244, 35175, 12288, 2048, 0, 0, 0, 2048, 6144, 31078, 65082, 52144, 14336, 2048, 0, 0, 4096, 26982, 65212, 49901, 22529, 8192, 10240, 39498, 65244, 39563, 8192, 2048, 2048, 4096, 12289, 43789, 65244, 35337, 10240, 4096, 32, 32, 32, 32, 2048, 4096, 14401, 30916, 64692, 64530, 35045, 12288, 4096, 2048, 4096, 10240, 18432, 24576, 26624, 28672, 26624, 26624, 26624, 26624, 26624, 26624, 28672, 34816, 38946, 56207, 65017, 37125, 20480, 4096, 0, 0, 4096, 18432, 32932, 64660, 64692, 32931, 18432, 4096, 0, 2048, 6144, 10240, 12288, 14336, 14336, 14336, 14336, 14336, 12288, 10240, 6144, 4096, 2048, 0, 6144, 22528, 39141, 49836, 65179, 56045, 30819, 14336, 4096, 0, 0, 0, 0, 0, 0, 1, 69, 268, 4625, 38367, 40446},
    {21494, 53247, 6902, 433, 235, 167, 35, 0, 0, 2048, 10240, 31014, 65244, 37515, 4096, 0, 0, 2048, 18562, 64919, 56305, 18465, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 24772, 64919, 54289, 12288, 2048, 0, 0, 4096, 35402, 65276, 35207, 12288, 2048, 0, 0, 0, 0, 4096, 18465, 56305, 65049, 26885, 4096, 0, 0, 2048, 12289, 58548, 64854, 32997, 14336, 24740, 60596, 64919, 18563, 6144, 2048, 4096, 8192, 24837, 64951, 58580, 18563, 6144, 2048, 32, 32, 32, 32, 2048, 4096, 14401, 30916, 64692, 64530, 35013, 14337, 4096, 4096, 8192, 20513, 32964, 37061, 39108, 39108, 39108, 39108, 39108, 39108, 37060, 37060, 39108, 43172, 45382, 64984, 60498, 30916, 14336, 4096, 0, 0, 4096, 18432, 32932, 64692, 64692, 32931, 18432, 4096, 0, 2048, 4096, 8192, 12288, 14336, 14336, 14336, 14336, 14336, 12288, 10240, 8192, 4096, 2048, 0, 8192, 32867, 49576, 65147, 56305, 34947, 24576, 8192, 2048, 0, 0, 0, 0, 0, 0, 1, 101, 268, 4625, 38399, 40479},
    {19446, 53247, 6870, 433, 235, 167, 35, 0, 0, 4096, 10240, 31014, 65244, 37515, 4096, 0, 0, 2048, 18563, 64952, 56305, 18465, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 24772, 64887, 54289, 12288, 2048, 0, 0, 4096, 33321, 65277, 35207, 12288, 2048, 0, 0, 0, 0, 2048, 10240, 33191, 65211, 45902, 10240, 2048, 0, 2048, 6144, 33257, 65244, 45643, 26624, 43432, 65212, 41709, 6144, 2048, 0, 4096, 14336, 43724, 65244, 35369, 6144, 2048, 0, 0, 0, 0, 0, 2048, 4096, 14337, 30884, 64692, 64530, 35012, 14336, 4096, 6144, 14336, 34980, 47528, 62481, 64692, 64691, 64659, 64659, 64627, 62579, 62579, 62547, 62546, 62547, 65082, 64919, 39271, 20513, 8192, 2048, 0, 0, 4096, 20480, 34980, 64692, 64692, 32932, 18432, 4096, 0, 4096, 14336, 26624, 34817, 34850, 34850, 34818, 34818, 34818, 32769, 30721, 24577, 16384, 8192, 2048, 6144, 30754, 47560, 65180, 49966, 34915, 24576, 8192, 0, 0, 0, 0, 0, 0, 0, 1, 69, 268, 4625, 36350, 40511},
    {19446, 53247, 8950, 433, 235, 135, 35, 0, 0, 2048, 10240, 33094, 65276, 37515, 4096, 0, 0, 2048, 18563, 64952, 56305, 18465, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 26820, 64854, 54289, 12288, 2048, 0, 0, 4096, 33321, 65277, 37288, 12288, 2048, 0, 0, 0, 0, 2048, 6144, 18530, 56435, 64984, 22756, 6144, 2048, 0, 4096, 14402, 64854, 64757, 43335, 64562, 64952, 20643, 2048, 0, 0, 4096, 24772, 64854, 62676, 20611, 4096, 0, 0, 0, 0, 0, 0, 0, 2048, 14336, 30884, 64692, 64497, 34980, 12288, 4096, 8192, 20513, 39141, 64692, 64952, 62579, 62611, 62611, 62643, 62644, 62676, 64756, 64757, 64789, 64789, 52111, 35077, 24642, 10240, 2048, 0, 0, 0, 4096, 20480, 34980, 64692, 62611, 32931, 18432, 4096, 2048, 12288, 28835, 41254, 51657, 55818, 55786, 53737, 53737, 53737, 53737, 51689, 41254, 28771, 16384, 6144, 6144, 20480, 35012, 58288, 65115, 47560, 28705, 10240, 2048, 0, 0, 0, 0, 0, 0, 1, 101, 268, 4625, 36318, 40511},
    {19414, 55295, 8951, 433, 235, 167, 35, 0, 0, 2048, 10240, 33094, 65276, 37515, 4096, 0, 0, 2048, 18563, 64952, 56304, 18433, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 24804, 64952, 52176, 12288, 2048, 0, 0, 4096, 33289, 65277, 37320, 10240, 2048, 0, 0, 0, 0, 0, 2048, 8192, 35337, 65212, 43724, 12288, 2048, 0, 0, 6144, 39433, 65049, 65115, 65212, 43724, 10240, 2048, 0, 2048, 8192, 39595, 65276, 39498, 10240, 2048, 0, 0, 0, 0, 0, 0, 0, 4096, 14337, 30884, 64692, 64497, 34980, 12288, 4096, 10240, 22594, 37125, 65050, 55980, 34915, 28770, 26755, 26755, 28803, 28803, 28835, 28803, 28803, 26755, 24706, 18465, 10240, 4096, 0, 0, 0, 0, 4096, 20480, 34980, 64724, 62611, 32931, 18432, 4096, 6144, 22528, 39238, 64887, 65244, 65179, 65179, 65180, 65179, 65179, 65180, 65212, 64855, 43399, 30786, 14336, 6144, 10240, 22561, 39174, 64725, 64692, 32996, 16417, 4096, 0, 0, 0, 0, 0, 0, 1, 101, 267, 4625, 36318, 40543},
    {19414, 55295, 8983, 433, 235, 167, 35, 0, 0, 2048, 10240, 33094, 65276, 37482, 4096, 0, 0, 2048, 18531, 64854, 58418, 18433, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 24804, 65114, 47950, 12288, 2048, 0, 0, 4096, 31208, 65277, 35272, 10240, 2048, 0, 0, 0, 0, 0, 0, 4096, 22659, 64724, 64822, 22659, 4096, 0, 0, 4096, 18465, 37320, 47853, 39498, 18530, 6144, 2048, 2048, 6144, 20675, 62773, 62741, 20643, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 4096, 14337, 30884, 64692, 64497, 34980, 12288, 6144, 10240, 22626, 35110, 65082, 55948, 30721, 18432, 14336, 14336, 14336, 14336, 14336, 14336, 14336, 14336, 10240, 6144, 2048, 0, 0, 0, 0, 0, 4096, 20480, 34980, 64724, 62579, 30883, 18432, 4096, 8192, 26625, 51786, 65211, 39465, 33094, 33062, 33094, 33094, 33094, 33094, 39368, 64919, 64790, 39206, 22561, 10240, 6144, 12288, 32802, 53802, 65147, 41416, 20546, 8192, 2048, 0, 0, 0, 0, 0, 1, 101, 268, 4625, 36286, 42591},
    {19382, 55295, 8983, 433, 235, 167, 35, 0, 0, 2048, 10240, 33127, 65276, 37482, 4096, 0, 0, 4096, 18466, 60530, 64822, 22659, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 8192, 33256, 65244, 41611, 12288, 2048, 0, 0, 6144, 35337, 65277, 35272, 10240, 2048, 0, 0, 0, 0, 0, 0, 4096, 14336, 41546, 65244, 37482, 6144, 2048, 0, 2048, 4096, 14402, 22756, 14402, 4096, 2048, 0, 2048, 12288, 39498, 65244, 39531, 8192, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 14337, 30884, 64692, 64465, 34980, 12288, 6144, 10240, 22626, 37158, 65082, 55948, 26625, 10240, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 2048, 2048, 2048, 0, 0, 0, 0, 0, 0, 4096, 20480, 35012, 64724, 62579, 30883, 18432, 6144, 8192, 28673, 55948, 65082, 35142, 24674, 18433, 16417, 16449, 16449, 20513, 30786, 43399, 65082, 54127, 28803, 16384, 4096, 8192, 26624, 41156, 65082, 49933, 26787, 12288, 2048, 0, 0, 0, 0, 0, 1, 101, 268, 4625, 36286, 42591},
    {17334, 55295, 8983, 433, 235, 167, 35, 0, 0, 2048, 10240, 28966, 65212, 41708, 8192, 2048, 0, 2048, 12288, 43659, 65244, 39530, 12288, 4096, 2048, 0, 0, 0, 0, 0, 2048, 4096, 8192, 22756, 58548, 64984, 26885, 8192, 2048, 0, 2048, 10240, 45869, 65211, 26917, 8192, 2048, 0, 0, 0, 0, 0, 0, 2048, 8192, 24772, 64854, 60661, 16514, 4096, 0, 0, 0, 2048, 4096, 4096, 0, 0, 0, 4096, 22658, 64756, 64822, 22691, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 14337, 30884, 64692, 64465, 34980, 12288, 6144, 10240, 22626, 37158, 65082, 55948, 26625, 8192, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 20480, 34980, 64725, 60498, 30883, 18432, 6144, 8192, 28673, 55947, 65049, 37158, 22561, 10240, 6144, 4096, 6144, 10240, 24576, 34947, 64691, 64724, 35012, 20480, 4096, 6144, 24576, 39076, 65017, 52046, 28835, 14336, 2048, 0, 0, 0, 0, 0, 1, 101, 268, 4624, 34205, 42623},
    {17333, 55295, 11064, 433, 267, 167, 35, 0, 0, 2048, 6144, 20676, 64984, 54354, 16450, 6144, 2048, 2048, 6144, 22724, 62774, 64984, 37352, 16384, 8192, 4096, 2048, 0, 2048, 2048, 6144, 12288, 24772, 54289, 65212, 41644, 10240, 2048, 0, 2048, 8192, 22659, 58483, 64886, 16515, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 10240, 43724, 65244, 35304, 8192, 2048, 0, 0, 0, 0, 0, 0, 0, 2048, 8192, 35402, 65276, 43659, 12288, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 14337, 30884, 64692, 64465, 34980, 12288, 6144, 12288, 24674, 37158, 65082, 55947, 26624, 10240, 4096, 4096, 4096, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 0, 0, 0, 0, 0, 4096, 20480, 35012, 64757, 60498, 30851, 18432, 6144, 8192, 28705, 55947, 65049, 35110, 20513, 8192, 2048, 0, 2048, 6144, 16384, 28771, 56304, 64887, 39141, 24576, 6144, 6144, 24576, 37028, 64952, 54159, 30883, 14336, 4096, 0, 0, 0, 0, 0, 1, 101, 268, 4624, 34205, 42623},
    {17301, 55295, 11064, 433, 235, 167, 35, 0, 0, 0, 4096, 12289, 50095, 65147, 31111, 10240, 2048, 0, 2048, 8192, 29063, 64886, 65082, 47853, 24772, 14337, 8192, 8192, 8192, 10240, 20578, 37353, 64789, 65179, 45837, 16417, 4096, 0, 0, 4096, 12288, 35240, 65147, 48015, 8192, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 26885, 64919, 60531, 18465, 4096, 0, 0, 0, 0, 0, 0, 0, 4096, 18563, 60628, 64887, 24772, 8192, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 14337, 30884, 64692, 64465, 34980, 12288, 4096, 10240, 22626, 37158, 65050, 53899, 28673, 18432, 14336, 14336, 14336, 14336, 14336, 14336, 14336, 12288, 12288, 12288, 10240, 10240, 6144, 4096, 2048, 0, 4096, 20480, 34980, 64757, 60466, 30851, 18432, 6144, 8192, 28705, 55947, 65049, 35110, 18498, 6144, 2048, 0, 0, 2048, 12288, 24675, 54191, 64952, 41189, 24576, 6144, 4096, 22528, 34980, 64887, 56272, 30884, 16384, 4096, 0, 0, 0, 0, 0, 2, 101, 267, 4592, 34205, 44703},
    {15253, 53247, 11064, 433, 235, 135, 35, 0, 0, 0, 0, 4096, 28966, 65017, 58482, 22659, 4096, 2048, 2048, 2048, 8192, 26918, 58418, 65212, 64887, 54289, 43821, 41676, 41676, 47982, 58547, 65212, 64887, 39498, 14401, 4096, 2048, 0, 2048, 8192, 26820, 60531, 65049, 24805, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 16384, 49966, 65179, 29063, 4096, 0, 0, 0, 0, 0, 0, 2048, 10240, 35304, 65244, 43756, 10240, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 14337, 30884, 64724, 64432, 34980, 12288, 4096, 8192, 20545, 37126, 65017, 56110, 37093, 35013, 35012, 35012, 35012, 35012, 34980, 34980, 34980, 34980, 34980, 34980, 34947, 30786, 24577, 14336, 4096, 0, 4096, 20480, 37060, 64789, 60466, 30883, 18432, 6144, 8192, 28705, 55948, 65017, 35110, 18465, 6144, 2048, 0, 0, 2048, 12288, 24674, 52111, 64984, 41157, 26624, 6144, 4096, 20480, 34980, 64822, 58385, 32932, 16384, 4096, 0, 0, 0, 0, 0, 1, 101, 267, 2544, 34172, 44703},
    {15253, 55295, 11064, 433, 267, 135, 35, 0, 0, 0, 0, 2048, 14336, 43724, 65212, 47950, 16482, 4096, 2048, 2048, 4096, 8192, 20579, 37418, 58515, 64951, 65179, 65244, 65244, 65146, 62773, 45902, 26852, 12288, 4096, 2048, 2048, 4096, 6144, 22724, 54256, 65211, 41643, 10240, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 10240, 28965, 65081, 52273, 12321, 4096, 0, 0, 0, 0, 0, 4096, 18498, 60563, 64919, 24804, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 14337, 30916, 64725, 64432, 34980, 12288, 4096, 6144, 16417, 35045, 62384, 65114, 64725, 64660, 64627, 64595, 64562, 64530, 64497, 64465, 64432, 64432, 64367, 64335, 62254, 60141, 39206, 26722, 14336, 4096, 6144, 20480, 37060, 64789, 60466, 30851, 16384, 6144, 8192, 28705, 55948, 65017, 35110, 18465, 6144, 2048, 0, 0, 2048, 12288, 24675, 49966, 65017, 43237, 26624, 6144, 4096, 20480, 32931, 64789, 60498, 32932, 18432, 4096, 0, 0, 0, 0, 0, 1, 101, 235, 2544, 34140, 44703},
    {15221, 53247, 11096, 433, 267, 167, 35, 0, 0, 0, 0, 0, 4096, 18563, 52111, 65212, 50030, 22692, 8192, 6144, 2048, 2048, 4096, 6144, 14402, 20643, 24804, 26884, 24804, 22724, 18595, 10240, 4096, 2048, 0, 2048, 6144, 10240, 24804, 54224, 65212, 45837, 14401, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 14401, 50096, 65146, 26917, 8192, 2048, 0, 0, 0, 2048, 6144, 33224, 65212, 47853, 16384, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 14369, 30916, 64725, 64432, 34980, 12288, 2048, 2048, 10240, 24610, 39173, 58125, 64497, 64530, 64562, 64594, 64627, 64659, 64660, 64692, 64757, 64789, 64822, 64822, 64887, 65114, 64854, 41286, 26625, 8192, 6144, 22528, 37060, 64790, 60466, 30883, 16384, 6144, 8192, 28706, 55980, 65017, 35110, 18498, 6144, 2048, 0, 0, 2048, 10240, 22626, 47885, 65082, 43237, 26624, 6144, 4096, 18432, 32931, 64724, 62611, 34980, 20480, 4096, 0, 0, 0, 0, 0, 1, 101, 235, 4592, 34140, 44735},
    {15221, 55295, 11128, 433, 267, 167, 35, 1, 0, 0, 0, 0, 2048, 6144, 22692, 52144, 65212, 62644, 31111, 14336, 6144, 4096, 2048, 2048, 6144, 6144, 8192, 8192, 8192, 6144, 4096, 4096, 2048, 2048, 4096, 6144, 14337, 31111, 62676, 65179, 45869, 16417, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 6144, 31111, 65146, 52111, 14336, 4096, 0, 0, 2048, 6144, 16450, 56435, 64984, 26885, 8192, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 14369, 30916, 64725, 64432, 34980, 14336, 4096, 2048, 6144, 14336, 24576, 28706, 32866, 32867, 30819, 30851, 30851, 32899, 30883, 30883, 30883, 30884, 32932, 32932, 32996, 39368, 65114, 60174, 32867, 10240, 6144, 22528, 37060, 64822, 58385, 30851, 18432, 6144, 10240, 30753, 58028, 64985, 35110, 18497, 6144, 2048, 0, 0, 2048, 10240, 22626, 45804, 65114, 43237, 28672, 8192, 6144, 18432, 32899, 62611, 64724, 35012, 20480, 4096, 0, 0, 0, 0, 0, 1, 69, 235, 4592, 34140, 44735},
    {15189, 53247, 11129, 433, 268, 167, 67, 1, 0, 0, 0, 0, 0, 2048, 6144, 18530, 41579, 65049, 65081, 43756, 26820, 14336, 8192, 6144, 4096, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 6144, 8192, 16384, 26820, 43724, 65017, 65049, 39498, 12321, 4096, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 20546, 56305, 65017, 26885, 8192, 4096, 2048, 4096, 12288, 35207, 65179, 45902, 12321, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 14369, 30916, 64757, 64432, 36996, 20480, 10240, 6144, 6144, 8192, 12288, 14336, 16384, 16384, 16384, 16384, 16384, 16384, 18432, 16384, 16384, 16384, 18432, 20480, 26625, 34980, 64627, 64595, 35012, 12288, 6144, 20480, 35012, 64822, 58385, 32899, 20480, 10240, 14336, 30754, 58028, 64985, 35110, 16449, 4096, 2048, 0, 0, 2048, 10240, 22626, 43691, 65147, 43237, 30720, 10240, 8192, 20480, 32899, 60466, 64789, 35012, 20480, 4096, 0, 0, 0, 0, 0, 1, 69, 235, 2543, 32059, 44767},
    {15188, 53247, 11129, 433, 268, 168, 68, 1, 0, 0, 0, 0, 0, 0, 2048, 4096, 14336, 31046, 56467, 65244, 64757, 43627, 26852, 16450, 10240, 6144, 4096, 4096, 4096, 4096, 4096, 8192, 16450, 26885, 45740, 64756, 65244, 58548, 28998, 12288, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 10240, 35240, 65211, 52144, 20644, 12288, 8192, 12288, 28901, 62611, 65017, 26917, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 10240, 28803, 64595, 64757, 41254, 28770, 22561, 18465, 16417, 16417, 18433, 18433, 18432, 18432, 18432, 18432, 18432, 18432, 18432, 18432, 18432, 18432, 20480, 22528, 26625, 34980, 64659, 64595, 32964, 10240, 6144, 18432, 35012, 64789, 62546, 34947, 26624, 20480, 22529, 32899, 58125, 64984, 35077, 16417, 4096, 0, 0, 0, 2048, 8192, 20578, 41481, 65180, 47496, 32769, 18432, 16384, 26625, 34980, 64659, 64757, 32964, 18432, 4096, 0, 0, 0, 0, 0, 1, 68, 235, 4592, 32059, 44767},
    {15188, 55295, 13209, 433, 300, 200, 68, 1, 0, 0, 0, 0, 0, 0, 0, 2048, 2048, 6144, 18562, 39530, 64821, 65244, 64984, 54322, 43788, 37418, 35337, 33224, 31208, 33256, 35369, 45902, 58515, 65114, 65212, 64821, 39530, 18530, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 14369, 50030, 65212, 60596, 48015, 45870, 47950, 64822, 65179, 39563, 12288, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8192, 24610, 45577, 65147, 64691, 41384, 37125, 37093, 35045, 35045, 37093, 37093, 37093, 37093, 37092, 37093, 35044, 35013, 35044, 35013, 35013, 35013, 37061, 37060, 37093, 41481, 65115, 58125, 30818, 8192, 4096, 14336, 28835, 58385, 64984, 41319, 37060, 34980, 35045, 41416, 64952, 64529, 34980, 12288, 2048, 0, 0, 0, 0, 4096, 16417, 35110, 64952, 62384, 39174, 34980, 34947, 37060, 47690, 65114, 54062, 26755, 14336, 4096, 0, 0, 0, 0, 0, 1, 69, 235, 4559, 32059, 46847},
    {13108, 53247, 13241, 466, 268, 201, 101, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 6144, 10240, 20611, 35337, 54354, 64919, 65211, 65276, 65277, 65277, 65277, 65277, 65277, 65147, 64854, 48047, 31144, 18530, 10240, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 4096, 18562, 45870, 62774, 65081, 65146, 65146, 60661, 35402, 12288, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4096, 16384, 26755, 41578, 64952, 65115, 64822, 64789, 64757, 64757, 64757, 64757, 64757, 64725, 64725, 64757, 64757, 64757, 64757, 64757, 64757, 64757, 64757, 64757, 64822, 65179, 64822, 39206, 22528, 6144, 2048, 8192, 20546, 37255, 64952, 65017, 64692, 64595, 64789, 65114, 64886, 41319, 24577, 8192, 0, 0, 0, 0, 0, 2048, 12288, 28803, 53997, 65180, 64790, 64497, 64497, 64692, 65147, 64562, 35077, 18465, 8192, 2048, 0, 0, 0, 0, 0, 1, 69, 235, 4559, 32027, 46847},
    {13108, 53247, 13241, 466, 301, 233, 135, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 2048, 2048, 4096, 12321, 18563, 26885, 33159, 35240, 37320, 37352, 37320, 35240, 26885, 20643, 10240, 4096, 2048, 2048, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 8192, 18563, 24772, 24772, 22724, 16515, 6144, 2048, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 6144, 14336, 24674, 35045, 53932, 62319, 64432, 64432, 64433, 64432, 64432, 64432, 64432, 64432, 64432, 64432, 64432, 64432, 64432, 64400, 64400, 64400, 64400, 64432, 53900, 37093, 24642, 12288, 2048, 0, 2048, 12288, 26690, 39205, 60238, 64497, 64562, 64432, 58093, 37125, 26690, 14336, 4096, 0, 0, 0, 0, 0, 0, 6144, 18433, 30851, 49706, 64497, 64724, 64724, 64594, 53867, 37093, 24642, 10240, 2048, 0, 0, 0, 0, 0, 0, 1, 69, 235, 2511, 29947, 46847},
    {10996, 49087, 19580, 530, 333, 235, 168, 101, 35, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 6144, 8192, 8192, 10240, 10240, 10240, 10240, 10240, 8192, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 6144, 6144, 8192, 6144, 4096, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 6144, 14336, 20481, 24610, 28738, 28770, 28771, 28771, 28771, 28771, 30819, 30819, 28770, 30819, 30819, 30819, 30819, 30819, 30819, 30819, 30819, 30819, 30819, 28706, 22529, 12288, 4096, 0, 0, 0, 4096, 14336, 22529, 28770, 30883, 30883, 30851, 28706, 22529, 12288, 4096, 0, 0, 0, 0, 0, 0, 0, 2048, 10240, 18433, 26658, 30851, 32932, 32932, 32899, 28738, 22529, 14336, 4096, 0, 0, 0, 0, 0, 0, 0, 1, 69, 235, 2544, 29947, 46847},
    {6837, 40639, 34303, 6803, 399, 269, 234, 168, 134, 101, 68, 34, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 2048, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 4096, 8192, 8192, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 12288, 12288, 10240, 10240, 6144, 2048, 0, 0, 0, 0, 0, 4096, 6144, 8192, 10240, 10240, 10240, 8192, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 4096, 8192, 12288, 14336, 14337, 12288, 10240, 6144, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 1, 101, 235, 2544, 29946, 48927},
    {530, 21628, 53247, 19448, 4658, 399, 301, 267, 234, 201, 168, 134, 100, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 34, 34, 34, 35, 35, 34, 34, 2, 34, 34, 2, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2048, 2048, 2048, 4096, 2048, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 35, 135, 301, 2577, 34173, 44735},
    {367, 6803, 38431, 49023, 17368, 4724, 466, 401, 334, 301, 267, 234, 200, 135, 68, 34, 0, 0, 0, 0, 1, 1, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 1, 2, 35, 68, 101, 102, 102, 135, 135, 135, 135, 135, 135, 135, 135, 135, 135, 134, 102, 101, 68, 35, 1, 0, 0, 0, 0, 0, 0, 1, 2, 35, 68, 69, 69, 68, 35, 1, 0, 0, 0, 0, 0, 0, 1, 1, 2, 35, 35, 68, 68, 68, 69, 69, 69, 101, 69, 69, 69, 69, 69, 68, 68, 68, 68, 68, 36, 36, 36, 36, 36, 35, 35, 35, 35, 35, 35, 3, 3, 35, 34, 34, 34, 34, 34, 34, 2, 2, 2, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 35, 68, 135, 235, 433, 6804, 42655, 36285},
    {203, 2478, 10963, 38463, 51167, 23773, 11063, 4757, 4691, 2545, 2511, 2446, 333, 267, 168, 101, 34, 0, 0, 2, 36, 101, 135, 167, 134, 69, 35, 2, 0, 0, 0, 1, 35, 101, 168, 201, 234, 267, 268, 300, 300, 300, 300, 300, 300, 300, 300, 300, 299, 299, 267, 266, 233, 168, 69, 3, 1, 0, 0, 0, 1, 68, 102, 168, 201, 234, 202, 201, 135, 101, 34, 1, 0, 0, 0, 2, 36, 101, 135, 168, 168, 201, 201, 201, 201, 202, 202, 202, 202, 202, 202, 202, 201, 201, 202, 201, 201, 201, 169, 201, 169, 169, 169, 169, 168, 168, 168, 168, 168, 136, 136, 136, 136, 135, 136, 135, 135, 135, 135, 135, 135, 135, 103, 102, 69, 36, 2, 1, 0, 0, 0, 0, 1, 2, 3, 35, 35, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 3, 69, 102, 136, 201, 268, 399, 4691, 21627, 53247, 21528},
    {103, 234, 2479, 6803, 30046, 51135, 53215, 42655, 34205, 27834, 19414, 13076, 8851, 4625, 334, 201, 69, 2, 2, 69, 202, 365, 2511, 4624, 4625, 366, 201, 69, 2, 1, 1, 68, 168, 268, 2512, 4690, 4690, 6771, 6771, 4723, 6771, 4723, 6771, 6771, 6771, 6771, 6771, 6771, 6771, 4723, 6771, 4690, 4690, 2512, 332, 136, 35, 1, 0, 1, 68, 168, 300, 399, 497, 2578, 2578, 432, 333, 202, 101, 35, 1, 1, 2, 69, 201, 301, 367, 432, 433, 465, 465, 465, 433, 433, 465, 465, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433, 401, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 368, 368, 368, 367, 368, 367, 367, 367, 367, 367, 367, 367, 335, 334, 269, 201, 102, 35, 1, 0, 0, 2, 69, 169, 267, 268, 235, 169, 70, 35, 1, 0, 0, 0, 0, 0, 1, 2, 102, 201, 268, 334, 368, 433, 498, 6803, 25754, 51135, 30079, 4756},
    {36, 102, 234, 366, 4625, 10963, 23608, 36286, 42655, 48991, 53247, 53247, 42751, 25821, 6804, 333, 135, 36, 69, 200, 2479, 8917, 27934, 34303, 32159, 15255, 6737, 234, 69, 2, 35, 135, 332, 8884, 21627, 32223, 32223, 32255, 34303, 34303, 34303, 34303, 34303, 34303, 34303, 34303, 34303, 34303, 34335, 34335, 34335, 34303, 34303, 25821, 8851, 332, 102, 2, 2, 68, 168, 365, 6738, 11095, 21628, 25887, 23806, 15289, 6771, 2446, 201, 69, 2, 2, 69, 234, 2479, 8884, 13143, 15322, 15354, 15354, 15355, 15355, 15354, 15354, 15322, 15322, 15322, 15322, 13242, 13242, 13241, 13241, 13209, 13209, 13209, 13209, 11129, 11129, 11128, 11128, 11096, 11096, 11096, 11096, 11064, 11063, 9015, 9015, 8983, 8983, 8983, 8983, 8951, 8982, 8950, 8950, 8950, 8950, 8950, 6870, 6870, 8917, 8851, 4624, 266, 103, 35, 1, 3, 102, 2379, 6704, 6771, 8884, 6770, 4624, 332, 135, 35, 1, 0, 0, 0, 0, 2, 134, 2380, 4624, 6771, 6805, 6837, 6870, 17403, 38495, 53247, 29980, 6837, 434},
    {1, 34, 101, 168, 266, 2413, 2511, 4658, 6804, 6870, 9015, 17434, 30078, 51167, 27934, 2544, 233, 103, 136, 300, 10963, 42719, 49055, 36383, 44799, 53215, 19514, 4592, 168, 70, 102, 201, 4657, 25821, 53215, 38527, 36415, 36447, 36415, 36415, 36415, 36415, 36415, 36415, 36415, 36415, 36415, 36415, 36415, 36415, 36383, 36415, 36415, 51135, 34239, 6737, 233, 69, 36, 103, 333, 8884, 32092, 53247, 51135, 44831, 46943, 53247, 38463, 10995, 2413, 169, 36, 37, 169, 2478, 19382, 46847, 55295, 53247, 53247, 53247, 53247, 53247, 53247, 53247, 53247, 53247, 55295, 55295, 53247, 53247, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 55295, 53247, 53247, 53247, 53247, 53247, 53247, 53247, 53247, 53247, 53247, 53215, 53215, 53215, 53215, 51167, 53183, 48959, 25721, 6738, 235, 69, 35, 103, 2445, 15188, 34238, 48991, 48959, 42655, 25754, 10996, 431, 167, 35, 1, 0, 0, 2, 102, 2413, 13043, 34206, 42655, 44799, 46943, 51135, 53247, 38527, 17401, 6804, 432, 302},
    {0, 0, 2, 68, 102, 168, 234, 269, 335, 400, 434, 531, 6803, 36351, 38463, 6738, 267, 137, 202, 2478, 23608, 53247, 13175, 6804, 8883, 36351, 42751, 8884, 299, 169, 169, 300, 6771, 38527, 34271, 8882, 4690, 4690, 4658, 4658, 4658, 4658, 4658, 4690, 4690, 4690, 4690, 4658, 4690, 4690, 4691, 4723, 6803, 21594, 53247, 13076, 2380, 103, 70, 201, 2545, 23708, 53215, 25753, 10963, 8851, 8884, 19448, 51103, 30013, 6706, 236, 103, 102, 268, 6738, 42623, 42655, 17302, 13076, 10963, 10963, 10963, 10995, 10995, 13043, 13043, 13043, 13043, 13076, 13076, 13076, 15156, 15156, 15156, 15156, 15188, 15188, 15188, 15189, 17269, 17269, 17269, 17301, 17301, 17301, 17302, 19382, 19382, 19382, 19414, 19414, 19415, 21495, 21495, 21495, 21495, 21527, 21527, 21527, 21527, 21560, 23608, 23608, 34204, 53215, 13176, 400, 136, 70, 235, 6770, 38463, 46943, 27899, 27867, 34271, 51167, 36415, 8883, 299, 102, 2, 0, 0, 35, 202, 6771, 34206, 51103, 32125, 29980, 27834, 23673, 15222, 6803, 498, 368, 268, 234},
    {0, 0, 0, 1, 34, 36, 69, 135, 168, 234, 269, 367, 4691, 30079, 40639, 6770, 300, 170, 236, 4559, 27866, 46943, 6870, 532, 563, 17434, 51199, 11028, 398, 267, 267, 365, 6803, 42719, 27999, 4691, 399, 334, 333, 333, 333, 332, 332, 332, 332, 332, 332, 332, 333, 333, 334, 367, 498, 11128, 53247, 15221, 2445, 137, 136, 266, 6738, 36447, 34335, 6803, 2512, 399, 465, 4691, 30012, 46879, 6836, 334, 136, 136, 334, 6804, 46911, 27866, 4691, 465, 432, 2479, 2479, 2479, 2479, 2479, 2479, 2479, 2479, 2479, 2479, 2479, 2479, 2479, 2479, 2479, 2479, 2479, 2511, 2479, 2479, 2479, 2512, 2511, 2511, 2511, 2511, 2512, 2512, 2512, 2512, 2512, 2512, 2512, 2512, 2512, 2512, 2512, 2512, 2544, 2544, 2544, 2545, 4657, 8915, 51135, 21629, 2578, 202, 136, 334, 8885, 48991, 25786, 4723, 2611, 2677, 15322, 53247, 19415, 2445, 168, 35, 1, 1, 69, 301, 8917, 46879, 29979, 4723, 2578, 2577, 2512, 2447, 365, 300, 235, 201, 168},
    {0, 0, 0, 0, 0, 1, 1, 34, 68, 135, 234, 301, 4691, 30079, 40639, 6770, 333, 203, 269, 4592, 29946, 46911, 6870, 435, 500, 13209, 53247, 15189, 464, 302, 302, 399, 6803, 44831, 25887, 2611, 301, 267, 234, 201, 201, 201, 201, 201, 201, 201, 201, 201, 202, 234, 267, 301, 466, 11096, 53247, 15221, 2478, 202, 169, 299, 6738, 38559, 30143, 4724, 367, 302, 336, 2578, 27866, 46911, 6837, 368, 169, 169, 367, 6837, 48991, 25786, 2578, 335, 301, 268, 268, 267, 267, 235, 235, 235, 235, 235, 235, 267, 267, 267, 267, 267, 267, 267, 268, 268, 268, 268, 268, 268, 268, 268, 268, 268, 268, 300, 269, 269, 301, 301, 301, 301, 301, 301, 301, 301, 301, 302, 335, 432, 6803, 46975, 23774, 2611, 267, 202, 368, 6870, 51103, 23641, 2546, 402, 500, 6871, 51135, 23640, 4559, 234, 69, 2, 2, 102, 335, 8950, 48991, 25753, 2578, 368, 302, 301, 267, 234, 201, 167, 102, 68},
    {0, 0, 0, 0, 0, 0, 0, 1, 2, 101, 201, 300, 2643, 28031, 40639, 6770, 333, 235, 269, 4625, 29947, 44831, 6838, 403, 467, 8983, 53247, 21560, 2611, 434, 402, 466, 8883, 46943, 23741, 2578, 268, 201, 135, 102, 101, 101, 101, 101, 101, 101, 101, 101, 102, 167, 202, 269, 466, 11128, 53247, 15221, 2478, 202, 170, 300, 6770, 38559, 30143, 4691, 334, 301, 302, 2545, 25753, 49023, 6870, 400, 202, 170, 368, 6837, 48991, 25786, 2577, 302, 268, 202, 168, 167, 167, 135, 135, 135, 135, 135, 135, 135, 167, 167, 167, 168, 168, 168, 168, 200, 168, 168, 168, 168, 168, 168, 201, 201, 168, 201, 201, 201, 201, 201, 201, 201, 201, 201, 202, 202, 234, 235, 268, 366, 6771, 42751, 27999, 2643, 300, 236, 401, 6903, 53215, 21528, 2512, 336, 434, 6870, 51103, 23640, 4592, 235, 102, 3, 3, 135, 367, 6902, 48991, 25753, 2545, 334, 268, 234, 168, 135, 101, 67, 34, 1},
    {0, 0, 0, 0, 0, 0, 0, 0, 1, 68, 168, 268, 4691, 28031, 40639, 6770, 333, 235, 270, 4625, 29979, 44799, 6837, 402, 369, 6804, 42655, 40511, 8950, 4790, 4757, 6836, 19448, 53247, 17402, 432, 201, 102, 35, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 69, 168, 268, 434, 11128, 53247, 17269, 2478, 235, 202, 300, 6770, 40607, 30111, 4691, 333, 268, 302, 2512, 25688, 51135, 6870, 401, 235, 203, 368, 6837, 49023, 25786, 2577, 301, 234, 167, 69, 68, 36, 35, 35, 35, 35, 35, 35, 35, 35, 67, 36, 36, 68, 68, 36, 68, 68, 68, 68, 68, 68, 68, 69, 69, 101, 101, 69, 101, 101, 101, 101, 102, 102, 102, 102, 134, 135, 168, 234, 333, 6771, 38495, 32223, 4691, 367, 302, 466, 9015, 53247, 19383, 2479, 335, 434, 6837, 51103, 25721, 4592, 268, 135, 36, 35, 136, 368, 6903, 49023, 25721, 2512, 301, 235, 168, 101, 35, 2, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 1, 68, 168, 300, 4691, 28031, 40639, 6770, 365, 236, 302, 4625, 32060, 44767, 4757, 401, 303, 2545, 19481, 49023, 48991, 40543, 38431, 40575, 53247, 34271, 8884, 333, 135, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 36, 167, 235, 433, 11096, 55295, 17269, 2479, 236, 202, 332, 6770, 40607, 30111, 4691, 301, 235, 269, 2511, 23608, 53215, 6902, 401, 236, 235, 369, 6837, 49023, 25753, 2545, 301, 202, 102, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 34, 35, 102, 201, 300, 4691, 32191, 40607, 8916, 531, 467, 2676, 21627, 51167, 10996, 398, 302, 401, 6837, 51071, 25753, 4625, 268, 135, 36, 36, 168, 400, 6902, 49023, 25720, 2512, 301, 234, 102, 3, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 1, 35, 135, 268, 4691, 28031, 40639, 6770, 366, 268, 302, 4625, 32092, 44767, 4756, 401, 236, 301, 4625, 13109, 29947, 36285, 38399, 36318, 23641, 8850, 2414, 169, 36, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 135, 235, 433, 11096, 55295, 17269, 2479, 236, 203, 332, 6770, 40607, 30079, 4691, 333, 235, 268, 2479, 21495, 53247, 6903, 434, 269, 269, 401, 6870, 49023, 25753, 2545, 301, 201, 101, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 68, 167, 267, 497, 19515, 53247, 27933, 13209, 11096, 21660, 49055, 34271, 6770, 333, 269, 401, 6837, 49023, 25753, 4625, 269, 168, 37, 36, 168, 400, 6903, 51103, 25720, 2512, 301, 201, 101, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 135, 267, 4658, 28031, 40639, 6770, 366, 269, 335, 4658, 32125, 42687, 4756, 368, 235, 201, 234, 2413, 2511, 4624, 6705, 4657, 2478, 299, 168, 36, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 135, 235, 433, 11096, 55295, 17269, 2479, 236, 235, 333, 6770, 40607, 30111, 4691, 300, 202, 235, 2478, 19414, 53247, 9016, 500, 336, 335, 467, 4855, 49055, 25753, 2512, 268, 168, 68, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 35, 134, 234, 366, 4691, 27901, 49023, 55295, 53247, 51167, 34271, 6869, 432, 268, 236, 400, 4757, 48991, 27834, 4625, 301, 168, 69, 36, 168, 400, 8983, 51135, 23640, 2512, 268, 201, 69, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 34, 134, 267, 4658, 30079, 40639, 6771, 399, 302, 336, 4691, 32125, 40575, 4724, 367, 201, 135, 102, 135, 201, 234, 235, 202, 168, 102, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 34, 134, 235, 433, 11096, 53247, 17269, 2479, 236, 235, 333, 6770, 40639, 30079, 4691, 300, 201, 202, 2413, 13108, 51167, 23773, 4757, 2578, 2610, 4757, 15322, 53247, 17302, 2479, 235, 167, 68, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 68, 168, 268, 398, 2577, 8850, 15156, 17302, 10996, 4690, 432, 301, 235, 235, 368, 4757, 48991, 27866, 4625, 301, 168, 69, 69, 169, 401, 8983, 51135, 23608, 2511, 268, 168, 68, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 102, 234, 4658, 30079, 42719, 6803, 465, 369, 435, 2677, 34206, 42623, 6771, 334, 168, 68, 3, 34, 35, 68, 68, 36, 35, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 134, 235, 433, 11096, 53247, 15221, 2479, 268, 235, 333, 6770, 40639, 30079, 4659, 300, 169, 136, 267, 4690, 27998, 53215, 32093, 21560, 19415, 27867, 51103, 38495, 6771, 333, 201, 102, 34, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 102, 201, 267, 333, 398, 2479, 2511, 2479, 366, 300, 234, 201, 202, 368, 4756, 46911, 27866, 4625, 301, 168, 69, 69, 201, 401, 8951, 51135, 23607, 2511, 268, 168, 68, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 69, 201, 2545, 25821, 49023, 13142, 4756, 2644, 4757, 8983, 46879, 36286, 4658, 268, 135, 35, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 102, 234, 433, 11096, 53247, 15221, 2479, 268, 236, 333, 6771, 40639, 28031, 4659, 300, 168, 103, 169, 399, 6803, 27867, 46879, 53215, 55295, 51039, 34205, 8884, 432, 267, 167, 68, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 35, 102, 200, 234, 235, 268, 268, 268, 235, 233, 168, 135, 170, 368, 4756, 46879, 29947, 4658, 301, 168, 70, 70, 202, 401, 8983, 53215, 23575, 2479, 268, 168, 68, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 35, 135, 366, 11030, 44799, 51103, 36351, 32093, 36319, 49023, 46943, 13110, 2446, 201, 101, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 101, 234, 433, 11096, 53247, 17270, 2479, 269, 268, 366, 6771, 40639, 28031, 4658, 267, 136, 69, 135, 234, 365, 4625, 6771, 8917, 8950, 6771, 4625, 398, 267, 168, 101, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 35, 101, 135, 168, 201, 201, 201, 168, 134, 102, 102, 169, 367, 4756, 46879, 29979, 4658, 301, 169, 70, 102, 202, 433, 8984, 53215, 21495, 2479, 267, 135, 35, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 69, 234, 2478, 10963, 27899, 40511, 44767, 40543, 29979, 10996, 2512, 267, 134, 35, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 69, 202, 433, 11096, 55295, 17302, 2479, 302, 301, 366, 6803, 40639, 27999, 4658, 267, 135, 36, 36, 135, 201, 269, 334, 367, 400, 367, 301, 234, 168, 101, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 35, 68, 101, 134, 101, 101, 36, 35, 69, 169, 367, 4756, 44799, 29979, 4658, 334, 202, 135, 136, 235, 434, 8984, 53247, 21462, 2479, 235, 135, 35, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 101, 201, 332, 4591, 6738, 6771, 6738, 4592, 365, 234, 134, 36, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 36, 169, 432, 11096, 55295, 17302, 2512, 302, 301, 399, 6803, 40671, 27999, 4658, 267, 102, 3, 2, 68, 102, 168, 201, 202, 234, 201, 168, 135, 100, 34, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 34, 34, 1, 1, 1, 68, 168, 367, 4756, 44799, 32060, 4658, 335, 235, 201, 234, 268, 467, 8984, 53247, 19414, 2478, 234, 134, 2, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 68, 135, 202, 268, 301, 268, 234, 168, 101, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 136, 399, 8950, 51167, 23640, 2577, 400, 335, 432, 8883, 42719, 27999, 4626, 234, 101, 2, 1, 1, 35, 68, 101, 102, 134, 102, 68, 35, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 35, 135, 333, 6771, 44735, 32125, 4723, 401, 301, 268, 269, 335, 531, 17434, 53247, 13108, 2446, 201, 101, 2, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 35, 68, 101, 101, 101, 69, 67, 34, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 102, 333, 6803, 44735, 38463, 8949, 4724, 2644, 4723, 17269, 51135, 25821, 2513, 201, 68, 1, 0, 0, 0, 1, 2, 34, 34, 34, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 34, 134, 268, 4657, 32093, 46911, 8982, 564, 434, 369, 402, 499, 8916, 38559, 40639, 6803, 332, 168, 68, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 36, 201, 2511, 19414, 51135, 42751, 30111, 25887, 32191, 51103, 42655, 8884, 366, 135, 35, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 69, 201, 2446, 15157, 49023, 38559, 13209, 6870, 4789, 6870, 11063, 34271, 51167, 15256, 2545, 235, 134, 35, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 69, 266, 4592, 15255, 30079, 40639, 44831, 40639, 25853, 8949, 2479, 234, 69, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 34, 102, 266, 2544, 15256, 40639, 53247, 44799, 40543, 40575, 49023, 46943, 19514, 4658, 333, 168, 101, 1, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 3, 103, 269, 400, 2578, 4722, 6803, 4723, 563, 401, 236, 103, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 36, 136, 269, 465, 8851, 19447, 32059, 36317, 36252, 27898, 8916, 2546, 334, 203, 135, 35, 1, 0, 0, 0, 0, 0, 0, 0}
};