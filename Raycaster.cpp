#include "MicroBit.h"
#include "Adafruit_ST7735.h"

MicroBit uBit;

// // Define the MICROBIT EDGE CONNECTOR pins where the display is connected...
#define LCD_PIN_CS      2
#define LCD_PIN_DC      1
#define LCD_PIN_RST     0
#define LCD_PIN_MOSI    15
#define LCD_PIN_MISO    14
#define LCD_PIN_SCLK    13

#define MAP_WIDTH   	24
#define MAP_HEIGHT  	24
#define SCREEN_WIDTH    160
#define SCREEN_HEIGHT   128

#define RED     0xFC00
#define GREEN   0x001F
#define BLUE    0x03E0
#define WHITE	0xFFFF
#define YELLOW	0xFFE0

int worldMap[MAP_WIDTH][MAP_HEIGHT]=
{
	{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,2,2,2,2,2,0,0,0,0,3,0,3,0,3,0,0,0,1},
	{1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,3,0,0,0,3,0,0,0,1},
	{1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,2,2,0,2,2,0,0,0,0,3,0,3,0,3,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,4,4,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,4,0,4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,4,0,0,0,0,5,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,4,0,4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,4,0,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,4,4,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

void setColor(ManagedBuffer buf, uint16_t value, int offset)
{
    uint16_t *p = (uint16_t *) &buf[offset];

	// draw up to specific point
    while(p < (uint16_t *) &buf[buf.length()]) {
        *p = value;
        p++;
    }
}

int main()
{
	uBit.init();

	ManagedBuffer img(SCREEN_WIDTH * SCREEN_HEIGHT * 2);
    Adafruit_ST7735 *lcd = new Adafruit_ST7735(LCD_PIN_CS, LCD_PIN_DC, LCD_PIN_RST,
							LCD_PIN_MOSI, LCD_PIN_MISO, LCD_PIN_SCLK);
    lcd->initR(INITR_GREENTAB);

    double posX = 22, posY = 12;      // Initial starting positions
    double dirX = -1, dirY = 0;    	  // Initial direction vector
    double planeX = 0, planeY = 0.66; // 2D Raycaster of camera plane

    double current_time = 0; // Time of current frame
    double prev_time = 0;    // Time of previous frame

	double cameraX;	// X-coordinate in camera space
	double rayDirX, rayDirY;
	double sideDistX, sideDistY;
	double deltaX, deltaY;
	double moveSpeed, rotSpeed;
	double perpWallDist;
	double frameRate;

	int mapX, mapY;
	int stepX, stepY;
	int hit, side;
	int h = SCREEN_HEIGHT;
	int w = SCREEN_WIDTH;
	int lineHeight;
	int drawStart, drawEnd;
	int color = 0;

	while(1) {
		for (int x = 0; x < w; x++) {
			cameraX = (2 * x) / (double)w - 1;
			rayDirX = dirX + (planeX * cameraX);
			rayDirY = dirY + (planeY * cameraX);

			mapX = int(posX);
			mapY = int(posY);

			deltaX = (rayDirX == 0) ? 1e30 : abs(1 / rayDirX);
			deltaY = (rayDirY == 0) ? 1e30 : abs(1 / rayDirY);

			hit = 0;

			if (rayDirX < 0) {
				stepX = -1;
				sideDistX = (posX - mapX) * deltaX;
			} else {
				stepX = 1;
				sideDistX = (mapX + 1.0 - posX) * deltaX;
			}
			if (rayDirY < 0) {
				stepY = -1;
				sideDistY = (posY - mapY) * deltaY;
			} else {
				stepY = 1;
				sideDistY = (mapY + 1.0 - posY) * deltaY;
			}

			// Perform DDA
			while (hit == 0) {
				if (sideDistX < sideDistY) {
					sideDistX += deltaX;
					mapX += stepX;
					side = 0;
				} else {
					sideDistY += deltaY;
					mapY += stepY;
					side = 1;
				}
				if (worldMap[mapX][mapY] > 0)
					hit = 1;
			}

			perpWallDist = (side == 0) ? (sideDistX - deltaX) :
							(sideDistY - deltaY);
			lineHeight = (int)(h / perpWallDist);

			drawStart = -(lineHeight / 2) + (h / 2);
			if (drawStart < 0) drawStart = 0;
			drawEnd = (lineHeight / 2) + (h / 2);
			if (drawEnd >= h) drawEnd = h - 1;

			// Add wall colour here
			switch (worldMap[mapX][mapY]) {
				case 1:  color = RED;     break;
				case 2:  color = GREEN;	  break;
				case 3:  color = BLUE;	  break;
				case 4:  color = WHITE;	  break;
				default: color = YELLOW;  break;
			}
			setColor(img, BLUE, 0);

			// draw pixels of stripe as vertical line
			lcd->sendData(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, img.getBytes());
		}
		prev_time = current_time;
		// get Ticks here
		frameRate = 1000.0 / (current_time - prev_time);
	}
}