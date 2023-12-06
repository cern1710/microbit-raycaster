#include "MicroBit.h"
#include "Adafruit_ST7735.h"
#include <vector>

MicroBit uBit;

// Define the MICROBIT EDGE CONNECTOR pins where the display is connected...
#define LCD_PIN_CS      2
#define LCD_PIN_DC      1
#define LCD_PIN_RST     0
#define LCD_PIN_MOSI    15
#define LCD_PIN_MISO    14
#define LCD_PIN_SCLK    13

#define MAP_WIDTH   	24
#define MAP_HEIGHT  	24
#define TEX_WIDTH		64
#define TEX_HEIGHT		64
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   160

#define RED     0xFC00
#define GREEN   0x001F
#define BLUE    0x03E0
#define WHITE	0xFFFF
#define YELLOW	0xFEE0
#define BLACK 	0x0000

#define MAX_VIEW_DISTANCE	5

int worldMap[MAP_WIDTH][MAP_HEIGHT]=
{
	{4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,7,7,7,7,7,7,7,7},
	{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7,0,0,0,0,0,0,7},
	{4,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7},
	{4,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7},
	{4,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,7,0,0,0,0,0,0,7},
	{4,0,4,0,0,0,0,5,5,5,5,5,5,5,5,5,7,7,0,7,7,7,7,7},
	{4,0,5,0,0,0,0,5,0,5,0,5,0,5,0,5,7,0,0,0,7,7,7,1},
	{4,0,6,0,0,0,0,5,0,0,0,0,0,0,0,5,7,0,0,0,0,0,0,8},
	{4,0,7,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7,7,7,1},
	{4,0,8,0,0,0,0,5,0,0,0,0,0,0,0,5,7,0,0,0,0,0,0,8},
	{4,0,0,0,0,0,0,5,0,0,0,0,0,0,0,5,7,0,0,0,7,7,7,1},
	{4,0,0,0,0,0,0,5,5,5,5,0,5,5,5,5,7,7,7,7,7,7,7,1},
	{6,6,6,6,6,6,6,6,6,6,6,0,6,6,6,6,6,6,6,6,6,6,6,6},
	{8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
	{6,6,6,6,6,6,0,6,6,6,6,0,6,6,6,6,6,6,6,6,6,6,6,6},
	{4,4,4,4,4,4,0,4,4,4,6,0,6,2,2,2,2,2,2,2,3,3,3,3},
	{4,0,0,0,0,0,0,0,0,4,6,0,6,2,0,0,0,0,0,2,0,0,0,2},
	{4,0,0,0,0,0,0,0,0,0,0,0,6,2,0,0,5,0,0,2,0,0,0,2},
	{4,0,0,0,0,0,0,0,0,4,6,0,6,2,0,0,0,0,0,2,2,0,2,2},
	{4,0,6,0,6,0,0,0,0,4,6,0,0,0,0,0,5,0,0,0,0,0,0,2},
	{4,0,0,5,0,0,0,0,0,4,6,0,6,2,0,0,0,0,0,2,2,0,2,2},
	{4,0,6,0,6,0,0,0,0,4,6,0,6,2,0,0,5,0,0,2,0,0,0,2},
	{4,0,0,0,0,0,0,0,0,4,6,0,6,2,0,0,0,0,0,2,0,0,0,2},
	{4,4,4,4,4,4,4,4,4,4,1,1,1,2,2,2,2,2,2,3,3,3,3,3}
};

uint32_t buffer[SCREEN_HEIGHT][SCREEN_WIDTH];

void setColor(ManagedBuffer buf, uint16_t value, int offset)
{
    uint16_t *p = (uint16_t *) &buf[offset];

	// draw up to specific point
    while(p < (uint16_t *) &buf[buf.length()]) {
        *p = value;
        p++;
    }
}

void verticalLine(ManagedBuffer buf, int x, int drawStart, int drawEnd, int color)
{
    uint16_t *p = (uint16_t *) &buf[0] + (x * SCREEN_WIDTH + drawStart);

    while (drawStart <= drawEnd) {
        *p = color;
        p++;
        drawStart++;
    }
}

void texturedVerticalLine(ManagedBuffer buf, int x, int drawStart, int drawEnd,
				int wallType, double wallX, double perpWallDist, int side, int lineHeight,
				double rayDirX, double rayDirY, double posX, double posY)
{
    uint16_t *p = (uint16_t *) &buf[0] + (x * SCREEN_WIDTH + drawStart);

    const int textureSize = 8;

    // Correct the wallX value to ensure it's within the range [0,1]
	wallX = (side == 1) ? posX + perpWallDist * rayDirX :
			posY + perpWallDist * rayDirY;
    wallX -= floor(wallX); // Remove the integer part

    // Calculate the offset on the texture
    int texX = int(wallX * double(textureSize));
    if ((side == 0 && rayDirX > 0) || (side == 1 && rayDirY < 0)) {
        texX = textureSize - texX - 1;
    }

	// Textures are swimming here...
    for (int y = drawStart; y < drawEnd; y++) {
        int d = (y - drawStart) * 128;
        int texY = ((d * textureSize) / lineHeight) / 128;
        texY %= textureSize; // Wrap around the texture

        int color;
        // Simple checkerboard pattern for texture
        if ((texX % 2 == 0) ^ (texY % 2 == 0)) {
            color = wallType;
        } else {
            color = BLACK;
        }

        *p = color;
        p++;
    }
}

bool isButtonPressedA = false;
bool isButtonPressedB = false;

void onButtonA(MicroBitEvent) {
    isButtonPressedA = true;
}

void onButtonB(MicroBitEvent) {
    isButtonPressedB = true;
}

int main()
{
	uBit.init();
	uBit.sleep(500);

	ManagedBuffer img(SCREEN_WIDTH * SCREEN_HEIGHT * 2);
    Adafruit_ST7735 *lcd = new Adafruit_ST7735(LCD_PIN_CS, LCD_PIN_DC, LCD_PIN_RST,
							LCD_PIN_MOSI, LCD_PIN_MISO, LCD_PIN_SCLK);
    lcd->initR(INITR_GREENTAB);

    double posX = 22, posY = 12;      // Initial starting positions
    double dirX = -1, dirY = 0;    	  // Initial direction vector
    double planeX = 0, planeY = 0.66; // 2D Raycaster of camera plane

	uint64_t startTime, endTime;

	double cameraX;	// X-coordinate in camera space
	double rayDirX, rayDirY;
	double sideDistX, sideDistY;
	double deltaX, deltaY;
	double moveSpeed, rotSpeed;
	double perpWallDist;
	double frameTime;

	int mapX, mapY;
	int stepX, stepY;
	int hit, side;
	int w = SCREEN_HEIGHT;
	int h = SCREEN_WIDTH;
	int lineHeight;
	int drawStart, drawEnd;
	int color = 0;

    // Register the button A press event handler
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_CLICK, onButtonA);
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_CLICK, onButtonB);

	std::vector<uint32_t> texture[8];
	for(int x = 0; x < TEX_WIDTH; x++)
		for(int y = 0; y < TEX_HEIGHT; y++) {
			int xorcolor = (x * 256 / TEX_WIDTH) ^ (y * 256 / TEX_HEIGHT);
			//int xcolor = x * 256 / texWidth;
			int ycolor = y * 256 / TEX_HEIGHT;
			int xycolor = y * 128 / TEX_HEIGHT + x * 128 / TEX_WIDTH;
			texture[0][TEX_WIDTH * y + x] = 65536 * 254 * (x != y && x != TEX_WIDTH - y); //flat red texture with black cross
			texture[1][TEX_WIDTH * y + x] = xycolor + 256 * xycolor + 65536 * xycolor; //sloped greyscale
			texture[2][TEX_WIDTH * y + x] = 256 * xycolor + 65536 * xycolor; //sloped yellow gradient
			texture[3][TEX_WIDTH * y + x] = xorcolor + 256 * xorcolor + 65536 * xorcolor; //xor greyscale
			texture[4][TEX_WIDTH * y + x] = 256 * xorcolor; //xor green
			texture[5][TEX_WIDTH * y + x] = 65536 * 192 * (x % 16 && y % 16); //red bricks
			texture[6][TEX_WIDTH * y + x] = 65536 * ycolor; //red gradient
			texture[7][TEX_WIDTH * y + x] = 128 + 256 * 128 + 65536 * 128; //flat grey texture
		}

	while(1) {
		startTime = system_timer_current_time(); // Time at start of the loop
        // Clear the screen at the start of each frame
        for (int i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT * 2; i += 2) {
            img[i] = BLACK & 0xFF;
            img[i + 1] = (BLACK >> 8) & 0xFF;
        }
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

			double wallHeightFactor = 1.0; // Default height factor
			switch (worldMap[mapX][mapY]) {
				case 1:  color = RED;     break;
				case 2:  color = GREEN;	  wallHeightFactor = 1.5; break;
				case 3:  color = BLUE;	  break;
				case 4:  color = WHITE;	  break;
				default: color = YELLOW;  break;
			}

			perpWallDist = (side == 0) ? (sideDistX - deltaX) :
							(sideDistY - deltaY);
			lineHeight = (int)(h / perpWallDist * wallHeightFactor);
			drawStart = -(lineHeight / 2) + (h / 2);
			drawEnd = (lineHeight / 2) + (h / 2);
			if (drawStart < 0) drawStart = 0;
			if (drawEnd >= h) drawEnd = h - 1;

			if (side == 1) {
				int red = ((color & 0xF800) >> 1) | 0x0400;
				int green = ((color & 0x07E0) >> 1) | 0x0040;
				int blue = ((color & 0x001F) >> 1) | 0x0001;

				color = (red & 0xF800) | (green & 0x07E0) | (blue & 0x001F);
			}
			// Apply distance shading (this is cooked)
			// float shade = 1.0 - (perpWallDist / MAX_VIEW_DISTANCE);
			// shade = (shade < 0.3) ? 0.3 : shade; // Clamp minimum shade

			// int red = (int)((((color & 0xF800) >> 11) * shade)) << 11;
			// int green = (int)((((color & 0x07E0) >> 5) * shade)) << 5;
			// int blue = (color & 0x001F) * shade;

			// color = red | green | blue;

			if (worldMap[mapX][mapY] == 3) { // For textured walls
				double wallX; // X coordinate on the texture
				if (side == 0) wallX = posY + perpWallDist * rayDirY;
				else           wallX = posX + perpWallDist * rayDirX;
				wallX -= floor((wallX)); // Remove the integer part to get the fractional part

				texturedVerticalLine(img, x, drawStart, drawEnd, BLUE, wallX,
					perpWallDist, side, lineHeight, rayDirX, rayDirY, posX, posY);
			} else
			verticalLine(img, x, drawStart, drawEnd, color);
		}
		endTime = system_timer_current_time();
		frameTime = (endTime - startTime) / 1000.0;

		moveSpeed = frameTime * 5.0; //the constant value is in squares/second
    	rotSpeed = frameTime * 3.0; //the constant value is in radians/second

		lcd->sendData(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, img.getBytes());

        if (uBit.buttonA.isPressed()) {
			if(worldMap[int(posX + dirX * moveSpeed)][int(posY)] == false)
				posX += dirX * moveSpeed;
			if(worldMap[int(posX)][int(posY + dirY * moveSpeed)] == false)
				posY += dirY * moveSpeed;
			isButtonPressedA = false;
        }
		if (uBit.buttonB.isPressed()) {
			//both camera direction and camera plane must be rotated
			double oldDirX = dirX;
			dirX = dirX * cos(-rotSpeed) - dirY * sin(-rotSpeed);
			dirY = oldDirX * sin(-rotSpeed) + dirY * cos(-rotSpeed);
			double oldPlaneX = planeX;
			planeX = planeX * cos(-rotSpeed) - planeY * sin(-rotSpeed);
			planeY = oldPlaneX * sin(-rotSpeed) + planeY * cos(-rotSpeed);
			isButtonPressedB = false;
		}
	}

}