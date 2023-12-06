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
#define TEX_WIDTH		16
#define TEX_HEIGHT		16
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   160

int worldMap[MAP_WIDTH][MAP_HEIGHT] =
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
	{4,0,6,0,6,0,0,0,0,4,5,0,0,0,0,0,5,0,0,0,0,0,0,2},
	{4,0,0,5,0,0,0,0,0,4,4,0,6,2,0,0,0,0,0,2,2,0,2,2},
	{4,0,6,0,6,0,0,0,0,4,3,0,6,2,0,0,5,0,0,2,0,0,0,2},
	{4,0,0,0,0,0,0,0,0,4,2,0,7,2,0,0,0,0,0,2,0,0,0,2},
	{4,4,4,4,4,4,4,4,4,4,1,1,1,2,2,2,2,2,2,3,3,3,3,3}
};

bool isButtonPressedA = false;
bool isButtonPressedB = false;

void onButtonA(MicroBitEvent)
{
    isButtonPressedA = true;
}

void onButtonB(MicroBitEvent)
{
    isButtonPressedB = true;
}

int main()
{
	uBit.init();
	uBit.sleep(500);

	ManagedBuffer img(SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(int16_t));
    Adafruit_ST7735 *lcd = new Adafruit_ST7735(LCD_PIN_CS, LCD_PIN_DC, LCD_PIN_RST,
							LCD_PIN_MOSI, LCD_PIN_MISO, LCD_PIN_SCLK);
    lcd->initR(INITR_GREENTAB);

    double posX = 22, posY = 11.5;      // Initial starting positions
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
	double wallX;

	int mapX, mapY;
	int stepX, stepY;
	int hit, side;
	int w = SCREEN_HEIGHT;
	int h = SCREEN_WIDTH;
	int lineHeight;
	int drawStart, drawEnd;
	int16_t color = 0;
	int pitch;
	int texNum;

    // Register the button A press event handler
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_CLICK, onButtonA);
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_CLICK, onButtonB);

	std::vector<int16_t> texture[9];
	for(int i = 0; i < 9; i++)
		texture[i].resize(TEX_WIDTH * TEX_HEIGHT);

	// NOTE: color representation is R-B-G!!!
	for (int x = 0; x < TEX_WIDTH; x++) {
		for (int y = 0; y < TEX_HEIGHT; y++) {
			int xorcolor = (x * 32 / TEX_WIDTH) ^ (y * 32 / TEX_HEIGHT);
			int ycolor = y - y * 32 / TEX_HEIGHT;
			int xycolor = y * 16 / TEX_HEIGHT + x * 16 / TEX_WIDTH;

			texture[0][TEX_WIDTH * y + x] = (20 * (x != y && x != TEX_WIDTH - y)) << 11; // Red with black cross
			texture[1][TEX_WIDTH * y + x] = xycolor << 11 | xycolor << 6 | xycolor; // Sloped greyscale
			texture[2][TEX_WIDTH * y + x] = xycolor << 6 | xycolor; // Sloped yellow gradient
			texture[3][TEX_WIDTH * y + x] = xorcolor << 11 | xorcolor << 6 | xorcolor; // XOR greyscale
			texture[4][TEX_WIDTH * y + x] = xorcolor; // XOR green
			texture[5][TEX_WIDTH * y + x] = (31 * (x % 4 && y % 4)) << 11; // Red bricks
			texture[6][TEX_WIDTH * y + x] = ycolor << 11; // Red gradient
			texture[7][TEX_WIDTH * y + x] = 16 + 16*32 + 24*2048; // Flat grey texture
			texture[8][TEX_WIDTH * y + x] = (20 * (x % 4 && y % 4)); // Green bricks
		}
	}

	while(1) {
		startTime = system_timer_current_time(); // Time at start of the loop

		// Floor casting
		for (int y = h / 2 + 1; y < h; ++y) {
			// rayDir for leftmost ray (x = 0) and rightmost ray (x = w)
			float rayDirX0 = dirX - planeX;
			float rayDirY0 = dirY - planeY;
			float rayDirX1 = dirX + planeX;
			float rayDirY1 = dirY + planeY;

			// Current y position compared to the center of the screen (the horizon)
			int p = y - h / 2;

			// Vertical position of the camera.
			float posZ = 0.5 * h;

			// Horizontal distance from the camera to the floor for the current row.
			// 0.5 is the z position exactly in the middle between floor and ceiling.
			float rowDistance = posZ / p;

			// calculate the real world step vector we have to add for each x (parallel to camera plane)
			// adding step by step avoids multiplications with a weight in the inner loop
			float floorStepX = rowDistance * (rayDirX1 - rayDirX0) / w;
			float floorStepY = rowDistance * (rayDirY1 - rayDirY0) / w;

			// real world coordinates of the leftmost column. This will be updated as we step to the right.
			float floorX = posX + rowDistance * rayDirX0;
			float floorY = posY + rowDistance * rayDirY0;

			for(int x = 0; x < w; ++x) {
				// the cell coord is simply got from the integer parts of floorX and floorY
				int cellX = (int)(floorX);
				int cellY = (int)(floorY);

				// get the texture coordinate from the fractional part
				int tx = (int)(TEX_WIDTH * (floorX - cellX)) & (TEX_WIDTH - 1);
				int ty = (int)(TEX_HEIGHT * (floorY - cellY)) & (TEX_HEIGHT - 1);

				floorX += floorStepX;
				floorY += floorStepY;

				// choose texture and draw the pixel
				int floorTexture = 0;
				int ceilingTexture = 8;
				int16_t color;

				// floor
				color = texture[floorTexture][TEX_WIDTH * ty + tx];
				color = (color >> 1) & 0x7BEF; // make a bit darker
				uint16_t *p = (uint16_t *) &img[0] + (x * SCREEN_WIDTH + y);
                *p = color; // Update the buffer at position (x, y)

				//ceiling (symmetrical, at screenHeight - y - 1 instead of y)
				color = texture[ceilingTexture][TEX_WIDTH * ty + tx];
				color = (color >> 1) & 0x7BEF; // make a bit darker
				p = (uint16_t *) &img[0] + (x * SCREEN_WIDTH + SCREEN_WIDTH - y - 1);
                *p = color; // Update the buffer at position (x, y)
			}
		}
		// Wall casting
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

			pitch = 0;
			drawStart = -(lineHeight / 2) + (h / 2) + pitch;
			drawEnd = (lineHeight / 2) + (h / 2) + pitch;
			if (drawStart < 0) drawStart = 0;
			if (drawEnd >= h) drawEnd = h - 1;

			texNum = worldMap[mapX][mapY] - 1;

			wallX = (side == 0) ? posY + perpWallDist * rayDirY :
			 		posX + perpWallDist * rayDirX;
			wallX -= floor(wallX); // Remove the integer part

			int texX = int(wallX * double(TEX_WIDTH));
			if(side == 0 && rayDirX > 0) texX = TEX_WIDTH - texX - 1;
			if(side == 1 && rayDirY < 0) texX = TEX_WIDTH - texX - 1;
			double step = 1.0 * TEX_HEIGHT / lineHeight;
			double texPos = (drawStart - pitch - h / 2 + lineHeight / 2) * step;

			for (int y = drawStart; y < drawEnd; y++) {
				// Cast the texture coordinate to integer, and mask with (texHeight - 1) in case of overflow
				int texY = (int)texPos & (TEX_HEIGHT - 1);
				texPos += step;
				color = texture[texNum][TEX_WIDTH * texY + texX];
				//make color darker for y-sides: R, G and B byte each divided through two with a "shift" and an "and"
				// if (side == 1)
				// 	color = ((color >> 1) & 0xFFEF);
                uint16_t *p = (uint16_t *) &img[0] + (x * SCREEN_WIDTH + y);
                *p = color; // Update the buffer at position (x, y)
			}
		}
		lcd->sendData(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, img.getBytes());
		endTime = system_timer_current_time();
		frameTime = (endTime - startTime) / 1000.0;

		moveSpeed = frameTime * 5.0; //the constant value is in squares/second
    	rotSpeed = frameTime * 3.0; //the constant value is in radians/second

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