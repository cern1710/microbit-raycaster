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

#define TEX_WIDTH		16
#define TEX_HEIGHT		16
#define MAP_WIDTH   	24
#define MAP_HEIGHT  	24
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   160
#define TEX_MASK		((TEX_HEIGHT) - 1)

#define FLOOR_TEXTURE		8
#define CEILING_TEXTURE		9
#define DISTANCE_THRESHOLD 	10

#define NUM_TEXTURES	11

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

// TODO: create a door animation by adjusting line height for certain walls (based on collision)
// TODO: create less fucky textures
// TODO: optimize program (it gets slow when it's close to a wall)
int main()
{
	uBit.init();

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
	double step;
	double texPos;

	int mapX, mapY;
	int stepX, stepY;
	int hit, side;
	int texX, texY;
	int lineHeight;
	int drawStart, drawEnd;
	int16_t color = 0;
	int texNum;
	uint16_t *p;
	int reducedLineHeight;
	int reducedDrawStart, reducedDrawEnd;

	float rayDirX0, rayDirY0, rayDirX1, rayDirY1;

	std::vector<int16_t> texture[NUM_TEXTURES];
	for(int i = 0; i < NUM_TEXTURES; i++)
		texture[i].resize(TEX_WIDTH * TEX_HEIGHT);

	// NOTE: color representation is R-B-G!!!
	// Used for texture mapping
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
			texture[7][TEX_WIDTH * y + x] = 16 + 16*32 + 16*2048; // Flat grey texture
			texture[8][TEX_WIDTH * y + x] = (20 * (x % 4 && y % 4)); // Green bricks
			texture[9][TEX_WIDTH * y + x] = (20 * (x % 4 && y % 4)) << 5; // Blue bricks
		}
	}

	uBit.sleep(200);

	while(1) {
		startTime = system_timer_current_time(); // Time at start of the loop

		// Floor casting (horizontal scanline)
		for (int y = SCREEN_WIDTH / 2 + 1; y < SCREEN_WIDTH; y++) {
			// rayDir for leftmost ray (x = 0) and rightmost ray (x = w)
			rayDirX0 = dirX - planeX;
			rayDirY0 = dirY - planeY;
			rayDirX1 = dirX + planeX;
			rayDirY1 = dirY + planeY;

			int p_y = y - SCREEN_WIDTH / 2;  // Current y-coord compared to horizon
			float posZ = 0.5 * SCREEN_WIDTH; // Vertical position of the camera.

			// Horizontal distance from the camera to the floor for the current row.
			// 0.5 is the z position exactly in the middle between floor and ceiling.
			float rowDistance = posZ / p_y;

			// calculate the real world step vector we have to add for each x (parallel to camera plane)
			// adding step by step avoids multiplications with a weight in the inner loop
			float floorStepX = rowDistance * (rayDirX1 - rayDirX0) / SCREEN_HEIGHT;
			float floorStepY = rowDistance * (rayDirY1 - rayDirY0) / SCREEN_HEIGHT;

			// real world coordinates of the leftmost column. This will be updated as we step to the right.
			float floorX = posX + rowDistance * rayDirX0;
			float floorY = posY + rowDistance * rayDirY0;

			// Linear interpolation for texture mapping
			for(int x = 0; x < SCREEN_HEIGHT; ++x) {
				// the cell coord is simply got from the integer parts of floorX and floorY
				int cellX = (int)(floorX);
				int cellY = (int)(floorY);

				// get the texture coordinate from the fractional part
				int tx = (int)(TEX_WIDTH * (floorX - cellX)) & (TEX_WIDTH - 1);
				int ty = (int)(TEX_HEIGHT * (floorY - cellY)) & (TEX_HEIGHT - 1);

				floorX += floorStepX;
				floorY += floorStepY;

				// We've inversed ceiling and floor!!!
				color = texture[CEILING_TEXTURE][TEX_WIDTH * ty + tx];
				color = (color >> 1) & 0x7BEF; // make a bit darker
				p = (uint16_t *) &img[0] + (x * SCREEN_WIDTH + y);
                *p = color; // Update the buffer at position (x, y)

				//ceiling (symmetrical, at screenHeight - y - 1 instead of y)
				color = texture[FLOOR_TEXTURE][TEX_WIDTH * ty + tx];
				color = (color >> 1) & 0x7BEF; // make a bit darker
				p = (uint16_t *) &img[0] + (x * SCREEN_WIDTH + SCREEN_WIDTH - y - 1);
                *p = color; // Update the buffer at position (x, y)
			}
		}

		// Wall casting
		for (int x = 0; x < SCREEN_HEIGHT; x++) {
			cameraX = (2 * x) / (double)SCREEN_HEIGHT - 1;
			rayDirX = dirX + (planeX * cameraX);
			rayDirY = dirY + (planeY * cameraX);

			mapX = int(posX);
			mapY = int(posY);

			// Length of ray from current position to next x or y-side
			deltaX = (rayDirX == 0) ? 1e30 : abs(1 / rayDirX);
			deltaY = (rayDirY == 0) ? 1e30 : abs(1 / rayDirY);

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

			// Digital differential analyzer (DDA) algorithm
			hit = 0;
			while (hit == 0) {
				// Jump to next map square in either x or y direction
				if (sideDistX < sideDistY) {
					sideDistX += deltaX;
					mapX += stepX;
					side = 0;
				} else {
					sideDistY += deltaY;
					mapY += stepY;
					side = 1;
				}
				if (worldMap[mapX][mapY] > 0) hit = 1;
			}

			perpWallDist = (side == 0) ? (sideDistX - deltaX) :
							(sideDistY - deltaY);	// Calculate distance of perpendicular ray
			lineHeight = (int)(SCREEN_WIDTH / perpWallDist);	// Calculate height of line to draw on screen

			// Calculate lowest and highest pixel to fill in current stripe
			drawStart = -(lineHeight / 2) + (SCREEN_WIDTH / 2);
			drawEnd = (lineHeight / 2) + (SCREEN_WIDTH / 2);
			if (drawStart < 0) drawStart = 0;
			if (drawEnd >= SCREEN_WIDTH) drawEnd = SCREEN_WIDTH - 1;

			texNum = worldMap[mapX][mapY] - 1;	// Subtract 1 to use texture 0

			wallX = (side == 0) ? posY + perpWallDist * rayDirY :
			 		posX + perpWallDist * rayDirX;
			wallX -= floor(wallX); // Where is the wall hit

			texX = int(wallX * double(TEX_WIDTH));	// X-coordinate of texture
			if ((side == 0 && rayDirX > 0) || (side == 1 && rayDirY < 0))
				texX = TEX_WIDTH - texX - 1;

			step = 1.0 * TEX_HEIGHT / lineHeight;
			texPos = (drawStart - SCREEN_WIDTH / 2 + lineHeight / 2) * step;

			p = (uint16_t *) &img[0] + (x * SCREEN_WIDTH);
			if (perpWallDist < DISTANCE_THRESHOLD) { // Render wall normally for closer walls
				for (int y = drawStart; y < drawEnd; y++, texPos += step) {
					// Cast the texture coordinate to integer, and mask in case of overflow
					texY = (int)texPos & TEX_MASK;
					color = texture[texNum][TEX_WIDTH * texY + texX];
					color = (side == 1) ? (color & 0xEFBB) : color; // Make color darker for y-sides
					p[y] = color; // Update the buffer at position (x, y)
				}
			} else { // Render wall with less detail for distant walls
				reducedLineHeight = lineHeight / 2; // Reduce the line height for distant walls
				reducedDrawStart = drawStart + reducedLineHeight / 2;
				reducedDrawEnd = drawEnd - reducedLineHeight / 2;

				for (int y = reducedDrawStart; y < reducedDrawEnd; y += 4, texPos += step) { // Skip every other pixel
					// Cast the texture coordinate to integer, and mask in case of overflow
					texY = (int)texPos & TEX_MASK;
					color = texture[texNum][TEX_WIDTH * texY + texX];
					p[y] = color; // Update the buffer at position (x, y)
				}
			}
		}
		lcd->sendData(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, img.getBytes());
		endTime = system_timer_current_time();
		frameTime = (endTime - startTime) / 1000.0;

		moveSpeed = frameTime * 5.0; //the constant value is in squares/second
		if (perpWallDist < 1.5)
			rotSpeed = frameTime * 4.0;
		else if (perpWallDist < 1.0)
			rotSpeed = frameTime * 5.0;
		else if (perpWallDist < 0.5)
			rotSpeed = frameTime * 6.0;
    	else
			rotSpeed = frameTime * 3.0; //the constant value is in radians/second

        if (uBit.buttonA.isPressed()) {
			if(worldMap[int(posX + dirX * moveSpeed)][int(posY)] == false)
				posX += dirX * moveSpeed;
			if(worldMap[int(posX)][int(posY + dirY * moveSpeed)] == false)
				posY += dirY * moveSpeed;
        }
		if (uBit.buttonB.isPressed()) {
			//both camera direction and camera plane must be rotated
			double oldDirX = dirX;
			dirX = dirX * cos(-rotSpeed) - dirY * sin(-rotSpeed);
			dirY = oldDirX * sin(-rotSpeed) + dirY * cos(-rotSpeed);
			double oldPlaneX = planeX;
			planeX = planeX * cos(-rotSpeed) - planeY * sin(-rotSpeed);
			planeY = oldPlaneX * sin(-rotSpeed) + planeY * cos(-rotSpeed);
		}
	}
}