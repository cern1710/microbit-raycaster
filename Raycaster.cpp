#include "MicroBit.h"
#include "Adafruit_ST7735.h"

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
#define TEX_MASK		((TEX_HEIGHT) - 1)

#define MAP_WIDTH   	24
#define MAP_HEIGHT  	24

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   160
#define SCREEN_HALF		((SCREEN_WIDTH) / 2)

#define FLOOR_TEXTURE		8
#define CEILING_TEXTURE		9
#define DISTANCE_THRESHOLD 	10

#define NUM_TEXTURES	11
#define NUM_SPRITES		19

// Parameters for scaling and moving the sprites
#define U_DIV 	1
#define V_DIV 	1
#define V_MOVE 	0.0

#define COLOR_MASK	0xEFBB

int worldMap[MAP_WIDTH][MAP_HEIGHT] =
{
  {8,8,8,8,8,8,8,8,8,8,8,4,4,6,4,4,6,4,6,4,4,4,6,4},
  {8,0,0,0,0,0,0,0,0,0,8,4,0,0,0,0,0,0,0,0,0,0,0,4},
  {8,0,3,3,0,0,0,0,0,8,8,4,0,0,0,0,0,0,0,0,0,0,0,6},
  {8,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6},
  {8,0,3,3,0,0,0,0,0,8,8,4,0,0,0,0,0,0,0,0,0,0,0,4},
  {8,0,0,0,0,0,0,0,0,0,8,4,0,0,0,0,0,6,6,6,0,6,4,6},
  {8,8,8,8,0,8,8,8,8,8,8,4,4,4,4,4,4,6,0,0,0,0,0,6},
  {7,7,7,7,0,7,7,7,7,0,8,0,8,0,8,0,8,4,0,4,0,6,0,6},
  {7,7,0,0,0,0,0,0,7,8,0,8,0,8,0,8,8,6,0,0,0,0,0,6},
  {7,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,6,0,0,0,0,0,4},
  {7,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,6,0,6,0,6,0,6},
  {7,7,0,0,0,0,0,0,7,8,0,8,0,8,0,8,8,6,4,6,0,6,6,6},
  {7,7,7,7,0,7,7,7,7,8,8,4,0,6,8,4,8,3,3,3,0,3,3,3},
  {2,2,2,2,0,2,2,2,2,4,6,4,0,0,6,0,6,3,0,0,0,0,0,3},
  {2,2,0,0,0,0,0,2,2,4,0,0,0,0,0,0,4,3,0,0,0,0,0,3},
  {2,0,0,0,0,0,0,0,2,4,0,0,0,0,0,0,4,3,0,0,0,0,0,3},
  {1,0,0,0,0,0,0,0,1,4,4,4,4,4,6,0,6,3,3,0,0,0,3,3},
  {2,0,0,0,0,0,0,0,2,2,2,1,2,2,2,6,6,0,0,5,0,5,0,5},
  {2,2,0,0,0,0,0,2,2,2,0,0,0,2,2,0,5,0,5,0,0,0,5,5},
  {2,0,0,0,0,0,0,0,2,0,0,0,0,0,2,5,0,5,0,5,0,5,0,5},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5},
  {2,0,0,0,0,0,0,0,2,0,0,0,0,0,2,5,0,5,0,5,0,5,0,5},
  {2,2,0,0,0,0,0,2,2,2,0,0,0,2,2,0,5,0,5,0,0,0,5,5},
  {2,2,2,2,1,2,2,2,2,2,2,1,2,2,2,5,5,5,5,5,5,5,5,5}
};

float zBuffer[SCREEN_HEIGHT];
int spriteOrder[NUM_SPRITES];
float spriteDistance[NUM_SPRITES];

struct Sprite {
	float x;
	float y;
	int texture;
};

Sprite sprite[NUM_SPRITES] =
{
	{20.5, 11.5, 10}, //green light in front of playerstart
	//green lights in every room
	{18.5,4.5, 10},
	{10.0,4.5, 10},
	{10.0,12.5,10},
	{3.5, 6.5, 10},
	{3.5, 20.5,10},
	{3.5, 14.5,10},
	{14.5,20.5,10},

	// Row of pillars in front of wall
	{18.5, 10.5, 9},
	{18.5, 11.5, 9},
	{18.5, 12.5, 9},

	// Barrels around the map
	{21.5, 1.5, 8},
	{15.5, 1.5, 8},
	{16.0, 1.8, 8},
	{16.2, 1.2, 8},
	{3.5,  2.5, 8},
	{9.5, 15.5, 8},
	{10.0, 15.1,8},
	{10.5, 15.8,8},
};

template <typename T>
void swap(T& a, T& b)
{
    T temp = a;
    a = b;
    b = temp;
}

void bubbleSortSprites(int* order, float* dist, int amount)
{
    bool swapped;
    for (int i = 0; i < amount - 1; i++) {
        swapped = false;
        for (int j = 0; j < amount - i - 1; j++) {
            if (dist[j] < dist[j + 1]) {
                // Swap the distances and the order
                swap(dist[j], dist[j + 1]);
                swap(order[j], order[j + 1]);
                swapped = true;
            }
        }
        if (!swapped) // Break if no two elements swapped by inner loop
            break;
    }
}

//sort the sprites based on distance
void sortSprites(int* order, float* dist, int amount)
{
    // Allocate memory for dynamic arrays for sorted value's arrays
    float* sortedDist = new float[amount];
    int* sortedOrder = new int[amount];

    for (int i = 0; i < amount; i++) {
        sortedDist[i] = dist[i];
        sortedOrder[i] = order[i];
    }
    bubbleSortSprites(sortedOrder, sortedDist, amount);

    // Restore in reverse order to go from farthest to nearest
    for (int i = 0; i < amount; i++) {
        dist[i] = sortedDist[amount - i - 1];
        order[i] = sortedOrder[amount - i - 1];
    }
    // Free the allocated memory
    delete[] sortedDist;
    delete[] sortedOrder;
}

int main()
{
	uBit.init();

	ManagedBuffer img(SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(int16_t));
    Adafruit_ST7735 *lcd = new Adafruit_ST7735(LCD_PIN_CS, LCD_PIN_DC, LCD_PIN_RST,
							LCD_PIN_MOSI, LCD_PIN_MISO, LCD_PIN_SCLK);
    lcd->initR(INITR_GREENTAB);

	uint64_t startTime, endTime;
	uint16_t *p, *tex_ptr;

    float posX = 22, posY = 11.5;      // Initial starting positions
    float dirX = -1, dirY = 0;    	  // Initial direction vector
    float planeX = 0, planeY = 0.66; // 2D Raycaster of camera plane
	float cameraX;	// X-coordinate in camera space
	float rayDirX, rayDirY;
	float sideDistX, sideDistY;
	float deltaX, deltaY;
	float moveSpeed, rotSpeed;
	float perpWallDist;
	float frameTime;
	float wallX;
	float step;
	float texPos;
	float rayDirX0, rayDirY0, rayDirX1, rayDirY1;
	float oldDirX, oldPlaneX;

	int mapX, mapY;
	int stepX, stepY;
	int hit, side;
	int texX;
	int lineHeight;
	int drawStart, drawEnd;
	int texNum;
	int reducedLineHeight;
	int reducedDrawStart, reducedDrawEnd;
	int16_t color = 0;

	// NOTE: color representation is R-B-G!!!
	// Used for texture mapping
	uint16_t texture[NUM_TEXTURES][TEX_WIDTH * TEX_HEIGHT];
	for (int x = 0; x < TEX_WIDTH; x++) {
		for (int y = 0; y < TEX_HEIGHT; y++) {
			int xorcolor = (x * 32 / TEX_WIDTH) ^ (y * 32 / TEX_HEIGHT);
			int xcolor = x - x * 32 / TEX_HEIGHT;
			// int ycolor = y - y * 32 / TEX_HEIGHT;
			int xycolor = y * 16 / TEX_HEIGHT + x * 16 / TEX_WIDTH;

			texture[0][TEX_WIDTH * y + x] = (21 * (x != y && x != TEX_WIDTH - y)) << 11; // Red with black cross
			texture[1][TEX_WIDTH * y + x] = xycolor << 11 | xycolor << 6 | xycolor; // Sloped greyscale
			texture[2][TEX_WIDTH * y + x] = xycolor << 6 | xycolor; // Sloped yellow gradient
			texture[3][TEX_WIDTH * y + x] = xorcolor << 11 | xorcolor << 6 | xorcolor; // XOR greyscale
			texture[4][TEX_WIDTH * y + x] = xorcolor; // XOR green
			texture[5][TEX_WIDTH * y + x] = (31 * (x % 4 && y % 4)) << 11; // Red bricks
			texture[6][TEX_WIDTH * y + x] = xcolor << 11; // Red gradient
			texture[7][TEX_WIDTH * y + x] = 16 + 16*32 + 16*2048; // Flat grey texture

			// Twin Peaks floor pattern
        	if (((y + (x % 8 < 4 ? x : 7 - x)) / 4) % 2 == 0)
				texture[8][TEX_WIDTH * y + x] = 0; // Black
			else
				texture[8][TEX_WIDTH * y + x] = 0xFFFF; // White

			texture[9][TEX_WIDTH * y + x] = (0b1010000001001001 * (x % 4 && y % 4)); // Blue bricks
			texture[10][TEX_WIDTH * y + x] = (20 * (x % 4 && y % 4)); // Green bricks
		}
	}

	// for (int x = 0; x < TEX_WIDTH; x++)
	// 	for (int y = 0; y < TEX_HEIGHT; y++)
			// texture[10][TEX_WIDTH * y + x] = 0b1010000001001001; // Black
	// texture[10][TEX_WIDTH * 15 + 8] = 1;
	// texture[10][TEX_WIDTH * 15 + 9] = 1;
	// texture[10][TEX_WIDTH * 14 + 8] = 1;
	// texture[10][TEX_WIDTH * 14 + 9] = 1;
	// texture[10][TEX_WIDTH * 13 + 8] = 1;
	// texture[10][TEX_WIDTH * 13 + 9] = 1;

	uBit.sleep(200);

	while (1) {
		startTime = system_timer_current_time(); // Time at start of the loop

		// Floor casting (horizontal scanline)
		for (int y = SCREEN_HALF + 1; y < SCREEN_WIDTH; y++) {
			// rayDir for leftmost ray (x = 0) and rightmost ray (x = w)
			rayDirX0 = dirX - planeX;
			rayDirY0 = dirY - planeY;
			rayDirX1 = dirX + planeX;
			rayDirY1 = dirY + planeY;

			int p_y = y - SCREEN_HALF;  // Current y-coord compared to horizon
			float posZ = SCREEN_HALF; 	// Vertical position of the camera.

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
			for (int x = 0; x < SCREEN_HEIGHT; ++x) {
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
				p = (uint16_t *) &img[0] + (x * SCREEN_WIDTH + y);
                *p = color; // make a bit darker

				// Floor (symmetrical, at screenHeight - y - 1 instead of y)
				color = texture[FLOOR_TEXTURE][TEX_WIDTH * ty + tx];
				p = (uint16_t *) &img[0] + (x * SCREEN_WIDTH + SCREEN_WIDTH - y - 1);
                *p = (color >> 1) & 0x7BEF; // make a bit darker
			}
		}

		// Wall casting
		for (int x = 0; x < SCREEN_HEIGHT; x++) {
			cameraX = (2 * x) / (float)SCREEN_HEIGHT - 1;
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

			perpWallDist = (side == 0) ? (sideDistX - deltaX) : (sideDistY - deltaY); // Calculate distance of perpendicular ray
			lineHeight = (int)(SCREEN_WIDTH / perpWallDist);	// Calculate height of line to draw on screen

			// Calculate lowest and highest pixel to fill in current stripe
			drawStart = SCREEN_HALF - (lineHeight / 2);
			drawEnd   = SCREEN_HALF + (lineHeight / 2);
			if (drawStart < 0) drawStart = 0;
			if (drawEnd >= SCREEN_WIDTH) drawEnd = SCREEN_WIDTH - 1;

			texNum = worldMap[mapX][mapY] - 1;	// Subtract 1 to use texture 0

			wallX = (side == 0) ? posY + perpWallDist * rayDirY : posX + perpWallDist * rayDirX;
			wallX -= floor(wallX); // Where is the wall hit

			texX = int(wallX * float(TEX_WIDTH));	// X-coordinate of texture
			if ((side == 0 && rayDirX > 0) || (side == 1 && rayDirY < 0))
				texX = TEX_WIDTH - texX - 1;

			step = 1.0 * TEX_HEIGHT / lineHeight;
			texPos = (drawStart - SCREEN_HALF + lineHeight / 2) * step;

			p = (uint16_t *) &img[0] + (x * SCREEN_WIDTH);
			tex_ptr = (uint16_t *) &texture[texNum][texX];

			if (perpWallDist < DISTANCE_THRESHOLD) { // Render wall normally for closer walls
				if (side == 1) {
					// Cast the texture coordinate to integer, and mask in case of overflow
					for (int y = drawStart; y < drawEnd; y++, texPos += step)
						p[y] = (tex_ptr[TEX_WIDTH * ((int)texPos & TEX_MASK)] & COLOR_MASK);
				} else {
					for (int y = drawStart; y < drawEnd; y++, texPos += step)
						p[y] = tex_ptr[TEX_WIDTH * ((int)texPos & TEX_MASK)];
				}
			} else { // Render wall with less detail for distant walls
				reducedLineHeight = lineHeight / 2; // Reduce the line height for distant walls
				reducedDrawStart = drawStart + reducedLineHeight / 2;
				reducedDrawEnd = drawEnd - reducedLineHeight / 2;

				// Skip every other pixel
				if (side == 1) {
					// Cast the texture coordinate to integer, and mask in case of overflow
					for (int y = reducedDrawStart; y < reducedDrawEnd; y += 4, texPos += 4 * step)
						p[y] = (tex_ptr[TEX_WIDTH * ((int)texPos & TEX_MASK)] & COLOR_MASK);
				} else {
					for (int y = reducedDrawStart; y < reducedDrawEnd; y += 4, texPos += 4 * step)
						p[y] = tex_ptr[TEX_WIDTH * ((int)texPos & TEX_MASK)];
				}
			}
			zBuffer[x] = perpWallDist;
		}

		// Sprite casting
		for (int i = 0; i < NUM_SPRITES; i++) {
			spriteOrder[i] = i;
			spriteDistance[i] = ((posX - sprite[i].x) * (posX - sprite[i].x) + (posY - sprite[i].y) * (posY - sprite[i].y)); //sqrt not taken, unneeded
		}

		sortSprites(spriteOrder, spriteDistance, NUM_SPRITES);
		// after sorting the sprites, do the projection and draw them
		for (int i = 0; i < NUM_SPRITES; i++) {
			//translate sprite position to relative to camera
			float spriteX = sprite[spriteOrder[i]].x - posX;
			float spriteY = sprite[spriteOrder[i]].y - posY;

			//transform sprite with the inverse camera matrix
			// [ planeX   dirX ] -1                                       [ dirY      -dirX ]
			// [               ]       =  1/(planeX*dirY-dirX*planeY) *   [                 ]
			// [ planeY   dirY ]                                          [ -planeY  planeX ]

			float invDet = 1.0 / (planeX * dirY - dirX * planeY); //required for correct matrix multiplication

			float transformX = invDet * (dirY * spriteX - dirX * spriteY);
			float transformY = invDet * (-planeY * spriteX + planeX * spriteY); //this is actually the depth inside the screen, that what Z is in 3D, the distance of sprite to player, matching sqrt(spriteDistance[i])

			int spriteScreenX = int((SCREEN_HEIGHT / 2) * (1 + transformX / transformY));

			int vMoveScreen = int(V_MOVE / transformY);

			//calculate height of the sprite on screen
			int spriteHeight = abs(int(SCREEN_WIDTH / (transformY))) / V_DIV; //using "transformY" instead of the real distance prevents fisheye
			//calculate lowest and highest pixel to fill in current stripe
			int drawStartY = -spriteHeight / 2 + SCREEN_WIDTH / 2 + vMoveScreen;
			if(drawStartY < 0) drawStartY = 0;
			int drawEndY = spriteHeight / 2 + SCREEN_WIDTH / 2 + vMoveScreen;
			if(drawEndY >= SCREEN_WIDTH) drawEndY = SCREEN_WIDTH - 1;

			//calculate width of the sprite
			int spriteWidth = abs(int (SCREEN_WIDTH / (transformY))) / U_DIV; // same as height of sprite, given that it's square
			int drawStartX = -spriteWidth / 2 + spriteScreenX;
			if (drawStartX < 0) drawStartX = 0;
			int drawEndX = spriteWidth / 2 + spriteScreenX;
			if (drawEndX > SCREEN_HEIGHT) drawEndX = SCREEN_HEIGHT;

			//loop through every vertical stripe of the sprite on screen
			for(int stripe = drawStartX; stripe < drawEndX; stripe++)
			{
				int texX = int(32 * (stripe - (-spriteWidth / 2 + spriteScreenX)) * TEX_WIDTH / spriteWidth) / 32;
				//the conditions in the if are:
				//1) it's in front of camera plane so you don't see things behind you
				//2) ZBuffer, with perpendicular distance
				if(transformY > 0 && transformY < zBuffer[stripe]) {
					for(int y = drawStartY; y < drawEndY; y++) {	//for every pixel of the current stripe
						int d = (y - vMoveScreen) * 32 - SCREEN_WIDTH * 16 + spriteHeight * 16; //256 and 128 factors to avoid floats
						int texY = ((d * TEX_HEIGHT) / spriteHeight) / 32;
						uint16_t color = texture[sprite[spriteOrder[i]].texture][TEX_WIDTH * texY + texX]; //get current color from the texture
						p = (uint16_t *) &img[0] + (stripe * SCREEN_WIDTH + y);
						if((color & 0x00FFFFFF) != 0)
							*p = color; //paint pixel if it isn't black, black is the invisible color
					}
				}
			}
		}

		lcd->sendData(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, img.getBytes());
		endTime = system_timer_current_time();
		frameTime = (endTime - startTime) / 1000.0;

		moveSpeed = frameTime * 4.0; // Constant value is in squares/second
		rotSpeed = frameTime * 2.0;  // Constant value is in radians/second

		if (uBit.buttonAB.isPressed()) {
			if(worldMap[int(posX + dirX * moveSpeed)][int(posY)] == false)
				posX += dirX * moveSpeed;
			if(worldMap[int(posX)][int(posY + dirY * moveSpeed)] == false)
				posY += dirY * moveSpeed;
		}
        else if (uBit.buttonA.isPressed()) {
			oldDirX = dirX;
			dirX = dirX * cos(rotSpeed) - dirY * sin(rotSpeed);
			dirY = oldDirX * sin(rotSpeed) + dirY * cos(rotSpeed);
			oldPlaneX = planeX;
			planeX = planeX * cos(rotSpeed) - planeY * sin(rotSpeed);
			planeY = oldPlaneX * sin(rotSpeed) + planeY * cos(rotSpeed);
        }
		else if (uBit.buttonB.isPressed()) {
			//both camera direction and camera plane must be rotated
			oldDirX = dirX;
			dirX = dirX * cos(-rotSpeed) - dirY * sin(-rotSpeed);
			dirY = oldDirX * sin(-rotSpeed) + dirY * cos(-rotSpeed);
			oldPlaneX = planeX;
			planeX = planeX * cos(-rotSpeed) - planeY * sin(-rotSpeed);
			planeY = oldPlaneX * sin(-rotSpeed) + planeY * cos(-rotSpeed);
		}
	}
}
