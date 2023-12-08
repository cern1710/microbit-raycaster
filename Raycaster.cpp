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
	{20.5, 11.5, 10}, // Green light in front of playerstart
	// Green lights in every room
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

int partition(int* order, float* dist, int left, int right)
{
    float pivot = dist[right];
    int i = (left - 1);

    for (int j = left; j <= right - 1; j++) {
        if (dist[j] < pivot) {
            i++;
            swap(dist[i], dist[j]);
            swap(order[i], order[j]);
        }
    }
    swap(dist[i + 1], dist[right]);
    swap(order[i + 1], order[right]);
    return (i + 1);
}

void quickSort(int* order, float* dist, int left, int right)
{
    if (left < right) {
        int pi = partition(order, dist, left, right);
        quickSort(order, dist, left, pi - 1);
        quickSort(order, dist, pi + 1, right);
    }
}

// Sort the sprites based on distance
void sortSprites(int* order, float* dist, int amount)
{
    // Allocate memory for dynamic arrays for sorted value's arrays
    float* sortedDist = new float[amount];
    int* sortedOrder = new int[amount];

    for (int i = 0; i < amount; i++) {
        sortedDist[i] = dist[i];
        sortedOrder[i] = order[i];
    }
	quickSort(order, dist, 0, amount - 1);

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

	// Wall casting
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
	int skipStep;
	int16_t color = 0;

	// Floor casting
	float floorX, floorY;
	float floorStepX, floorStepY;
	float realFloorStepX, realFloorStepY;
	float rowDistance;
	int cellX, cellY;
	int tx, ty;

	// Sprite casting
	float invDet;
	float spriteX, spriteY;
	float transformX, transformY;

	int texY;
	int drawStartX, drawEndX;
	int drawStartY, drawEndY;
	int spriteWidth, spriteHeight;
	int dist;
	int spriteScreenX, vMoveScreen;

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

	uBit.sleep(200);	// Start up

	while (1) {
		startTime = system_timer_current_time(); // Time at start of the loop

		// Floor casting (horizontal scanline)
		p = (uint16_t *) &img[0];
		rayDirX0 = dirX - planeX;	// Ray direction for leftmost ray (x = 0)
		rayDirY0 = dirY - planeY;
		rayDirX1 = dirX + planeX;	// Ray direction for rightmost ray (x = w)
		rayDirY1 = dirY + planeY;
		floorStepX = (rayDirX1 - rayDirX0) / SCREEN_HEIGHT;
		floorStepY = (rayDirY1 - rayDirY0) / SCREEN_HEIGHT;

		for (int y = SCREEN_HALF + 1; y < SCREEN_WIDTH; y++) {
			// Horizontal distance from the camera to the floor for the current row.
			// 0.5 is the z position exactly in the middle between floor and ceiling.
			// (Vertical position of camera) / (current y-coord compared to horizon)
			rowDistance = float(SCREEN_HALF) / (y - SCREEN_HALF);

			// Calculate the real world step vector we have to add for each x (parallel to camera plane)
			// Adding step by step avoids multiplications with a weight in the inner loop
			realFloorStepX = rowDistance * floorStepX;
			realFloorStepY = rowDistance * floorStepY;

			// Real world coordinates of the leftmost column. This will be updated as we step to the right.
			floorX = posX + rowDistance * rayDirX0;
			floorY = posY + rowDistance * rayDirY0;

			// Linear interpolation for texture mapping
			for (int x = 0; x < SCREEN_HEIGHT; x++) {
				// The cell coord is simply got from the integer parts of floorX and floorY
				cellX = (int)(floorX);
				cellY = (int)(floorY);

				// Get texture coordinate from the fractional part
				tx = (int)(TEX_WIDTH * (floorX - cellX)) & (TEX_MASK);
				ty = (int)(TEX_HEIGHT * (floorY - cellY)) & (TEX_MASK);

				floorX += realFloorStepX;
				floorY += realFloorStepY;

				// Inverse ceiling and floor
				color = texture[CEILING_TEXTURE][TEX_WIDTH * ty + tx];
                p[x * SCREEN_WIDTH + y] = color; // Make a bit darker

				// Floor (symmetrical, at screenHeight - y - 1 instead of y)
				color = texture[FLOOR_TEXTURE][TEX_WIDTH * ty + tx];
                p[(x + 1) * SCREEN_WIDTH - (y + 1)] = (color >> 1) & 0x7BEF;
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
			drawStart = max(0, SCREEN_HALF - (lineHeight / 2));
			drawEnd   = min(SCREEN_WIDTH, SCREEN_HALF + (lineHeight / 2));

			texNum = worldMap[mapX][mapY] - 1;	// Subtract 1 to use texture 0

			wallX = (side == 0) ? posY + perpWallDist * rayDirY : posX + perpWallDist * rayDirX;
			wallX -= floor(wallX); // Where is the wall hit

			texX = int(wallX * float(TEX_WIDTH));	// X-coordinate of texture
			if ((side == 0 && rayDirX > 0) || (side == 1 && rayDirY < 0))
				texX = TEX_WIDTH - texX - 1;

			step = 1.0 * TEX_HEIGHT / lineHeight;
			texPos = (drawStart - SCREEN_HALF + lineHeight / 2) * step;

			if (perpWallDist < DISTANCE_THRESHOLD) { // Render wall normally for closer walls
				skipStep = 1;
			} else { // Render wall with less detail for distant walls
				skipStep = 4;
				lineHeight /= 2;  // Reduce line height for distant walls
				drawStart += lineHeight / 2;
				drawEnd -= lineHeight / 2;
			}

			// Texture rendering
			tex_ptr = (uint16_t *) &texture[texNum][texX];
			if (side == 1) {
				// Cast the texture coordinate to integer, and mask in case of overflow
				for (int y = drawStart; y < drawEnd; y += skipStep, texPos += skipStep * step) {
					p[y] = (tex_ptr[TEX_WIDTH * ((int)texPos & TEX_MASK)] & COLOR_MASK);
				}
			} else {
				for (int y = drawStart; y < drawEnd; y += skipStep, texPos += skipStep * step)
					p[y] = tex_ptr[TEX_WIDTH * ((int)texPos & TEX_MASK)];
			}
			zBuffer[x] = perpWallDist;	// Update z buffer for current x value on screen
			p += SCREEN_WIDTH;
		}

		// Sprite casting
		p = (uint16_t *) &img[0];
		invDet = 1.0 / (planeX * dirY - dirX * planeY);	// Required for correct matrix multiplication
		for (int i = 0; i < NUM_SPRITES; i++)
			spriteOrder[i] = i;
		for (int i = 0; i < NUM_SPRITES; i++)
			spriteDistance[i] = (posX - sprite[i].x) * (posX - sprite[i].x) +
								(posY - sprite[i].y) * (posY - sprite[i].y); // sqrt not taken, unneeded
		sortSprites(spriteOrder, spriteDistance, NUM_SPRITES);

		// After sorting the sprites, do the projection and draw them
		for (int i = 0; i < NUM_SPRITES; i++) {
			// Translate sprite position to relative to camera
			spriteX = sprite[spriteOrder[i]].x - posX;
			spriteY = sprite[spriteOrder[i]].y - posY;

			// Transform sprite with the inverse camera matrix
			// [ planeX   dirX ] -1             [ dirY      -dirX ]
			// [               ]     = invDet * [                 ]
			// [ planeY   dirY ]                [ -planeY  planeX ]
			transformX = invDet * (dirY * spriteX - dirX * spriteY);

			// This is the depth inside the screen, that what Z is in 3D,
			// the distance of sprite to player, matching sqrt(spriteDistance[i])
			transformY = invDet * (-planeY * spriteX + planeX * spriteY);

			spriteScreenX = int((SCREEN_HEIGHT / 2) * (1 + transformX / transformY));
			vMoveScreen = int(V_MOVE / transformY);

			// Calculate height of the sprite on screen (lowest and highest pixel to fill current stripe)
			spriteHeight = abs(int(SCREEN_WIDTH / (transformY))) / V_DIV; // transformY instead of real distance prevents fisheye
			drawStartY = max(0, -spriteHeight / 2 + SCREEN_HALF + vMoveScreen);
			drawEndY = min(SCREEN_WIDTH, spriteHeight / 2 + SCREEN_HALF + vMoveScreen);

			// Calculate width of the sprite
			spriteWidth = abs(int(SCREEN_WIDTH / (transformY))) / U_DIV;
			drawStartX = max(0, -spriteWidth / 2 + spriteScreenX);
			drawEndX = min(SCREEN_HEIGHT, spriteWidth / 2 + spriteScreenX);

			// Loop through every vertical stripe of the sprite on screen
			for (int stripe = drawStartX; stripe < drawEndX; stripe++) {
				texX = int(32 * (stripe - (-spriteWidth / 2 + spriteScreenX)) * TEX_WIDTH / spriteWidth) / 32;
				// 1) It's in front of camera plane so you don't see things behind you
				// 2) zBuffer, with perpendicular distance
				if (transformY > 0 && transformY < zBuffer[stripe]) {
					for (int y = drawStartY; y < drawEndY; y++) {	// For every pixel of the current stripe
						dist = (y - vMoveScreen) * 32 + (spriteHeight - SCREEN_WIDTH) * 16; // Avoid floating point w/ 32 and 16
						texY = ((dist * TEX_HEIGHT) / spriteHeight) / 32;
						color = texture[sprite[spriteOrder[i]].texture][TEX_WIDTH * texY + texX]; // Get current color from the texture
						if ((color & 0xFFFF) != 0)
							p[stripe * SCREEN_WIDTH + y] = color; // Black is the invisible color
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
			if (worldMap[int(posX + dirX * moveSpeed)][int(posY)] == false)
				posX += dirX * moveSpeed;
			if (worldMap[int(posX)][int(posY + dirY * moveSpeed)] == false)
				posY += dirY * moveSpeed;
		}
		// Both camera direction and camera planes must be rotated
        else if (uBit.buttonA.isPressed()) {
			oldDirX = dirX;
			oldPlaneX = planeX;
			dirX = dirX * cos(rotSpeed) - dirY * sin(rotSpeed);
			dirY = oldDirX * sin(rotSpeed) + dirY * cos(rotSpeed);
			planeX = planeX * cos(rotSpeed) - planeY * sin(rotSpeed);
			planeY = oldPlaneX * sin(rotSpeed) + planeY * cos(rotSpeed);
        }
		else if (uBit.buttonB.isPressed()) {
			oldDirX = dirX;
			oldPlaneX = planeX;
			dirX = dirX * cos(-rotSpeed) - dirY * sin(-rotSpeed);
			dirY = oldDirX * sin(-rotSpeed) + dirY * cos(-rotSpeed);
			planeX = planeX * cos(-rotSpeed) - planeY * sin(-rotSpeed);
			planeY = oldPlaneX * sin(-rotSpeed) + planeY * cos(-rotSpeed);
		}
	}
}
