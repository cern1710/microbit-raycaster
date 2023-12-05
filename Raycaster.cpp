#include "MicroBit.h"

MicroBit uBit;

#define MAP_WIDTH   	24
#define MAP_HEIGHT  	24
#define SCREEN_WIDTH    160
#define SCREEN_HEIGHT   128

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

int main()
{
	uBit.init();

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

	while(1) {
		for (int x = 0; x < w; x++) {
			cameraX = (2 * x) / (double)w - 1;
			rayDirX = dirX + (planeX * cameraX);
			rayDirY = dirY + (planeY * cameraX);

			mapX = int(posX);
			mapY = int(posY);

			deltaX = (rayDirX == 0) ? 1e30 : std::abs(1 / rayDirX);
			deltaY = (rayDirY == 0) ? 1e30 : std::abs(1 / rayDirY);

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

			drawStart = -lineHeight / 2 + h / 2;
			if (drawStart < 0) drawStart = 0;
			drawEnd = lineHeight / 2 + h / 2;
			if (drawEnd >= h) drawEnd = h - 1;

			// Add wall colour here

			// draw pixels of stripe as vertical line
		}
		prev_time = current_time;
		// get Ticks here
		frameRate = 1000.0 / (current_time - prev_time);
	}
}