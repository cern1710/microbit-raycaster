#include "MicroBit.h"
#include "Adafruit_ST7735.h"

// Microbit edge connector pins where the display is connected
#define LCD_PIN_CS      2
#define LCD_PIN_DC      1
#define LCD_PIN_RST     0
#define LCD_PIN_MOSI    15
#define LCD_PIN_MISO    14
#define LCD_PIN_SCLK    13

#define STARTUP_TIME_MS	200

#define TEX_WIDTH		16
#define TEX_HEIGHT		16
#define TEX_MASK		((TEX_HEIGHT) - 1)

#define MAP_WIDTH   	24
#define MAP_HEIGHT  	24

// Note: We invert all usage of width and height
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   160
#define SCREEN_HALF		((SCREEN_WIDTH) / 2)

#define CEILING_TEXTURE				2
#define FLOOR_TEXTURE				5
#define FLOOR_DISTANCE_THRESHOLD 	10

#define NUM_TEXTURES	11
#define NUM_SPRITES		20
#define SPRITE_DISTANCE_THRESHOLD	100	// (10^2; we don't take sqrt for sprite distance)

#define BULLET_SPEED	0.2
#define BULLET_TEXTURE	10
#define BULLET_COLLISION_THRESHOLD	0.25

#define COLOR_MASK	0xEFBB
#define EMPTY_MASK	0xFFFF

#define MAX_FLOAT	(3.402823466e+38F)

#define LARGE_STEP	4

#define INITIAL_POSX	22
#define INITIAL_POSY	11.5
#define INITIAL_DIRX	-1
#define INITIAL_DIRY	0
#define INITIAL_PLANEX	0
#define INITIAL_PLANEY	0.66

#define MOV_SPEED_MULTIPLIER	4.0
#define ROT_SPEED_MULTIPLIER	2.0

#define FOV				65
#define FOV_RADIANS		((FOV) * PI / 180)

/**
 * Distance from center of camera's view to edge of camera plane along its width
 *
 * Determines how objects in the 3D space will be projected onto the 2D screen
 */
#define CAMERA_PLANE_HALF_LENGTH	(tan((FOV_RADIANS) / 2.0))

struct __attribute__((packed)) Sprite {
	float x;
	float y;
	int8_t texture;
	bool isActive;
};	// Pack this struct for SpriteContext

struct Player {
	float posX;
	float posY;
	float dirX;
	float dirY;
	float planeX;
	float planeY;
	float moveSpeed;
	float rotSpeed;
};

struct FloorContext {
	float floorX;
	float floorY;
	float floorStepX;
	float floorStepY;
	float realFloorStepX;
	float realFloorStepY;
	float rowDistance;
	float rayDirX0;
	float rayDirX1;
	float rayDirY0;
	float rayDirY1;
	int8_t cellX;
	int8_t cellY;
	int8_t texX;
	int8_t texY;
};

struct WallContext {
	float cameraX;	// X-coordinate in camera space
	float rayDirX;
	float rayDirY;
	float sideDistX;
	float sideDistY;
	float deltaX;
	float deltaY;
	float perpWallDist;
	float wallX;
	float step;
	float texPos;
	int lineHeight;
	int drawStart;
	int drawEnd;
	int8_t mapX;
	int8_t mapY;
	int8_t texX;
	int8_t texNum;
	int8_t stepX;
	int8_t stepY;
	int8_t skipStep;
	int8_t side;
};

struct SpriteContext {
	float invDet;
	float spriteX;
	float spriteY;
	float transformX;
	float transformY;
	int drawStartX;
	int drawEndX;
	int drawStartY;
	int drawEndY;
	int spriteHeight;
	int spriteScreenX;
	int8_t texX;
	int8_t texY;
	Sprite currentSprite;
};

struct BulletContext {
    float shotX;
    float shotY;
    float shotDirX;
    float shotDirY;
};

bool beginAnimation = false;	// Global state on bullet animation
bool lastButtonState = false;	// Used for shooting mechanism (P8)
float ZBuffer[SCREEN_HEIGHT];	// Used to handle sprite-wall occlusion
float spriteDistance[NUM_SPRITES];
int8_t spriteOrder[NUM_SPRITES];
uint16_t texture[NUM_TEXTURES][TEX_WIDTH * TEX_HEIGHT];

MicroBit uBit;
MicroBitPin P8(MICROBIT_ID_IO_P8, MICROBIT_PIN_P8, PIN_CAPABILITY_DIGITAL);
MicroBitPin P14(MICROBIT_ID_IO_P14, MICROBIT_PIN_P14, PIN_CAPABILITY_DIGITAL);

Sprite sprite[NUM_SPRITES] =
{
	{-1, -1, -1, false},	// Bullet sprite

	{20.5, 11.5, 6, true},
	{18.5, 4.5,  6, true},
	{10.0, 4.5,  6, true},
	{10.0, 12.5, 6, true},
	{3.5,  6.5,  6, true},
	{3.5,  20.5, 6, true},
	{3.5,  14.5, 6, true},
	{14.5, 20.5, 6, true},

	{18.5, 10.5, 5, true},
	{18.5, 11.5, 5, true},
	{18.5, 12.5, 5, true},

	{21.5, 1.5,  9, true},
	{15.5, 1.5,  9, true},
	{16.0, 1.8,  9, true},
	{16.2, 1.2,  9, true},
	{3.5,  2.5,  9, true},
	{9.5, 15.5,  9, true},
	{10.0, 15.1, 9, true},
	{10.5, 15.8, 9, true}
};

const char worldMap[MAP_WIDTH][MAP_HEIGHT] =
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

/*******************************
 * Sprite helper functions
 *******************************/

template <class T>
void swap(T& a, T& b)
{
    T temp = a;
    a = b;
    b = temp;
}

int8_t partition(int8_t* order, float* dist, int8_t left, int8_t right)
{
    float pivot = dist[right];
    int8_t i = left - 1;

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

/* Praise Tony Hoare \o/ */
void quickSort(int8_t* order, float* dist, int8_t left, int8_t right)
{
    if (left < right) {
        int8_t pi = partition(order, dist, left, right);
        quickSort(order, dist, left, pi - 1);
        quickSort(order, dist, pi + 1, right);
    }
}

/* Sort sprites from farthest to nearest using quicksort */
void sortSprites(int8_t* order, float* dist, int amount)
{
	int8_t start = 0, end = amount - 1;

	quickSort(order, dist, 0, amount - 1);

	// Reverse sorted sprites
    for (; start < end; start++, end--) {
        swap(dist[start], dist[end]);
        swap(order[start], order[end]);
    }
}

/*******************************
 * Create textures
 *******************************/

void createTextures()
{
	uint16_t xorcolor, xcolor, index;

	// Note: colours are represented in 565 R-B-G
	for (int x = 0; x < TEX_WIDTH; x++) {
		for (int y = 0; y < TEX_HEIGHT; y++) {
			xorcolor = (x * 32 / TEX_WIDTH) ^ (y * 32 / TEX_HEIGHT);
			xcolor = x - x * 32 / TEX_HEIGHT;
			index = TEX_WIDTH * y + x;

			texture[0][index] = (21 * (x != y && x != TEX_WIDTH - y)) << 11;
			texture[1][index] = xorcolor;
			texture[2][index] = (31 * (x % 4 && y % 4)) << 11;
			texture[3][index] = xcolor << 11;
			texture[4][index] = 21 + 21*32 + 22*2048;

			// Twin Peaks floor pattern
        	if (((y + (x % 8 < 4 ? x : 7 - x)) / 4) % 2 == 0)
				texture[5][index] = 0; // Black
			else
				texture[5][index] = 0xFFFF; // White

			texture[6][index] = (0b1010000011001001 * (x % 3 && y % 3));
			texture[7][index] = xorcolor << 3;
			texture[8][index] = xcolor << 7;

        	if (((x + (y % 8 < 4 ? y : 7 - y)) / 4) % 2 == 0)
				texture[9][index] = 0; // Black
			else
				texture[9][index] = 0b1111100000011111; // Yellow

			if (x >= 6 && x <= 9 && y >= 6 && y <= 9) {
				texture[10][index] = 0b1111100000000000;
			}
		}
	}
}

/*******************************
 * Floor casting
 *******************************/

/* Linear interpolation for texture mapping onto 3D environment */
void linearInterpolation(uint16_t *img_ptr, FloorContext *f, int current_y)
{
	uint16_t color;

	for (int x = 0; x < SCREEN_HEIGHT; x++) {
		// The cell coord is simply got from the integer parts of floorX and floorY
		f->cellX = (int8_t)(f->floorX);
		f->cellY = (int8_t)(f->floorY);

		// Get texture coordinate from the fractional part
		f->texX = (int8_t)(TEX_WIDTH * (f->floorX - f->cellX)) & TEX_MASK;
		f->texY = (int8_t)(TEX_HEIGHT * (f->floorY - f->cellY)) & TEX_MASK;

		f->floorX += f->realFloorStepX;
		f->floorY += f->realFloorStepY;

		// Inverse ceiling and floor
		color = texture[CEILING_TEXTURE][TEX_WIDTH * f->texY + f->texX];
		img_ptr[x * SCREEN_WIDTH + current_y] = color; // Make a bit darker

		// Floor (symmetrical, at screenHeight - y - 1 instead of y)
		color = texture[FLOOR_TEXTURE][TEX_WIDTH * f->texY + f->texX];
		img_ptr[(x + 1) * SCREEN_WIDTH - (current_y + 1)] = color;
	}
}

/* Horizontal scanline */
void floorCasting(uint16_t *img_ptr, Player *p, FloorContext *f)
{
	f->rayDirX0 = p->dirX - p->planeX;	// Ray direction for leftmost ray (x = 0)
	f->rayDirY0 = p->dirY - p->planeY;
	f->rayDirX1 = p->dirX + p->planeX;	// Ray direction for rightmost ray (x = w)
	f->rayDirY1 = p->dirY + p->planeY;
	f->floorStepX = (f->rayDirX1 - f->rayDirX0) / SCREEN_HEIGHT;
	f->floorStepY = (f->rayDirY1 - f->rayDirY0) / SCREEN_HEIGHT;

	for (int y = SCREEN_HALF + 1; y < SCREEN_WIDTH; y++) {
		// Horizontal distance from the camera to the floor for the current row.
		// 0.5 is the z position exactly in the middle between floor and ceiling.
		// (Vertical position of camera) / (current y-coord compared to horizon)
		f->rowDistance = float(SCREEN_HALF) / (y - SCREEN_HALF);

		// Calculate real world step vector added for each x (parallel to camera plane)
		f->realFloorStepX = f->rowDistance * f->floorStepX;
		f->realFloorStepY = f->rowDistance * f->floorStepY;

		// Real world coordinates of the leftmost column; updated as we step to the right
		f->floorX = p->posX + f->rowDistance * f->rayDirX0;
		f->floorY = p->posY + f->rowDistance * f->rayDirY0;

		// Linear interpolation for texture mapping
		linearInterpolation(img_ptr, f, y);
	}
}

/*******************************
 * Wall casting
 *******************************/

/* Calculates where to cast ray based on player position and direction */
void calculateRay(Player *p, WallContext *w)
{
	// Compute ray direction based on player's direction w.r.t. camera plane
	w->rayDirX = p->dirX + (p->planeX * w->cameraX);
	w->rayDirY = p->dirY + (p->planeY * w->cameraX);

	// Player's position on the map (from floating point to int)
	w->mapX = int(p->posX);
	w->mapY = int(p->posY);

	// Length of ray from current position to next x or y-side
	w->deltaX = (w->rayDirX == 0) ? MAX_FLOAT : abs(1 / w->rayDirX);
	w->deltaY = (w->rayDirY == 0) ? MAX_FLOAT : abs(1 / w->rayDirY);

	// Determines ray step direction and inital distance to first x/y side
	if (w->rayDirX < 0) {
		w->stepX = -1;
		w->sideDistX = (p->posX - w->mapX) * w->deltaX;
	} else {
		w->stepX = 1;
		w->sideDistX = (w->mapX + 1.0 - p->posX) * w->deltaX;
	}
	if (w->rayDirY < 0) {
		w->stepY = -1;
		w->sideDistY = (p->posY - w->mapY) * w->deltaY;
	} else {
		w->stepY = 1;
		w->sideDistY = (w->mapY + 1.0 - p->posY) * w->deltaY;
	}
}

/**
 * Digital differential analyser (DDA) algorithm
 *
 * Used to follow ray through the grid to see where it hits the wall
 */
void performDDA(WallContext *w)
{
	while (1) {
		// Jump to next map square in either x or y direction
		if (w->sideDistX < w->sideDistY) {
			w->sideDistX += w->deltaX;
			w->mapX += w->stepX;
			w->side = 0;
		} else {
			w->sideDistY += w->deltaY;
			w->mapY += w->stepY;
			w->side = 1;
		}
		if (worldMap[w->mapX][w->mapY] > 0) return;	// Wall was hit
	}
}

void calculateWallStripe(Player *p, WallContext *w)
{
	if (w->side == 0) {
		// perpWallDist is the distance to the wall along direction of ray
		// This prevents texture swimming
		w->perpWallDist = w->sideDistX - w->deltaX;
		w->wallX = p->posY + w->perpWallDist * w->rayDirY;
	} else {
		w->perpWallDist = w->sideDistY - w->deltaY;
		w->wallX = p->posX + w->perpWallDist * w->rayDirX;
	}
	w->lineHeight = (int)(SCREEN_WIDTH / w->perpWallDist); // Height of wall stripe to draw
	w->wallX -= floor(w->wallX); // Where the wall was hit horizontally

	// Calculate lowest and highest pixel to fill in current stripe
	w->drawStart = max(0, SCREEN_HALF - (w->lineHeight / 2));
	w->drawEnd = min(SCREEN_WIDTH, SCREEN_HALF + (w->lineHeight / 2));
}

void renderWallTexture(uint16_t *img_ptr, Player *p, WallContext *w)
{
	uint16_t *tex_ptr;
	uint16_t mask = (w->side == 1) ? COLOR_MASK : EMPTY_MASK;

	w->texX = int(w->wallX * float(TEX_WIDTH));	// X-coordinate of texture
	if ((w->side == 0 && w->rayDirX > 0) || (w->side == 1 && w->rayDirY < 0))
		w->texX = TEX_WIDTH - w->texX - 1;

	w->step = 1.0 * TEX_HEIGHT / w->lineHeight;
	w->texPos = (w->drawStart - SCREEN_HALF + w->lineHeight / 2) * w->step;

	if (w->perpWallDist < FLOOR_DISTANCE_THRESHOLD) { // Render wall normally for closer walls
		w->skipStep = 1;
	} else { // Render wall with less detail for distant walls
		w->skipStep = LARGE_STEP;	// Take a larger step for distant walls
		w->lineHeight /= 2;  // Reduce line height for distant walls
		w->drawStart += w->lineHeight / 2;
		w->drawEnd -= w->lineHeight / 2;
	}

	// Texture rendering
	// Cast the texture coordinate to integer, and mask in case of overflow
	w->texNum = worldMap[w->mapX][w->mapY] - 1;	// Subtract 1 to use texture 0
	tex_ptr = (uint16_t *) &texture[w->texNum][w->texX];
	for (int y = w->drawStart; y < w->drawEnd; y += w->skipStep) {
		img_ptr[y] = (tex_ptr[TEX_WIDTH * ((int)w->texPos & TEX_MASK)] & mask);
		w->texPos += w->skipStep * w->step;
	}
}

void wallCasting(uint16_t *img_ptr, Player *p, WallContext *w)
{
	for (int x = 0; x < SCREEN_HEIGHT; x++) {
		w->cameraX = (2 * x / float(SCREEN_HEIGHT)) - 1;
		calculateRay(p, w);
		performDDA(w);
		calculateWallStripe(p, w);
		renderWallTexture(img_ptr, p, w);

		ZBuffer[x] = w->perpWallDist;	// Update Z buffer for current x value on screen
		img_ptr += SCREEN_WIDTH;	// Update image pointer
	}
}

/*******************************
 * Sprite casting
 *******************************/

/* Convert world -> camera coords of sprites, then projects them on the 2D plane */
void calculateSpriteProjection(Player *p, SpriteContext *s)
{
	// Translate sprite position from world to position relative to camera
	s->spriteX = s->currentSprite.x - p->posX;
	s->spriteY = s->currentSprite.y - p->posY;

	// Compute the depth of the sprite relative to the player's position in the
	// screen space (Z-axis in 3D) using the inverse camera matrix
	s->transformY = s->invDet * (-p->planeY * s->spriteX + p->planeX * s->spriteY);
	if (s->transformY <= 0)  return; // Skip this sprite if it's behind player

	// Transform sprite with the inverse camera matrix
	s->transformX = s->invDet * (p->dirY * s->spriteX - p->dirX * s->spriteY);
	s->spriteScreenX = int((SCREEN_HEIGHT / 2) * (1 + s->transformX / s->transformY));

	// Calculate height of the sprite on screen (lowest and highest pixel to fill in
	// current stripe). Use transformY instead of real distance to prevent fisheye effect
	s->spriteHeight = abs(int(SCREEN_WIDTH / (s->transformY)));
	s->drawStartY = max(0, -s->spriteHeight / 2 + SCREEN_HALF);
	s->drawEndY = min(SCREEN_WIDTH, s->spriteHeight / 2 + SCREEN_HALF);

	// Calculate width of the sprite (assumes same as height)
	s->drawStartX = max(0, -s->spriteHeight / 2 + s->spriteScreenX);
	s->drawEndX = min(SCREEN_HEIGHT, s->spriteHeight / 2 + s->spriteScreenX);
}

void renderSprite(uint16_t *img_ptr, SpriteContext *s)
{
	uint16_t color;
	uint16_t *tex_ptr;

	// Loop through every vertical stripe of the sprite on screen (x)
	tex_ptr = (uint16_t *) texture[s->currentSprite.texture];
	for (int x = s->drawStartX; x < s->drawEndX; x++) {
		s->texX = (x + (s->spriteHeight / 2 - s->spriteScreenX)) * TEX_WIDTH / s->spriteHeight;
		// The conditions to draw the sprites are:
		// 1: It's in front of camera plane
		// 2: Check if not obscured by other objects using ZBuffer
		if (s->transformY > 0 && s->transformY < ZBuffer[x] && spriteDistance) {
			for (int y = s->drawStartY; y < s->drawEndY; y++) {
				// Use fixed point arithmetic to calculate texY
				s->texY = (y * 2 + s->spriteHeight - SCREEN_WIDTH) * TEX_WIDTH
							/ (2 * s->spriteHeight);
				color = tex_ptr[TEX_WIDTH * s->texY + s->texX];
				// Only paint if pixel not 0x0000 (our invisible color)
				if ((color & EMPTY_MASK) != 0)
					img_ptr[x * SCREEN_WIDTH + y] = color;
			}
		}
	}
}

bool checkBulletSpriteCollision(BulletContext *b, Sprite s)
{
    float dx = b->shotX - s.x;
    float dy = b->shotY - s.y;

    return ((dx * dx) + (dy * dy)) < BULLET_COLLISION_THRESHOLD;
}

void checkBulletCollision(BulletContext *b)
{
    if (sprite[0].isActive) {
        b->shotX += b->shotDirX * BULLET_SPEED;
        b->shotY += b->shotDirY * BULLET_SPEED;
        sprite[0] = {b->shotX, b->shotY, BULLET_TEXTURE, true};

        if (worldMap[int(b->shotX)][int(b->shotY)] > 0) {
            sprite[0].isActive = false;
        } else {
            // Check collision with other sprites
            for (int i = 1; i < NUM_SPRITES; i++) {
                if (sprite[i].isActive && checkBulletSpriteCollision(b, sprite[i])) {
                    sprite[i].isActive = false; // Deactivate the sprite
            		sprite[0].isActive = false;
					return;	// Early exit to avoid multiple collisions
                }
			}
        }
    }
}

void spriteCasting(uint16_t *img_ptr, Player *p, SpriteContext *s, BulletContext *b)
{
	// Inverse camera matrix calculation; used to transform sprite coordinates
	s->invDet = 1.0 / (p->planeX * p->dirY - p->dirX * p->planeY);

	checkBulletCollision(b);

	for (int i = 0; i < NUM_SPRITES; i++) {
		spriteOrder[i] = i;
		// Euclidean distance unnecessary; square the threshold value instead
		spriteDistance[i] = (p->posX - sprite[i].x) * (p->posX - sprite[i].x) +
							(p->posY - sprite[i].y) * (p->posY - sprite[i].y);
	}
	sortSprites(spriteOrder, spriteDistance, NUM_SPRITES);

	// Project and draw sprites after sorting
	for (int i = 0; i < NUM_SPRITES; i++) {
		// Skip sprite if it its distance to player is greater than threshold
		if (spriteDistance[i] >= SPRITE_DISTANCE_THRESHOLD) continue;

		// Skip sprite if it's not active (i.e. collided with a bullet)
		s->currentSprite = sprite[spriteOrder[i]];
		if (!s->currentSprite.isActive) continue;

		// Render sprite otherwise
		calculateSpriteProjection(p, s);
		renderSprite(img_ptr, s);
	}
}

/* Update bullet context with current user position and direction */
void bulletAnimation(Player *p, BulletContext *b)
{
	if (beginAnimation) {
		b->shotX = p->posX;
		b->shotY = p->posY;
		b->shotDirX = p->dirX;
		b->shotDirY = p->dirY;

		sprite[0] = {b->shotX, b->shotY, BULLET_TEXTURE, true};
		beginAnimation = false;
	}
}

/*******************************
 * Raycaster initialisation
 *******************************/

void initRaycaster(Adafruit_ST7735 **lcd, Player **p, FloorContext **f,
					WallContext **w, SpriteContext **s, BulletContext **b)
{
    *lcd = new Adafruit_ST7735(LCD_PIN_CS, LCD_PIN_DC, LCD_PIN_RST,
							LCD_PIN_MOSI, LCD_PIN_MISO, LCD_PIN_SCLK);
    (*lcd)->initR(INITR_GREENTAB);

	*p = new Player;
	*f = new FloorContext;
	*w = new WallContext;
	*s = new SpriteContext;
	*b = new BulletContext;

	// Initial starting positions
	(*p)->posX = INITIAL_POSX;
	(*p)->posY = INITIAL_POSY;

	// Initial direction vector
	(*p)->dirX = INITIAL_DIRX;
	(*p)->dirY = INITIAL_DIRY;

	// 2D raycaster of camera plane
	(*p)->planeX = INITIAL_PLANEX;
	(*p)->planeY = INITIAL_PLANEY;

	createTextures();

	// Initialise Microbit
	uBit.init();
	uBit.sleep(STARTUP_TIME_MS);
}

/*******************************
 * Update player movement
 *******************************/

void checkPlayerPosition(Player *p)
{
	bool currentButtonState = P8.getDigitalValue();

	if (P14.getDigitalValue()) {	// Move forward
		if (!worldMap[int(p->posX + p->dirX * p->moveSpeed)][int(p->posY)])
			p->posX += p->dirX * p->moveSpeed;
		if (!worldMap[int(p->posX)][int(p->posY + p->dirY * p->moveSpeed)])
			p->posY += p->dirY * p->moveSpeed;
	}
	else if (currentButtonState && lastButtonState && !sprite[0].isActive)	{ // Pew pew
		beginAnimation = true;
	}
	lastButtonState = currentButtonState;	// Prevent debouncing
}

void checkCameraDirection(Player *p)
{
	float oldDirX, oldPlaneX;

	// Both camera direction and camera planes must be rotated
	if (uBit.buttonA.isPressed()) {  // Turn left
		oldDirX = p->dirX;
		oldPlaneX = p->planeX;
		p->dirX = p->dirX * cos(p->rotSpeed) - p->dirY * sin(p->rotSpeed);
		p->dirY = oldDirX * sin(p->rotSpeed) + p->dirY * cos(p->rotSpeed);
		p->planeX = p->planeX * cos(p->rotSpeed) - p->planeY * sin(p->rotSpeed);
		p->planeY = oldPlaneX * sin(p->rotSpeed) + p->planeY * cos(p->rotSpeed);
	}
	else if (uBit.buttonB.isPressed()) {  // Turn right
		oldDirX = p->dirX;
		oldPlaneX = p->planeX;
		p->dirX = p->dirX * cos(-p->rotSpeed) - p->dirY * sin(-p->rotSpeed);
		p->dirY = oldDirX * sin(-p->rotSpeed) + p->dirY * cos(-p->rotSpeed);
		p->planeX = p->planeX * cos(-p->rotSpeed) - p->planeY * sin(-p->rotSpeed);
		p->planeY = oldPlaneX * sin(-p->rotSpeed) + p->planeY * cos(-p->rotSpeed);
	}
}

void updateMovement(Player *p)
{
	checkPlayerPosition(p);
	checkCameraDirection(p);
}

/**
 * Updates camera plane and direction vectors.
 *
 * Avoids FOV shifting by aligning camera plane to direction vector.
 * Additional step to normalise direction vector to have unit length.
 */
void normaliseVector(Player *p)
{
	// The camera plane is perpendicular to the direction vector
	p->planeX = CAMERA_PLANE_HALF_LENGTH * p->dirY;
	p->planeY = CAMERA_PLANE_HALF_LENGTH * -p->dirX;

	// Direction magnitude based on Euclidean distance
	// We don't need inverse sqrt here
    float dirMag = sqrt(p->dirX * p->dirX + p->dirY * p->dirY);
    if (dirMag != 0) {
        p->dirX /= dirMag;
        p->dirY /= dirMag;
    }
}

void runRaycaster(Adafruit_ST7735 **lcd, Player **p, FloorContext **f,
					WallContext **w, SpriteContext **s, BulletContext **b)
{
	ManagedBuffer img(SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(int16_t));
	uint64_t startTime, endTime;
	uint16_t *img_ptr = (uint16_t *)&img[0];
	float frameTime;

	while (1) {
		startTime = system_timer_current_time(); // Time at start of the loop

		// Main raycasting logic: floor -> wall -> sprite
		floorCasting(img_ptr, *p, *f);
		wallCasting(img_ptr, *p, *w);
		spriteCasting(img_ptr, *p, *s, *b);

		// Send image buffer to the screen
		(*lcd)->sendData(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, img.getBytes());

		// Tie our movement/rotation speed to framerate
		endTime = system_timer_current_time();
		frameTime = (endTime - startTime) / 1000.0;

		(*p)->moveSpeed = frameTime * MOV_SPEED_MULTIPLIER; // Squares/second
		(*p)->rotSpeed = frameTime * ROT_SPEED_MULTIPLIER;  // Radians/second

		updateMovement(*p);
		normaliseVector(*p);
		bulletAnimation(*p, *b);
	}
}

int main()
{
	Adafruit_ST7735 *lcd;
	Player *p;
	FloorContext *f;
	WallContext *w;
	SpriteContext *s;
	BulletContext *b;

	static_assert (sizeof(Sprite) == 10, "Size is not correct");
	static_assert (sizeof(Player) == 32, "Size is not correct");
	static_assert (sizeof(FloorContext) == 48, "Size is not correct");
	static_assert (sizeof(WallContext) == 64, "Size is not correct");
	static_assert (sizeof(SpriteContext) == 56, "Size is not correct");
	static_assert (sizeof(BulletContext) == 16, "Size is not correct");

	initRaycaster(&lcd, &p, &f, &w, &s, &b);
	runRaycaster(&lcd, &p, &f, &w, &s, &b);
}
