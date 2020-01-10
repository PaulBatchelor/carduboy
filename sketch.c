#include "arduboy.h"

ArduboyC arduboy;

#define AVATAR_SPEED 5

typedef struct {
    int x, y;
} avatar;

avatar avi;

void avatar_init(avatar *a)
{
    a->x = 0;
    a->y = 32;
}

void avatar_draw(avatar *a, ArduboyC *ab)
{
    arduboy_draw_rect(ab, a->x, a->y, 8, 8, WHITE);
}

void avatar_move_right(avatar *a)
{
    a->x += AVATAR_SPEED;
    if(a->x > 120) a->x = 120;
}

void avatar_move_left(avatar *a)
{
    a->x -= AVATAR_SPEED;
    if(a->x < 0) a->x = 0;
}

void avatar_move_up(avatar *a)
{
    a->y -= AVATAR_SPEED;
    if(a->y < 0) a->y = 0;
}

void avatar_move_down(avatar *a)
{
    a->y += AVATAR_SPEED;
    if(a->y > 56) a->y = 56;
}

void setup(void)
{
    arduboy_init(&arduboy);
    arduboy_begin(&arduboy);
    arduboy_setframerate(&arduboy, 30);
    avatar_init(&avi);
}

void loop(void)
{
    int bs;
    int rc;

    rc = arduboy_nextframe(&arduboy);

    if (!rc) return;

    arduboy_clear(&arduboy);

    bs = arduboy_buttons_state();

    if (bs & RIGHT_BUTTON) {
        avatar_move_right(&avi);
    } else if (bs & LEFT_BUTTON) {
        avatar_move_left(&avi);
    } else if (bs & UP_BUTTON) {
        avatar_move_up(&avi);
    } else if (bs & DOWN_BUTTON) {
        avatar_move_down(&avi);
    }


    avatar_draw(&avi, &arduboy);
    arduboy_display(&arduboy, true);
}
