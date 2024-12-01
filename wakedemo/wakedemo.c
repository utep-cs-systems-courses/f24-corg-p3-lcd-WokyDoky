//Old and working Debug text

#include <msp430.h>
#include <libTimer.h>
#include "lcdutils.h"
#include "lcddraw.h"

// WARNING: LCD DISPLAY USES P1.0.  Do not touch!!! 

#define LED BIT6		/* note that bit zero req'd for display */

#define SW1 1
#define SW2 2
#define SW3 4
#define SW4 8

#define SWITCHES 15

#define PADDLE_WIDTH 30
#define PADDLE_HEIGHT 10
#define PADDLE_SPEED 5

// Screen dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 160

// Global variables for paddle
short paddleX = (SCREEN_WIDTH - PADDLE_WIDTH) / 2;  // Start at center
short paddleY = 10;  // Paddle positioned near the top of the screen
short paddleY2 = SCREEN_HEIGHT - 20;

//Added wheel
int colorWheel [] = {COLOR_RED, COLOR_GREEN, COLOR_BLACK};
int colorFromWheel = 0;
int COLOR_OF_BALL = COLOR_WHITE;
char blue = 31, green = 0, red = 31;
unsigned char step = 0;

char lives = 51;

static char 
switch_update_interrupt_sense()
{
  char p2val = P2IN;
  /* update switch interrupt to detect changes from current buttons */
  P2IES |= (p2val & SWITCHES);	/* if switch up, sense down */
  P2IES &= (p2val | ~SWITCHES);	/* if switch down, sense up */
  return p2val;
}

void 
switch_init()			/* setup switch */
{  
  P2REN |= SWITCHES;		/* enables resistors for switches */
  P2IE |= SWITCHES;		/* enable interrupts from switches */
  P2OUT |= SWITCHES;		/* pull-ups for switches */
  P2DIR &= ~SWITCHES;		/* set switches' bits for input */
  switch_update_interrupt_sense();
}

int switches = 0;

void
switch_interrupt_handler()
{
  char p2val = switch_update_interrupt_sense();
  switches = ~p2val & SWITCHES;
}



// axis zero for col, axis 1 for row

short drawPos[2] = {1,10}, controlPos[2] = {2, 10};
short colVelocity = 3, colLimits[2] = {1, SCREEN_WIDTH-20};

short drawPosSec[2] = {20,100};
short control[2] = {21,100};

short colVeSecond = -3;
short colera[2] = {1, SCREEN_WIDTH-20};

int sizeOfBall = 10;
int sizeOfBallSec = 10;

void draw_ball(int col, int row, unsigned short color, int sizeOfBall)
{
  fillRectangle(col-1, row-1, sizeOfBall, sizeOfBall, color);
}

void screen_update_ball()
{
  char position_changed = (drawPos[0] != controlPos[0]) | (drawPos[1] != controlPos[1]);
  sizeOfBall += position_changed;
  draw_ball(drawPos[0], drawPos[1], position_changed ? COLOR_BLUE : COLOR_WHITE, sizeOfBall);
  sizeOfBall -= position_changed;
  drawPos[0] = controlPos[0];
  drawPos[1] = controlPos[1];
  draw_ball(drawPos[0], drawPos[1], COLOR_WHITE, sizeOfBall);
}
void screen_update_second_ball(){
  char position_changed = (drawPosSec[0] != control[0]) | (drawPosSec[1] != control[1]);

  // Increment size only if position changed
  sizeOfBallSec += position_changed;
  draw_ball(drawPosSec[0], drawPosSec[1], position_changed ? COLOR_BLUE : colorWheel[colorFromWheel], sizeOfBallSec);

  drawPosSec[0] = control[0];
  drawPosSec[1] = control[1];

  draw_ball(drawPosSec[0], drawPosSec[1], colorWheel[colorFromWheel], sizeOfBallSec);

  sizeOfBallSec -= position_changed;
}

void draw_paddle(int x, int y, unsigned short color) {
  fillRectangle(x, y, PADDLE_WIDTH, PADDLE_HEIGHT, color);
  fillRectangle(x, paddleY2, PADDLE_WIDTH, PADDLE_HEIGHT, color);
}

// Function to update paddle position
void update_paddle_position() {
  draw_paddle(paddleX, paddleY, COLOR_BLUE);

  // Update paddle position based on switches
  if (switches & SW1) { // Move left
    if (paddleX > 0) { // Ensure paddle doesn't go off-screen
      paddleX -= PADDLE_SPEED;
    }
  }
  if (switches & SW2) { // Move right
    if (paddleX < SCREEN_WIDTH - PADDLE_WIDTH) { // Ensure paddle doesn't go off-screen
      paddleX += PADDLE_SPEED;
    }
  }

  // Redraw the paddle in the new position
  draw_paddle(paddleX, paddleY, COLOR_WHITE);
}


short redrawScreen = 1;
u_int controlFontColor = COLOR_GREEN;

short rowVelocity = 3, rowLimits[2] = {1, SCREEN_HEIGHT-20};
short rowVeSecond = -3;
short rowera[2] = {1, SCREEN_HEIGHT-20};


//Absolute big change:

int is_ball_colliding_with_paddle(short ballX, short ballY, int ballSize) {
  // Check if the ball intersects the paddle's rectangle
  return (ballY + ballSize >= paddleY &&       // Ball is at or below paddle's top edge
          ballY <= paddleY + PADDLE_HEIGHT && // Ball is above paddle's bottom edge
          ballX + ballSize >= paddleX &&      // Ball is at or beyond paddle's left edge
          ballX <= paddleX + PADDLE_WIDTH);   // Ball is at or before paddle's right edge
}
int is_ball_colliding_with_second_paddle(short ballX, short ballY, int ballSize) {
  // Check if the ball intersects the paddle's rectangle
  return (ballY + ballSize >= paddleY2 &&       // Ball is at or below paddle's top edge
          ballY <= paddleY2 + PADDLE_HEIGHT && // Ball is above paddle's bottom edge
          ballX + ballSize >= paddleX &&      // Ball is at or beyond paddle's left edge
          ballX <= paddleX + PADDLE_WIDTH);   // Ball is at or before paddle's right edge
}

void wdt_c_handler(){
  static int secCount = 0;

  secCount ++;
  if (secCount >= 18) {

    { /* Move first ball */
      short oldCol = controlPos[0];
      short newCol = oldCol + colVelocity;
      short oldRow = controlPos[1];
      short newRow = oldRow + rowVelocity;

      // Check collision between balls
      int ballDistance = ((newCol - control[0]) * (newCol - control[0])) +
                         ((newRow - control[1]) * (newRow - control[1]));
      int collisionThreshold = (sizeOfBall * 2) * (sizeOfBallSec * 2);

      if (ballDistance <= collisionThreshold) {
        // Swap velocities on collision
        colVelocity = -colVelocity;
        rowVelocity = -rowVelocity;
        colVeSecond = -colVeSecond;
        rowVeSecond = -rowVeSecond;
        if (colorFromWheel >= *(&colorWheel + 1) - colorWheel) colorFromWheel = 0;
        colorFromWheel++;
      }
      if (is_ball_colliding_with_paddle(newCol, newRow, sizeOfBall)) {

        rowVelocity = -rowVelocity;
      }
      if (is_ball_colliding_with_second_paddle(newCol, newRow, sizeOfBall)) {

        rowVelocity = -rowVelocity;
      }

      // Screen boundary checks
      if (newCol <= colLimits[0] || newCol >= colLimits[1])
        colVelocity = -colVelocity;
      else
        controlPos[0] = newCol;

      if (newRow <= rowLimits[0] || newRow >= rowLimits[1]){
        rowVelocity = -rowVelocity;
        lives--;
        if (lives == 48) lives = 51;
        drawPos = {1,10};
      }

      else
        controlPos[1] = newRow;
    }

    { /* Move second ball */
      short oldColon = control[0];
      short newColon = oldColon + colVeSecond;
      short oldRowSec = control[1];
      short newRowSec = oldRowSec + rowVeSecond;
      if (is_ball_colliding_with_paddle(newColon, newRowSec, sizeOfBallSec)) {
        rowVeSecond = -rowVeSecond;
      }
      if (is_ball_colliding_with_second_paddle(newColon, newRowSec, sizeOfBallSec)) {

        rowVeSecond = -rowVeSecond;
      }

      // Screen boundary checks
      if (newColon <= colera[0] || newColon >= colera[1])
        colVeSecond = -colVeSecond;
      else
        control[0] = newColon;

      if (newRowSec <= rowera[0] || newRowSec >= rowera[1])
        rowVeSecond = -rowVeSecond;
      else
        control[1] = newRowSec;
    }

    {
      if (step <= 30)
        step ++;
      else
        step = 0;
      secCount = 0;
    }
    if (switches & SW4) return;
    redrawScreen = 1;
  }
}

void update_shape();

void main()
{
  P1DIR |= LED;		/**< Green led on when CPU on */
  P1OUT |= LED;
  configureClocks();
  lcd_init();
  switch_init();

  enableWDTInterrupts();      /**< enable periodic interrupt */
  or_sr(0x8);	              /**< GIE (enable interrupts) */

  clearScreen(COLOR_BLUE);
  //SCREEN_HEIGHT
  //SCREEN_WIDTH
  while (1) {			/* forever */
    if (redrawScreen) {
      redrawScreen = 0;
      update_shape();
    }
    P1OUT &= ~LED;	/* led off */
    or_sr(0x10);	/**< CPU OFF */
    P1OUT |= LED;	/* led on */
  }
}

void update_shape() {
  drawChar5x7(SCREEN_WIDTH - 10, SCREEN_HEIGHT - 10, lives, COLOR_YELLOW, COLOR_BLUE);
  screen_update_ball();
  screen_update_second_ball();
  update_paddle_position();
}

void
__interrupt_vec(PORT2_VECTOR) Port_2(){
  if (P2IFG & SWITCHES) {	      /* did a button cause this interrupt? */
    P2IFG &= ~SWITCHES;		      /* clear pending sw interrupts */
    switch_interrupt_handler();	/* single handler for all switches */
  }
}
