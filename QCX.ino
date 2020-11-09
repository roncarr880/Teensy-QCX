/*
 *   QCX with Teensy 3.6, Audio shield, and ILI9341 touchscreen added on QCX Dev Board.
 * 

   Free pins when using audio board and display.
   Teensy 3.6.  Many more pins unused but not populated with headers.
   
  0,1    used for Serial1  Rx, Tx
  2
  3 
  4
  5
  8      used for touch CS
 16
 17

 *******************************************************************************/


#include "SPI.h"
#include "ILI9341_t3.h"
#include <XPT2046_Touchscreen.h>


         //  4bpp pallett  0-3, 4-7, 8-11, 12-15
const uint16_t EGA[] = { 
         ILI9341_BLACK,    ILI9341_NAVY,    ILI9341_DARKGREEN, ILI9341_DARKCYAN,  \
         ILI9341_MAROON,   ILI9341_PURPLE,  ILI9341_OLIVE,     ILI9341_LIGHTGREY, \
         ILI9341_DARKGREY, ILI9341_BLUE,    ILI9341_GREEN,     ILI9341_CYAN,      \
         ILI9341_RED,      ILI9341_MAGENTA, ILI9341_YELLOW,    ILI9341_WHITE      \
         }; 
uint16_t GRAY[16];
const uint16_t WF[] = {
         0b0, 0b11, 0b111, 0b1100, 0b10010, \
         0b11001, 0b111111, 0b111111100, 0b11101111111100, 0b111101111111000,  \
         0b1100011100010000,  0b1110011100000000, \
         0b1111101111100000, 0b1111101000001000, 0b1111110000110000, 0xffff  \
         };



// alternate display connections for use with audio board
#define CS_PIN  8       // touchscreen
#define TFT_DC      20
#define TFT_CS      21
#define TFT_RST    255  // 255 = unused, connect to 3.3V
#define TFT_MOSI     7
#define TFT_SCLK    14
#define TFT_MISO    12
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);
XPT2046_Touchscreen ts(CS_PIN);


#define cat Serial1

// radio defines for vfo_mode.
#define VFO_A  0
#define VFO_B  1
#define VFO_SPLIT 2
#define VFO_RIT   4
#define VFO_UNUSED 8

//  radio variables
int32_t  vfo_a, vfo_b, rit, stp = 100, rit_stp = 10;
uint8_t  vfo_mode = VFO_UNUSED;    // set to invalid value so prints status line on startup

// cat polling
#define QUFA 0b00000001
#define QUFB 0b00000010
#define QUIF 0b00101100
#define QUTB 0b01000000
uint32_t cmd_tm;           // command interval timer
uint8_t qu_flags;

// cw decode
#define DECODE_LINES 5
#define D_LEFT  8               // margins
#define D_BOTTOM 238            // 
#define D_SIZE   20             // line spacing
char dtext[DECODE_LINES][26];   // buffer
int dline = DECODE_LINES - 1;   // current working line
int dpos;                       // current cursor position

// screen owners
#define DECODE 0
#define KEYBOARD 1
uint8_t  screen_owner = DECODE;

char kb[5][10] = { \
   { '1','2','3','4','5','6','7','8','9','0' }, \
   { 'Q','W','E','R','T','Y','U','I','O','P' }, \
   { 'A','S','D','F','G','H','J','K','L', 8 }, \
   { 'Z','X','C','V','B','N','M',',','.','/' }, \
   { '*','*',' ',' ','?','=',' ',' ',' ',' ' }  \
};
/********************************************************************************/
void setup() {
int i,j,r,g,b;

  Serial.begin(38400);     // for debug or PC control
  cat.begin(38400);        // qcx CAT control

  // build the gray scale pallett
  for( i = 0; i < 16; ++i ){
    r = 2*i + 1;   g = 4*i + 3;  b = 2*i + 1;
    GRAY[i] = r << 11 | g << 5 | b;
  }

  // clear the decode text buffer with spaces
  for( i = 0; i < DECODE_LINES; ++i ){
     for( j = 0; j < 25; ++j ) dtext[i][j] = ' ';
     dtext[i][25] = 0;                               // terminate as a string
  }
  
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(1);

  ts.begin();
  ts.setRotation(1);
  
}


void loop() {
static int c_state;
int32_t t;

   // poll radio once a second, process qu_flags as a priority
   if( ( millis() - cmd_tm > 1000 ) ){
      if( qu_flags & QUIF ) cat.print("IF;");
      else if( qu_flags & QUFB ) cat.print("FB;");
      else if( qu_flags & QUFA ) cat.print("FA;");
      else if( qu_flags & QUTB && screen_owner == DECODE ) cat.print("TB;");
      else if( qu_flags & QUTB ) cat.print("TB0;");
      else{
          switch(c_state){                         // poll radio round robin for missed packet recovery
              case 0: case 2: case 4: case 6: case 8:
              case 10: case 12: case 14:  cat.print("IF;"); break;
              case 1: cat.print("FA;");   break;
              case 3: cat.print("QU1;");  break;   // enable flags
              case 5: cat.print("FB;");   break;
              case 7: if( screen_owner == DECODE ) cat.print("TB1;");
                      else cat.print("TB0;");  
                      break;   // enable/disable decode flag
              case 9: if( screen_owner == DECODE ) cat.print("TB;");
                      break;        
         }
         ++c_state;
         c_state &= 15;
      }
      cmd_tm = millis();
   }

   if( cat.available() ) radio_control();

   t = touch();
   if( t ){
      // dispatch to who owns the touch screen
      tft.setTextSize(2);  // !!! all debug code here
      tft.setTextColor(ILI9341_YELLOW,ILI9341_NAVY);
      tft.setCursor(0,45);
      tft.println("          ");
      tft.println("          ");
      tft.setCursor(0,45);
      tft.println(t >> 8);   // x
      tft.println(t & 0xff);  // y

      // test keyboard
      key_tx(t);
   }


}


void key_tx( int32_t t ){
int x,y;
char c;

   if( screen_owner != KEYBOARD ){     // need to display the keyboard
      screen_owner = KEYBOARD;
      // !!! use some upper bits(change type) in qu_flags to que a TB0;
      tft.fillRect(0, 80, 320, 160, EGA[0]);
      tft.setTextSize(2);
      tft.setTextColor( ILI9341_YELLOW, ILI9341_MAROON );
      for( y = 0;  y < 5; ++y ){
         for( x = 0; x < 10; ++x ){
             tft.fillRect(  32*x +3, 32*y +3 +80, 32-6, 32-6, ILI9341_MAROON );
             tft.setCursor( 32*x +6, 32*y +6 +80);
             if( kb[y][x] == 8 ) tft.print("BS");
             else tft.write(kb[y][x]);
         }
      }
      // fixups for rx and space
      tft.fillRect( 0, 80 + 3 + 4*32, 32+32 - 6, 32-6, ILI9341_MAROON );
      tft.setCursor( 6, 80 + 6 + 4*32 );
      tft.print("RX");
      tft.fillRect( 6*32+3, 80 + 4*32+3, 4*32 -6, 32-6 , ILI9341_MAROON);
      tft.setCursor( 6*32 + 6, 80 + 4*32 + 6 );
      tft.print("SPACE");
   }

   x = t >> 8;   y = t & 0xff;
   tft.setCursor(80-12,80-12);
   y -= 80;
   x /= 32;   y /= 32;

   c = kb[y][x];
   if( x >= 0 && x < 10 && y >= 0 && y < 5 )  tft.write( c );

   if( c == '*' ){
     tft.fillRect(0, 80, 320, 160, EGA[0]);
     screen_owner = DECODE;
     cat.print("TB1;");
   }
}

int32_t touch(){
static uint32_t  tm;
static int16_t  x,y;
static uint8_t   z;

   // control rate of updates,  library is at 3 ms.
   if( millis() - tm < 5 ) return 0;
   tm = millis();

   if( ts.touched() ){
     ++z;
     TS_Point p = ts.getPoint();
     //ts.readData(&x,&y,&z);
     x += map( p.x, 250, 3650, 0, 320 );
     y += map( p.y, 330, 3700, 0, 240 );
     //x = p.x;  y = p.y;
   }
   else z = 0;
   if( z == 0 ) x = y = 0;

   if( z == 4 ){
     x >>= 2;
     y >>= 2;
     return ( x << 8 ) | y;
   }
   return 0;
}

void vfo_mode_disp(){
int a,b,s,r;
int pos;

   a = b = s = r = 0;                       // set colors of the vfo mode
   if( vfo_mode & VFO_B ) b = 10;
   else a = 10;
   if( vfo_mode & VFO_SPLIT ) s = 10;
   if( vfo_mode & VFO_RIT ) r = 10;
   
   tft.setTextSize(1);
   tft.setTextColor(EGA[4+a],0);
   tft.setCursor(45,2);
   tft.print("VFO A");
   tft.setCursor(110,2);
   tft.setTextColor(EGA[4+s],0);
   tft.print("Split");
   tft.setCursor(170,2);
   tft.setTextColor(EGA[4+b],0);
   tft.print("VFO B");
   tft.setCursor(280,2);
   tft.setTextColor(EGA[4+r],0);
   tft.print("RIT"); 
   tft.drawLine(0,40,640,40,EGA[4]);
   tft.drawLine(0,41,640,41,EGA[4]);
   
       //  underline active tuning digit with best guess at step
   s = stp;    
   if( r ) pos = 278, s = rit_stp;    // rit active
   else if( b ) pos = 183;            // vfo b
   else pos = 53;                     // vfo a
   tft.drawLine(pos+s,40,pos+s+10,40,EGA[14]); 
   tft.drawLine(pos+s,41,pos+s+10,41,EGA[14]); 
}

void vfo_freq_disp(){
int val;

   tft.setTextSize(2);
   tft.setCursor(5,20);
   tft.setTextColor(EGA[10],0);
   if( vfo_a < 10000000 ) tft.write(' ');
   tft.print(vfo_a / 1000);
   tft.write('.');
   p_leading(vfo_a % 1000,3);
   
   tft.setCursor(135,20);
   if( vfo_b < 10000000 ) tft.write(' ');
   tft.print(vfo_b / 1000);
   tft.write('.');
   p_leading(vfo_b % 1000,3 );

   val = rit;
   if( val < 0 ){
      val = - val;
      tft.setTextColor(EGA[12],0);       // change color instead of displaying the minus sign
   }
   tft.setCursor(265,20);
   p_leading(val,4);

}


void p_leading( int val, int digits ){    // print a number with leading zeros without using sprintf
int test;

   test = pow(10,digits-1);
   while( val < test && test >= 10 ){
       tft.write('0');
       test /= 10;
   }
   tft.print(val);
  
}


#define CMDLEN 128
char response[CMDLEN];

void radio_control(){
static int len;
char c;

    c = 0;
    while( cat.available() ){
       c = cat.read();
       response[len] = c;
       if( c == ';' ) break;
       if(++len >= CMDLEN ) len = 0;      // somethings wrong, reset
    }
    
    if( c != ';' ) return;               // command not complete yet

    // debug   !!! comment out
    response[len+1] = 0;
    Serial.println(response);            // !!! debug

    response[len] = 0;                   // terminate string removing ; on the end

    if( response[0] == 'F' ){
      switch(response[1]){
        case 'A': cat_freq(&vfo_a);  qu_flags &= ~QUFA;  break;
        case 'B': cat_freq(&vfo_b);  qu_flags &= ~QUFB;  break;
        // case 'R': cat_mode(2);       break;   // using IF command to get this info now
        // case 'T': cat_mode(1);       break;
      }      
    }
    if( response[0] == 'I' && response[1] == 'F' ) cat_mode();
    if( response[0] == 'Q' && response[1] == 'U' ) cat_qu_flags();
    if( response[0] == 'T' && response[1] == 'B' ) cat_decode();

    len = 0;     // reset string to start for next command
}

void cat_decode(){
int i;
char c;

  qu_flags &= ~QUTB;
  if( strlen(response) < 6 ) return;                       // no data
  if( screen_owner != DECODE ) return;

  i = 5;
  tft.setTextColor(EGA[14],0);
  tft.setTextSize(2);
  while( (c = response[i++]) ){
     if( dpos >= 25 ) scroll_dtext();                      // scroll needed
     // save each char in buffer and print to the screen
     dtext[dline][dpos] = c;
     tft.setCursor( D_LEFT + 12*dpos, D_BOTTOM - D_SIZE );
     tft.write(c);
     ++dpos;
  }
}


void scroll_dtext(){
int i,j,k;

      dtext[dline][25] = 0;                                // terminate string redundantly
      dpos = 0;
      if( ++dline >= DECODE_LINES ) dline = 0;             // move index instead of data
      for( i = 0; i < 25; ++i ) dtext[dline][i] = ' ';     // clear line
      dtext[dline][25] = 0;

      // write all the buffer to the screen
      j = dline + 1;                                       // first line printed is one more than new dline
      if( j >= DECODE_LINES ) j = 0;
      k = D_BOTTOM - D_SIZE * DECODE_LINES;                // screen position of the 1st write
      tft.setTextColor(EGA[14],0);

      for( i = 0; i < DECODE_LINES; ++i ){                 // print to the screen
         tft.setCursor(D_LEFT,k);
         tft.print(&dtext[j][0]);
         k += D_SIZE;
         if( ++j >= DECODE_LINES ) j = 0;
      }
  
}

void cat_qu_flags(){                       // process the qu flags response

  qu_flags |= atoi(&response[2]);             // merge new flags
  qu_flags &= (QUFA + QUFB + QUIF + QUTB);    // clear unused flags
}

void cat_freq( int32_t *vfo ){             // update a vfo from cat response
int32_t temp, s;

    if( strlen(response) != 13 ) return;   // ??
    temp = atoi( &response[2] );
    s = *vfo - temp;
    if( s ){
       *vfo = temp;
       vfo_freq_disp();

       temp = stp;                       // get tuning rate from how much the vfo changed
       if( s < 0 ) s = -s;
       if( s >= 1000 ) temp = 0;         // temp is the pixel positions for the digit underline
       else if( s == 500 ) temp = 12;
       else if( s >= 100 ) temp = 24;
       else temp = 36;

       if( temp != stp ){
           stp = temp;
           vfo_mode_disp();
       }
    }
}


void cat_mode( ){     // from IF command
uint8_t temp;
int32_t temp2;

   if( strlen(response) < 35 ) return;
    // build new vfo_mode bits
   temp = 0;
   if( response[30] == '1' ) temp |= VFO_B;
   if( response[32] == '1' ) temp |= VFO_SPLIT;
   if( response[23] == '1' ) temp |= VFO_RIT;    
   if( temp != vfo_mode ){                    // compare to displayed value, avoid duplicate screen update
      vfo_mode = temp;
      vfo_mode_disp();
   }

   response[23] = 0;           // get the rit value
   temp2 = atoi( &response[18] );
   if( rit != temp2 ){
      rit = temp2;
      vfo_freq_disp();
   }

   if( vfo_mode & ( VFO_B + VFO_SPLIT ) ) qu_flags |= QUFB;   // que a read of vfo B
   else{                      // not sure which vfo is in the IF when in split or VFO B mode
      response[13] = 0;       // but should be vfo A in normal operation
      cat_freq( &vfo_a );     // get vfo A from IF response
      qu_flags &= ~QUFA;      // and clear any A flag that may be set
   }

   qu_flags &= ~QUIF;
}


/******************************************************************/

 //writeRect4BPP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pixels, const uint16_t * palette );

/*
uint8_t pix[20*20];    // write rec buffer example
int x,y;
static int loc;

 for(y = 0; y < 16; ++y ){        // write 16 20 by 20 size color samples using 4bpp colormap
   for(x = 0; x < 20*20; ++x ){   // buffer holds colors in linear char array, pointer to pallet
        pix[x] = y + 16*y;
   }
   tft.writeRect4BPP(loc, 100, 20, 20, pix, EGA );
   tft.writeRect4BPP(loc, 140, 20, 20, pix, GRAY );
   tft.writeRect4BPP(loc, 180, 20, 20, pix, WF );
   
   loc += 20;
   if( loc >= 320 ) loc = 0;
 }
 
 delay(20000);
 loc += 20;      // pan the colors each loop
 if( loc >= 320 ) loc = 0;
*/
//  tft.setTextColor(ILI9341_YELLOW);
//  tft.setTextSize(1);
//  tft.println("Waiting for Arduino Serial Monitor...");
//  tft.setTextSize(2);
//  tft.println("Not Really...");
//  tft.setTextSize(3);
//  tft.setTextColor(ILI9341_DARKGREEN);
//  tft.println(" 7122.030");
//  tft.setTextColor(ILI9341_GREEN);
//  tft.print(" 7038.600");
