/*
 *   QCX with Teensy 3.6, Audio shield, and ILI9341 touchscreen added on QCX Dev Board.
 *   Building upon ZL2CTM inspired code.
 * 
     
   Free pins when using audio board and display.
   Teensy 3.6.  Many more pins unused but not populated with headers.
   
  0,1    used for Serial1  Rx, Tx
  2
  3 
  4
  5
  8      used for touch CS
 16      A2
 17      A3


 Considering: re-sample qcx audio and lineout audio and using DAC0 to pass audio
 to the top of the volume control.  This will allow software selection of the
 output as well as access to audio data for decoders. The qcx audio can be provided
 with AGC and/or clipping/filter for hearing protection.

 *******************************************************************************/


#include <ILI9341_t3.h>
#include <XPT2046_Touchscreen.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>

#include "hilbert.h"


// GUItool: begin automatically generated code
// IIR bandwidth filter version.
// FIR constants are picked up in reverse order, so may have to switch H45plus and H45minus
// as they seem to have the same constants, just reverse order.
AudioInputI2S            LineIn;           //xy=55,254
AudioFilterFIR           H45minus;           //xy=168,338
AudioFilterFIR           H45plus;           //xy=169,175
//AudioAnalyzeFFT256       LSBscope;       //xy=349,445
AudioMixer4              LSBmixer;         //xy=352,328
AudioMixer4              USBmixer;         //xy=355,181
//AudioAnalyzeFFT256       USBscope;       //xy=356,63
AudioMixer4              SSBselect;         //xy=455,256
//AudioAnalyzePeak         peak1;          //xy=485.55556869506836,107.77777481079102
//AudioAnalyzeRMS          rms1;           //xy=615.5555839538574,139.99999809265137
AudioOutputI2S           LineOut;           //xy=619.4443702697754,391.0000190734863
AudioFilterBiquad        BandWidth;        //xy=645,255
AudioConnection          patchCord1(LineIn, 0, H45plus, 0);
AudioConnection          patchCord2(LineIn, 1, H45minus, 0);
AudioConnection          patchCord3(H45minus, 0, USBmixer, 2);
AudioConnection          patchCord4(H45minus, 0, LSBmixer, 2);
AudioConnection          patchCord5(H45plus, 0, USBmixer, 1);
AudioConnection          patchCord6(H45plus, 0, LSBmixer, 1);
//AudioConnection          patchCord7(LSBmixer, LSBscope);
AudioConnection          patchCord8(LSBmixer, 0, SSBselect, 2);
//AudioConnection          patchCord9(USBmixer, USBscope);
AudioConnection          patchCord10(USBmixer, 0, SSBselect, 1);
//AudioConnection          patchCord11(USBmixer, peak1);
AudioConnection          patchCord12(SSBselect, BandWidth);
//AudioConnection          patchCord13(BandWidth, rms1);
AudioConnection          patchCord14(BandWidth, 0, LineOut, 0);
AudioConnection          patchCord15(BandWidth, 0, LineOut, 1);
AudioControlSGTL5000     codec;     //xy=86,63
// GUItool: end automatically generated code



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
#define MENUS    2
uint8_t  screen_owner = DECODE;

char kb[5][10] = { \
   { '1','2','3','4','5','6','7','8','9','0' }, \
   { 'Q','W','E','R','T','Y','U','I','O','P' }, \
   { 'A','S','D','F','G','H','J','K','L', 8 }, \
   { 'Z','X','C','V','B','N','M',',','.','/' }, \
   { '*','*',' ',' ','?','=',' ',' ',' ',' ' }  \
};

// Menu's

void (* menu_dispatch )( int32_t );    // pointer to function that is processing screen touches


struct menu {
   char title[16];
   char *menu_item[8];
   int param[8];
   int y_size;                  // x size will be half the screen, two items on a line for now
   int color;
   int current;
};

// mode menu items
char m_qcx[] = " OFF";    // native qcx mode using hardware decode, built in filter, no agc
char m_cw[]  = " CW 400";
char m_lsb[] = " LSB";
char m_usb[] = " USB";
char m_am[]  = " AM";
char m_sam[] = " SAM";
char m_data[]= " DATA";

struct menu mode_menu_data = {
   { "SDR Mode" },
   { m_qcx,m_cw,m_lsb,m_usb,m_am,m_sam,m_data },
   {0,2,1,2,-1,-1,-1,-1},
   48,
   ILI9341_PURPLE,
   2
};

char w_am[] =     " AM 3900";
char w_bypass[] = " AM Wide";
char w_2500[] =   " 2500";
char w_2700[] =   " 2700";
char w_2900[] =   " 2900";
char w_3100[] =   " 3100";
char w_3300[] =   " 3300";
char w_data[] =   " DATA";
struct menu band_width_menu_data = {
   { "Band Width" },
   { w_am, w_bypass, w_2500, w_2700, w_2900, w_3100, w_3300, w_data },
   { 5000, 10000, 3200, 3400, 3600, 3800, 4000, 2100 },      // set wider, 12 db down point?
   48,
   ILI9341_MAROON,
   6
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
  
  ts.begin();                        // touchscreen
  ts.setRotation(1);
 
  menu_dispatch = &hidden_menu;     // function pointer for screen touch

   // Setup the audio shield
  AudioNoInterrupts();
  AudioMemory(20);
  codec.enable();
  codec.volume(0.6);                        // headphones, not used eventually
  codec.unmuteHeadphone();
  codec.inputSelect( AUDIO_INPUT_LINEIN );  
  codec.lineInLevel(12);                    // 0 to 15  !!! use as attenuator, reduce gain on loud signals
  codec.lineOutLevel(20);                   // 13 to 31 with 13 the loudest.  Use to balance gain with
                                            // the QCX signal level.  One time adjustment.
  // dacVolume  ? does it effect lineout or just headphones
  // autoVolume ? Built in AGC.  Is it input or output feature?
 // H45plus.begin(h45p,HILBERT_SIZE);
 // H45minus.begin(h45m,HILBERT_SIZE);
  H45plus.begin(h90p,HILBERT_SIZE);        // try the 90deg and 0deg phase shift filters
  H45minus.begin(h00m,HILBERT_SIZE);
  USBmixer.gain(1,1.0);                    // add signals get USB.   Lower gain if addition causes clipping.
  USBmixer.gain(2,1.0);
  LSBmixer.gain(1,1.0);                    // sub signals get LSB
  LSBmixer.gain(2,-1.0);

  SSBselect.gain(1,0.0);                   // turn off USB
  SSBselect.gain(2,1.0);                   // LSB is default on startup

  BandWidth.setLowpass(0,4000,0.7);        // do all of these need to be configured?
  BandWidth.setLowpass(1,4000,0.7);
  BandWidth.setLowpass(2,4000,0.7);
  BandWidth.setLowpass(3,4000,0.7);        // 12 db down at 4000 now?
    
  AudioInterrupts();
  
}



// touch the screen top,middle,bottom to bring up different menus.  Assign menu_dispatch.
// This is default touch processing.   No menu on the screen.
void hidden_menu( int32_t t ){

   screen_owner = MENUS;
   
   // just check the y value of touch to see what menu 
   t = t & 0xff;
   if( t < 60 ){
      menu_display( &mode_menu_data );
      menu_dispatch = &mode_menu;        // screen touch goes to mode_menu() now
   }
   else if ( t < 140 ){ 
      menu_display( &band_width_menu_data );
      menu_dispatch = &band_width_menu;
   }
   
   else screen_owner = DECODE;           // screen area not defined 
}

void mode_menu( int32_t t ){
int selection;

   selection = touch_decode( t, mode_menu_data.y_size );
   if( mode_menu_data.param[selection] != -1 ){
      mode_menu_data.current = selection;
      selection = mode_menu_data.param[selection];
      // selection 0 is a special case.  Disable SDR and enable QCX.
      if( selection == 0 ){
         // !!! enable the QCX audio
      }
      else{
        if( selection == 1 ){
            SSBselect.gain(1,0.0);                   // turn on LSB
            SSBselect.gain(2,1.0);
        }
        if( selection == 2 ){
            SSBselect.gain(1,1.0);                   // turn on USB
            SSBselect.gain(2,0.0);  
        }
      }
   }

   menu_cleanup(); 
}

void menu_cleanup(){

   // exit touch menu and back to normal screen
   menu_dispatch = &hidden_menu;
   tft.fillScreen(ILI9341_BLACK);
   screen_owner = DECODE;
   vfo_mode_disp();
   vfo_freq_disp();
}

void band_width_menu( int32_t t ){
int sel;

   sel = touch_decode( t, band_width_menu_data.y_size );
   if( band_width_menu_data.param[sel] != -1 ){
      band_width_menu_data.current = sel;
      sel = band_width_menu_data.param[sel];
     //Serial.println(sel);
      BandWidth.setLowpass(0,sel,0.7);        // do all of these need to be configured?
      BandWidth.setLowpass(1,sel,0.7);
      BandWidth.setLowpass(2,sel,0.7);
      BandWidth.setLowpass(3,sel,0.7);        // 12 db down at 4000 now?
   }
   
   menu_cleanup();
}

void decode_menu( int32_t t ){
  
}

void menu_display( struct menu *m ){    // display any of the menus on the screen
int i,x,y;                              // other functions handle selections

   tft.setTextColor( ILI9341_WHITE, m->color );  // text is white on background color 
   tft.fillScreen(ILI9341_BLACK);                // border of menu items is black

   // title box
   tft.fillRect(5,5,320-10,m->y_size-10,m->color);
   tft.setCursor( 10,10 );
   if( m->y_size > 45 ) tft.setTextSize(3);
   else tft.setTextSize(2);
   tft.print(m->title);
   
   // draw some menu boxes, two per row for now
   y = m->y_size; x = 0;
   for( i = 0; i < 8; ++i ){
      if( m->menu_item[i][0] == 0 ) break;       // null option
      if( y + m->y_size-10  > 239 ) break;       // screen is full
      tft.fillRect(x+5,y+5,160-10,m->y_size-10,m->color);
      tft.setCursor( x+10,y+10 );
      if( i == m->current ) tft.setTextColor( ILI9341_YELLOW, m->color );
      else tft.setTextColor( ILI9341_WHITE, m->color );
      tft.print(m->menu_item[i]);
      x += 160;
      if( x >= 320 ) x = 0, y += m->y_size;
   }
}

int touch_decode( int32_t t, int y_size ){    // selection for 2 items wide touch menu
int32_t x,y;

   y = t & 0xff;
   x = t >> 8;
   y -= y_size;    // top row is the title
   y /= y_size;    // row of the touch
   x /= 160;       // column 
   return 2*y + x; // two menu items per row
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
      // goto where menu_dispatch() points
      (*menu_dispatch)(t);    // off to whoever owns the touchscreen
   }


}


void key_tx( int32_t t ){
int x,y;
char c;

   if( screen_owner != KEYBOARD ){     // need to display the keyboard
      screen_owner = KEYBOARD;
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
// int pos;

   if( screen_owner != DECODE ) return;
   a = b = s = r = 0;                       // set colors of the vfo mode
   if( vfo_mode & VFO_B ) b = 10;
   else a = 10;
   if( vfo_mode & VFO_SPLIT ) s = 10;
   if( vfo_mode & VFO_RIT ) r = 10;
   
   tft.setTextSize(1);
   tft.setTextColor(EGA[4+a],0);
   tft.setCursor(15,2);
   tft.print("VFO A");
   
   tft.setCursor(60,2);
   tft.setTextColor(EGA[4+b],0);
   tft.print("VFO B");
   
   tft.setCursor(105,2);
   tft.setTextColor(EGA[4+s],0);
   tft.print("Split");

   tft.setCursor(150,2);
   tft.print("CW");

   tft.setCursor(175,2);
   tft.print("USB");

   tft.setCursor(205,2);
   tft.print("LSB");

   tft.setCursor(235,2);
   tft.print("AM");
   
   tft.setCursor(280,2);
   tft.setTextColor(EGA[4+r],0);
   tft.print("RIT");
    
   tft.drawLine(0,60,640,60,EGA[4]);
   tft.drawLine(0,61,640,61,EGA[4]);
   
       //  underline active tuning digit with best guess at step
    /*   
   s = stp;    
   if( r ) pos = 278, s = rit_stp;    // rit active
   else if( b ) pos = 183;            // vfo b
   else pos = 53;                     // vfo a
   tft.drawLine(pos+s,40,pos+s+10,40,EGA[14]); 
   tft.drawLine(pos+s,41,pos+s+10,41,EGA[14]);
   */ 
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


//  alternate freq display.  Simulate something that looks like the Argonaut V
/* segments */
#define A_  1
#define B_  2
#define C_  4
#define D_  8
#define E_  16
#define F_  32
#define G_  64
#define DP_ 128
uint8_t segment[16] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x79,0x71};

void vfo_freq_disp(){
int32_t vfo;
int32_t digit;
int i,mult;

   if( screen_owner != DECODE ) return;
   if( vfo_mode & VFO_B ) vfo = vfo_b;
   else vfo = vfo_a;

   if( 1 ) vfo -= 700;   // !!! cw mode flag needed
   
   // 40 meter radio, ignore 10 meg digit for now
   mult = 1000000;
   for( i = 0; i < 7; ++i ){
       digit = vfo / mult;
       disp_segments(i,digit);
       vfo -= digit*mult;
       mult /= 10;
   }  

   tft.setTextSize(2);
   tft.setTextColor(EGA[10],0);
   int val = rit;
   if( val < 0 ){
      val = - val;
      tft.setTextColor(EGA[12],0);       // change color instead of displaying the minus sign
   }
   tft.setCursor(265,20);
   p_leading(val,4);
}

void disp_segments( int pos, int32_t digit ){    // ?? maybe a font table would be better
uint16_t color;

   digit = segment[digit];
   color = ( digit & A_ ) ? EGA[10] : GRAY[1] ;
   draw_A( pos,color );
   color = ( digit & B_ ) ? EGA[10] : GRAY[1] ;
   draw_B( pos,color );
   color = ( digit & C_ ) ? EGA[10] : GRAY[1] ;
   draw_C( pos,color );
   color = ( digit & D_ ) ? EGA[10] : GRAY[1] ;
   draw_D( pos,color );
   color = ( digit & E_ ) ? EGA[10] : GRAY[1] ;
   draw_E( pos,color );
   color = ( digit & F_ ) ? EGA[10] : GRAY[1] ;
   draw_F( pos,color );
   color = ( digit & G_ ) ? EGA[10] : GRAY[1] ;
   draw_G( pos,color );
   color = ( pos == 0 || pos == 3 ) ? EGA[10] : GRAY[1] ;
   draw_DP( pos, color);
  
}

#define VB 20     //20
#define VS 35     //40
#define HB 20     //20
#define HS 32     //30

 // slanted verticals have strange looking jaggies, try all horizontal lines
void draw_A( int pos, uint16_t color ){
int zz;
int i;

   zz = HB + HS * pos;
   for( i = 0; i < 4; ++i ) tft.drawFastHLine( zz+12,VB+i,13,color);
}

void draw_G( int pos, uint16_t color ){
int zz;
int i,k;
  
   zz = HB + HS * pos;
   for( i = 0; i < 4; ++i ){
      k = ( i == 1 || i == 2 ) ? 2 : 0;
      tft.drawFastHLine( zz+9-k,VB+VS/2+i-2,13 + 2*k,color);
   }
}

void draw_D( int pos, uint16_t color ){
int zz;
int i;
  
   zz = HB + HS * pos;
   for( i = 0; i < 4; ++i ) tft.drawFastHLine( zz+6,VB+VS-i,13,color);
}

void draw_B( int pos, uint16_t color ){
int zz;
int i,len;

   zz = HB + HS * pos +27;
   len = 1;
   for( i = 0; i < VS/2; ++i ){
      tft.drawFastHLine( zz,VB+i,len,color);
      if( i < 4 && len < 4 ) ++len;
      if( i > VS/2 - 4 ) ++zz, --len;
      if( i == VS/8 ) --zz;
      if( i == VS/4 ) --zz;
      if( i == 3*VS/8 ) --zz;
   }
}

void draw_C( int pos, uint16_t color ){
int zz;
int i,len;
  
   zz = HB + HS * pos + 27;
   len = 1;
   for( i = 0; i < VS/2; ++i ){
      tft.drawFastHLine( zz,VB+VS/2+i,len,color);
      if( i < 4 && len < 4 ) ++len, --zz;
      if( i > VS/2 - 4 ) --len;
      if( i == VS/8 ) --zz;
      if( i == VS/4 ) --zz;
      if( i == 3*VS/8 ) --zz;      
   } 
}

void draw_E( int pos, uint16_t color ){
int zz;
int i,len;

   zz = HB + HS * pos + 4;

   len = 1;
   for( i = 0; i < VS/2; ++i ){
      tft.drawFastHLine( zz,VB+VS/2+i,len,color);
      if( i < 4 && len < 4 ) ++len;
      if( i > VS/2 - 4 ) --len, ++zz;
      if( i == VS/8 ) --zz;
      if( i == VS/4 ) --zz;
      if( i == 3*VS/8 ) --zz;      
   } 
}

void draw_F( int pos, uint16_t color ){
int zz;
int i,len;

   zz = HB + HS * pos + 10;

   len = 1;
   for( i = 0; i < VS/2; ++i ){
      tft.drawFastHLine( zz,VB+i,len,color);
      if( i < 4 && len < 4 ) ++len, --zz;
      if( i > VS/2 - 4 ) --len, ++zz;
      if( i == VS/8 ) --zz;
      if( i == VS/4 ) --zz;
      if( i == 3*VS/8 ) --zz;      
   } 
}


void draw_DP( int pos, uint16_t color ){
int zz;

   zz = HB + HS * pos;
   tft.fillRect(zz+HS-5, VB+VS-1, 3, 3, color);
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
    //response[len+1] = 0;
    //Serial.println(response);            // !!! debug

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
      vfo_freq_disp();
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


/***************************** old code
  void vfo_freq_disp(){
int val;
int32_t VFO;

   VFO = vfo_a + tuning_offset - 700;

   vfo_freq_disp_alt(); // !!! testing
   return;
   
   tft.setTextSize(2);
   tft.setCursor(5,20);
   tft.setTextColor(EGA[10],0);
   if( VFO < 10000000 ) tft.write(' ');
   tft.print(VFO / 1000);
   tft.write('.');
   p_leading(VFO % 1000,3);
   
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

70

  (short)(32768 * -0.000287988910943357),
  (short)(32768 * -0.000383511439791303),
  (short)(32768 * -0.000468041804899774),
  (short)(32768 * -0.000529324432676899),
  (short)(32768 * -0.000569479602046985),
  (short)(32768 * -0.000616670267768531),
  (short)(32768 * -0.000731530748681977),
  (short)(32768 * -0.001002372095321225),
  (short)(32768 * -0.001525299390682192),
  (short)(32768 * -0.002370114347025230),
  (short)(32768 * -0.003539247773172147),
  (short)(32768 * -0.004932965382552984),
  (short)(32768 * -0.006337182914262393),
  (short)(32768 * -0.007448193692118567),
  (short)(32768 * -0.007940501940620482),
  (short)(32768 * -0.007570802072162988),
  (short)(32768 * -0.006296120449841751),
  (short)(32768 * -0.004371955618154949),
  (short)(32768 * -0.002391875073164555),
  (short)(32768 * -0.001236984700413469),
  (short)(32768 * -0.001922560128827416),
  (short)(32768 * -0.005356720327533458),
  (short)(32768 * -0.012055656297010635),
  (short)(32768 * -0.021882952959947619),
  (short)(32768 * -0.033888748300090733),
  (short)(32768 * -0.046312736456333638),
  (short)(32768 * -0.056783367797647665),
  (short)(32768 * -0.062699937453677912),
  (short)(32768 * -0.061735375084135742),
  (short)(32768 * -0.052358513976237808),
  (short)(32768 * -0.034257179158167443),
  (short)(32768 * -0.008554500746482946),
  (short)(32768 * 0.022249911747384360),
  (short)(32768 * 0.054622962942346594),
  (short)(32768 * 0.084568844473140448),
  (short)(32768 * 0.108316122839950818),
  (short)(32768 * 0.122979341462627859),
  (short)(32768 * 0.127056096658453188),
  (short)(32768 * 0.120656295327679283),
  (short)(32768 * 0.105420364259485699),
  (short)(32768 * 0.084152608145489444),
  (short)(32768 * 0.060257510644444748),
  (short)(32768 * 0.037105711921879434),
  (short)(32768 * 0.017464092086704748),
  (short)(32768 * 0.003100559033325746),
  (short)(32768 * -0.005373489802481697),
  (short)(32768 * -0.008418211280310166),
  (short)(32768 * -0.007286730644726664),
  (short)(32768 * -0.003638388931163832),
  (short)(32768 * 0.000858330713630433),
  (short)(32768 * 0.004847436504682235),
  (short)(32768 * 0.007476399317750315),
  (short)(32768 * 0.008440227567663121),
  (short)(32768 * 0.007898970420636600),
  (short)(32768 * 0.006314366257036837),
  (short)(32768 * 0.004261033495040515),
  (short)(32768 * 0.002261843500794377),
  (short)(32768 * 0.000680212977485724),
  (short)(32768 * -0.000319493110301691),
  (short)(32768 * -0.000751893569425181),
  (short)(32768 * -0.000752248417868501),
  (short)(32768 * -0.000505487955986662),
  (short)(32768 * -0.000184645628631330),
  (short)(32768 * 0.000087913008490067),
  (short)(32768 * 0.000253106348867209),
  (short)(32768 * 0.000306473486382603),
  (short)(32768 * 0.000277637042003169),
  (short)(32768 * 0.000207782317481292),
  (short)(32768 * 0.000132446796990356),
  (short)(32768 * 0.000072894261560354)


  (short)(32768 * -0.000072894261560345),
  (short)(32768 * -0.000132446796990344),
  (short)(32768 * -0.000207782317481281),
  (short)(32768 * -0.000277637042003168),
  (short)(32768 * -0.000306473486382623),
  (short)(32768 * -0.000253106348867259),
  (short)(32768 * -0.000087913008490148),
  (short)(32768 * 0.000184645628631233),
  (short)(32768 * 0.000505487955986583),
  (short)(32768 * 0.000752248417868491),
  (short)(32768 * 0.000751893569425298),
  (short)(32768 * 0.000319493110301983),
  (short)(32768 * -0.000680212977485245),
  (short)(32768 * -0.002261843500793748),
  (short)(32768 * -0.004261033495039842),
  (short)(32768 * -0.006314366257036280),
  (short)(32768 * -0.007898970420636345),
  (short)(32768 * -0.008440227567663343),
  (short)(32768 * -0.007476399317751102),
  (short)(32768 * -0.004847436504683540),
  (short)(32768 * -0.000858330713632029),
  (short)(32768 * 0.003638388931162351),
  (short)(32768 * 0.007286730644725833),
  (short)(32768 * 0.008418211280310565),
  (short)(32768 * 0.005373489802483816),
  (short)(32768 * -0.003100559033321630),
  (short)(32768 * -0.017464092086698697),
  (short)(32768 * -0.037105711921871905),
  (short)(32768 * -0.060257510644436532),
  (short)(32768 * -0.084152608145481672),
  (short)(32768 * -0.105420364259479538),
  (short)(32768 * -0.120656295327675800),
  (short)(32768 * -0.127056096658453216),
  (short)(32768 * -0.122979341462631633),
  (short)(32768 * -0.108316122839958146),
  (short)(32768 * -0.084568844473150454),
  (short)(32768 * -0.054622962942358168),
  (short)(32768 * -0.022249911747396132),
  (short)(32768 * 0.008554500746472333),
  (short)(32768 * 0.034257179158159054),
  (short)(32768 * 0.052358513976232306),
  (short)(32768 * 0.061735375084133286),
  (short)(32768 * 0.062699937453678217),
  (short)(32768 * 0.056783367797650072),
  (short)(32768 * 0.046312736456337288),
  (short)(32768 * 0.033888748300094730),
  (short)(32768 * 0.021882952959951244),
  (short)(32768 * 0.012055656297013388),
  (short)(32768 * 0.005356720327535105),
  (short)(32768 * 0.001922560128828006),
  (short)(32768 * 0.001236984700413229),
  (short)(32768 * 0.002391875073163812),
  (short)(32768 * 0.004371955618154038),
  (short)(32768 * 0.006296120449840938),
  (short)(32768 * 0.007570802072162439),
  (short)(32768 * 0.007940501940620253),
  (short)(32768 * 0.007448193692118624),
  (short)(32768 * 0.006337182914262643),
  (short)(32768 * 0.004932965382553323),
  (short)(32768 * 0.003539247773172483),
  (short)(32768 * 0.002370114347025498),
  (short)(32768 * 0.001525299390682370),
  (short)(32768 * 0.001002372095321316),
  (short)(32768 * 0.000731530748682004),
  (short)(32768 * 0.000616670267768521),
  (short)(32768 * 0.000569479602046963),
  (short)(32768 * 0.000529324432676881),
  (short)(32768 * 0.000468041804899765),
  (short)(32768 * 0.000383511439791304),
  (short)(32768 * 0.000287988910943362)


85
(short)( 32768 * 1.936360238116910E-6  ),
(short)( 32768 * 8.596907941616230E-6  ),
(short)( 32768 * 0.000021313539094513  ),
(short)( 32768 * 0.000039922581007478  ),
(short)( 32768 * 0.000063665037839926  ),
(short)( 32768 * 0.000096588018442902  ),
(short)( 32768 * 0.000151185079450219  ),
(short)( 32768 * 0.000240048671089178  ),
(short)( 32768 * 0.000355313244093721  ),
(short)( 32768 * 0.000455480575306277  ),
(short)( 32768 * 0.000487525618897985  ),
(short)( 32768 * 0.000448429344559743  ),
(short)( 32768 * 0.000441905673857701  ),
(short)( 32768 * 0.000656436179838934  ),
(short)( 32768 * 0.001228260442994080  ),
(short)( 32768 * 0.002057747170786644  ),
(short)( 32768 * 0.002744169021256951  ),
(short)( 32768 * 0.002783365328971788  ),
(short)( 32768 * 0.001991048270971564  ),
(short)( 32768 * 0.000870152935399601  ),
(short)( 32768 * 0.000543348954385515  ),
(short)( 32768 * 0.002086895284405703  ),
(short)( 32768 * 0.005571916961201433  ),
(short)( 32768 * 0.009524166504366735  ),
(short)( 32768 * 0.011460470244062732  ),
(short)( 32768 * 0.009509885883912047  ),
(short)( 32768 * 0.004211375504597140  ),
(short)( 32768 *-0.000921851712276400  ),
(short)( 32768 *-0.001000876711154889  ),
(short)( 32768 * 0.006952244638533937  ),
(short)( 32768 * 0.021004287244179118  ),
(short)( 32768 * 0.034012187697005251  ),
(short)( 32768 * 0.037165922099714394  ),
(short)( 32768 * 0.026008149014849609  ),
(short)( 32768 * 0.005368479982471630  ),
(short)( 32768 *-0.010563366460562079  ),
(short)( 32768 *-0.004422967169111801  ),
(short)( 32768 * 0.034346004889288143  ),
(short)( 32768 * 0.101161721542181893  ),
(short)( 32768 * 0.175050839863564234  ),
(short)( 32768 * 0.226418705484803112  ),
(short)( 32768 * 0.230647590757408094  ),
(short)( 32768 * 0.180578149437779611  ),
(short)( 32768 * 0.091011367822764075  ),
(short)( 32768 *-0.007523287528724512  ),
(short)( 32768 *-0.082646532411566984  ),
(short)( 32768 *-0.114415065769738644  ),
(short)( 32768 *-0.102741450977249640  ),
(short)( 32768 *-0.064695558637794720  ),
(short)( 32768 *-0.024031568766843151  ),
(short)( 32768 * 0.000838971257744977  ),
(short)( 32768 * 0.004123979400998993  ),
(short)( 32768 *-0.007737243216200090  ),
(short)( 32768 *-0.022502153655438228  ),
(short)( 32768 *-0.030011089016335601  ),
(short)( 32768 *-0.027030298718197423  ),
(short)( 32768 *-0.017135589787907100  ),
(short)( 32768 *-0.006884134779572964  ),
(short)( 32768 *-0.001323305572550949  ),
(short)( 32768 *-0.001543570640730744  ),
(short)( 32768 *-0.005147424323709140  ),
(short)( 32768 *-0.008593379072179084  ),
(short)( 32768 *-0.009543776366052220  ),
(short)( 32768 *-0.007839810879186379  ),
(short)( 32768 *-0.004919969493187661  ),
(short)( 32768 *-0.002482037048876100  ),
(short)( 32768 *-0.001413220860136188  ),
(short)( 32768 *-0.001545100215161085  ),
(short)( 32768 *-0.002116104245595165  ),
(short)( 32768 *-0.002427342712861818  ),
(short)( 32768 *-0.002226516476618989  ),
(short)( 32768 *-0.001681043647118040  ),
(short)( 32768 *-0.001107511122709350  ),
(short)( 32768 *-0.000718246000668050  ),
(short)( 32768 *-0.000537418783099531  ),
(short)( 32768 *-0.000470701289670866  ),
(short)( 32768 *-0.000418493599034147  ),
(short)( 32768 *-0.000339467237498280  ),
(short)( 32768 *-0.000244872331566725  ),
(short)( 32768 *-0.000160562163030922  ),
(short)( 32768 *-0.000099429006230605  ),
(short)( 32768 *-0.000058841179210850  ),
(short)( 32768 *-0.000031458009497279  ),
(short)( 32768 *-0.000013170642801227  ),
(short)( 32768 *-3.004924640076950E-6  )  



 (short)( 32768 * -3.004924640077050E-6  ),
(short)( 32768 *-0.000013170642801227  ),
(short)( 32768 *-0.000031458009497278  ),
(short)( 32768 *-0.000058841179210850  ),
(short)( 32768 *-0.000099429006230612  ),
(short)( 32768 *-0.000160562163030940  ),
(short)( 32768 *-0.000244872331566747  ),
(short)( 32768 *-0.000339467237498281  ),
(short)( 32768 *-0.000418493599034105  ),
(short)( 32768 *-0.000470701289670803  ),
(short)( 32768 *-0.000537418783099538  ),
(short)( 32768 *-0.000718246000668246  ),
(short)( 32768 *-0.001107511122709762  ),
(short)( 32768 *-0.001681043647118495  ),
(short)( 32768 *-0.002226516476619129  ),
(short)( 32768 *-0.002427342712861341  ),
(short)( 32768 *-0.002116104245594171  ),
(short)( 32768 *-0.001545100215160246  ),
(short)( 32768 *-0.001413220860136486  ),
(short)( 32768 *-0.002482037048878108  ),
(short)( 32768 *-0.004919969493190803  ),
(short)( 32768 *-0.007839810879188832  ),
(short)( 32768 *-0.009543776366051852  ),
(short)( 32768 *-0.008593379072175104  ),
(short)( 32768 *-0.005147424323703379  ),
(short)( 32768 *-0.001543570640727314  ),
(short)( 32768 *-0.001323305572553909  ),
(short)( 32768 *-0.006884134779583109  ),
(short)( 32768 *-0.017135589787920066  ),
(short)( 32768 *-0.027030298718205129  ),
(short)( 32768 *-0.030011089016331063  ),
(short)( 32768 *-0.022502153655421273  ),
(short)( 32768 *-0.007737243216180045  ),
(short)( 32768 * 0.004123979401006588  ),
(short)( 32768 * 0.000838971257727133  ),
(short)( 32768 *-0.024031568766887061  ),
(short)( 32768 *-0.064695558637848288  ),
(short)( 32768 *-0.102741450977284071  ),
(short)( 32768 *-0.114415065769726015  ),
(short)( 32768 *-0.082646532411495471  ),
(short)( 32768 *-0.007523287528607515  ),
(short)( 32768 * 0.091011367822891140  ),
(short)( 32768 * 0.180578149437874147  ),
(short)( 32768 * 0.230647590757439319  ),
(short)( 32768 * 0.226418705484765947  ),
(short)( 32768 * 0.175050839863480301  ),
(short)( 32768 * 0.101161721542088010  ),
(short)( 32768 * 0.034346004889218650  ),
(short)( 32768 *-0.004422967169139688  ),
(short)( 32768 *-0.010563366460552682  ),
(short)( 32768 * 0.005368479982499039  ),
(short)( 32768 * 0.026008149014873309  ),
(short)( 32768 * 0.037165922099721326  ),
(short)( 32768 * 0.034012187696995183  ),
(short)( 32768 * 0.021004287244161254  ),
(short)( 32768 * 0.006952244638519390  ),
(short)( 32768 *-0.001000876711159727  ),
(short)( 32768 *-0.000921851712272130  ),
(short)( 32768 * 0.004211375504605129  ),
(short)( 32768 * 0.009509885883917896  ),
(short)( 32768 * 0.011460470244063530  ),
(short)( 32768 * 0.009524166504363295  ),
(short)( 32768 * 0.005571916961196730  ),
(short)( 32768 * 0.002086895284402516  ),
(short)( 32768 * 0.000543348954384910  ),
(short)( 32768 * 0.000870152935400873  ),
(short)( 32768 * 0.001991048270973229  ),
(short)( 32768 * 0.002783365328972698  ),
(short)( 32768 * 0.002744169021256828  ),
(short)( 32768 * 0.002057747170785915  ),
(short)( 32768 * 0.001228260442993352  ),
(short)( 32768 * 0.000656436179838569  ),
(short)( 32768 * 0.000441905673857703  ),
(short)( 32768 * 0.000448429344559912  ),
(short)( 32768 * 0.000487525618898127  ),
(short)( 32768 * 0.000455480575306319  ),
(short)( 32768 * 0.000355313244093691  ),
(short)( 32768 * 0.000240048671089135  ),
(short)( 32768 * 0.000151185079450196  ),
(short)( 32768 * 0.000096588018442900  ),
(short)( 32768 * 0.000063665037839931  ),
(short)( 32768 * 0.000039922581007480  ),
(short)( 32768 * 0.000021313539094513  ),
(short)( 32768 * 8.596907941615600E-6  ),
(short)( 32768 * 1.936360238116770E-6  )

 */
