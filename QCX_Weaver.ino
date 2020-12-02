/*
 *   QCX with Teensy 3.6, Audio shield, and ILI9341 touchscreen added on QCX Dev Board.
 *     Weaver version
         Runs the baseband version for the band scope and AM detector
         and runs a weaver SSB demodulator for audio.

 *   A note:  putting capacitors in C4 C7 caused the front end to pick up screen draw noise.  Line In is
 *   connected to those nodes for connection of I and Q.  The difference with and without is substantial.
 * 
 * to work on  
 *   Fix my antenna cable or purchase a new one.
 *   Wire a fet to switch dit or dah to ground for hellschreiber
 *   See if TX works and power level ok.
 *   make a hole in the top case
 *   Add a USB external wire for usb audio
 *   CW, RTTY, PSK31 decoders - software only
 *   Hellschrieber RX and TX
 *      
 *   
     
   Free pins when using audio board and display.
   Teensy 3.6.  Many more pins unused but not populated with headers.
   
  0,1    used for Serial1  Rx, Tx
  2      mute switch 
  3 
  4
  5
  8      used for touch CS
 16      A2  
 17      A3  


 
 *******************************************************************************/


#include <ILI9341_t3.h>
#include <XPT2046_Touchscreen.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>

#include "hilbert_W.h"

#define QCX_MUTE 2              // pin 2 high mutes qcx audio

// #define USE_USB_AUDIO        // not needed for self contained radio but works really slick
                                // with HDSDR.  Can include later.

// peak1 and peak2 used to avoid exceeding int16 size in the usb,lsb adders.


// GUItool: begin automatically generated code
AudioInputI2S            IQ_in;          //xy=68.66668701171875,360.3333740234375
AudioFilterFIR           PhaseI;           //xy=206,292
AudioFilterFIR           PhaseQ;           //xy=212,458
AudioFilterBiquad        I_filter;        //xy=253.3333333333333,659.9999999999999
AudioFilterBiquad        Q_filter;        //xy=254.16668701171875,799.5833435058594
#ifdef USE_USB_AUDIO
AudioOutputUSB           usb1;           //xy=361.66668701171875,369
#endif
AudioFilterFIR           H90plus;        //xy=364.66668701171875,293.3333435058594
AudioFilterFIR           H00minus;       //xy=375.66668701171875,456.3333740234375
AudioSynthWaveformSine   cosBFO;          //xy=426.66666666666663,739.9999999999999
AudioSynthWaveformSine   sinBFO;          //xy=426.6666946411133,888.3332710266113
AudioEffectMultiply      I_mixer;      //xy=514.9999999999999,668.3333333333333
AudioEffectMultiply      Q_mixer;      //xy=518.3333930969238,806.6667461395264
AudioMixer4              USBmixer;       //xy=546.6666870117188,299.3333435058594
AudioMixer4              LSBmixer;       //xy=549.6666870117188,450.3333740234375
AudioAnalyzeFFT256       LSBscope;       //xy=557.8095550537109,571.9046897888184
AudioAnalyzeFFT256       USBscope;       //xy=560.5238571166992,184.1904640197754
AudioMixer4              SSBselect;         //xy=714.9999961853027,731.6667022705078
AudioAnalyzePeak         peak2;          //xy=730.666690826416,452.3333683013916
AudioFilterFIR           IF12r7;         //xy=733.4763412475586,358.6190776824951
AudioAnalyzePeak         peak1;          //xy=736.1905746459961,299.6190242767334
AudioEffectRectifier     AMdet;          //xy=876.3334197998047,359.2855930328369
AudioAnalyzeRMS          rms1;           //xy=907.7141952514648,659.0952682495117
AudioOutputI2S           LineOut;        //xy=911.7617797851562,730.8573341369629
AudioConnection          patchCord1(IQ_in, 0, PhaseI, 0);
AudioConnection          patchCord2(IQ_in, 1, PhaseQ, 0);
AudioConnection          patchCord3(PhaseI, H90plus);
#ifdef USE_USB_AUDIO
AudioConnection          patchCord4(PhaseI, 0, usb1, 0);
AudioConnection          patchCord7(PhaseQ, 0, usb1, 1);
#endif
AudioConnection          patchCord8(PhaseQ, Q_filter);
AudioConnection          patchCord5(PhaseI, I_filter);
AudioConnection          patchCord6(PhaseQ, H00minus);
AudioConnection          patchCord9(I_filter, 0, I_mixer, 0);
AudioConnection          patchCord10(Q_filter, 0, Q_mixer, 0);
AudioConnection          patchCord11(H90plus, 0, USBmixer, 1);
AudioConnection          patchCord12(H90plus, 0, LSBmixer, 1);
AudioConnection          patchCord13(H00minus, 0, USBmixer, 2);
AudioConnection          patchCord14(H00minus, 0, LSBmixer, 2);
AudioConnection          patchCord15(cosBFO, 0, I_mixer, 1);
AudioConnection          patchCord16(sinBFO, 0, Q_mixer, 1);
AudioConnection          patchCord17(I_mixer, 0, SSBselect, 1);
AudioConnection          patchCord18(Q_mixer, 0, SSBselect, 2);
AudioConnection          patchCord19(USBmixer, USBscope);
AudioConnection          patchCord20(USBmixer, peak1);
AudioConnection          patchCord21(USBmixer, IF12r7);
AudioConnection          patchCord22(LSBmixer, LSBscope);
AudioConnection          patchCord23(LSBmixer, peak2);
AudioConnection          patchCord24(SSBselect, rms1);
AudioConnection          patchCord25(SSBselect, 0, LineOut, 0);
AudioConnection          patchCord26(SSBselect, 0, LineOut, 1);
AudioConnection          patchCord27(IF12r7, AMdet);
AudioConnection          patchCord28(AMdet, 0, SSBselect, 0);
AudioControlSGTL5000     codec;          //xy=227.6666717529297,181.3333342075348
// GUItool: end automatically generated code


// 16 EGA colors from the possible 65k colors. 
         //  4bpp pallett  0-3, 4-7, 8-11, 12-15
const uint16_t EGA[] = { 
         ILI9341_BLACK,    ILI9341_NAVY,    ILI9341_DARKGREEN, ILI9341_DARKCYAN,
         ILI9341_MAROON,   ILI9341_PURPLE,  ILI9341_OLIVE,     ILI9341_LIGHTGREY,
         ILI9341_DARKGREY, ILI9341_BLUE,    ILI9341_GREEN,     ILI9341_CYAN,
         ILI9341_RED,      ILI9341_MAGENTA, ILI9341_YELLOW,    ILI9341_WHITE
         }; 
uint16_t GRAY[16];

const uint16_t WF[] = {
         0b0000000000000000, 0b0000000000000001,0b0000000000000011,0b0000000000000111,
         0b0000000000011111, 0b0000000001111110,0b0000000111111000,0b0000011111100000,
         0b1111110000000000, 0b1111111000000000,0b1111111100000000,0b1111111110000000,
         0b1111111111000000, 0b1111111111110000,0b1111111111111100,0b1111111111111111,          
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
#define VFO_A  1
#define VFO_B  2
#define VFO_SPLIT 4
#define VFO_RIT   8
#define VFO_CW    16
#define VFO_LSB   32
#define VFO_USB   64
#define VFO_AM   128

//  radio variables
int32_t  vfo_a, vfo_b, rit, stp = 100, rit_stp = 10;
uint8_t  vfo_mode;           // undefined, should pick up value on init

// cat polling
#define QUFA 0b00000001
#define QUFB 0b00000010
#define QUIF 0b00101100
#define QUTB 0b01000000
uint32_t cmd_tm;              // command interval timer
uint8_t qu_flags;

// printing cw decode
#define DECODE_LINES 5
#define D_LEFT  8               // margins
#define D_BOTTOM 238            // 
#define D_SIZE   20             // line spacing
char dtext[DECODE_LINES][26];   // buffer
int dline = DECODE_LINES - 1;   // current working line bottom of the screen
int dpos;                       // current cursor position

// screen owners
#define DECODE 0
#define KEYBOARD 1
#define MENUS    2
uint8_t  screen_owner = DECODE;

char kb[5][10] = {
   { '1','2','3','4','5','6','7','8','9','0' },
   { 'Q','W','E','R','T','Y','U','I','O','P' },
   { 'A','S','D','F','G','H','J','K','L', 8 },      // 8 is backspace char
   { 'Z','X','C','V','B','N','M',',','.','/' },
   { '*','*',' ',' ','?','=',' ',' ',' ',' ' }
};

// Menu's

void (* menu_dispatch )( int32_t );    // pointer to function that is processing screen touches

struct menu {
   char title[16];
   const char *menu_item[8];
   int param[8];
   int y_size;                  // x size will be half the screen, two items on a line for now
   int color;
   int current;
};

// mode menu items
const char m_qcx[] = " CW QCX";          // qcx audio sampled on A2 via audio library
const char m_cw[]  = " CW sdr";
const char m_lsb[] = " LSB";
const char m_usb[] = " USB";
const char m_am[]  = " AM";
const char m_sam[] = " ";
const char m_data[]= " ";
const char m_phase[] = "Phase";

struct menu mode_menu_data = {
   { "SDR Mode" },
   { m_qcx,m_cw,m_lsb,m_usb,m_am,m_sam,m_data,m_phase },
   {0,1,2,3,4,-1,-1,7},
   48,
   ILI9341_PURPLE,
   2
};

const char w_3600[] =   " 3600";
const char w_3400[] =   " 3400";
const char w_3200[] =   " 3200";
const char w_3000[] =   " 3000";
const char w_2800[] =   " 2800";
const char w_2600[] =   " 2600";
const char w_2400[] =   " 2400";
const char w_1000[] =   " 1000";
struct menu band_width_menu_data = {
   { "Band Width" },
   { w_3600, w_3400,  w_3200, w_3000, w_2800, w_2600, w_2400, w_1000 },
   { 3600, 3400, 3200, 3000, 2800, 2600, 2400, 1000 },
   48,
   ILI9341_PURPLE,
   3
};

uint8_t ManInTheMiddle;        // act as a USB -> <- QCX serial repeater for CAT commands


// phase correction FIRs. Apparently sometimes the I2s audio samples are 1 step out of phase
// and waterfall shows very little opposite sideband suppression
int16_t  PhaseIfir[2] = { 32767, 0 };    // swap constants to change phasing
int16_t  PhaseQfir[2] = { 32767, 0 };

int peak_atn = 15;        // auto attenuation of line_input, atn will be peak_atn - 15
float agc_gain = 1.0;
//int mux_selected;
float svalue;
float rms_value;

// keyboard transmit
#define TBUFSIZE  16
char tbuf[TBUFSIZE];            // power of two buffer
int  t_in, t_out;
uint8_t tx_in_progress;

// Weaver folding bfo, set to 1/2 the bandwidth
int bfo = 1500;

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

  pinMode(QCX_MUTE,OUTPUT);
  digitalWriteFast(QCX_MUTE,HIGH);
  
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(1);
  
  ts.begin();                        // touchscreen
  ts.setRotation(1);
  
  tft.setTextSize(2);                // sign on message, else see a blank screen until qcx boots.
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(80,200);
  tft.print("QCX+ +SDR");
 
  menu_dispatch = &hidden_menu;     // function pointer for screen touch
  vfo_mode = VFO_LSB;                

   // Setup the audio shield
  AudioNoInterrupts();
  AudioMemory(20);
  codec.enable();
  //codec.volume(0.5);                        // headphone volume, not used eventually
  //codec.unmuteHeadphone();
  codec.inputSelect( AUDIO_INPUT_LINEIN );
  // codec analog gains  
  codec.lineInLevel(15);                    // 0 to 15, used as attenuator, 3.12v to 0.24v
  codec.lineOutLevel(30);                   // 13 to 31 with 13 the loudest. 3.16v to 1.16v
  //codec.adcHighPassFilterDisable();         // less noise ? don't notice any change
  //codec.adcHighPassFilterFreeze();          // try this one

  // bandscope processing                                           
  H90plus.begin(h90p,HILBERT_AM);          // Bandscope and AM detector hilbert
  H00minus.begin(h00m,HILBERT_AM);
  PhaseI.begin(PhaseIfir,2);               // Phasing change delays if needed
  PhaseQ.begin(PhaseQfir,2);
  USBmixer.gain(1,1.0);                    // add signals get USB.   
  USBmixer.gain(2,1.0);
  LSBmixer.gain(1,1.0);                    // sub signals get LSB
  LSBmixer.gain(2,-1.0);

  // weaver audio
  SSBselect.gain(0,0.0);                   // AM off
  SSBselect.gain(1,1.0);                   // !!! gains will be agc_gain or -agc_gain
  SSBselect.gain(2,1.0);                   // -1 for the other sideband
  //mux_selected = 2;              // !!! what does this do now

  IF12r7.begin(AM12r7fir,30);              // AM filter at 12.7k audio IF frequency

  set_Weaver_bandwidth(3000);
  
  AudioInterrupts();

  USBscope.averageTogether(50);            // or 40 for faster waterfall
  LSBscope.averageTogether(50);

}



void set_Weaver_bandwidth(int bandwidth){
  
  bfo = bandwidth/2;                     // weaver audio folding at 1/2 bandwidth
  I_filter.setLowpass(0,bfo,0.51);       // filters are set to 1/2 the desired audio bandwidth
  I_filter.setLowpass(1,bfo,0.60);       // with Butterworth Q's for 4 cascade
  I_filter.setLowpass(2,bfo,0.90);
  I_filter.setLowpass(3,bfo,2.56);

  Q_filter.setLowpass(0,bfo,0.51);
  Q_filter.setLowpass(1,bfo,0.60);
  Q_filter.setLowpass(2,bfo,0.90);
  Q_filter.setLowpass(3,bfo,2.56);

  AudioNoInterrupts();                     // need so cos and sin start with correct phase

    // complex BFO
  cosBFO.amplitude(0.9);                   // what is correct bfo level, sig vs overload in adder
  cosBFO.frequency(bfo);
  cosBFO.phase(90);                        // cosine 
  sinBFO.amplitude(0.9);
  sinBFO.frequency(bfo);
  sinBFO.phase(0);                         // sine

  AudioInterrupts();
  
}


// touch the screen top,middle,bottom to bring up different menus.  Assign menu_dispatch.
// This is default touch processing.   No menu on the screen.
void hidden_menu( int32_t t ){
int32_t  yt, xt;


   screen_owner = MENUS;
   yt = t & 0xff;
   xt = t >> 8;
   
   // check the y value of touch to see what menu area
   if( yt < 60 ){
      menu_display( &mode_menu_data );
      menu_dispatch = &mode_menu;        // screen touch goes to mode_menu() now
   }
   else if ( yt < 140 ){ 
      menu_display( &band_width_menu_data );
      menu_dispatch = &band_width_menu;
   }
   else if( yt > 190 && xt > 270 && (vfo_mode & VFO_CW) ){  // keyboard CW sending
      menu_dispatch = &key_tx;
      key_tx(0);
   }
                           
   else menu_cleanup();                  // not active part of the screen, return to normal op.
}

void mode_menu( int32_t t ){
int selection;
int current;

   current = mode_menu_data.current;
   selection = touch_decode( t, mode_menu_data.y_size );
   if( mode_menu_data.param[selection] != -1 ){
      if( selection != 7 ) mode_menu_data.current = selection;
      selection = mode_menu_data.param[selection];

      if( selection != 7 ) vfo_mode &= 0x0f;    // save the qcx mode flags, clear the sdr mode flags
      switch( selection ){
        case 0:     // QCX audio selected
                 vfo_mode |= VFO_CW;
                 digitalWriteFast(QCX_MUTE,LOW);          // select qcx audio      
                 SSBselect.gain(1,0.0);                   // mute LSB,USB,AM
                 SSBselect.gain(2,0.0);
                 SSBselect.gain(0,0.0);
        break;
        case 1:  vfo_mode |= VFO_CW;                      // SDR CW mode on USB
                 digitalWriteFast(QCX_MUTE,HIGH);                 
                 SSBselect.gain(0,0.0);                   // turn off AM
                 SSBselect.gain(1,agc_gain);
                 SSBselect.gain(2,-agc_gain);             // USB audio
        break;
        case 2:  vfo_mode |= VFO_LSB;  
                 digitalWriteFast(QCX_MUTE,HIGH);
                 SSBselect.gain(0,0.0);
                 SSBselect.gain(1,agc_gain);
                 SSBselect.gain(2,agc_gain); 
        break;
        case 3:  vfo_mode |= VFO_USB;
                 digitalWriteFast(QCX_MUTE,HIGH);
                 SSBselect.gain(0,0.0);                   // AM off
                 SSBselect.gain(1,agc_gain);
                 SSBselect.gain(2,-agc_gain);
        break;
        case 4:  vfo_mode |= VFO_AM;
                 digitalWriteFast(QCX_MUTE,HIGH);
                 SSBselect.gain(0,agc_gain);               // AM on
                 SSBselect.gain(1,0.0);
                 SSBselect.gain(2,0.0);
        break;
        case 7:  PhaseChange(1);  break;                  // phasing correction    
      }
     
   }
   qsy_mode( current, mode_menu_data.current, bfo, bfo );   
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
int old_bfo;

   if( vfo_mode & VFO_AM ) return;              // am bandwidth is fixed

   old_bfo = bfo;
   sel = touch_decode( t, band_width_menu_data.y_size );
   if( band_width_menu_data.param[sel] != -1 ){
      band_width_menu_data.current = sel;
      sel = band_width_menu_data.param[sel];
      set_Weaver_bandwidth(sel);
      qsy_mode( 0, 0, old_bfo, bfo );  // tuning change needed
   }
   
   menu_cleanup();
}

void decode_menu( int32_t t ){
  
}

// I2S audio sometimes starts with I and Q out of order
void PhaseChange(uint8_t chg){
static int val;

   if( chg ){
      if( ++val > 2 ) val = 0;                            // rotate through the settings
   }
      // print 
   tft.setTextSize( 1 );
   tft.setCursor(262,66);
   tft.setTextColor(EGA[14],0);
   tft.print("Ph:  ");

   switch( val ){
      case 0:  
         PhaseIfir[0] = 32767;   PhaseIfir[1] = 0;        // normal in phase
         PhaseQfir[0] = 32767;   PhaseQfir[1] = 0;
         tft.print("1010");
      break;
      case 1:
         PhaseIfir[0] = 32767;   PhaseIfir[1] = 0;    
         PhaseQfir[0] = 0;       PhaseQfir[1] = 32767;    // delay Q  ( delay I if fir runs backward )
         tft.print("1001");
      break;
      case 2:
         PhaseIfir[0] = 0;       PhaseIfir[1] = 32767;    // delay I
         PhaseQfir[0] = 32767;   PhaseQfir[1] = 0;
         tft.print("0110");
      break;
   }   

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
   y -= y_size;    // top row is the title  !!! if sub menus needed, need to decode title touch also
   y /= y_size;    // row of the touch
   x /= 160;       // column 
   return 2*y + x; // two menu items per row
}


void loop() {
static int c_state;
int32_t t;

   // poll radio once a second, process qu_flags as a priority
   if( ( millis() - cmd_tm > 1000 ) && ManInTheMiddle == 0 ){
      if( tx_in_progress || t_in != t_out ) cat.print("KY;");
      else if( qu_flags & QUIF ) cat.print("IF;");
      else if( qu_flags & QUFB ) cat.print("FB;");
      else if( qu_flags & QUFA ) cat.print("FA;");
      else if( qu_flags & QUTB && screen_owner == DECODE && (vfo_mode & VFO_CW ) ) cat.print("TB;");
      else{
          switch(c_state){                         // poll radio round robin for missed packet recovery
              case 0: case 2: case 4: case 6: case 8:
              case 10: case 12: case 14:  cat.print("IF;"); break;
              case 1: cat.print("FA;");   break;
              case 3: cat.print("QU1;");  break;   // enable flags
              case 5: cat.print("FB;");   break;
              case 7: if( screen_owner == DECODE && (vfo_mode & VFO_CW ) ) cat.print("TB1;");
                      //  else cat.print("TB0;");  keep enabled but don't get the info unless CW mode
                      break;   // enable/disable decode flag
              case 9: if( screen_owner == DECODE && (vfo_mode & VFO_CW ) ) cat.print("TB;");
                      break;        
         }
         ++c_state;
         c_state &= 15;
      }
      cmd_tm = millis();
   }

   if( cat.available() ) radio_control();
   if( Serial.available() ){
       char c = Serial.read();
       if( ManInTheMiddle ) cat.write(c);         // CAT command repeater
       else if( c == ';' ) ManInTheMiddle = 1;    // looks like a CAT command on USB serial
   }

   t = touch();
   if( t ){
      // goto where menu_dispatch() points
      (*menu_dispatch)(t);    // off to whoever owns the touchscreen
   }

   USBwaterfall();
   LSBwaterfall();

   auto_atn();             // front end gain reduction if rcv very loud signals
   agc();

   // balance_schmoo();    // !!! testing
}

// change the qcx frequency when changing modes to remain on the same frequency
// change the frequency when bandwidth changes for Weaver demod.
void qsy_mode(int old_mode, int new_mode, int old_bfo, int new_bfo ){
int32_t freq;
char buf[33];
char buf2[33];
   
   freq = vfo_a;
   strcpy(buf,"FA000");
   if( vfo_mode & VFO_B ) freq = vfo_b, buf[1] = 'B';

   if( old_mode != new_mode ){
      switch( old_mode ){                                  // remove current offsets
         case 0:  freq += 700;  break;                     // qsx mode
         case 1:  freq -= (bfo-700);  break;               // cw sdr
         case 2:  freq += bfo;  break;                     // lsb
         case 3:  freq -= bfo;  break;                     // usb
         case 4:  freq += 12700;  break;                       // am filter center
      }
      switch( new_mode ){
         case 0:  freq -= 700;  break;
         case 1:  freq += (bfo-700);  break;
         case 2:  freq -= bfo;  break;                     // lsb
         case 3:  freq += bfo;  break;                     // usb
         case 4:  freq -= 12700;   break;
      }
   }

   if( old_bfo != new_bfo ){                            // bandwidth change
     if( vfo_mode & VFO_LSB ){
       freq += old_bfo;
       freq -= new_bfo;
     }
     else{
       freq -= old_bfo;
       freq += new_bfo;
     }
   }
 
   if( freq < 10000000 ) strcat(buf,"0");               //add a zero
   itoa( freq, buf2, 10 );
   strcat( buf, buf2 );
   strcat( buf,";");
   delay(300);                                          // wait any current command
   cat.print(buf);
   cmd_tm = millis();                                   // delay future commands, allow this one to process
}

// avoid overload.  This looks at 40khz of signal on USB and LSB so not a replacement for AGC
// both sideband signals feed the bandscope even if not listened to
// !!!should the select mux gain be increased to avoid desense on loud out of bandwidth signals
void auto_atn(){           // lower front end gain if signals approach overload
static uint32_t tm;        
static int no_chg_cnt;     // slowly raise gain

   if( millis() - tm < 10 ) return;
   if( peak1.available() == 0 ) return;
   if( peak2.available() == 0 ) return;
   ++no_chg_cnt;
   tm = millis();
   
   if( peak_atn > 0 && (peak1.read() > 0.9 || peak2.read() > 0.9) ){    // lower gain on strong signal
       --peak_atn;
       codec.lineInLevel(peak_atn);
       vfo_mode_disp();
       no_chg_cnt = 0;
   }
   // raise gain if no strong signals for awhile
   if( no_chg_cnt > 200 && peak_atn < 15 ){        // 200 == 2 seconds
       ++peak_atn;
       codec.lineInLevel(peak_atn);
       vfo_mode_disp();
       no_chg_cnt = 0;
   }
}


#define AGC_FLOOR 0.07      // 0.10 0.15
#define AGC_SLOPE 6.0       //  8.0
#define AGC_HANG 300
void agc(){
static uint32_t tm;
static float sig;
static int hang;
float reading;
int ch;

   if( millis() - tm < 1 ) return;      // working at 1000hz
   if( rms1.available() == 0 ) return;
   tm = millis();

   ch = 0;
   rms_value = reading = rms1.read();
   if( reading > sig && reading > AGC_FLOOR ){         // attack
       sig = sig + 0.001;
       ch = 1;
       hang = 0;
   }
   else if( sig > AGC_FLOOR && hang++ > AGC_HANG ){    // decay
       sig = sig - 0.00005;
       ch = 1;
   }

   if( ch ){
     agc_gain = sig - AGC_FLOOR;
     agc_gain = agc_gain * AGC_SLOPE;
     agc_gain = 1.0 - agc_gain;     
     // SSBselect.gain(mux_selected,agc_gain);
 //    if( vfo_mode & VFO_AM ) ModeSelect.gain(0,agc_gain);
 //    else ModeSelect.gain(1,agc_gain);
       // Serial.println(agc_gain);
     if( vfo_mode & VFO_AM ) SSBselect.gain(0,agc_gain);
     else{
        SSBselect.gain(1,agc_gain);
        if( vfo_mode & VFO_LSB ) SSBselect.gain(2,agc_gain);
        else SSBselect.gain(2,-agc_gain);
     }
     if( screen_owner == DECODE ){
        tft.setTextSize(1);
        tft.setTextColor(EGA[14],0);
        tft.setCursor(262,114);  
        tft.print("Sig: ");
        tft.print(sig); 
     } 
   }

   smeter();
  
}


#define ybase 250              // vertical position of all
void smeter(){
static uint8_t  onscreen;
float angle;
int x,y;
static int lastx = 160,lasty = 239;
int diff;
float tvalue;
float ftemp;

   if( screen_owner != DECODE || (vfo_mode & VFO_CW) ){       // show the meter ?
      onscreen = 0;
      return;
   }

   if( onscreen == 0 ){         // init the meter area
      onscreen = 1;
      tft.fillRect(5, 129, 310, 110, EGA[0]);
      for( ftemp = 135; ftemp > 45; ftemp -= 0.2 ){
            y = ybase - 90 * sin(ftemp/57.3);
            x = 160 + 180 * cos(ftemp/57.3);
            tft.drawPixel(x,y,EGA[15]);
            tft.drawPixel(x,y-1,EGA[15]);
            tft.drawPixel(x,y+1,EGA[15]);
            if( int(ftemp) == 90 )  tft.drawLine(x,y,x,y-7,EGA[15]);
            if( int(ftemp) >= 133 ) tft.drawLine(x,y,x-4,y-6,EGA[15]);
            if( int(ftemp) <= 46 )  tft.drawLine(x,y,x+4,y-6,EGA[15]);
            if( int(ftemp) == 123 ) tft.drawLine(x,y,x-3,y-6,EGA[15]);
            if( int(ftemp) == 111 ) tft.drawLine(x,y,x-2,y-6,EGA[15]);
            if( int(ftemp) == 99  ) tft.drawLine(x,y,x-1,y-6,EGA[15]);
            if( int(ftemp) == 75  ) tft.drawLine(x,y,x+1,y-6,EGA[15]);
            if( int(ftemp) == 60  ) tft.drawLine(x,y,x+2,y-6,EGA[15]);
            tft.setTextColor(EGA[15],0);
            tft.setTextSize(2);
            tft.setCursor(25,ybase-90);
            tft.write('1');
            tft.setCursor(90,ybase-109);
            tft.write('5');
            tft.setCursor(156,ybase-115);
            tft.write('9');
            tft.setCursor(210,ybase-109);
            tft.print("20");
            tft.setCursor(250,ybase-100);
            tft.print("40");
      }
   }

   tvalue = rms_value/agc_gain;                // what the signal would be without the agc
   ftemp  = map( peak_atn, 0, 15, 312, 24 );   // voltage ratio due to front end attenuation
   tvalue = ftemp * tvalue / 24.0;             // what the signal would be without the attenuator
   tvalue *= 160;                              // fudge factor scaling to make it look good
   tvalue = log10(tvalue);
   if( tvalue > svalue ) svalue += 0.02;       // smooth out the meter movement
   if( tvalue < svalue ) svalue -= 0.005;

   //angle = map( svalue*10, 1, 120, 1350, 450 );    // S1 to S12 mapped to 90 deg +- 45 scaled by 10.
   angle = map( svalue * 100, 1, 300, 13500, 4500 );   // more fudging
   angle /= 100.0;
  // Serial.println(angle);         // !!! debug, remove
   if( angle < 45 ) angle = 45;
   if( angle > 135 ) angle = 135;

   y = ybase - 80 * sin(angle/57.3);
   x = 160 + 180 * cos(angle/57.3);

   diff = x - lastx;
   if( diff < 0 ) diff = -diff;
   if( diff > 2 ){
      tft.drawLine(lastx-1,lasty,159,ybase+140,EGA[0]);
      tft.drawLine(lastx,lasty,160,ybase+140,EGA[0]);
      tft.drawLine(lastx+1,lasty,161,ybase+140,EGA[0]);

      tft.drawLine(x-1,y,159,ybase+140,EGA[12]);
      tft.drawLine(x,y,160,ybase+140,EGA[12]);           // lines clip to screen in plot pixel
      tft.drawLine(x+1,y,161,ybase+140,EGA[12]);

      lastx = x;
      lasty = y;
   }
}

// send cw via a touchscreen keyboard
void key_tx( int32_t t ){
int x,y;
char c;
static char buff[3][25];    // screen display
static int pos;

   if( screen_owner != KEYBOARD ){     // need to display the keyboard
      screen_owner = KEYBOARD;
      tft.fillScreen(ILI9341_BLACK);
      // tft.fillRect(0, 80, 320, 160, EGA[0]);
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
      return;                // ignore this touch
   }

   x = t >> 8;   y = t & 0xff;
   // tft.setCursor(80-12,80-12);
   y -= 80;
   x /= 32;   y /= 32;

   c = ' ';
   if( x >= 0 && x < 10 && y >= 0 && y < 5 ){
      c = kb[y][x];
      if( ++pos >= 24 ){       // scroll text
          buff[2][24] = 0;
          for(y = 0; y < 2; ++y){
             for( x = 0; x < 25; ++x ) buff[y][x] = buff[y+1][x];
          }
          for( x = 0; x < 24; ++x ) buff[2][x] = ' ';
          buff[2][24] = 0;
          tft.setTextSize(2);
          tft.setTextColor(EGA[14],0);
          for( y = 0; y < 3; ++y ){
             tft.setCursor( 5, 5 + 22*y );
             buff[y][24] = 0;
             tft.print(&buff[y][0]);
          }
          pos = 0;
      }
      
      tft.setCursor( 5 + 12*pos, 5 + 22 * 2 );
      tft.setTextSize(2);
      tft.setTextColor(EGA[15],0);
      tft.write( c );
      buff[2][pos] = c;
      tbuf[t_in++] = c;
      t_in &= (TBUFSIZE-1);
      cat.print("KY;");
   }

   if( c == '*' ){
     //tft.fillRect(0, 80, 320, 160, EGA[0]);
     //screen_owner = DECODE;
     menu_cleanup();
     //cat.print("TB1;");
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
int u,l,c,m;

   if( screen_owner != DECODE ) return;
   a = b = s = r = 0;                       // set colors of the vfo mode
   c = u = l = m = 0;
   
   if( vfo_mode & VFO_B ) b = 10;
   if( vfo_mode & VFO_A ) a = 10;
   if( vfo_mode & VFO_SPLIT ) s = 10;
   if( vfo_mode & VFO_RIT ) r = 10;
   if( vfo_mode & VFO_CW ) c = 10;
   if( vfo_mode & VFO_LSB ) l = 10;
   if( vfo_mode & VFO_USB ) u = 10;
   if( vfo_mode & VFO_AM )  m = 10;
   
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
   tft.setTextColor(EGA[4+c],0);
   tft.print("CW");

   tft.setCursor(175,2);
   tft.setTextColor(EGA[4+u],0);
   tft.print("USB");

   tft.setCursor(205,2);
   tft.setTextColor(EGA[4+l],0);
   tft.print("LSB");

   tft.setCursor(235,2);
   tft.setTextColor(EGA[4+m],0);
   tft.print("AM");
   
   tft.setCursor(280,2);
   tft.setTextColor(EGA[4+r],0);
   tft.print("RIT");
    
   tft.drawLine(0,61,640,60,EGA[4]);
   tft.drawLine(0,62,640,61,EGA[4]);

   PhaseChange(0);           // just print current value
   tft.setCursor(262,78);    // print bandwidth ( spacing 12 pixel ? )
   tft.setTextSize(1);
   tft.setTextColor(EGA[14],0);
   tft.print("BW: ");
   tft.print(band_width_menu_data.menu_item[band_width_menu_data.current]);
   tft.setCursor(262,90);
   tft.print("Stp: ");
   if(stp <= 1000 ) p_leading(stp,4);
   tft.setCursor(262,102);
   tft.print("Atn: ");
   tft.print(peak_atn - 15);   tft.write(' ');
   tft.setCursor(262,114);
   //tft.print("Sig: ");
   //tft.print(rms1.read());  
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

   if( (vfo_mode & VFO_AM ) == VFO_AM ) vfo += 12000;   // AM IF is 12.7 but sound better 1k low
   else if( ( vfo_mode & VFO_CW ) == 0 ) vfo -= 700;
   if( (vfo_mode & VFO_USB ) ) vfo -= bfo;
   if( (vfo_mode & VFO_LSB ) ) vfo += bfo;
   if( (vfo_mode & VFO_CW ) &&  mode_menu_data.current == 1) vfo -= bfo;
   
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
       if( ManInTheMiddle ) Serial.write(c);  // Serial repeater mode
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
    if( response[0] == 'K' && response[1] == 'Y' ) cat_transmit();

    len = 0;     // reset string to start for next command
}


void cat_transmit(){    // sending via the touch keyboard

   if( response[2] == '1' ) tx_in_progress = 1;
   else{
       tx_in_progress = 0;
       if( t_in != t_out ) cat_transmit2();    // punt
   }
}

void cat_transmit2(){
char msg[16];
int  i;
char c;

   strcpy(msg,"KY ");
   i = 3;
   while( t_in != t_out ){
      c = tbuf[t_out++];
      t_out &= ( TBUFSIZE-1);
      if( c == '*' ) break;         // switch to receive
      msg[i++] = c;
      if( i > 12 ) break;
   }
   msg[i++] = ';';
   msg[i] = 0;
   if( i > 4 ) cat.print(msg);      // avoid null message KY ;
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
       if( s >= 1000 ) temp = 1000;   
       else temp = s;

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
   temp = vfo_mode & 0xf0;
   if( response[30] == '1' ) temp |= VFO_B;
   else temp |= VFO_A;
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



void balance_schmoo(){    //  setup function.  Find the least signal on USB

   // setup with a test signal received on LSB side.  This will measure the audio image on USB side.
   // and print out the usb gain for best IQ balance at whatever audio freq was tuned.
static float startval, endval,val, stp;
static uint32_t  tm;
static float minval;
static float minpeak;
float rdpeak;

   // starting values
   startval = 0.70;   endval = 1.3;  stp = 0.001;
   if( val < startval || val > endval ){
      Serial.print( minval,4);   Serial.write(' ');
      Serial.println( minpeak,4);
      minpeak = 2.0;
      val = startval;
      tm = millis();
      USBmixer.gain(1,val);
      USBmixer.gain(2,1/val);
   }

   if( millis() - tm < 10 ) return;
   if( peak1.available() == 0 ) return;
   tm = millis();
   
   rdpeak = peak1.read();
   if( rdpeak < minpeak ){
       minval = val;
       minpeak = rdpeak;
   }
   val += stp;
   USBmixer.gain(1,val);
 
}


// find a waterfall pallet color 0 to 15
uint8_t lg2( float f, int side){    // log base 2, don't exceed 15.  f 0.0 to 1.0
uint32_t val;
uint8_t  r;
uint16_t  bw_low,bw_high;

    // show bandwidth on waterfall                                             // USB audio is on LSB waterfall
    if( side <= 0 && (vfo_mode & (VFO_USB + VFO_CW))){                         // LSB bandwidth coloring
       bw_high = band_width_menu_data.param[band_width_menu_data.current];
       bw_high /= 172;
       bw_low = bfo/172;
       bw_high = bw_low - bw_high;
       side = -side;
       if( mode_menu_data.current != 0 ){
          if( side == bw_high || side == bw_low ) return 10;
       }
    }
    else if(side >= 0 && (vfo_mode & ( VFO_LSB + VFO_AM ))){  // USB or AM
       bw_high = band_width_menu_data.param[band_width_menu_data.current];
       bw_high /= 172;
       if( vfo_mode & VFO_AM ){
          bw_low = 73 - bw_high;
          bw_high += 73;
       }
       else{
        bw_low = bfo/172;
        bw_high = bw_low - bw_high;
       }
       if( side == bw_low || side == bw_high ) return 10;
    }

    // show the ends of the waterfall
    if( side == 127 || side == -127 ) return 8;
    
    if( f < 0.0015 ) return 0;
    f -= 0.0015;            // set min signal level, noise just shows on waterfall 0015
    val = 20000 * f;        // scale here for best display < 32000 has unused pallett
    r = 0;
    while( val ){           // log base 2
      ++r;
      val >>= 1;
      if( r == 15 ) break;
    }
    //r += bw_add;
    if( r > 15 ) r = 15;
    return r;
}

void USBwaterfall(){
static uint8_t data[64][64];
int i,j;
uint8_t *p;
uint8_t d;

    if( USBscope.available() == 0 ) return;
    if( screen_owner != DECODE ) return;
    
    // scroll the data the most obvious and probably slowest way
    for( i = 63; i > 0; --i ){
        for( j = 0; j < 64; ++j ) data[i][j] = data[i-1][j];
    }
    // get fft result and convert to 4 bits
    j = 0;
    for( i = 0; i < 64; ++i ){
       d = lg2( USBscope.read(j),j );
       ++j;
       data[0][i] = d << 4;
       d = lg2( USBscope.read(j),j );
       ++j;
       data[0][i] |= d;  
    }

    // display on screen
    p = &data[0][0];
    tft.writeRect4BPP( 131,64,128,64,p,WF );
  
}


void LSBwaterfall(){
static uint8_t data[64][64];
int i,j;
uint8_t *p;
uint8_t d;

    if( LSBscope.available() == 0 ) return;
    if( screen_owner != DECODE ) return;
    
    // scroll the data the most obvious and probably slowest way
    for( i = 63; i > 0; --i ){
        for( j = 0; j < 64; ++j ) data[i][j] = data[i-1][j];
    }
    // get fft result and convert to 4 bits
    j = 127;
    for( i = 0; i < 64; ++i ){
       d = lg2( LSBscope.read(j),-j );      // neg j for lower sideband bandwidth edges
       --j;
       data[0][i] = d << 4;
       d = lg2( LSBscope.read(j),-j );
       --j;
       data[0][i] |= d;  
    }

    // display on screen
    p = &data[0][0];
    tft.writeRect4BPP( 130-126,64,128,64,p,WF );


/*
    static unsigned int c = 0;
    ++c;
    if( (c & 31) == 0 ) {
    float mn,mx;                    // signal levels here !!! debug
    mn = 1; mx = 0;
    for( i = 0; i < 128; ++i ){
        f = LSBscope.read(i);
        if( f > mx ) mx = f;
        if( f < mn ) mn = f;
    }
    Serial.print(mn,4);  Serial.write(' ');
    Serial.println(mx,4);
    }  
    */
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
