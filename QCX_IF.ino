/*
 *   QCX with Teensy 3.6, Audio shield, and ILI9341 touchscreen added on QCX Dev Board.
 *     IF version
         Runs the baseband version for the band scope, and AM detector
         and runs a 6.6 khz IF for SSB

 *   A note:  putting capacitors in C4 C7 caused the front end to pick up screen draw noise.  Line In is
 *   connected to those nodes for connection of I and Q.  The difference with and without is substantial.
 * 
 * to work on
 *   Message buffer transmit
 *   Consider changing what determines the mode we are in, decode, vfo_mode or what?
 *   Choose a bfo placement on the roofing filter. Maybe use different placement for CW vs SSB.
 *   Change one of the low pass filters to bandpass for Narrow band modes.
 *   Add manual attenuation
 *   make a hole in the top case
 *   Add an external cable for usb audio, PC CAT mode.  Low priority for me as this is a standalone SDR.
 *   RTTY, PSK31 decoders - software only.  Low priority as will not be transmitting these modes.
 *   Could disable Audio objects when they are not in use (example AM detector, Tone detectors, Roof filter)  
 *   
     
   Free pins when using audio board and display.
   Teensy 3.6.  Many more pins unused but not populated with headers.
   
  0,1    used for Serial1  Rx, Tx
  2      mute switch 
  3      key ( PPS )
  4
  5
  8      used for touch CS
 16      A2  
 17      A3  


Have decided to expand upon this version.  The general plan is when one chooses the QCX CW mode, you have 
a QCX without any changes, enhancements are for SDR modes.  The baseband and Weaver versions may or may not
be updated with some of these features.  The Weaver version looks promising for different hardware.

Change log:
  Added a CW tuning indicator.
  Lower the 1100 hz filter for CW work.  Trying a 900hz lowpass.
  Added a separate CW decoder for SDR CW mode.  Had some trouble with the audio library Tone objects
     returning zero. Added an audio object with a gain of 10 and some workaround code.
  Move SDR CW mode to the IF to get away from the baseband noise.
     Operate in a tracking split mode to keep TX freq correct.
     Added RIT which moves the bfo when in CW SDR mode.  The QCX reports RIT changes in split mode
     but does not implement with tuning changes.
  Verified power out ok, tracks spec well, but expected better than 65% efficiency with class E.
     1.2 watts at 9 volts, 50% eff.
     2 watts at 10 volts, 60% eff.   2.5 watts at 11 volts.  3 watts at 12.  3.8 watts at 13.
  Added digital gain to the agc function to compensate for when loud signals outside the listening
     bandwidth reduce the front end gain ( from auto attenuation in the SSB mixers when peak signals added
     approach 32767 - max value for int16_t. )
  Peak2 seems redundant, commented out its code.
  New Waterfall algorithm for less time used per call.     
 
 *******************************************************************************/


#include <ILI9341_t3.h>
#include <XPT2046_Touchscreen.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>

#include "hilbert_IF.h"
#include "my_morse.h"           // morse and baudot tables
#include "helldefs.h"

#define QCX_MUTE 2              // pin 2 high mutes qcx audio
#define QCX_KEY  3              // keys PPS via BS170 FET


// peak1 used to avoid exceeding int16 size in the usb,lsb adders.

// GUItool: begin automatically generated code
AudioInputI2S            IQ_in;          //xy=142,624
AudioFilterFIR           PhaseI;         //xy=280,556
AudioFilterFIR           PhaseQ;         //xy=286,722
AudioMixer4              SSBselect;      //xy=325,1019
AudioFilterFIR           H90plus;        //xy=438,557
AudioFilterFIR           H00minus;       //xy=449,720
AudioFilterFIR           SSBroof;        //xy=499,1017
AudioMixer4              USBmixer;       //xy=620,563
AudioMixer4              LSBmixer;       //xy=623,714
AudioAnalyzeFFT256       LSBscope;       //xy=631,835
AudioAnalyzeFFT256       USBscope;       //xy=634,448
AudioSynthWaveformSine   BFO;            //xy=700,1150
AudioEffectMultiply      Second_mixer;   //xy=708,1026
// AudioAnalyzePeak         peak2;          //xy=804,716
AudioFilterFIR           IF12r7;         //xy=807,622
AudioAnalyzePeak         peak1;          //xy=810,563
AudioMixer4              ModeSelect;     //xy=899,1034
AudioEffectRectifier     AMdet;          //xy=950,623
AudioFilterBiquad        BandWidth;      //xy=1059,1032
AudioAmplifier           amp1;           //xy=1065.7142857142856,1138.5714285714284
AudioAnalyzeToneDetect   tone700;          //xy=1222,1143
AudioAnalyzeToneDetect   tone800;          //xy=1222,1191
AudioAnalyzeToneDetect   tone600;          //xy=1223,1096
AudioAnalyzeRMS          rms1;           //xy=1261,949
AudioOutputI2S           LineOut;        //xy=1262,1029
AudioConnection          patchCord1(IQ_in, 0, PhaseI, 0);
AudioConnection          patchCord2(IQ_in, 1, PhaseQ, 0);
AudioConnection          patchCord3(PhaseI, H90plus);
AudioConnection          patchCord4(PhaseQ, H00minus);
AudioConnection          patchCord5(SSBselect, SSBroof);
AudioConnection          patchCord6(H90plus, 0, USBmixer, 1);
AudioConnection          patchCord7(H90plus, 0, LSBmixer, 1);
AudioConnection          patchCord8(H00minus, 0, USBmixer, 2);
AudioConnection          patchCord9(H00minus, 0, LSBmixer, 2);
AudioConnection          patchCord10(SSBroof, 0, Second_mixer, 0);
AudioConnection          patchCord11(USBmixer, USBscope);
AudioConnection          patchCord12(USBmixer, peak1);
AudioConnection          patchCord13(USBmixer, IF12r7);
AudioConnection          patchCord14(USBmixer, 0, SSBselect, 1);
AudioConnection          patchCord15(LSBmixer, LSBscope);
//AudioConnection          patchCord16(LSBmixer, peak2);
AudioConnection          patchCord17(LSBmixer, 0, SSBselect, 2);
AudioConnection          patchCord18(BFO, 0, Second_mixer, 1);
AudioConnection          patchCord19(Second_mixer, 0, ModeSelect, 1);
AudioConnection          patchCord20(IF12r7, AMdet);
AudioConnection          patchCord21(ModeSelect, BandWidth);
AudioConnection          patchCord22(AMdet, 0, ModeSelect, 0);
AudioConnection          patchCord23(BandWidth, rms1);
AudioConnection          patchCord24(BandWidth, 0, LineOut, 0);
AudioConnection          patchCord25(BandWidth, 0, LineOut, 1);
AudioConnection          patchCord26(BandWidth, amp1);
AudioConnection          patchCord27(amp1, tone600);
AudioConnection          patchCord28(amp1, tone700);
AudioConnection          patchCord29(amp1, tone800);
AudioControlSGTL5000     codec;          //xy=301,445
// GUItool: end automatically generated code

/*
// IF version with baseband CW and CW tuning display for best decoding
// removing the baseband CW to see if can make the vfo's track for a CW offset of 8k
// GUItool: begin automatically generated code
AudioInputI2S            IQ_in;          //xy=142,624
AudioFilterFIR           PhaseI;         //xy=280,556
AudioFilterFIR           PhaseQ;         //xy=286,722
AudioMixer4              SSBselect;      //xy=325,1019
AudioFilterFIR           H90plus;        //xy=438,557
AudioFilterFIR           H00minus;       //xy=449,720
AudioFilterFIR           SSBroof;        //xy=499,1017
AudioMixer4              USBmixer;       //xy=620,563
AudioMixer4              LSBmixer;       //xy=623,714
AudioAnalyzeFFT256       LSBscope;       //xy=631,835
AudioAnalyzeFFT256       USBscope;       //xy=634,448
AudioSynthWaveformSine   BFO;            //xy=700,1150
AudioEffectMultiply      Second_mixer;   //xy=708,1026
AudioAnalyzePeak         peak2;          //xy=804,716
AudioFilterFIR           IF12r7;         //xy=807,622
AudioAnalyzePeak         peak1;          //xy=810,563
AudioMixer4              ModeSelect;     //xy=899,1034
AudioEffectRectifier     AMdet;          //xy=950,623
AudioFilterBiquad        BandWidth;      //xy=1059,1032
AudioAnalyzeToneDetect   tone700;          //xy=1222,1143
AudioAnalyzeToneDetect   tone800;          //xy=1222,1191
AudioAnalyzeToneDetect   tone600;          //xy=1223,1096
AudioAnalyzeRMS          rms1;           //xy=1261,949
AudioOutputI2S           LineOut;        //xy=1262,1029
AudioConnection          patchCord1(IQ_in, 0, PhaseI, 0);
AudioConnection          patchCord2(IQ_in, 1, PhaseQ, 0);
AudioConnection          patchCord3(PhaseI, H90plus);
AudioConnection          patchCord5(PhaseQ, H00minus);
AudioConnection          patchCord7(SSBselect, SSBroof);
AudioConnection          patchCord8(H90plus, 0, USBmixer, 1);
AudioConnection          patchCord9(H90plus, 0, LSBmixer, 1);
AudioConnection          patchCord10(H00minus, 0, USBmixer, 2);
AudioConnection          patchCord11(H00minus, 0, LSBmixer, 2);
AudioConnection          patchCord12(SSBroof, 0, Second_mixer, 0);
AudioConnection          patchCord13(USBmixer, USBscope);
AudioConnection          patchCord14(USBmixer, peak1);
AudioConnection          patchCord15(USBmixer, IF12r7);
AudioConnection          patchCord16(USBmixer, 0, SSBselect, 1);
//AudioConnection          patchCord17(USBmixer, 0, ModeSelect, 2);   // baseband cw
AudioConnection          patchCord18(LSBmixer, LSBscope);
AudioConnection          patchCord19(LSBmixer, peak2);
AudioConnection          patchCord20(LSBmixer, 0, SSBselect, 2);
AudioConnection          patchCord21(BFO, 0, Second_mixer, 1);
AudioConnection          patchCord22(Second_mixer, 0, ModeSelect, 1);
AudioConnection          patchCord23(IF12r7, AMdet);
AudioConnection          patchCord24(ModeSelect, BandWidth);
AudioConnection          patchCord25(AMdet, 0, ModeSelect, 0);
AudioConnection          patchCord26(BandWidth, rms1);
AudioConnection          patchCord27(BandWidth, 0, LineOut, 0);
AudioConnection          patchCord28(BandWidth, 0, LineOut, 1);
AudioConnection          patchCord29(BandWidth, tone600);
AudioConnection          patchCord30(BandWidth, tone700);
AudioConnection          patchCord31(BandWidth, tone800);
AudioControlSGTL5000     codec;          //xy=301,445
// GUItool: end automatically generated code
*/

// #define USE_USB_AUDIO        // not needed for self contained radio but works really slick
                                // with HDSDR.  Can include later.
#ifdef USE_USB_AUDIO
  AudioOutputUSB           usb1;           //xy=435,633
  AudioConnection          patchCord4(PhaseI, 0, usb1, 0);
  AudioConnection          patchCord6(PhaseQ, 0, usb1, 1);
#endif



// we have 65k colors.  How to pick one? Limit to just 16.
         //  4bpp pallett  0-3, 4-7, 8-11, 12-15
const uint16_t EGA[] = { 
         ILI9341_BLACK,    ILI9341_NAVY,    ILI9341_DARKGREEN, ILI9341_DARKCYAN,
         ILI9341_MAROON,   ILI9341_PURPLE,  ILI9341_OLIVE,     ILI9341_LIGHTGREY,
         ILI9341_DARKGREY, ILI9341_BLUE,    ILI9341_GREEN,     ILI9341_CYAN,
         ILI9341_RED,      ILI9341_MAGENTA, ILI9341_YELLOW,    ILI9341_WHITE
         }; 
uint16_t GRAY[16];

// waterfall pallet, try just sliding bits
//const uint16_t WF[] = {
//         0b0000000000000000, 0b0000000000000001,0b0000000000000011,0b0000000000000111,
//         0b0000000000001111, 0b0000000000011111,0b0000000000111111,0b0000000001111111,
//         0b0000000011111111, 0b0000000111111111,0b0000001111111111,0b0000011111111111,
//         0b0000111111111111, 0b0001111111111111,0b0011111111111111,0b0111111111111111,          
//         };
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
// !!! not sure we need step, as just guessing at the value from how far the vfo moves per update.
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
   { '*','*',' ','#','?','=',' ',' ',' ',' ' }
};

// Menu's

void (* menu_dispatch )( int32_t );    // pointer to function that is processing screen touches

struct menu {
   char title[16];
   const char *menu_item[8];    // array of pointers to strings
   int param[8];
   int y_size;                  // x size will be half the screen, two items on a line for now
   int color;
   int current;
};

// mode menu items
const char m_qcx[] = " CW QCX";          // qcx audio pass through mode
const char m_cw[]  = " CW SDR";
const char m_lsb[] = " LSB";
const char m_usb[] = " USB";
const char m_am[]  = " AM";
const char m_sam[] = " ";
const char m_data[]= " ";
const char m_phase[] = "Phase";          // codec issue with samples order ( twin peaks issue )

struct menu mode_menu_data = {
   { "SDR Mode" },
   { m_qcx,m_cw,m_lsb,m_usb,m_am,m_sam,m_data,m_phase },
   {0,1,2,3,4,-1,-1,7},
   48,
   ILI9341_PURPLE,
   2
};

const char w_6k[] =     " 6000";
const char w_4k[] =     " 4500";
const char w_3600[] =   " 3600";
const char w_3300[] =   " 3300";
const char w_3000[] =   " 3000";
const char w_2700[] =   " 2700";
const char w_2400[] =   " 2400";
const char w_1100[] =   " 900";
struct menu band_width_menu_data = {
   { "Band Width" },
   { w_6k, w_4k,  w_3600, w_3300, w_3000, w_2700, w_2400, w_1100 },
   { 6000, 4500, 3600, 3300, 3000, 2700, 2400, 900 },
   48,
   ILI9341_PURPLE,
   4
};

#define DSMETER 0
#define DCW 1
#define DHELL 2
const char d_smeter[] = "S Meter";
const char d_cw[]     = " CW";
const char d_hell[]   = "F Hell";
struct menu decode_menu_data = {
   {"Decode Mode"},
   { d_smeter, d_cw, d_hell },
   { DSMETER, DCW, DHELL, -1, -1, -1, -1, -1 },
   48,
   ILI9341_PURPLE,
   0
};

#define MSG_BUF_SIZE  600           // storing 1st 8 messages, 4x100 + 4x50
char msg_buf[MSG_BUF_SIZE];         // one big buffer

struct menu qcx_message = {
   {"Xmit Message"},
   { &msg_buf[0], &msg_buf[100], &msg_buf[200], &msg_buf[300], &msg_buf[400], &msg_buf[450],
     &msg_buf[500], &msg_buf[550] },
   { -2, -2, -2, -2, -2, -2, -2, -2 },
   45,                                      // 45 max for text size 2
   ILI9341_NAVY,
   0
};


int ManInTheMiddle;        // act as a USB -> <- QCX serial repeater for CAT commands


// phase correction FIRs. Apparently sometimes the I2s audio samples are 1 step out of phase
// and waterfall shows very little opposite sideband suppression
int16_t  PhaseIfir[2] = { 32767, 0 };    // swap constants to change phasing
int16_t  PhaseQfir[2] = { 32767, 0 };

int peak_atn = 15;        // auto attenuation of line_input, atn will be peak_atn - 15
float agc_gain = 1.0;
int mux_selected;
float svalue;
float rms_value;

// keyboard transmit
#define TBUFSIZE  64
char tbuf[TBUFSIZE];            // power of two buffer for transmit cw, hell
int  t_in, t_out;
int  tx_in_progress;

// Audio IF
#define BFO_FREQ 9000
int bfo = BFO_FREQ;             // bfo on the upper edge of the roofing filter

// cw decode
int cread_buf[16];
int cread_indx;
int dah_table[8];
int dah_in;

// Feld Hell - make double use of the rms1 signal level
IntervalTimer RXsigs;
float rms_buf[8];
int rms_in;
int rms_out;
volatile int hflag;         // ready to feed next transmit char to Hellschreiber transmit?
char hchar;
int hcount;                 // number of special characters in the buffer, used for word spacing

// 4 line Feld Hell display or a larger easier to read 3 line display that borrows some waterfall space, pick one
#define USE_3_LINE
// #define USE_4_LINE


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
  digitalWriteFast(QCX_MUTE,HIGH);       // mute QCX, start in SDR mode
  pinMode(QCX_KEY,OUTPUT);
  digitalWriteFast(QCX_KEY,LOW);         // RX mode
  
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
  AudioMemory(30);
  codec.enable();
  //codec.volume(0.5);                        // headphone volume, not used eventually
  //codec.unmuteHeadphone();
  codec.inputSelect( AUDIO_INPUT_LINEIN );
  // codec analog gains  
  codec.lineInLevel(15);                    // 0 to 15, used as attenuator, 3.12v to 0.24v
  codec.lineOutLevel(28);                   // 13 to 31 with 13 the loudest. 3.16v to 1.16v, was 30
  //codec.adcHighPassFilterDisable();         // less noise ? don't notice any change
  //codec.adcHighPassFilterFreeze();          // try this one
                                             
//  H45plus.begin(h45p,HILBERT_IF);          // SSB 3.75k wide at IF 11k
//  H45minus.begin(h45m,HILBERT_IF);
  H90plus.begin(h90p,HILBERT_AM);          // Bandscope and AM detector hilbert
  H00minus.begin(h00m,HILBERT_AM);
  PhaseI.begin(PhaseIfir,2);               // Phasing change delays if needed
  PhaseQ.begin(PhaseQfir,2);
  SSBroof.begin(RoofFir,80);               // 40 taps for the previous version
  USBmixer.gain(1,1.0);                    // add signals get USB.   
  USBmixer.gain(2,1.0);
  LSBmixer.gain(1,1.0);                    // sub signals get LSB
  LSBmixer.gain(2,-1.0);

  // the BFO inverts the audio, so get USB audio from the LSB selection, LSB from USB selection
  SSBselect.gain(1,1.0);
  SSBselect.gain(2,0.0);                   // Select USB, listen to LSB
  mux_selected = 1;

  BFO.amplitude(1.0);                      // turn on the bfo
  BFO.frequency(bfo);                     // put on upper band edge of the Roofing filter
                                           // adjust for best sound vs image rejection
                                           // no commands for the 2nd mixer
  ModeSelect.gain(0,0.0);                  // turn off AM
  ModeSelect.gain(1,1.0);                  // turn on SSB audio path
  // ModeSelect.gain(2,0.0);                  // turn off CW baseband, not used now, using IF CW.

  BandWidth.setLowpass(0,3000,0.67);       // actual butterworth Q's are used when changing bandwidth
  BandWidth.setLowpass(1,3000,1.10);
  BandWidth.setLowpass(2,3000,0.707);
  BandWidth.setLowpass(3,3000,1.00);       // 4 IIR lowpass cascade

  IF12r7.begin(AM12r7fir,30);              // AM filter at 12.7k audio IF frequency
                                           // !!! we could turn this off when not used
                                           
  tone600.frequency(600,6);                // try to get these to all run at 10ms rep rate
  tone700.frequency(700,7);
  tone800.frequency(800,8);
  
  AudioInterrupts();

  USBscope.averageTogether(50);            // or 40 for faster waterfall
  LSBscope.averageTogether(50);

  amp1.gain(10.0);                         // will this avoid zero's being returned by tone objects?
                                           // no, but seems to help
  RXsigs.begin( RXfun, 4081.632653 );      // Feld Hell half pixel rate

  message_query();                         // read out the stored messages for local use

  delay(5000);                             // view any bootup messages
  tft.fillScreen(ILI9341_BLACK);           // clean up the screen again

}


// timer interrupt at Feld Hell half pixel rate
void RXfun(){
static int row = 0;
static int col = 1;
static int state = 0;
static struct HELFONT hc;

  
    rms_buf[rms_in++] = rms1.read();
    rms_in &= 7;
    
    if( decode_menu_data.current != DHELL ) return;
    
    // Feld Hell transmitting,  idle columns or send what is in the buffer
    // function from my TenTec Rebel
     switch ( state ){
     case 0:         /* idle a complete column, also idling during receive */
       col <<= 1;
       if( col == ( 1 << 14 ) ){
          row = 0, col = 1;
          if( hflag ){            /* check for next char during the last bit time in the column */
             if( hchar < ' ' || hchar > 'Z' ) hc = heltab[0];     /* no good so "tx" a space */
             else hc = heltab[hchar - ' '];                       /* look up hchar in font table */
             state = 1;
             hflag = 0;
          }
       }
     break;
     
     case 1:         /* tx bit at row and column */
       if( hc.font[row] & col ) digitalWriteFast( QCX_KEY, HIGH );  /* tx on */
       else digitalWriteFast( QCX_KEY, LOW );                       /* tx off */
       col <<= 1;
       if( col == ( 1 << 14 )) col= 1,  ++row;
       if( row > 4 ) col = 1, state = 2;
     break;
   
     case 2:      /* idle a complete column */
       col <<= 1;
       if( col == ( 1 << 14 )) col= 1,  state = 0;
     break;
  } 

    
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
   else if( yt > 140 ){
      menu_display( &decode_menu_data );
      menu_dispatch = &decode_menu;
   }
                           
   else menu_cleanup();                  // not active part of the screen, return to normal op.
}

void mode_menu( int32_t t ){
int selection;
int current;

   current = mode_menu_data.current;
   selection = touch_decode( t, mode_menu_data.y_size );
   if( mode_menu_data.param[selection] != -1 ){
      if( selection != 7 ) mode_menu_data.current = selection;  // added phasing to this menu
      selection = mode_menu_data.param[selection];              // perhaps it should be elsewhere
                                                                // as is just special case processing here

      if( selection != 7 ) vfo_mode &= 0x0f;    // save the qcx mode flags, clear the sdr mode flags
      switch( selection ){
        case 0:     // QCX audio selected
                 vfo_mode |= VFO_CW;
                 digitalWriteFast(QCX_MUTE,LOW);          // select qcx audio      
                 SSBselect.gain(1,0.0);                   // mute LSB,USB,AM
                 SSBselect.gain(2,0.0);
                 ModeSelect.gain(0,0.0);
                 ModeSelect.gain(1,0.0);
                 //ModeSelect.gain(2,0.0);
                 mux_selected = 3;                        // no connection on 3
        break;
//        case 1:  vfo_mode |= VFO_CW;                       // baseband sdr CW
//                 digitalWriteFast(QCX_MUTE,HIGH);                 
//                 ModeSelect.gain(2,agc_gain);              // turn on CW
//                 ModeSelect.gain(0,0.0);                   // turn off AM
//                 ModeSelect.gain(1,0.0);                   // turn off SSB
//                 SSBselect.gain(1,0.0);
//                 SSBselect.gain(2,0.0);                   // USB audio
//                 mux_selected = 2;
//        break;
//   leaving modeselect 2 in place in case wish to go back to baseband CW someday
//   but for now getting CW SDR through the IF bandpass, bfo processing
        case 1:  vfo_mode |= VFO_CW;                       // IF SDR CW instead of baseband
                 digitalWriteFast(QCX_MUTE,HIGH);
                 ModeSelect.gain(1,agc_gain);              // SSB audio
                 ModeSelect.gain(0,0.0);                   // AM off
                 //ModeSelect.gain(2,0.0);                   // baseband cw off
                 SSBselect.gain(1,0.0);
                 SSBselect.gain(2,1.0);
                 mux_selected = 1;
        break;
        case 2:  vfo_mode |= VFO_LSB;  
                 digitalWriteFast(QCX_MUTE,HIGH);
                 ModeSelect.gain(1,agc_gain);              // SSB audio
                 ModeSelect.gain(0,0.0);
                 //ModeSelect.gain(2,0.0);                   // am cw off
                 SSBselect.gain(1,1.0);
                 SSBselect.gain(2,0.0);                    // usb to listen to LSB audio
                 mux_selected = 1;
        break;
        case 3:  vfo_mode |= VFO_USB;
                 digitalWriteFast(QCX_MUTE,HIGH);
                 ModeSelect.gain(1,agc_gain);
                 ModeSelect.gain(0,0.0);                   // AM off
                 //ModeSelect.gain(2,0.0);
                 SSBselect.gain(1,0.0);
                 SSBselect.gain(2,1.0);
                 mux_selected = 1;
        break;
        case 4:  vfo_mode |= VFO_AM;
                 digitalWriteFast(QCX_MUTE,HIGH);
                 ModeSelect.gain(1,0.0);                  // ssb off
                 ModeSelect.gain(0,agc_gain);             // AM on
                 //ModeSelect.gain(2,0.0);                  // cw off ( baseband cw )
                 mux_selected = 0;
        break;
        case 7:  PhaseChange(1);  break;                  // phasing correction    
      }
     
   }
   qsy_mode( current, mode_menu_data.current );   
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
      BandWidth.setLowpass(0,sel,0.51);       // 0.51  butterworth constants
      BandWidth.setLowpass(1,sel,0.60);       // 0.60
      BandWidth.setLowpass(2,sel,0.90);       // 0.90
      BandWidth.setLowpass(3,sel,2.56);       // 2.56
   }
   
   menu_cleanup();
}

void decode_menu( int32_t t ){
int sel;

    sel = touch_decode( t, decode_menu_data.y_size );
    if( decode_menu_data.param[sel] != -1 ){
        decode_menu_data.current = decode_menu_data.param[sel];
    }
  
  menu_cleanup();
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
char buf[20];

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
      strncpy(buf,m->menu_item[i],12);    buf[12] = 0;
      tft.print(buf);
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
      else if( qu_flags & QUTB && screen_owner == DECODE && (mode_menu_data.current == 0 ) ) cat.print("TB;");
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
              case 9: if( screen_owner == DECODE && (mode_menu_data.current == 0 ) ) cat.print("TB;");
                      break;
              case 11: cat.print("KS;");  break;              
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

   USBwaterfall();         // new algorithm for less time, but need 65 calls to complete writes
   LSBwaterfall();
   
   auto_atn();             // front end gain reduction if rcv very loud signals

   if( rms_out != rms_in ){                  // agc, smeter, and Feld Hell RX data stream
      rms_value = rms_buf[rms_out++];
      rms_out &= 7;
      agc();
      smeter();
      rx_hell();
   }

   cw_tune();
   if( decode_menu_data.current == DCW ) code_read(); 

   // Feld Hell transmit, feed the next char to the Hell transmit process
   if( decode_menu_data.current == DHELL && hflag == 0 && t_out != t_in && hcount){
      hchar = tbuf[t_out++];
      if( hchar < 'A' ) --hcount;
      if( hcount < 0 ) hcount = 0;
      hflag = 1;
      t_out &= ( TBUFSIZE-1);
   }
   
   // balance_schmoo();    // !!! testing
}


// ***************   group of functions for a morse decoder

// indicator to show when tuned spot on 700 hz.  1st steps for another cw decoder.
// this function doesn't work for QCX CW mode as the audio path to the tone detectors is muted.
int cw_tune(){
static float t6, t7, t8;        // tone results
float av, f, t;                 // average of min buckets, meter force, max tone
static int loc;                 // location of the tune meter needle
int det;                        // cw mark space detect
int new_loc, c;                 // raw new meter needle location, color of needle changes mark/space
static int count;               // mark,space counts
static float rav;               // running average of signals

   //if( mode_menu_data.current != 1 ) return 1;
   
   if( tone700.available() == 0 ) return 0;      // should run at 10ms rate
   t7 = tone700.read();
   if( t7 == 0.0 ) t7 = rav;
                                                 // these objects sometimes read as zero
   if( tone800.available() ) t8 = tone800.read();
   if( t8 == 0.0 ) t8 = rav;
   if( tone600.available() ) t6 = tone600.read();
   if( t6 == 0.0 ) t6 = rav;

// perhaps we should see what the signals are before getting too carried away.  Seems noisy.
 //   static int mod;
  //  ++mod;
  //  if( mod > 20 ){
  //    mod = 0;
  //    Serial.print(t6,4);  Serial.write(' ');
  //    Serial.print(t7,4);  Serial.write(' ');
  //    Serial.println(t8,4);
  //  }

   t = f = av = 0.0;
   
   if( t7 > t6 && t7 > t8 ){                   // desired case 700hz
       av = ( t6 + t8 )/2;
       t = t7;
       f = ( t8 - t6 )/av;
   }
   if( t6 > t7 && t6 > t8 ){                   // tuned low
       av = ( t7 + t8 )/2;
       t = t6;
       f = ( av - t6 )/av;
   }
   if( t8 > t7 && t8 > t6 ){                   // tuned high
       av = ( t6 + t7 )/2;
       t = t8;
       f = ( t8 - av )/av;
   }

   av = (t6+t7+t8-t/2.0)/3.0;
  // rav = 15.0*rav + av;
  // rav /= 16.0;
   rav = 31.0*rav + av;     // try a longer time constant here.  Maybe shorter for faster code and
   rav /= 32.0;             // longer for slower code would work best to avoid noise in between characters.
                            // trade off for fast fading signals being lost with longer constant.
   
   det = ( t > 2.0*rav ) ? 1 : 0;       //2.0 - 3.0  noise to ok

   det = cw_denoise( det );

   // debug arduino graph values to see signals
   //Serial.print( 10*t ); Serial.write(' '); Serial.print(10*rav); Serial.write(' ');
   //Serial.write(' '); Serial.println(det);


   if( det ){                // marking
      if( count > 0 ){
         if( count < 99 ) storecount(count);
         count = 0;
      }
      --count;
   }
   else{                     // spacing
      if( count < 0 ){
        storecount(count);
        count = 0; 
      }
      ++count;
      if( count == 99 ) storecount(count);  // one second no signal
   }
   
   new_loc = 15 * f;
   new_loc = constrain(new_loc, -15, 15 );

   if( det > 0 ){
      if( loc < new_loc ) ++loc;
      if( loc > new_loc ) --loc;
      c = EGA[10];
   }
   else c = EGA[12];

// !!! inhibit these writes if position and color are same as last time
// void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
   if( screen_owner == DECODE ){
      tft.drawFastVLine(290+loc-2,40,12,0);    // erase like a sprite
      tft.drawFastVLine(290+loc-1,40,12,c);
      tft.drawFastVLine(290+loc,40,12,c);
      tft.drawFastVLine(290+loc+1,40,12,c);
      tft.drawFastVLine(290+loc+2,40,12,0);
   }

   return 1;              // ok to run time consuming waterfall now
}

// slide some bits around to remove 1 reversal of mark space
int cw_denoise( int m ){
static int val;

   if( m ){
      val <<= 1;
      val |= 1;
   }
   else val >>= 1;

   val &= 7;
   if( m ) return val & 4;      // need 3 marks in a row to return true
   else return val & 2;         // 1 extra mark returned when spacing
                                // so min mark count we see is -2 if this works correctly
                                // this shortens mark count by 1 which may be ok as
                                // the tone detect seems to stretch the tone present time
}

// store mark space counts for cw decode
void storecount( int count ){

     cread_buf[cread_indx++] = count;
     cread_indx &= 15;

     if( count < 0 ){      // save dah counts
        count = -count;
        if( count >= 12 ){
          dah_table[dah_in++] = count;
          dah_in &= 7;
        }
     }
}

void shuffle_down( int count ){    /* consume the stored code read counts */
int i;

  for( i= count; i < cread_indx; ++i ){
    cread_buf[i-count] = cread_buf[i];
  }
  
  cread_indx -= count;
  cread_indx &= 15;     // just in case out of sync  
}


int code_read_scan(int slice){  /* find a letter space */
int ls, i;

/* scan for a letter space */
   ls = -1;
   for( i= 0; i < cread_indx; ++i ){
      if( cread_buf[i] > slice ){
        ls = i;
        break;
      }
   }
   return ls;   
}


unsigned char morse_lookup( int ls, int slicer){
unsigned char m_ch, ch;
int i,elcount;

   /* form morse in morse table format */
   m_ch= 0;  elcount= 1;  ch= 0;
   for( i = 0; i <= ls; ++i ){
     if( cread_buf[i] > 0 ) continue;   /* skip the spaces */
     if( cread_buf[i] < -slicer ) m_ch |= 1;
     m_ch <<= 1;
     ++elcount;
   }
   m_ch |= 1;
   /* left align */
   while( elcount++ < 8 ) m_ch <<= 1;

   /* look up in table */
   for( i = 0; i < 47; ++i ){
      if( m_ch == morse[i] ){
        ch = i+',';
        break;
      }
   }

   return ch;  
}

// routines from my TenTec Rebel code
void code_read(){  /* convert the stored mark space counts to a letter on the screen */
int slicer;
int i;
unsigned char m_ch;
int ls,force;
static int wt;    /* heavy weighting will mess up the algorithm, so this compensation factor */
static int singles;
static int farns,ch_count;
static int eees;

   if( cread_indx < 2 ) return;    // need at least one mark and one space in order to decode something
   if( screen_owner != DECODE ) return;
   if( decode_menu_data.current != DCW ) return;
   
   /* find slicer from dah table */
   slicer= 0;   force= 0;
   for( i = 0; i < 8; ++i ){
     slicer += dah_table[i];
   }
   slicer >>= 4;   /* divide by 8 and take half the value */

   ls = code_read_scan(slicer + wt);
   
   if( ls == -1 && cread_indx == 15 ){   // need to force a decode
      for(i= 1; i < 30; ++i ){
        ls= code_read_scan(slicer + wt - i);
        if( ls >= 0 ) break;
      } 
      --wt;    /* compensate for short letter spaces */
      force= 1;
   }
   
   if( ls == -1 ) return;
   
   m_ch = morse_lookup( ls, slicer );
   
   /* are we getting just E and T */
   if( m_ch == 'E' || m_ch == 'T' ){   /* less weight compensation needed */
      if( ++singles == 4 ){
         ++wt;
         singles = 0;
      }
   }
   else if( m_ch ) singles = 0;   
 
   /* are we getting just e,i,s,h,5 ?   High speed limit reached.  Attempt to receive above 30 wpm */
  // if( m_ch > 0 && ( m_ch == 'S' || m_ch == 'H' || m_ch == 'I' || m_ch == '5' )) ++shi5;
  // else if( m_ch > 0 && m_ch != 'E' ) --shi5;   // E could be just noise so ignore
  // if( shi5 < 0 ) shi5 = 0;
 
   /* if no char match, see if can get a different decode */
   if( m_ch == 0 && force == 0 ){
     //if( ( slicer + wt ) < 10 ) ls = code_read_scan( slicer + wt -1 );
     //else ls = code_read_scan( slicer + wt -2 );
     ls = code_read_scan( slicer + wt - ( slicer >> 2 ) );
     m_ch = morse_lookup( ls, slicer );
     if( m_ch > 64 ) m_ch += 32;       // lower case for this algorithm
     //if( m_ch ) --wt;     this doesn't seem to be a good idea
   }
 
   if( m_ch ){   /* found something so print it */
      ++ch_count;
      if( m_ch == 'E' || m_ch == 'I' ) ++eees;         // just noise ?
      else eees = 0;
      if( eees < 5 ) decode_print(m_ch);
      if( cread_buf[ls] > 3*slicer + farns ){   // check for word space
        if( ch_count == 1 ) ++farns;    // single characters, no words printed
        ch_count= 0;
        decode_print(' ');
      }
   }
     
   if( ls < 0 ) ls = 0;   // check if something wrong just in case  
   shuffle_down( ls+1 );  

   /* bounds for weight */
   if( wt > slicer ) wt = slicer;
   if( wt < -(slicer >> 1)) wt= -(slicer >> 1);
   
   if( ch_count > 10 ) --farns;
   
}

//  ***************   end of morse decode functions


// change the qcx frequency when changing modes to remain on the same frequency
// needed due to receiving on an IF freq instead of baseband
// sometimes qcx doesn't change freq, maybe will be better when I flash the new version
void qsy_mode(int old_mode, int new_mode ){
int32_t freq;
char buf[33];
char buf2[33];

   if( old_mode == new_mode ) return;
   
   freq = vfo_a;
   strcpy(buf,"FA000");
   if( vfo_mode & VFO_B ) freq = vfo_b, buf[1] = 'B';

   switch( old_mode ){                                  // remove current offsets
      case 0:  /*freq += 700;*/  break;                     // qsx mode
//      case 1:  freq += 700;  break;                     // cw sdr baseband
      case 1:  freq -= bfo;  break;                     // CW IF USB
      case 2:  freq += bfo;  break;                     // lsb IF
      case 3:  freq -= bfo;  break;                     // usb IF
      case 4:  freq += 12700;  break;                   // am filter center
   }
   switch( new_mode ){
      case 0:  /*freq -= 700;*/  break;
//      case 1:  freq -= 700;  break;
      case 1:  freq += bfo;  break;                     // IF CW mode
      case 2:  freq -= bfo;  break;                     // lsb
      case 3:  freq += bfo;  break;                     // usb
      case 4:  freq -= 12700;   break;
   }
 
   if( freq < 10000000 ) strcat(buf,"0");               //add a zero
   itoa( freq, buf2, 10 );
   strcat( buf, buf2 );
   strcat( buf,";");
   delay(100);                                          // wait qcx processing any current command
   cat.print(buf);
   cmd_tm = millis();                                   // delay future command
}

// avoid overload.  This looks at 40khz of signal on USB and LSB so not a replacement for AGC
// this basically keeps the audio processing out of saturation ( +-32767 ).
void auto_atn(){           // lower front end gain if signals approach overload
static uint32_t tm;        
static int no_chg_cnt;     // slowly raise gain

   if( millis() - tm < 10 ) return;
   if( peak1.available() == 0 ) return;
   //if( peak2.available() == 0 ) return;
   ++no_chg_cnt;
   tm = millis();
   
   if( peak_atn > 0 && (peak1.read() > 0.9 /*|| peak2.read() > 0.9*/) ){    // lower gain on strong signal
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
// static uint32_t tm;
static float sig;
static int hang;
float reading;
int ch;
float digital_gain = 1.0 , digital_target;

  // if( millis() - tm < 1 ) return;      // working at 1000hz
  // if( rms1.available() == 0 ) return;
  // tm = millis();

   reading = rms_value;
   ch = 0;
     /// rms_value = reading = rms1.read();
   if( reading > sig && reading > AGC_FLOOR ){         // attack
       sig = sig + 0.001;
       ch = 1;
       hang = 0;
       if( digital_gain > 1.0 ) digital_gain -= 0.01;
   }
   else if( sig > AGC_FLOOR && hang++ > AGC_HANG ){    // decay
       sig = sig - 0.00005;
       ch = 1;
   }
   digital_target  = map( peak_atn, 0, 15, 312, 24 );     // voltage ratio due to front end attenuation
   digital_target = digital_target / 24.0;                // what the signal would be without the attenuator
   if( sig <= AGC_FLOOR && digital_gain != digital_target ){
       if( digital_target > digital_gain + 0.02 ) digital_gain += 0.001, ch = 1;
       if( digital_target < digital_gain - 0.02 ) digital_gain -= 0.001, ch = 1;
   }


   
   if( ch ){
    // any loud signal on bandscope +-20khz can lower the signal levels of the desired signal
    // compensate with digital gain in the later stages.
     agc_gain = sig - AGC_FLOOR;
     agc_gain = agc_gain * AGC_SLOPE;
     agc_gain = digital_gain - agc_gain;                // digital gain is 1.0 when no attenuation 
     ModeSelect.gain(mux_selected,agc_gain);            // which is when analog gain is 15
     //if( vfo_mode & VFO_AM ) ModeSelect.gain(0,agc_gain);
     //else ModeSelect.gain(1,agc_gain);
       // Serial.println(agc_gain);

     if( screen_owner == DECODE && decode_menu_data.current != DHELL ){
        tft.setTextSize(1); 
        tft.setTextColor(EGA[14],0);
        tft.setCursor(262,114);  
        tft.print("Sig: ");
        tft.print(sig); 
     }
   }
}


#ifdef USE_4_LINE
// paint the screen with feld hell receive 
void rx_hell(){
static uint8_t  data[28][4];      // try 8 columns each write, packed into 4 wide data
uint8_t  *p;
static int16_t prow,pcolumn;          // 4 rows 80 writes of 4 colums for 320 pixels, lose one pixel row at bottom.
//static int pixel;
static int row = 13, col = 0;     // first pixel position
static float base;                // average signals 
float sig;
int  sigi;

  if( screen_owner != DECODE || decode_menu_data.current != DHELL ) return;

  base = 255.0*base + rms_value;               // 128 maybe ok
  base /= 256.0;

  sig = ( rms_value/base ) * 16 - 16;          // contrast and base color as white
  sig = 16 - sig;
  sigi = sig;
//Serial.print( sigi );  Serial.write(' ');  
  sigi = constrain( sigi, 0, 15 );
  if( (col & 1) == 0 ){                      // pack 4 bits per pixel and duplicate for print
    data[row][col/2] = sigi << 4;
    data[row+14][col/2] = sigi << 4;
  }
  else{
    data[row][col/2] |= sigi;
    data[row+14][col/2] |= sigi;
  }

  --row;
 // ++pixel;
  if( row < 0 ) row = 13, ++col;  // pixel = 0;
  if( col >= 8 ){
     p = &data[0][0];
     //writeRect4BPP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pixels, const uint16_t * palette );
     tft.writeRect4BPP( pcolumn, 128+28*prow, 8, 28, p, GRAY );
     col = 0;
     pcolumn += 8;
     if( pcolumn >= 319 ) pcolumn = 0, prow += 1;
     if( prow >= 4 ) prow = 0;
  }
   
}
#endif

#ifdef USE_3_LINE

//  .25 lowpass, no window
const float fh_k[14] = {
-0.023146590714119390,
-0.053367438848668208,
-0.049667786106291537,
 0.004798037517529520,
 0.101060536773946719,
 0.204651121687276810,
 0.271600842184059443,
 0.271600842184059443,
 0.204651121687276810,
 0.101060536773946719,
 0.004798037517529520,
-0.049667786106291537,
-0.053367438848668208,
-0.023146590714119390
};


// 42 pixel high double line print, interpolate by 3, decimate by 2, 1.5X speed increase
// need to borrow 14 rows of pixels from the waterfall for this version
void rx_hell(){
static float dline[14];
static int deci;
int i,j;
float sum;
  
    if( screen_owner != DECODE || decode_menu_data.current != DHELL ) return;
    
    for( i = 0; i < 3; ++i ){                                // sample rate * 3
       for( j = 0; j < 13; ++j ) dline[j] = dline[j+1];      // move delay line
       dline[13] = ( i == 0 ) ? rms_value : 0.0 ;            // sample at end
       sum = 0;
       for( j = 0; j < 14; ++j ) sum += dline[j] * fh_k[j];  // mult and add FIR filter
       deci ^= 1;
       if( deci ) rx_hell3( sum );                           // decimate by 2      
    }
}

void rx_hell3( float val ){
static uint8_t  data[42][4];      // try 8 columns each write, packed into 4 wide data
uint8_t  *p;
static int16_t prow,pcolumn;      // 4 rows 80 writes of 4 colums for 320 pixels
static int row = 20, col = 0;     // first pixel position
static float base;                // average signals 
float sig;
int  sigi;

  base = 255.0*base + val;               // 128 maybe ok
  base /= 256.0;

  sig = ( val/base ) * 16 - 16;          // contrast and base color as white
  sig = 16 - sig;
  sigi = sig;
//Serial.print( sigi );  Serial.write(' ');  
  sigi = constrain( sigi, 0, 15 );
  if( (col & 1) == 0 ){                      // pack 4 bits per pixel and duplicate for print
    data[row][col/2] = sigi << 4;
    data[row+21][col/2] = sigi << 4;
  }
  else{
    data[row][col/2] |= sigi;
    data[row+21][col/2] |= sigi;
  }

  --row;
  if( row < 0 ) row = 20, ++col;  // pixel = 0;
  if( col >= 8 ){
     p = &data[0][0];
     //writeRect4BPP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pixels, const uint16_t * palette );
     tft.writeRect4BPP( pcolumn, 114+42*prow, 8, 42, p, GRAY );
     col = 0;
     pcolumn += 8;
     if( pcolumn >= 319 ) pcolumn = 0, prow += 1;
     if( prow >= 3 ) prow = 0;
  }  
}
#endif


#define ybase 250              // vertical position of all
void smeter(){
static uint8_t  onscreen;
float angle;
int x,y;
static int lastx = 160,lasty = 239;
int diff;
float tvalue;
float ftemp;

   if( screen_owner != DECODE || decode_menu_data.current != DSMETER ){       // not show the meter ?
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
      }
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
      lastx = 160,lasty = 239;
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
      if( c < 'A' && decode_menu_data.current == DHELL ) ++hcount;
      t_in &= (TBUFSIZE-1);
      if( decode_menu_data.current != DHELL ) cat.print("KY;");      // cw mode, check if ok to send more data
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
    
   tft.drawLine(0,61,319,61,EGA[4]);      
   tft.drawLine(0,62,319,62,EGA[4]);
   if( decode_menu_data.current != DHELL ){
      tft.drawLine(0,128,319,128,EGA[4]);
      tft.drawLine(0,129,319,129,EGA[4]);
   }
  

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

   // qcx reports vfo 700 hz higher than actual as it is a CW receiver
   // remove that offset for AM, USB, LSB modes.  Add subtract BFO freq.
   if( (vfo_mode & VFO_AM ) == VFO_AM ) vfo += 12000;   // AM IF is 12.7 
   else if( ( vfo_mode & VFO_CW ) == 0 ) vfo -= 700;    // not CW, remove offset.
   if( (vfo_mode & VFO_USB ) ) vfo -= BFO_FREQ;
   if( (vfo_mode & VFO_LSB ) ) vfo += BFO_FREQ;
   if( (vfo_mode & VFO_CW ) &&  mode_menu_data.current == 1) vfo -= BFO_FREQ;   // SDR CW mode
   
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
    if( response[0] == 'K' && response[1] == 'S' ) cat_keyspeed();
    if( response[0] == 'Q' && response[1] == 'M' ) cat_message_query();

    len = 0;     // reset string to start for next command
}

// setup function, query the QCX for the stored messages
void message_query(){
int i, trys,j;
char cmd[20];

   delay(1000);                      // allow qcx to boot up
   tft.setCursor(0,0);
   tft.setTextSize(1);
   tft.setTextColor(ILI9341_WHITE);
   strcpy( cmd, "QM2.01;" );
   for( i = 0;  i < 8; ++i ){      // attempt 3 times to read out 1st 8 stored messages from QCX
       if( i < 7 ) cmd[5] = i + '3';
       else cmd[4] = '1', cmd[5] = (i+3)%10 + '0';
       for( trys = 0; trys < 3; ++trys ){
          cat.print(cmd);
            //Serial.print(cmd);    Serial.write(' ');  
          for( j = 0; j < 1000; ++j ){
              delay(1);
              radio_control();
              if( qcx_message.param[i] != -2 ) break;  
          }
          if( qcx_message.param[i] != -2 ) break;
       }
   }
}

void cat_message_query(){      // save the QCX message to local storage array
int bufin;
int i,j;
char c;

   tft.println(response);
   // Serial.println(response);    //  QM26 4NAME IS RON; is a response.  Format not same as documentation
   j = 0;   i = 0;
   // find first space char
   while( response[j] != ' ' ){
      if( response[j] == 0 ) return;
      ++j;
   }
   ++j;
   while( response[j] >= '0' && response[j] <= '9'){    // get index
      i = i * 10;
      if( response[j] == 0 ) return;
      i = i + response[j] - '0';
      ++j;
   }
   --i;                                   // numbers start at 1, index starts at zero
   if( i >= 8 ) return;                   // out of bounds of local array
  // copy the message
  //  ++j;                   // skip period  --  not in the response, maybe there in future revisions
  bufin = 100 * i;           // 1st 4 are 100 characters long, rest are 50
  if( i > 4 ) bufin = bufin - 50 * ( i-4);
  msg_buf[bufin++] = ' ';    // preceed with a space for menu printing or appending messages
  qcx_message.param[i] = ( response[j] == 0 ) ? -1 : 1;    // flag we found it and if it is valid
  for(;;++j){
      if( bufin == MSG_BUF_SIZE ){      // out of space, quit and leave string terminated
          msg_buf[MSG_BUF_SIZE-1] = 0;
          return;
      }
      c = response[j];
      msg_buf[bufin++] = c;
      if( c == 0 ) break;
  }
  
}


void cat_transmit(){    // sending via the touch keyboard

   if( decode_menu_data.current == DHELL ) return;     // not in CW mode
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

// replicate above for the SDR cw decoder
void decode_print( char c ){

  tft.setTextColor(EGA[14],0);
  tft.setTextSize(2);

     if( dpos >= 25 ) scroll_dtext();                      // scroll needed
     // save each char in buffer and print to the screen
     dtext[dline][dpos] = c;
     tft.setCursor( D_LEFT + 12*dpos, D_BOTTOM - D_SIZE );
     tft.write(c);
     ++dpos;
  
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

//  force straight key mode for Feld Hell mode by setting keyer speed to zero
//  to use a physical straight key, must set the straight key mode in the QCX menu system
void cat_keyspeed(){
static int kspeed;
int ks;
char buf[30];

   ks = atoi(&response[2]);
   if( decode_menu_data.current == DHELL ){
     if( ks != 0 ) cat.print("KS0;");         // set to straight key mode
   }
   else{                                      // normal mode, save result or return to normal
     if( ks != 0 ) kspeed = ks;
     else{                                    // no longer in Hellschreiber mode, set previous speed
         strcpy( buf,"KS" );
         itoa( kspeed, &buf[2], 10 );
         if( buf[3] == 0 ) buf[3] = ';', buf[4] = 0;
         else buf[4] = ';', buf[5] = 0;
         cat.print(buf);
     }
   }
}

void cat_freq( int32_t *vfo ){             // update a vfo from cat response
int32_t temp, s;
char buf[33];
char buf2[33];

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

    // tracking split for CW mode using IF
    // if split and cw sdr mode, keep vfo B on freq
    if( mode_menu_data.current == 1 && (vfo_mode & VFO_SPLIT) ){
        if( vfo_a - vfo_b != BFO_FREQ && rit == 0 ){           // normal qcx operation
           temp = vfo_a - BFO_FREQ;
           strcpy(buf,"FB000"); 
           if( temp < 10000000 ) strcat(buf,"0");         //add a zero
           itoa( temp, buf2, 10 );
           strcat( buf, buf2 );
           strcat( buf,";");
           cat.print(buf);
        }
        // qcx does not implement RIT when in split mode but allows it to half work
        // so change vfo_a to simulate RIT keeping vfo_b fixed now, final effect
        // works like a normal split.  ( usb rit works backwards )
        // this has a very slow response due to 1 second between cat commands.
        // turning rit off will leave the radio on the wrong freq
        // Let's try messing with the bfo instead of moving vfo_a
        //else if( rit != 0 && vfo_a - vfo_b != bfo + rit ){
        //   temp = vfo_b + bfo + rit;
        //   strcpy(buf,"FA000"); 
        //   if( temp < 10000000 ) strcat(buf,"0");
        //   itoa( temp, buf2, 10 );
        //   strcat( buf, buf2 );
        //   strcat( buf,";");
        //   cat.print(buf);           
        //}
    }
}


void cat_mode( ){     // from IF command
uint8_t temp;
int32_t temp2;
static int old_rit;

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

   // sdr cw mode, and not split check and send cat here
   if( mode_menu_data.current == 1 ){
       if( (vfo_mode & VFO_SPLIT) == 0 ) cat.print("FR2;");   // set split
       if( rit != old_rit ){
          old_rit = rit;
          BFO.frequency( BFO_FREQ + rit );          // move the bfo for RIT in split mode
       }
   }
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
uint8_t log2( float f, int side){    // log base 2, don't exceed 15.  f 0.0 to 1.0
uint32_t val;
uint8_t  r;
uint16_t  bw_low,bw_high;
static uint8_t dashline;

    // show bandwidth on waterfall                                     // USB audio is on LSB waterfall
    if( side <= 0 && (vfo_mode & VFO_USB )){      
       bw_high = band_width_menu_data.param[band_width_menu_data.current];
       bw_high /= 172;
       bw_low = bfo/172;
       bw_high = bw_low - bw_high;
       side = -side;
       if( mode_menu_data.current != 0 ){
          if( side == bw_high || side == bw_low ) return 10;
       }
    }
    else if(side >= 0 && (vfo_mode & ( VFO_LSB + VFO_AM ))){           // LSB or AM
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

    if( vfo_mode & VFO_CW ){
        if( side == 4 && mode_menu_data.current == 0 ){
            ++dashline;
            if( (dashline & 0xc) == 0xc ) return 15;     // CW tuning position    10    
        }
        if( side == (-bfo+700)/172 && mode_menu_data.current == 1 ){   // CW SDR on IF freq
            ++dashline;
            if( (dashline & 0xc) == 0xc ) return 15;     // CW tuning position                  
        }
    }
    
    // show the ends of the waterfall
    //if( side == 127 || side == -127 ) return 8;
    
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

// see if we can improve speed of the waterfall processing from about 5ms to ____ new value
// new algorithm scrolls the data by writing to a different line each time and uses less time
// per call by breaking up the screen writes into 64 separate writes. 
void USBwaterfall(){
static uint8_t data[64][64];
static int line;                  // working line
static int pline;                 // printing line
static int ppos;                  // where to print
static int state;                 // calculating state or printing state
int i,j;
uint8_t *p;
uint8_t d;

    if( screen_owner != DECODE ) return;
    if( state == 0 ){
       if( USBscope.available() == 0 ) return;
    
       // get fft result and convert to 4 bits per pixel
       j = 0;
       for( i = 0; i < 64; ++i ){
          d = log2( USBscope.read(j),j );
          ++j;
          data[line][i] = d << 4;
          d = log2( USBscope.read(j),j );
          ++j;
          data[line][i] |= d;  
       }
       pline = line;                  // 1st line to print
       if( --line < 0 ) line = 63;    // scroll data by writing to a new line next time
       ppos = 0;                      // position to print
       state = 1;                     // change to print mode
    }
    else{
    // display on screen one line per function call
       p = &data[pline][0];
       tft.writeRect4BPP( 131,64+ppos,128,1,p,WF );
       ++ppos;                              // next position
       if( ++pline > 63 ) pline = 0;        // next data line to print
       if( pline == line ) state = 0;       // done, back to looking for new data
       #ifdef USE_3_LINE
       if( ppos == 64-14 && decode_menu_data.current == DHELL ) state = 0;
       #endif
    }

     //writeRect4BPP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pixels, const uint16_t * palette );
}

/*
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
       d = log2( USBscope.read(j),j );
       ++j;
       data[0][i] = d << 4;
       d = log2( USBscope.read(j),j );
       ++j;
       data[0][i] |= d;  
    }

    // display on screen
    p = &data[0][0];
    tft.writeRect4BPP( 131,64,128,64,p,WF );
  
}
*/


void LSBwaterfall(){
static uint8_t data[64][64];
static int line;                  // working line
static int pline;                 // printing line
static int ppos;                  // where to print
static int state;                 // calculating state or printing state
int i,j;
uint8_t *p;
uint8_t d;

    if( screen_owner != DECODE ) return;
    if( state == 0 ){
       if( LSBscope.available() == 0 ) return;
    
       // get fft result and convert to 4 bits
       j = 127;
       for( i = 0; i < 64; ++i ){
          d = log2( LSBscope.read(j),-j );      // neg j for lower sideband bandwidth edges
          --j;
          data[line][i] = d << 4;
          d = log2( LSBscope.read(j),-j );
          --j;
          data[line][i] |= d;  
       }
       pline = line;                  // 1st line to print
       if( --line < 0 ) line = 63;    // scroll data by writing to a new line next time
       ppos = 0;                      // position to print
       state = 1;                     // change to print mode
    }
    else{
    // display on screen one line per function call
       p = &data[pline][0];                 //    tft.writeRect4BPP( 130-126,64,128,64,p,WF );
       tft.writeRect4BPP( 130-126,64+ppos,128,1,p,WF );
       ++ppos;                              // next position
       if( ++pline > 63 ) pline = 0;        // next data line to print
       if( pline == line ) state = 0;       // done, back to looking for new data
       #ifdef USE_3_LINE
       if( ppos == 64-14 && decode_menu_data.current == DHELL ) state = 0;    // borrow 14 lines
       #endif
    }

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
