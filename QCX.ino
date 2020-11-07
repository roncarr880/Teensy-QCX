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
#define TFT_DC      20
#define TFT_CS      21
#define TFT_RST    255  // 255 = unused, connect to 3.3V
#define TFT_MOSI     7
#define TFT_SCLK    14
#define TFT_MISO    12
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);

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

void setup() {
int r,g,b;

  Serial.begin(38400);     // for debug or PC control
  cat.begin(38400);        // qcx CAT control

  // build the gray scale pallett
  for( int i = 0; i < 16; ++i ){
    r = 2*i + 1;   g = 4*i + 3;  b = 2*i + 1;
    GRAY[i] = r << 11 | g << 5 | b;
  }
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(1);
  
}


void loop() {
static int c_state;


   // poll radio once a second, process qu_flags as a priority
   if( ( millis() - cmd_tm > 1000 ) ){
      if( qu_flags & QUIF ) cat.print("IF;");
      else if( qu_flags & QUFB ) cat.print("FB;");
      else if( qu_flags & QUFA ) cat.print("FA;");
      else if( qu_flags & QUTB ) cat.print("TB;");
      else{
          switch(c_state){                         // poll radio round robin for missed packet recovery
              case 0: case 2: case 4: case 6: case 8:
              case 10: case 12: case 14:  cat.print("IF;"); break;
              case 1: cat.print("FA;");   break;
              case 3: cat.print("QU1;");  break;   // enable flags
              case 5: cat.print("FB;");   break;
              case 7: cat.print("TB1;");  break;   // enable decode flag
              case 9: cat.print("TB;");   break;   // !!! redundant ? does TB1 also return decoded text        
         }
         ++c_state;
         c_state &= 15;
      }
      cmd_tm = millis();
   }

   if( cat.available() ) radio_control();


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
    
       //  underline active tuning digit with best guess at step
   s = stp;    
   if( r ) pos = 278, s = rit_stp;    // rit active
   else if( b ) pos = 183;            // vfo b
   else pos = 53;                     // vfo a
   tft.drawLine(pos+s,40,pos+s+10,40,EGA[14]); 
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

  qu_flags &= ~QUTB;
}

void cat_qu_flags(){                       // process the qu flags response

  qu_flags |= atoi(&response[2]);             // merge new flags
  qu_flags &= (QUFA + QUFB + QUIF + QUTB);   // clear unused flags
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

#ifdef NOWAY
// REBEL emulation code as radio rather than as contolling computer
/************************************************************************/

      //  K3 emulation code
            
void get_freq(unsigned long vfo ){   /* report vfo */

    if( mode == CW && fun_selected[0] != WIDE ) vfo = vfo + (( sideband == USB ) ? mode_offset : - mode_offset);     
    stage_str("000");
    if( vfo < 10000000 ) stage('0');
    stage_num(vfo);  
}

void radio_control() {

static String command = "";
String lcommand;
char c;
int rit;
int sm;
int bat;  /* put battery voltage in front panel revision */

    if (Serial.available() == 0) return;
    
    while( Serial.available() ){
       c = Serial.read();
       command += c;
       if( c == ';' ) break;
    }
    
    if( c != ';' ) return;  /* command not complete yet */
  
    lcommand = command.substring(0,2);
 
    if( command.substring(2,3) == ";" || command.substring(2,4) == "$;" || command.substring(0,2) == "RV" ){      /* it is a get command */
      stage_str(lcommand);  /* echo the command */
      if( command.substring(2,3) == "$") stage('$');
      
      if (lcommand == "IF") {
/*
RSP format: IF[f]*****+yyyyrx*00tmvspbd1*; where the fields are defined as follows:
[f] Operating frequency, excluding any RIT/XIT offset (11 digits; see FA command format)
* represents a space (BLANK, or ASCII 0x20)
+ either "+" or "-" (sign of RIT/XIT offset)
yyyy RIT/XIT offset in Hz (range is -9999 to +9999 Hz when computer-controlled)
r 1 if RIT is on, 0 if off
x 1 if XIT is on, 0 if off
t 1 if the K3 is in transmit mode, 0 if receive
m operating mode (see MD command)
v receive-mode VFO selection, 0 for VFO A, 1 for VFO B
s 1 if scan is in progress, 0 otherwise
p 1 if the transceiver is in split mode, 0 otherwise
b Basic RSP format: always 0; K2 Extended RSP format (K22): 1 if present IF response
is due to a band change; 0 otherwise
d Basic RSP format: always 0; K3 Extended RSP format (K31): DATA sub-mode,
if applicable (0=DATA A, 1=AFSK A, 2= FSK D, 3=PSK D)
*/      
        get_freq(tx_vfo);
        stage_str("     ");
        rit= rit_offset;
        if( rit >= 0 ) stage_str("+0");
        else{
          stage_str("-0"); 
          rit = - rit;
        }
        if( rit < 100 ) stage('0');
        if( rit < 10 ) stage('0');    //IF[f]*****+yyyyrx*00tmvspbd1*;
        stage_num(rit);
        stage_str("10 0003");    /* rit,xit,xmit,cw mode */
        if( split == 2 ) stage_str("10");
        else stage_str("00");
        if( split ) stage('1');
        else stage('0');
        stage_str("001 ");      
      }
      else if(lcommand == "FA") get_freq( rx_vfo );
      else if(lcommand == "FB") get_freq( tx_vfo );
      else if(lcommand == "RT") stage('1');
      else if(lcommand == "FR") stage('0');
      else if(lcommand == "FT"){
         if( split ) stage('1');
         else stage('0');
      }
      else if( lcommand == "KS"){
        stage('0');
        stage_num(wpm);
      }
      else if(lcommand == "XF") stage_num(fun_selected[0]);
      else if(lcommand == "AG") stage_str("030");
      else if(lcommand == "RG") stage_str("250");
      else if(lcommand == "PC") stage_str("005");
      else if(lcommand == "FW") stage_str("0000") , stage_num(fun_selected[0]);
      else if(lcommand == "IS") stage_str("0000");
      else if(lcommand == "AN") stage('1');
      else if(lcommand == "GT") stage_str("004");
      else if(lcommand == "TQ") stage_num(transmitting);
      else if(lcommand == "PA" || lcommand == "XT" || lcommand == "NB" ) stage('0');
      else if(lcommand == "RA") stage_str("00");
      else if(lcommand == "OM") stage_str("-----F------");
      else if(lcommand == "LK") stage_num(user[LOCK]);
      else if(lcommand == "MD") stage('3');
      else if(lcommand == "RV" && command.substring(2,3) == "F"){  /* battery voltage in the revision field */
        stage(command.charAt(2));
        bat = battery(0);
        stage_num(bat/10);
        stage('.');
        stage_num(bat % 10);
        stage('0');
      }
      else if(lcommand == "RV" && command.substring(2,3) == "A"){  /* swap status in revision field */
        stage(command.charAt(2));
        if( split == 2 ) stage_str("SWAP ");
        else stage_str("    ");
      }
      else if(lcommand == "RV"){   // revisions
        stage(command.charAt(2));
        stage_str("     ");
      }
      else if(lcommand == "SM"){
        stage_str("00");
        sm = smeter(0);
        if( sm < 10 ) stage('0');
        stage_num(sm);
      }   
      else{
         stage('0');  /* don't know what it is */
      }
 
    stage(';');   /* response terminator */
    }
    
    else  set_k3(lcommand,command);    /* else it is a set command ? */
   
    command = "";   /* clear for next command */
}


void set_k3(String lcom, String com ){
String arg;
long val;
char buf[25];

 
    if( lcom == "FA" || lcom == "FB" ){    /* set vfo freq */
      arg = com.substring(2,13);
      arg.toCharArray(buf,25);
      val = atol(buf);
      if( mode == CW && fun_selected[0] != WIDE ) val = val - (( sideband == USB ) ? mode_offset : - mode_offset);     
      cat_band_change((unsigned long)val);
      if( lcom == "FB" || split == 0 ) tx_vfo = val;
      if( lcom == "FA" ) rx_vfo = val;
      update_frequency(DISPLAY_UPDATE);  /* listen on new freq */
    }
    else if( lcom == "KS" ){    /* keyer speed */
      arg= com.substring(2,5);
      arg.toCharArray(buf,25);
      val = atol(buf);
      wpm = val;
    }
    else if( lcom == "LK" ){     /* lock vfo's */
      val = com.charAt(2);
      if( val == '$' ) val = com.charAt(3);
      user[LOCK] = val - '0';
    }
    else if( lcom == "FW" ){     /* xtal filter select */
      val = com.charAt(6) - '0';
      if( val < 4 && val != 0 ){
        fun_selected[0] = val;
        set_band_width(val);
      }
    }
    else if( lcom == "FT" ){     /* enter split */
      val = com.charAt(2) - '0';
      if( val == 0 ){
        if( split == 2 ) rx_vfo = tx_vfo;
        else tx_vfo = rx_vfo;        
      }
      split = user[SPLIT] = val;
      user[SWAPVFO] = 0;        
    }
    else if( lcom == "FR" ){    /* cancel split ? */
      val = com.charAt(2);
      if( val == '0' ){
        if( split == 2 ) rx_vfo = tx_vfo;
        else tx_vfo = rx_vfo;
        split = user[SPLIT] = user[SWAPVFO] = 0;
      }
    }
    else if( com == "SWT11;" ){    /* A/B tap. swap (listen) vfo */
      if( split < 2 ){            /* turns on split if off */
        split = 2;
        user[SPLIT]= user[SWAPVFO] = 1;
      }
      else{                        /* back to listen on RX freq, stay in split */
       split = 1;
       user[SWAPVFO] = 0;
      }
      update_frequency(DISPLAY_UPDATE);  /* listen on selected vfo */
    } 
            
    write_sleds(sleds[fun_selected[function]]);
    write_fleds(fleds[function], 1);  /* update on/off status  on FGRN led */
    led_on_timer = 1000;            /* works different now with battery saver mode */
    
}
        /* end of K3 emulation functions */

#endif

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
