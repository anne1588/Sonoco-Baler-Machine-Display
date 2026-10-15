#include <lvgl.h>
#include <stdio.h>
#include "FT6336U.h"
#include "SPI.h"
#include <TFT_eSPI.h>
#include "RF24.h"
#include <Wire.h>


//----------------------------- CAPACITIVE SCREEN ----------------------------------//
static const uint16_t screenWidth  = 320;       //Ori 320
static const uint16_t screenHeight = 480;       //Ori 480

#define I2C_SDA 4               //  CTP SDA                   (GPIO04)
#define I2C_SCL 5               //  CTP SDA                   (GPIO05)
#define RST_N_PIN 7             //  CTP RST                   (GPIO03)
#define INT_N_PIN 15            //  CTP INT                   (GPIO02)

//static lv_color_t buf[320 * 40];
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ]; 
TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight);
FT6336U ft6336u(RST_N_PIN, INT_N_PIN);                // FOR CAP SCREEN

//------------------------------- LCD OUTPUT ---------------------------------------//
#define TFT_CS 14             //  Lcd Chip Select           (GPIO14)
#define TFT_DC 12             //  Lcd DC                    (GPIO12)
#define TFT_RST 13            //  Lcd Reset                 (GPIO13)
#define MOSI 11               //  Lcd MOSI                  (GPIO11)
#define MISO 8                //  Lcd MISO                  (GPIO08)
#define SCK 10                //  Lcd SCK                   (GPIO10)
int BL = 9;                   //  Lcd back light 

//-------------------------- TOUCHSCREEN COORDINATES -------------------------------//
int x, y;
int a, b;
int z;


#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.println(buf);
    Serial.flush();
}
#endif
// UI-only simulator: LVGL v8.3, portrait 320x480.
// Insert your working display/touch/tick initialization in setup().
// Does NOT control any real motor, hydraulic, or safety circuit.

enum Phase { READY, FEEDING, HOPPER_FULL, ADVANCING, RETRACTING,
             PRESSURE_REACHED, WAIT_DOOR_OPEN, DISCHARGE_FEED,
             DISCHARGE_PUSH };
enum UiSeverity{ UI_READY, UI_INFO, UI_WAIT, UI_ALERT };
static lv_color_t severityColor(UiSeverity s);
static lv_obj_t *processPanel;
static lv_obj_t *hopperFill;
static void transition(Phase p);
static void render();
static void machineUpdate();
static void drawMachine();
static Phase phase=READY;
static lv_obj_t *scr,*processLabel,*instructionLabel,*statusLabel,*draw,*waste,*blue,*door;
static lv_timer_t *simTimer;
static uint32_t phaseAt;
static int hopper=0,stroke=0,wastePct=0,pressure=0,cycleCount=0;
static const int strokeLimit=75;        // operator setpoint as % of total plunger travel
static const int pressureLimit=80;      // operator setpoint in simulated pressure units
static bool dischargeMode=false;
static uint32_t last_tick = 0;

static lv_color_t severityColor(UiSeverity s){
  switch(s){
    case UI_READY: return lv_color_hex(0x22C55E);  // green
    case UI_INFO:  return lv_color_hex(0x5BC0EB);  // blue
    case UI_WAIT:  return lv_color_hex(0xF2B94B);  // amber
    case UI_ALERT: return lv_color_hex(0xEF4444);  // red
    default:       return lv_color_hex(0xFFFFFF);
  }
}

static lv_obj_t *rectangle(lv_obj_t *p,int x,int y,int w,int h,uint32_t hex,bool border=false){
  lv_obj_t *o=lv_obj_create(p);
  lv_obj_set_pos(o,x,y);
  lv_obj_set_size(o,w,h);
  lv_obj_set_style_pad_all(o,0,0);
  lv_obj_set_style_radius(o,0,0);
  lv_obj_set_style_bg_color(o,lv_color_hex(hex),0);
  lv_obj_set_style_bg_opa(o,border?LV_OPA_TRANSP:LV_OPA_COVER,0);
  lv_obj_set_style_border_color(o,lv_color_hex(hex),0);
  lv_obj_set_style_border_width(o,border?1:0,0);
  lv_obj_clear_flag(o,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
  return o;
}
static lv_obj_t *caption(lv_obj_t *p,int x,int y,int w,const char *text){
  lv_obj_t *l=lv_label_create(p);
  lv_obj_set_pos(l,x,y);
  lv_obj_set_width(l,w);
  lv_label_set_long_mode(l,LV_LABEL_LONG_WRAP);
  lv_label_set_text(l,text);
  lv_obj_set_style_text_color(l,lv_color_white(),0);
  lv_obj_set_style_text_align(l,LV_TEXT_ALIGN_CENTER,0);
  return l;
}
static void drawMachine(){
  /* Main machine card */
  lv_obj_t *p = rectangle(scr, 8, 8, 304, 152, 0x1E2B34);
  lv_obj_set_style_radius(p, 8, 0);
  lv_obj_set_style_border_width(p, 1, 0);
  lv_obj_set_style_border_color(p, lv_color_hex(0x4C6573), 0);

  /* Hopper outline */
  static const lv_point_t hopperPts[] = {
    {112, 18}, {194, 18}, {207, 42},
    {183, 70}, {123, 70}, {99, 42},
    {112, 18}
  };
  lv_obj_t *hopperLine = lv_line_create(p);
  lv_line_set_points(hopperLine, hopperPts, 7);
  lv_obj_set_style_line_color(hopperLine, lv_color_hex(0xD5E7EE), 0);
  lv_obj_set_style_line_width(hopperLine, 2, 0);

  /* Hopper level bar */
  rectangle(p, 121, 67, 64, 4, 0x334550);   // track
  hopperFill = rectangle(p, 121, 67, 0, 4, 0x24D83E);

  /* Chamber */
  rectangle(p, 42, 84, 220, 50, 0xD5E7EE, true);

  /* Door (left side, opens upward) */
  door = rectangle(p, 44, 85, 10, 48, 0x24D83E);
  lv_obj_set_style_radius(door, 2, 0);

  /* Plunger indicator area background */
  rectangle(p, 56, 86, 204, 46, 0x263743);

  /* Plunger indicator (right -> left) */
  blue = rectangle(p, 260, 86, 0, 46, 0x7EC8EE);
  lv_obj_set_style_radius(blue, 2, 0);

  /* Optional small labels */
  lv_obj_t *lbl;

  lbl = lv_label_create(p);
  lv_label_set_text(lbl, "HOPPER");
  lv_obj_set_pos(lbl, 122, 0);
  lv_obj_set_style_text_color(lbl, lv_color_hex(0xBFD4DD), 0);

  lbl = lv_label_create(p);
  lv_label_set_text(lbl, "DOOR");
  lv_obj_set_pos(lbl, 38, 136);
  lv_obj_set_style_text_color(lbl, lv_color_hex(0xBFD4DD), 0);

  lbl = lv_label_create(p);
  lv_label_set_text(lbl, "PLUNGER");
  lv_obj_set_pos(lbl, 190, 136);
  lv_obj_set_style_text_color(lbl, lv_color_hex(0xBFD4DD), 0);
}
static void machineUpdate(){
  /* Hopper fill bar */
  int hopperW = (64 * hopper) / 100;
  lv_obj_set_width(hopperFill, hopperW);

  if(hopperW <= 0) lv_obj_add_flag(hopperFill, LV_OBJ_FLAG_HIDDEN);
  else lv_obj_clear_flag(hopperFill, LV_OBJ_FLAG_HIDDEN);

  /* Plunger stroke strictly capped by strokeLimit */
  int maxTravelPx = 204;
  int sw = (maxTravelPx * stroke) / 100;

  if(sw > 0){
    lv_obj_clear_flag(blue, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_x(blue, 260 - sw);
    lv_obj_set_width(blue, sw);
  } else {
    lv_obj_add_flag(blue, LV_OBJ_FLAG_HIDDEN);
  }

  /* Door animation: 0 = closed, 100 = fully open upward */
  /*
  int closedY = 85;
  int openY   = 37;
  int doorOpenPct = 0;

  if(dischargeMode || phase == WAIT_DOOR_OPEN)
    doorOpenPct = (lv_obj_get_y(door) <= openY) ? 100 : 0;

  int y = closedY - ((closedY - openY) * doorOpenPct) / 100;
  lv_obj_set_y(door, y);
  */
}
static void transition(Phase p){
  phase=p;
  phaseAt=lv_tick_get();
 }
static void render(){
  const char *name = "";
  const char *instruction = "";
  UiSeverity sev = UI_INFO;

  switch(phase){
    case READY:
      name = "FEED READY";
      instruction = "CONVEYOR START PERMITTED";
      sev = UI_READY;
      break;

    case FEEDING:
      name = "MATERIAL FEEDING";
      instruction = "CONVEYOR RUNNING";
      sev = UI_INFO;
      break;

    case HOPPER_FULL:
      name = "HOPPER HIGH LEVEL";
      instruction = "CONVEYOR STOPPED";
      sev = UI_WAIT;
      break;

    case ADVANCING:
      name = "COMPRESSION STROKE";
      instruction = "FEED INHIBITED";
      sev = UI_INFO;
      break;

    case RETRACTING:
      name = "PLUNGER RETURN STROKE";
      instruction = "AWAIT FEED READY";
      sev = UI_INFO;
      break;

    case PRESSURE_REACHED:
      name = "PRESSURE SETPOINT REACHED";
      instruction = "AWAIT BALER OPERATOR";
      sev = UI_WAIT;
      break;

    case WAIT_DOOR_OPEN:
      name = "AWAIT DISCHARGE DOOR OPEN";
      instruction = "KEEP CONVEYOR STOPPED";
      sev = UI_WAIT;
      break;

    case DISCHARGE_FEED:
      name = "DISCHARGE FEED READY";
      instruction = "START FEED OR AWAIT DOOR CLOSE";
      sev = UI_WAIT;
      break;

    case DISCHARGE_PUSH:
      name = "BALE DISCHARGE STROKE";
      instruction = "FEED INHIBITED";
      sev = UI_ALERT;
      break;
  }

  lv_color_t c = severityColor(sev);

  lv_label_set_text(processLabel, name);
  lv_label_set_text(instructionLabel, instruction);

  lv_obj_set_style_text_color(processLabel, c, 0);
  lv_obj_set_style_text_color(instructionLabel, c, 0);
  lv_obj_set_style_border_color(processPanel, c, 0);

  lv_label_set_text_fmt(
    statusLabel,
    "Pressure: %d/%d  |  Stroke: %d/%d%%\n"
    "Hopper: %d%%  |  Compactions: %d",
    pressure, pressureLimit,
    stroke, strokeLimit,
    hopper, cycleCount
  );

  /* Optional: status text also changes on alert */
  if(sev == UI_ALERT)
    lv_obj_set_style_text_color(statusLabel, severityColor(UI_ALERT), 0);
  else
    lv_obj_set_style_text_color(statusLabel, lv_color_hex(0xD7E3EA), 0);

  machineUpdate();
}
static void tick(lv_timer_t *t){(void)t;uint32_t ms=lv_tick_elaps(phaseAt);
  switch(phase){
    case FEEDING:hopper=(int)(ms*100/2500);if(hopper>=100){hopper=100;transition(HOPPER_FULL);}break;
    case HOPPER_FULL:if(ms>=700)transition(dischargeMode?DISCHARGE_PUSH:ADVANCING);break;
    case ADVANCING:
    case DISCHARGE_PUSH:
      // Fixed stroke setpoint: NEVER exceeds strokeLimit.
      // In discharge mode, feeding and compression repeat until door-closed signal.
      stroke=(int)((uint32_t)strokeLimit*(ms>2300?2300:ms)/2300);
      if(ms>=2300){
        stroke=strokeLimit;
        if(phase==ADVANCING){
          cycleCount++;pressure=cycleCount*20; // simulated PLC-calculated pressure (replace with PLC reading)
          wastePct=cycleCount*11;if(wastePct>80)wastePct=80;
        }else {wastePct=80;} // continued feed pushes finished bale (schematic)
        transition(RETRACTING);
      }break;
    case RETRACTING:
      stroke=strokeLimit*(int)(1800-(ms>1800?1800:ms))/1800;
      if(ms>=1800){stroke=0;hopper=0;
        if(dischargeMode)transition(DISCHARGE_FEED);
        else if(pressure>=pressureLimit)transition(PRESSURE_REACHED);
        else transition(READY);
      }break;
    case PRESSURE_REACHED:if(ms>900)transition(WAIT_DOOR_OPEN);break;
    default:break;
  }
  render();
}
void setup(){
    Serial.begin(115200);
    pinMode(BL, OUTPUT); 
    analogWrite(BL, 255);
    tft.init();                           // ST7796S LCD initialize
    tft.setRotation(0);                   // ST7796S LCD set orientation potrait
    tft.invertDisplay(0);
    tft.fillScreen(TFT_BLACK);
    delay (50);

    ft6336u.begin();                      // FOR CAP SCREEN
    lv_init();
    last_tick = millis();

    /* register print function for debugging */
    #if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print );
    #endif

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );
    
    
    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    
    
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );
    
  // INSERT your working lv_init(), display registration, touch and tick setup HERE.
  scr=lv_scr_act();
  lv_obj_set_style_bg_color(scr,lv_color_hex(0x101A24),0);
  lv_obj_clear_flag(scr,LV_OBJ_FLAG_SCROLLABLE);
  drawMachine();
  processPanel = rectangle(scr, 7, 171, 306, 300, 0x243441);
  lv_obj_set_style_radius(processPanel, 10, 0);
  lv_obj_set_style_border_width(processPanel, 2, 0);
  lv_obj_set_style_border_color(processPanel, lv_color_hex(0x5BC0EB), 0);

  lv_obj_t *title = caption(processPanel, 5, 20, 296, "CURRENT BALER PROCESS");
  lv_obj_set_style_text_color(title, lv_color_hex(0xAFC7D3), 0);

  processLabel = caption(processPanel, 8, 74, 290, "");
  instructionLabel = caption(processPanel, 8, 120, 290, "");
  statusLabel = caption(processPanel, 8, 170, 290, "");

  lv_obj_set_style_text_font(title, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_font(processLabel, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_font(instructionLabel, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_font(statusLabel, &lv_font_montserrat_18, 0);

  lv_obj_set_style_text_color(statusLabel, lv_color_hex(0xD7E3EA), 0);
  transition(READY);
  render();
  simTimer=lv_timer_create(tick,50,NULL);
}
void loop(){
  lv_timer_handler();
  delay(5);
}
