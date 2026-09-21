#include "ui_screen.h"
#include "ili9341.h"
#include "colors.h"
#include <string.h>
#include <stdio.h>
#include "robot_params.h"
/* ================= NAME3 : ODOM SCREEN ================= */
#include <stdint.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>   // abs
#include "pid_params_flash.h"
#include "robot_params.h"
///////////////////////////////////////////////////////////
#define GRAPH_WIDTH  200   // chiều ngang đồ thị (pixel)
#define GRAPH_HEIGHT 120   // chiều cao đồ thị (pixel)
#define GRAPH_MAX_W 260

static float V1_hist[GRAPH_MAX_W];
static float V2_hist[GRAPH_MAX_W];
static float VD1_hist[GRAPH_MAX_W];
static float VD2_hist[GRAPH_MAX_W];
uint16_t xpix_prev = 0;
int yv1_prev=0, yvd1_prev=0, yv2_prev=0, yvd2_prev=0;
uint8_t first = 1;
static int16_t prev_y_v1[GRAPH_MAX_W];
static int16_t prev_y_v2[GRAPH_MAX_W];
static int16_t prev_y_vd1[GRAPH_MAX_W];
static int16_t prev_y_vd2[GRAPH_MAX_W];
static uint8_t prev_valid = 0;
static uint16_t hist_len = 200;
#define COL_V1   0xF800   // Đỏ đậm
#define COL_VD1  0x001F   // Xanh dương đậm
#define COL_V2   0x07E0   // Xanh lá đậm
#define COL_VD2  0x780F   // Tím đậm
#define W   GRAPH_MAX_W

static float V1_buf[W], VD1_buf[W];
static float V2_buf[W], VD2_buf[W];
#define STEP_X  4
// 1: dày nhất, 2: thưa vừa, 3~4: thưa rõ

static uint16_t head = 0;      // vị trí sẽ ghi tiếp theo (write index)
static uint8_t  filled = 0;    // đã đầy chưa
static uint16_t len = 0;       // số mẫu hợp lệ (<=W)
// x: 0..(len-1)  (0 là trái - cũ nhất, len-1 là phải - mới nhất)
static inline uint16_t idx_from_x(uint16_t x)
{
    // oldest = head (khi buffer đã đầy)
    // idx = (head + x) % W
    return (uint16_t)((head + x) % W);
}
static inline uint16_t idx_from_x_runtime(uint16_t x)
{
    if (!filled) {
        // chưa đầy: cũ nhất ở 0
        return x;
    }
    // đã đầy: cũ nhất ở head
    return (uint16_t)((head + x) % W);
}




static uint16_t graph_x0;   // gốc đồ thị X
static uint16_t graph_y0;   // gốc đồ thị Y

static uint16_t graph_w, graph_h;

#define YMAX 1.0f
#define GRID_COL 0x7BEF

static uint16_t g1_x0, g1_y0, g1_w, g1_h, g1_midY;
static uint16_t g2_x0, g2_y0, g2_w, g2_h, g2_midY;
static float g1_yscale, g2_yscale;

static uint8_t frame_drawn = 0;


////////////////////////////////////////////////////////////////
/* Text positions per page (FIX: tránh overwrite gây giá trị rác) */
static UI_TextPos_t page1_pos[8];   // Enc1, Enc2, Speed1, Speed2
static UI_TextPos_t page2_pos[3];   // Kp, Ki, Kd
static UI_TextPos_t page3_pos[3];   // Odom X, Y, Theta

static uint8_t pid_target = 1;     // 1 = PID1, 2 = PID2
static Rect_t pid_btn_save;
static Rect_t pid_tab1, pid_tab2;

void Graph_Push(float v1,float v2,float vd1,float vd2)
{
    V1_buf[head]  = v1;
    V2_buf[head]  = v2;
    VD1_buf[head] = vd1;
    VD2_buf[head] = vd2;

    head++;
    if (head >= W) head = 0;

    if (!filled) {
        len++;
        if (len >= W) { len = W; filled = 1; }
    }
}
static uint8_t g_pid_loaded_once = 0;

static UI_State_t g_ui;

uint16_t val_off = 70;   // <-- THIẾU CÁI NÀY

/* Format float to fixed-point with N decimals, without %f */
static void draw_thick_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    for(int dx=-1; dx<=1; dx++)
        for(int dy=-1; dy<=1; dy++)
            lcdDrawPixel(x+dx, y+dy, color);
}

static void ui_fmt_fixed_round6(char *out, size_t n, float v)
{
    if (n == 0) return;

    // Bảo vệ NaN / Inf
    if (!(v == v) || v > 1e30f || v < -1e30f) {
        snprintf(out, n, "0");
        return;
    }

    int32_t sign = (v < 0.0f) ? -1 : 1;
    float av = (v < 0.0f) ? -v : v;

    const int32_t scale = 1000000;   // 10^6  → 6 chữ số sau dấu chấm

    int32_t ip = (int32_t)av;        // phần nguyên

    // 👉 LÀM TRÒN ở chữ số thứ 6
    float frac_f = (av - (float)ip) * (float)scale + 0.5f;
    int32_t fp = (int32_t)frac_f;

    // xử lý carry (ví dụ 1.9999999 → 2.000000)
    if (fp >= scale) {
        fp = 0;
        ip += 1;
    }

    // Tạo chuỗi phần thập phân với zero-padding
    char frac[8];        // đủ chứa "000000\0"
    frac[6] = 0;
    for (int i = 5; i >= 0; i--) {
        frac[i] = (char)('0' + (fp % 10));
        fp /= 10;
    }

    if (sign < 0)
        snprintf(out, n, "-%ld.%s", (long)ip, frac);
    else
        snprintf(out, n, "%ld.%s", (long)ip, frac);
}



//static void UI_Draw_Name3(void)
//{
//    ILI9341_FillScreen(ILI9341_BLACK);
//
//    ILI9341_DrawString(ODOM_LABEL_X, ODOM_X_Y,
//                       "X:", ILI9341_WHITE, ILI9341_BLACK, 2);
//
//    ILI9341_DrawString(ODOM_LABEL_X, ODOM_Y_Y,
//                       "Y:", ILI9341_WHITE, ILI9341_BLACK, 2);
//
//    ILI9341_DrawString(ODOM_LABEL_X, ODOM_THETA_Y,
//                       "Theta:", ILI9341_WHITE, ILI9341_BLACK, 2);
//}



// ===== utils =====
static inline uint8_t pointInRect(uint16_t px, uint16_t py, Rect_t r) {
  return (px >= r.x && px < (uint16_t)(r.x + r.w) &&
          py >= r.y && py < (uint16_t)(r.y + r.h));
}

//static inline void UI_PrintXY(uint16_t x, uint16_t y, const char *s) {
//  lcdSetCursor(x, y);
//  lcdPrintf("%s", s);
//}
static void UI_PrintClipped(uint16_t x, uint16_t y, const char* s)
{
    uint16_t right = g_ui.panel.x + g_ui.panel.w - 2;
    if (right <= x) return;

    uint16_t max_px = right - x;
    uint16_t cw = lcdGetTextFont()->Width; // Font12 width
    uint16_t max_chars = max_px / cw;

    char tmp[64];
    if (max_chars >= sizeof(tmp)) max_chars = sizeof(tmp) - 1;

    strncpy(tmp, s, max_chars);
    tmp[max_chars] = 0;

    lcdSetCursor(x, y);
    lcdPrintf("%s", tmp);
}

static inline void UI_SetText(uint16_t fg, uint16_t bg) {
  lcdSetTextColor(fg, bg);
}

// ước lượng canh giữa (font Font12 của bạn ~ 12px cao; rộng 6-8px/char tuỳ font)
static void UI_DrawCenteredText(Rect_t r, const char *txt) {
  uint16_t charW = 8;     // ước lượng
  uint16_t fontH = 12;    // Font12
  uint16_t len = (uint16_t)strlen(txt);
  uint16_t textW = (uint16_t)(len * charW);

  uint16_t x = r.x + (r.w > textW ? (r.w - textW)/2 : 2);
  uint16_t y = r.y + (r.h > fontH ? (r.h - fontH)/2 : 2);

  UI_PrintClipped(x, y, txt);
}

static void UI_LayoutInit(void) {
  g_ui.header = (Rect_t){0, 0, UI_LCD_W, UI_HEADER_H};
  g_ui.panel  = (Rect_t){UI_LEFT_W, UI_HEADER_H, (uint16_t)(UI_LCD_W-UI_LEFT_W), (uint16_t)(UI_LCD_H-UI_HEADER_H)};

  uint16_t btnH = (uint16_t)((UI_LCD_H - UI_HEADER_H) / UI_BTN_COUNT);
  for (int i=0;i<UI_BTN_COUNT;i++){
    g_ui.btn[i].x = 0;
    g_ui.btn[i].y = (uint16_t)(UI_HEADER_H + i*btnH);
    g_ui.btn[i].w = UI_LEFT_W;
    g_ui.btn[i].h = btnH;
  }

  g_ui.active = UI_PAGE_INFO;
}
//void PIDFlash_Save_All(void)
//{
//    // TODO: gọi hàm save thật của bạn ở đây.
//    // Bạn search trong pid_params_flash.c xem hàm save thật tên gì.
//
//    // Ví dụ nếu trong file có hàm: bool PIDFlash_Save(const PIDGains_t* g)
//    // thì làm như sau:
//
//    PIDGains_t g;
//    g.kp1 = (float)Kp1; g.ki1 = (float)Ki1; g.kd1 = (float)Kd1;
//    g.kp2 = (float)Kp2; g.ki2 = (float)Ki2; g.kd2 = (float)Kd2;
//
//    PIDFlash_Save(&g);   // <-- đổi đúng theo hàm save thật trong file bạn
//}
//
//void UI_LoadPIDFromFlashOnce(void)
//{
//    if (g_pid_loaded_once) return;
//    g_pid_loaded_once = 1;
//
//    PIDGains_t g;
//
//    if (PIDFlash_Load(&g))  // load từ Flash nếu có
//    {
//        // CẬP NHẬT BIẾN THẬT (control dùng)
//        Kp1 = g.kp1; Ki1 = g.ki1; Kd1 = g.kd1;
//        Kp2 = g.kp2; Ki2 = g.ki2; Kd2 = g.kd2;
//    }
//
//    // 🔥 QUAN TRỌNG: LUÔN ĐỒNG BỘ UI THEO BIẾN THẬT
//    Kp_fromLCD1 = Kp1; Ki_fromLCD1 = Ki1; Kd_fromLCD1 = Kd1;
//    Kp_fromLCD2 = Kp2; Ki_fromLCD2 = Ki2; Kd_fromLCD2 = Kd2;
//
//    pid_update_pending = 1;   // báo control task cập nhật (nếu bạn có dùng)
//}


//static void PID_ApplyFromUI(void)
//{
//    SpeedPID1.Kp = Kp1;
//    SpeedPID1.Ki = Ki1;
//    SpeedPID1.Kd = Kd1;
//}

static void UI_DrawHeader(void) {
  lcdFillRect(g_ui.header.x, g_ui.header.y, g_ui.header.w, g_ui.header.h, UI_BG);
  lcdDrawRect(g_ui.header.x, g_ui.header.y, g_ui.header.w, g_ui.header.h, UI_FG);

  UI_SetText(UI_FG, UI_BG);
  UI_DrawCenteredText(g_ui.header, "Robotmango Zackon Chao ban den voi giao dien ky thuat");
}

static void UI_DrawButtons(void)
{
  static const char *names[UI_BTN_COUNT] = {"ENCODER","PID","ODOM","SETUP","FIGURE"};

  // Map page -> button index
  // UI_PAGE_INFO: không highlight
  int active_btn = -1;
  if (g_ui.active >= UI_PAGE_1 && g_ui.active <= UI_PAGE_5) {
    active_btn = (int)g_ui.active - (int)UI_PAGE_1;   // PAGE_1->0, PAGE_2->1, ...
  }

  for (int i = 0; i < UI_BTN_COUNT; i++) {
    uint16_t bg = (i == active_btn) ? UI_BTN_ACTIVE : UI_BTN_IDLE;

    lcdFillRect(g_ui.btn[i].x, g_ui.btn[i].y, g_ui.btn[i].w, g_ui.btn[i].h, bg);
    lcdDrawRect(g_ui.btn[i].x, g_ui.btn[i].y, g_ui.btn[i].w, g_ui.btn[i].h, UI_FG);

    UI_SetText(UI_FG, bg);
    UI_DrawCenteredText(g_ui.btn[i], names[i]);
  }
}


static void UI_ClearPanel(void) {
  lcdFillRect(g_ui.panel.x, g_ui.panel.y, g_ui.panel.w, g_ui.panel.h, UI_BG);
  lcdDrawRect(g_ui.panel.x, g_ui.panel.y, g_ui.panel.w, g_ui.panel.h, UI_FG);
}

void UI_ShowPage(UI_Page_t page) {
  if (page >= UI_PAGE_COUNT) return;

  g_ui.active = page;
  lcdDrawRect(g_ui.panel.x, g_ui.panel.y, g_ui.panel.w, g_ui.panel.h, COLOR_RED);

  // highlight lại button
  UI_DrawButtons();

  // clear + frame panel
  UI_ClearPanel();

  // vẽ nội dung demo trong panel
  uint16_t x0 = g_ui.panel.x + 10;
  uint16_t y0 = g_ui.panel.y + 10;

  UI_SetText(UI_FG, UI_BG);

  switch(page){

   case UI_PAGE_INFO:
     UI_PrintClipped(x0, y0, "Chao mung ban!");
     UI_PrintClipped(x0, y0+16, "Hay cham NAME1..NAME5 de mo tab.");
     UI_PrintClipped(x0, y0+32, "Dang cho thao tac...");
     break;


   case UI_PAGE_1:
   {
       lcdFillRect(g_ui.panel.x, g_ui.panel.y,
                   g_ui.panel.w, g_ui.panel.h, UI_BG);

       uint16_t x  = g_ui.panel.x + 10;
       uint16_t y  = g_ui.panel.y + 10;
       uint16_t dy = 20;
       uint16_t val_off = 70;

       UI_SetText(UI_FG, UI_BG);

       // Labels
       UI_PrintClipped(x, y + 0*dy, "ENC1 :");
       UI_PrintClipped(x, y + 1*dy, "ENC2 :");
       UI_PrintClipped(x, y + 2*dy, "V1   :");
       UI_PrintClipped(x, y + 3*dy, "VD1   :");
       UI_PrintClipped(x, y + 4*dy, "V2  :");
       UI_PrintClipped(x, y + 5*dy, "VD2  :");
       UI_PrintClipped(x, y + 6*dy, "PWM1 :");
       UI_PrintClipped(x, y + 7*dy, "PWM2 :");

       // Value positions
       page1_pos[0] = (UI_TextPos_t){x + val_off, y + 0*dy};
       page1_pos[1] = (UI_TextPos_t){x + val_off, y + 1*dy};
       page1_pos[2] = (UI_TextPos_t){x + val_off, y + 2*dy};
       page1_pos[3] = (UI_TextPos_t){x + val_off, y + 3*dy};
       page1_pos[4] = (UI_TextPos_t){x + val_off, y + 4*dy};
       page1_pos[5] = (UI_TextPos_t){x + val_off, y + 5*dy};
       page1_pos[6] = (UI_TextPos_t){x + val_off, y + 6*dy};
       page1_pos[7] = (UI_TextPos_t){x + val_off, y + 7*dy};
   }
   break;


   case UI_PAGE_2:
   {
       lcdFillRect(g_ui.panel.x, g_ui.panel.y,
                   g_ui.panel.w, g_ui.panel.h, UI_BG);

       uint16_t x0 = g_ui.panel.x + 10;
       uint16_t y0 = g_ui.panel.y + 30;
       uint16_t dy = 22;

       UI_SetText(UI_FG, UI_BG);

       // ================= TAB PID1 / PID2 (THÊM Ở ĐÂY) =================
       pid_tab1 = (Rect_t){ g_ui.panel.x + 10,  g_ui.panel.y + 6,  60, 18 };
       pid_tab2 = (Rect_t){ g_ui.panel.x + 80,  g_ui.panel.y + 6,  60, 18 };

       uint16_t bg1 = (pid_target == 1) ? COLOR_YELLOW : UI_BG;
       uint16_t bg2 = (pid_target == 2) ? COLOR_YELLOW : UI_BG;

       lcdFillRect(pid_tab1.x, pid_tab1.y, pid_tab1.w, pid_tab1.h, bg1);
       lcdFillRect(pid_tab2.x, pid_tab2.y, pid_tab2.w, pid_tab2.h, bg2);

       lcdDrawRect(pid_tab1.x, pid_tab1.y, pid_tab1.w, pid_tab1.h, UI_FG);
       lcdDrawRect(pid_tab2.x, pid_tab2.y, pid_tab2.w, pid_tab2.h, UI_FG);

       UI_SetText(UI_FG, bg1);
       UI_DrawCenteredText(pid_tab1, "PID1");

       UI_SetText(UI_FG, bg2);
       UI_DrawCenteredText(pid_tab2, "PID2");

       UI_SetText(UI_FG, UI_BG);
       // ===============================================================

       UI_PrintClipped(x0, y0 + 0*dy, "Kp:");
       UI_PrintClipped(x0, y0 + 1*dy, "Ki:");
       UI_PrintClipped(x0, y0 + 2*dy, "Kd:");

       page2_pos[0] = (UI_TextPos_t){x0 + 50, y0 + 0*dy};
       page2_pos[1] = (UI_TextPos_t){x0 + 50, y0 + 1*dy};
       page2_pos[2] = (UI_TextPos_t){x0 + 50, y0 + 2*dy};

       // ✅ BUTTON ROW (CỰC QUAN TRỌNG)
       pid_row[0] = (Rect_t){x0, y0 + 0*dy, 200, 18};
       pid_row[1] = (Rect_t){x0, y0 + 1*dy, 200, 18};
       pid_row[2] = (Rect_t){x0, y0 + 2*dy, 200, 18};

       // < O >
       uint16_t by = y0 + 4*dy;
       pid_btn_left  = (Rect_t){x0,       by, 40, 22};
       pid_btn_mid   = (Rect_t){x0 + 50,  by, 40, 22};
       pid_btn_right = (Rect_t){x0 + 100, by, 40, 22};

       lcdDrawRect(pid_btn_left.x,  pid_btn_left.y,  pid_btn_left.w,  pid_btn_left.h,  UI_FG);
       lcdDrawRect(pid_btn_mid.x,   pid_btn_mid.y,   pid_btn_mid.w,   pid_btn_mid.h,   UI_FG);
       lcdDrawRect(pid_btn_right.x, pid_btn_right.y, pid_btn_right.w, pid_btn_right.h, UI_FG);

       UI_DrawCenteredText(pid_btn_left, "<");
       UI_DrawCenteredText(pid_btn_mid,  "O");
       UI_DrawCenteredText(pid_btn_right, ">");

       // ================= NÚT SAVE (THÊM Ở ĐÂY) =================
       // đặt ngay bên phải nút '>'
       pid_btn_save = (Rect_t){ x0 + 150, by, 60, 22 };

       lcdDrawRect(pid_btn_save.x, pid_btn_save.y, pid_btn_save.w, pid_btn_save.h, UI_FG);
       UI_DrawCenteredText(pid_btn_save, "Apply");
       // =========================================================

       UI_PrintClipped(x0, by + 26, "Step:");
   }
   break;


   case UI_PAGE_3:
   {
       lcdFillRect(g_ui.panel.x, g_ui.panel.y,
                   g_ui.panel.w, g_ui.panel.h, UI_BG);

       uint16_t x = g_ui.panel.x + 10;
       uint16_t y = g_ui.panel.y + 20;
       uint16_t dy = 20;

       UI_SetText(UI_FG, UI_BG);

       UI_PrintClipped(x, y + 0*dy, "X:");
       UI_PrintClipped(x, y + 1*dy, "Y:");
       UI_PrintClipped(x, y + 2*dy, "Theta:");

       page3_pos[0] = (UI_TextPos_t){x + 80, y + 0*dy};
       page3_pos[1] = (UI_TextPos_t){x + 80, y + 1*dy};
       page3_pos[2] = (UI_TextPos_t){x + 80, y + 2*dy};
   }
   break;


    case UI_PAGE_4:
      UI_PrintClipped(x0, y0, "PAGE 4 (NAME4): Su kien 4");
      break;
    case UI_PAGE_5:
        UI_ClearPanel();

        frame_drawn = 0;
        prev_valid = 0;

        first = 1;
        xpix_prev = 0;
        yv1_prev = 0;  yvd1_prev = 0;
        yv2_prev = 0;  yvd2_prev = 0;

        UI_VelGraph_DrawFramesOnce();  // tạo g1_w/g2_w trước
        Graph_Reset();                  // tính hist_len theo g1_w

        break;






  }
}

void UI_InitAndDrawOnce(void) {
  if (g_ui.inited) return;

  UI_LayoutInit();

  // đảm bảo landscape đúng layout 320x240
  lcdSetOrientation(LCD_ORIENTATION_LANDSCAPE);

  // background
  lcdFillRect(0, 0, UI_LCD_W, UI_LCD_H, UI_BG);

  // static parts
  UI_DrawHeader();
  UI_DrawButtons();
  UI_ClearPanel();

  // default page
  UI_ShowPage(UI_PAGE_INFO);

//  uint16_t W = lcdGetWidth();
//  uint16_t H = lcdGetHeight();
//
//  g_ui.panel.x = 60;      // chiều rộng cột NAME1..5
//  g_ui.panel.y = 0;
//  g_ui.panel.w = W - 60;
//  g_ui.panel.h = H;

}

//void UI_OnTouch(uint16_t x, uint16_t y, uint8_t pressed) {
//  // chỉ trigger khi vừa nhấn xuống (0->1)
//  if (pressed && !g_ui.last_pressed) {
//    for (int i=0;i<UI_BTN_COUNT;i++){
//      if (pointInRect(x, y, g_ui.btn[i])) {
//    	  UI_ShowPage((UI_Page_t)(UI_PAGE_1 + i));
//
//        break;
//      }
//    }
//  }
//  g_ui.last_pressed = pressed;
//}
void UI_OnTouch(uint16_t x, uint16_t y, uint8_t pressed)
{

	    // Nếu nhả tay -> reset edge
	    if (!pressed) {
	        g_ui.last_pressed = 0;
	        return;
	    }
    // chỉ xử lý ở cạnh lên (pressed từ 0 -> 1)
    if (pressed && !g_ui.last_pressed)
    {
        // ===== 1) MENU NAME1..NAME5 =====
        for (int i = 0; i < UI_BTN_COUNT; i++)
        {
            if (pointInRect(x, y, g_ui.btn[i]))
            {
                UI_ShowPage((UI_Page_t)i + 1);
                g_ui.last_pressed = pressed;
                return;
            }

        }

        // ===== 2) PAGE-SPECIFIC TOUCH =====
        if (g_ui.active == UI_PAGE_2)   // <<< NAME2 = PID
        {
            // ---- TAB PID1/PID2 (nếu bạn đã vẽ pid_tab1/pid_tab2) ----
            if (pointInRect(x, y, pid_tab1)) { pid_target = 1; UI_UpdatePIDPage(); g_ui.last_pressed = pressed; return; }
            if (pointInRect(x, y, pid_tab2)) { pid_target = 2; UI_UpdatePIDPage(); g_ui.last_pressed = pressed; return; }

            // ---- chọn Kp / Ki / Kd (bấm vào dòng) ----
            Rect_t row0 = { page2_pos[0].x - 50, page2_pos[0].y, 140, 16 };
            Rect_t row1 = { page2_pos[1].x - 50, page2_pos[1].y, 140, 16 };
            Rect_t row2 = { page2_pos[2].x - 50, page2_pos[2].y, 140, 16 };

            if (pointInRect(x, y, row0)) { pid_selected = 0; UI_UpdatePIDPage(); g_ui.last_pressed = pressed; return; }
            if (pointInRect(x, y, row1)) { pid_selected = 1; UI_UpdatePIDPage(); g_ui.last_pressed = pressed; return; }
            if (pointInRect(x, y, row2)) { pid_selected = 2; UI_UpdatePIDPage(); g_ui.last_pressed = pressed; return; }

//            // ---- trỏ tới bộ PID đang chọn (PID1 hoặc PID2) ----
//            float *pKp = (pid_target == 1) ? &Kp_fromLCD1 : &Kp_fromLCD2;
//            float *pKi = (pid_target == 1) ? &Ki_fromLCD1 : &Ki_fromLCD2;
//            float *pKd = (pid_target == 1) ? &Kd_fromLCD1 : &Kd_fromLCD2;

            // ---- nút < ----
            if (pointInRect(x, y, pid_btn_left)) {
            	if (pid_target == 1)
            	{
					if (pid_selected == 0) { Kp_fromLCD1 -= (float)pid_step; if (Kp_fromLCD1 < 0) Kp_fromLCD1 = 0; }
					if (pid_selected == 1) { Ki_fromLCD1 -= (float)pid_step; if (Ki_fromLCD1 < 0) Ki_fromLCD1 = 0; }
					if (pid_selected == 2) { Kd_fromLCD1 -= (float)pid_step; if (Kd_fromLCD1 < 0) Kd_fromLCD1 = 0; }
            	}
            	else
            	{
            		if (pid_selected == 0) { Kp_fromLCD2 -= (float)pid_step; if (Kp_fromLCD2 < 0) Kp_fromLCD2 = 0; }
            		if (pid_selected == 1) { Ki_fromLCD2 -= (float)pid_step; if (Ki_fromLCD2 < 0) Ki_fromLCD2 = 0; }
            		if (pid_selected == 2) { Kd_fromLCD2 -= (float)pid_step; if (Kd_fromLCD2 < 0) Kd_fromLCD2 = 0; }
            	}

//                // 🔥 ĐỒNG BỘ NGAY VỚI BIẾN PID THẬT
//                if (pid_target == 1) {
//                    Kp1 = Kp_fromLCD1;
//                    Ki1 = Ki_fromLCD1;
//                    Kd1 = Kd_fromLCD1;
//                } else {
//                    Kp2 = Kp_fromLCD2;
//                    Ki2 = Ki_fromLCD2;
//                    Kd2 = Kd_fromLCD2;
//                }

                UI_UpdatePIDPage();   // cập nhật màn hình ngay
//                pid_update_pending = 1;  // báo task điều khiển cập nhật PID
                UI_PrintClipped(g_ui.panel.x + 10, pid_btn_save.y + 26, "Updated");

                g_ui.last_pressed = pressed;
                return;
            }

            // ---- nút > ----
            if (pointInRect(x, y, pid_btn_right)) {
            	if (pid_target == 1)
            	{
					if (pid_selected == 0) { Kp_fromLCD1 += (float)pid_step; }
					if (pid_selected == 1) { Ki_fromLCD1 += (float)pid_step; }
					if (pid_selected == 2) { Kd_fromLCD1 += (float)pid_step; }
            	}
            	else
            	{
					if (pid_selected == 0) { Kp_fromLCD2 += (float)pid_step; }
					if (pid_selected == 1) { Ki_fromLCD2 += (float)pid_step; }
					if (pid_selected == 2) { Kd_fromLCD2 += (float)pid_step; }
            	}

//                // 🔥 ĐỒNG BỘ NGAY VỚI BIẾN PID THẬT
//                if (pid_target == 1) {
//                    Kp1 = Kp_fromLCD1;
//                    Ki1 = Ki_fromLCD1;
//                    Kd1 = Kd_fromLCD1;
//                } else {
//                    Kp2 = Kp_fromLCD2;
//                    Ki2 = Ki_fromLCD2;
//                    Kd2 = Kd_fromLCD2;
//                }

                UI_UpdatePIDPage();
//                pid_update_pending = 1;
                UI_PrintClipped(g_ui.panel.x + 10, pid_btn_save.y + 26, "Updated");

                g_ui.last_pressed = pressed;
                return;
            }


            // ---- nút SAVE ----
            if (pointInRect(x, y, pid_btn_save)) {

                UI_PrintClipped(g_ui.panel.x + 10, pid_btn_save.y + 26, "Applying...");

                // 🔥 BƯỚC 1: đồng bộ UI → biến thật
                Kp1 = Kp_fromLCD1;
                Ki1 = Ki_fromLCD1;
                Kd1 = Kd_fromLCD1;
                Kp2 = Kp_fromLCD2;
                Ki2 = Ki_fromLCD2;
                Kd2 = Kd_fromLCD2;

                pid_update_pending = 1;

//                // 🔥 BƯỚC 2: lưu vào Flash
//                PIDGains_t g;
//                g.kp1 = Kp1; g.ki1 = Ki1; g.kd1 = Kd1;
//                g.kp2 = Kp2; g.ki2 = Ki2; g.kd2 = Kd2;
//
//                (void)PIDFlash_Save(&g);

                UI_PrintClipped(g_ui.panel.x + 10, pid_btn_save.y + 26, "Applied.   ");

                g_ui.last_pressed = pressed;
                return;
            }

            // ---- nút O (đổi step) ----
            if (pointInRect(x, y, pid_btn_mid)) {
                switch(pid_step){
                  case 1:    pid_step = 10; break;
                  case 10:   pid_step = 100; break;
                  case 100:  pid_step = 1000; break;
                  default:   pid_step = 1; break;
                }
                g_ui.last_pressed = pressed;
                return;
            }
        }
        g_ui.last_pressed = pressed;
    }
}
UI_Page_t UI_GetActivePage(void) {
  return g_ui.active;

}
static void UI_ClearValueLine(uint16_t x, uint16_t y)
{
    // Xoá từ x tới hết panel (không bao giờ tràn)
    uint16_t right = g_ui.panel.x + g_ui.panel.w - 2;
    if (right <= x) return;

    uint16_t w = right - x;
    uint16_t h = lcdGetTextFont()->Height;   // hoặc Font12->Height
    lcdFillRect(x, y, w, h, UI_BG);
}

void UI_UpdateEncoderPage(int32_t enc1, int32_t enc2,
                          float v1, float v2,
                          float v1d, float v2d,
                          float pwm1, float pwm2)
{
    if (g_ui.active != UI_PAGE_1) return;

    char buf[32];
    UI_SetText(UI_FG, UI_BG);

    // ENC1
    UI_ClearValueLine(page1_pos[0].x, page1_pos[0].y);
    snprintf(buf, sizeof(buf), "%ld", (long)enc1);
    UI_PrintClipped(page1_pos[0].x, page1_pos[0].y, buf);

    // ENC2
    UI_ClearValueLine(page1_pos[1].x, page1_pos[1].y);
    snprintf(buf, sizeof(buf), "%ld", (long)enc2);
    UI_PrintClipped(page1_pos[1].x, page1_pos[1].y, buf);

    // V1
    UI_ClearValueLine(page1_pos[2].x, page1_pos[2].y);
    ui_fmt_fixed_round6(buf, sizeof(buf), v1);
    UI_PrintClipped(page1_pos[2].x, page1_pos[2].y, buf);
    UI_PrintClipped(page1_pos[2].x + 72, page1_pos[2].y, " m/s");

    // VD1
    UI_ClearValueLine(page1_pos[3].x, page1_pos[3].y);
    ui_fmt_fixed_round6(buf, sizeof(buf), v1d);
    UI_PrintClipped(page1_pos[3].x, page1_pos[3].y, buf);
    UI_PrintClipped(page1_pos[3].x + 72, page1_pos[3].y, " m/s");
    // V2
    UI_ClearValueLine(page1_pos[4].x, page1_pos[4].y);
    ui_fmt_fixed_round6(buf, sizeof(buf), v2);
    UI_PrintClipped(page1_pos[4].x, page1_pos[4].y, buf);
    UI_PrintClipped(page1_pos[4].x + 72, page1_pos[4].y, " m/s");
    // VD2
    UI_ClearValueLine(page1_pos[5].x, page1_pos[5].y);
    ui_fmt_fixed_round6(buf, sizeof(buf), v2d);
    UI_PrintClipped(page1_pos[5].x, page1_pos[5].y, buf);
    UI_PrintClipped(page1_pos[5].x + 72, page1_pos[5].y, " m/s");

    // PWM1
    UI_ClearValueLine(page1_pos[6].x, page1_pos[6].y);
    snprintf(buf, sizeof(buf), "%.2f", (double)pwm1);
    UI_PrintClipped(page1_pos[6].x, page1_pos[6].y, buf);

    // PWM2
    UI_ClearValueLine(page1_pos[7].x, page1_pos[7].y);
    snprintf(buf, sizeof(buf), "%.2f", (double)pwm2);
    UI_PrintClipped(page1_pos[7].x, page1_pos[7].y, buf);
}

//static void UI_ClearRect(Rect_t r, uint16_t bg)
//{
//    lcdFillRect(r.x+1, r.y+1, r.w-2, r.h-2, bg);
//}


//void UI_UpdatePIDPage(void)
//{
//
//
//    if (g_ui.active != UI_PAGE_2) return;
//
//    char buf[32];
//
//    // Xoá vùng value trước khi in để không tràn
//    uint16_t right = g_ui.panel.x + g_ui.panel.w - 2;
//    uint16_t h = 16;
//
//    // Kp
//    lcdFillRect(enc_pos[0].x, enc_pos[0].y, right - enc_pos[0].x, h, UI_BG);
//    snprintf(buf, sizeof(buf), "%.0f", (double)Kp1);
//    UI_PrintClipped(enc_pos[0].x, enc_pos[0].y, buf);
//
//    // Ki
//    lcdFillRect(enc_pos[1].x, enc_pos[1].y, right - enc_pos[1].x, h, UI_BG);
//    snprintf(buf, sizeof(buf), "%.0f", (double)Ki1);
//    UI_PrintClipped(enc_pos[1].x, enc_pos[1].y, buf);
//
//    // Kd
//    lcdFillRect(enc_pos[2].x, enc_pos[2].y, right - enc_pos[2].x, h, UI_BG);
//    snprintf(buf, sizeof(buf), "%.0f", (double)Kd1);
//    UI_PrintClipped(enc_pos[2].x, enc_pos[2].y, buf);
//
//    // Step
//    snprintf(buf, sizeof(buf), "%d", pid_step);
//    UI_PrintClipped(g_ui.panel.x + 70, pid_btn_left.y + 26, buf);
//}
void UI_UpdatePIDPage(void)
{
    if (g_ui.active != UI_PAGE_2) return;

    char buf[32];

    // ===== 1) TÔ NỀN 3 DÒNG =====
    for (int i=0;i<3;i++){
        uint16_t bg = (i==pid_selected) ? COLOR_YELLOW : UI_BG;

        lcdFillRect(pid_row[i].x, pid_row[i].y,
                    pid_row[i].w, pid_row[i].h, bg);

        UI_SetText(UI_FG, bg);
        if (i==0) UI_PrintClipped(pid_row[i].x + 6, pid_row[i].y + 2, "Kp:");
        if (i==1) UI_PrintClipped(pid_row[i].x + 6, pid_row[i].y + 2, "Ki:");
        if (i==2) UI_PrintClipped(pid_row[i].x + 6, pid_row[i].y + 2, "Kd:");
    }

    // ===== 2) HIỂN THỊ GIÁ TRỊ =====
    // ===== 2) HIỂN THỊ GIÁ TRỊ =====
    double val[3];
    if (pid_target == 1) {
        val[0] = Kp_fromLCD1;
        val[1] = Ki_fromLCD1;
        val[2] = Kd_fromLCD1;
    } else {
        val[0] = Kp_fromLCD2;
        val[1] = Ki_fromLCD2;
        val[2] = Kd_fromLCD2;
    }

    // cập nhật lại TAB màu cho đúng (nhẹ)
    uint16_t bg1 = (pid_target==1) ? COLOR_YELLOW : UI_BG;
    uint16_t bg2 = (pid_target==2) ? COLOR_YELLOW : UI_BG;
    lcdFillRect(pid_tab1.x, pid_tab1.y, pid_tab1.w, pid_tab1.h, bg1);
    lcdFillRect(pid_tab2.x, pid_tab2.y, pid_tab2.w, pid_tab2.h, bg2);
    lcdDrawRect(pid_tab1.x, pid_tab1.y, pid_tab1.w, pid_tab1.h, UI_FG);
    lcdDrawRect(pid_tab2.x, pid_tab2.y, pid_tab2.w, pid_tab2.h, UI_FG);
    UI_SetText(UI_FG, bg1); UI_DrawCenteredText(pid_tab1, "PID1");
    UI_SetText(UI_FG, bg2); UI_DrawCenteredText(pid_tab2, "PID2");

    for (int i=0;i<3;i++){
        uint16_t bg = (i==pid_selected) ? COLOR_YELLOW : UI_BG;
        UI_SetText(UI_FG, bg);

        uint16_t vx = page2_pos[i].x;
        uint16_t vy = page2_pos[i].y;
        uint16_t right = g_ui.panel.x + g_ui.panel.w - 2;

        lcdFillRect(vx, vy, right - vx, 16, bg);
        snprintf(buf, sizeof(buf), "%.0f", val[i]);
        UI_PrintClipped(vx, vy, buf);
    }

    // ===== 3) HIỂN THỊ STEP =====
    UI_SetText(UI_FG, UI_BG);
    lcdFillRect(g_ui.panel.x + 60, pid_btn_left.y + 26, 80, 16, UI_BG);
    snprintf(buf, sizeof(buf), "%d", pid_step);
    UI_PrintClipped(g_ui.panel.x + 60, pid_btn_left.y + 26, buf);
}

//static void UI_Update_Name3(void)
//{
//    /* odom data lấy từ control / odom module */
//    extern float odom_x;
//    extern float odom_y;
//    extern float odom_theta;
//
//    /* clear value area */
//    ILI9341_FillRect(ODOM_VALUE_X, ODOM_X_Y,     160, 24, ILI9341_BLACK);
//    ILI9341_FillRect(ODOM_VALUE_X, ODOM_Y_Y,     160, 24, ILI9341_BLACK);
//    ILI9341_FillRect(ODOM_VALUE_X, ODOM_THETA_Y, 160, 24, ILI9341_BLACK);
//
//    snprintf(odom_str, sizeof(odom_str), "%.3f", odom_x);
//    ILI9341_DrawString(ODOM_VALUE_X, ODOM_X_Y,
//                       odom_str, ILI9341_GREEN, ILI9341_BLACK, 2);
//
//    snprintf(odom_str, sizeof(odom_str), "%.3f", odom_y);
//    ILI9341_DrawString(ODOM_VALUE_X, ODOM_Y_Y,
//                       odom_str, ILI9341_GREEN, ILI9341_BLACK, 2);
//
//    /* nếu theta là rad → đổi sang deg tại đây */
//    snprintf(odom_str, sizeof(odom_str), "%.2f", odom_theta);
//    ILI9341_DrawString(ODOM_VALUE_X, ODOM_THETA_Y,
//                       odom_str, ILI9341_GREEN, ILI9341_BLACK, 2);
//}
void UI_UpdateOdomPage(float x, float y, float theta)
{
    if (g_ui.active != UI_PAGE_3) return;

    char buf[32];
    UI_SetText(UI_FG, UI_BG);

    // X
    UI_ClearValueLine(page3_pos[0].x, page3_pos[0].y);
    ui_fmt_fixed_round6(buf, sizeof(buf), x);
    UI_PrintClipped(page3_pos[0].x, page3_pos[0].y, buf);

    // Y
    UI_ClearValueLine(page3_pos[1].x, page3_pos[1].y);
    ui_fmt_fixed_round6(buf, sizeof(buf), y);
    UI_PrintClipped(page3_pos[1].x, page3_pos[1].y, buf);

    // Theta
    UI_ClearValueLine(page3_pos[2].x, page3_pos[2].y);
    ui_fmt_fixed_round6(buf, sizeof(buf), theta);
    UI_PrintClipped(page3_pos[2].x, page3_pos[2].y, buf);
}
//void UI_Graph_Push(float v1, float v2, float vd1, float vd2)
//{
//    for (int i = 0; i < (int)hist_len - 1; i++) {
//        V1_hist[i]  = V1_hist[i+1];
//        V2_hist[i]  = V2_hist[i+1];
//        VD1_hist[i] = VD1_hist[i+1];
//        VD2_hist[i] = VD2_hist[i+1];
//    }
//    V1_hist[hist_len-1]  = v1;
//    V2_hist[hist_len-1]  = v2;
//    VD1_hist[hist_len-1] = vd1;
//    VD2_hist[hist_len-1] = vd2;
//}`

void Graph_Reset(void)
{
    // reset ring buffer
    head = 0;
    filled = 0;
    len = 0;

    // số điểm hiển thị theo STEP_X (mỗi điểm cách STEP_X pixel)
    hist_len = g1_w / STEP_X;
    if (hist_len < 2) hist_len = 2;
    if (hist_len > GRAPH_MAX_W) hist_len = GRAPH_MAX_W;

    // reset trạng thái vẽ
    prev_valid = 0;
    first = 1;
}


void UI_Graph_Push(float v1, float v2, float vd1, float vd2)
{
    // shift phải
//    for (int i = (int)hist_len - 1; i > 0; i--) {
//        V1_hist[i]  = V1_hist[i-1];
//        V2_hist[i]  = V2_hist[i-1];
//        VD1_hist[i] = VD1_hist[i-1];
//        VD2_hist[i] = VD2_hist[i-1];
//    }
	for (uint16_t x = 0; x < len; x++) {
	    uint16_t k = idx_from_x_runtime(x);

	    float v1  = V1_buf[k];
	    float vd1 = VD1_buf[k];

	    // map -> y
	    int yv1  = (int)(g1_midY - v1  * g1_yscale);
	    int yvd1 = (int)(g1_midY - vd1 * g1_yscale);

	    // ... clamp rồi vẽ tại (g1_x0 + x, y)
	}
    // sample mới ở bên trái (x nhỏ)
    V1_hist[0]  = v1;
    V2_hist[0]  = v2;
    VD1_hist[0] = vd1;
    VD2_hist[0] = vd2;
}
static float Graph_GetMaxAbs(void)
{
    float m = 0.1f;
    for (int i=0;i<(int)hist_len;i++){
        float a;

        a = (V1_hist[i]  < 0) ? -V1_hist[i]  : V1_hist[i];  if (a>m) m=a;
        a = (V2_hist[i]  < 0) ? -V2_hist[i]  : V2_hist[i];  if (a>m) m=a;
        a = (VD1_hist[i] < 0) ? -VD1_hist[i] : VD1_hist[i]; if (a>m) m=a;
        a = (VD2_hist[i] < 0) ? -VD2_hist[i] : VD2_hist[i]; if (a>m) m=a;
    }
    return m;
}

void UI_DrawVelocityGraph(void)
{
    // 1) clear vùng graph
    lcdFillRect(graph_x0, graph_y0, graph_w, graph_h, UI_BG);

    // 2) khung
    lcdDrawRect(graph_x0, graph_y0, graph_w, graph_h, UI_FG);

    // 3) lưới + trục
    uint16_t midY = graph_y0 + graph_h/2;

    // trục 0
    lcdDrawLine(graph_x0, midY, graph_x0 + graph_w, midY, UI_FG);

    // lưới ngang (4 phần)
    for (int k=1;k<=3;k++){
        uint16_t yy = graph_y0 + (uint16_t)(k * (graph_h/4));
        lcdDrawLine(graph_x0, yy, graph_x0 + graph_w, yy, 0x7BEF); // xám nhạt
    }

    // lưới dọc + tick thời gian (mỗi 1s)
    // dt=0.1s, 1s = 10 samples
    for (int i=0;i<(int)graph_w;i+=50){
        lcdDrawLine(graph_x0+i, graph_y0, graph_x0+i, graph_y0+graph_h, 0x7BEF);
    }

    // 4) scale trục Y
    float ymax = Graph_GetMaxAbs();       // auto scale
    // làm tròn ymax cho đẹp: ví dụ 0.83 -> 1.0; 1.6 -> 2.0; 2.3 -> 3.0
    float nice = 1.0f;
    if (ymax <= 0.5f) nice = 0.5f;
    else if (ymax <= 1.0f) nice = 1.0f;
    else if (ymax <= 2.0f) nice = 2.0f;
    else if (ymax <= 3.0f) nice = 3.0f;
    else if (ymax <= 5.0f) nice = 5.0f;
    else nice = 10.0f;

    float scale = (graph_h/2 - 2) / nice;

    // 5) ghi nhãn trục Y: +nice, 0, -nice
    char buf[16];
    UI_SetText(UI_FG, UI_BG);

    snprintf(buf,sizeof(buf), "+%.1f", (double)nice);
    UI_PrintClipped(graph_x0 - 36, graph_y0 + 2, buf);

    UI_PrintClipped(graph_x0 - 36, midY - 6, "0.0");

    snprintf(buf,sizeof(buf), "-%.1f", (double)nice);
    UI_PrintClipped(graph_x0 - 36, graph_y0 + graph_h - 14, buf);

    // 6) trục thời gian (hiển thị “seconds back”)
    // ví dụ: graph_w samples * 0.1s/sample -> total seconds
    float total_s = (float)hist_len * 0.1f;
    snprintf(buf,sizeof(buf), "%.1fs", (double)total_s);
    UI_PrintClipped(graph_x0 + graph_w - 40, graph_y0 + graph_h + 2, buf);
    UI_PrintClipped(graph_x0, graph_y0 + graph_h + 2, "time");

    // 7) vẽ 4 đường
    for (int i=1;i<(int)hist_len;i++){
        int x0 = graph_x0 + (i-1);
        int x1 = graph_x0 + i;

        int y0_v1  = midY - (int)(V1_hist[i-1]  * scale);
        int y1_v1  = midY - (int)(V1_hist[i]    * scale);

        int y0_v2  = midY - (int)(V2_hist[i-1]  * scale);
        int y1_v2  = midY - (int)(V2_hist[i]    * scale);

        int y0_vd1 = midY - (int)(VD1_hist[i-1] * scale);
        int y1_vd1 = midY - (int)(VD1_hist[i]   * scale);

        int y0_vd2 = midY - (int)(VD2_hist[i-1] * scale);
        int y1_vd2 = midY - (int)(VD2_hist[i]   * scale);

        // cắt biên tránh vẽ ra ngoài
        if (y0_v1<graph_y0) y0_v1=graph_y0; if (y0_v1>graph_y0+graph_h-1) y0_v1=graph_y0+graph_h-1;
        if (y1_v1<graph_y0) y1_v1=graph_y0; if (y1_v1>graph_y0+graph_h-1) y1_v1=graph_y0+graph_h-1;
        if (y0_v2<graph_y0) y0_v2=graph_y0; if (y0_v2>graph_y0+graph_h-1) y0_v2=graph_y0+graph_h-1;
        if (y1_v2<graph_y0) y1_v2=graph_y0; if (y1_v2>graph_y0+graph_h-1) y1_v2=graph_y0+graph_h-1;
        if (y0_vd1<graph_y0) y0_vd1=graph_y0; if (y0_vd1>graph_y0+graph_h-1) y0_vd1=graph_y0+graph_h-1;
        if (y1_vd1<graph_y0) y1_vd1=graph_y0; if (y1_vd1>graph_y0+graph_h-1) y1_vd1=graph_y0+graph_h-1;
        if (y0_vd2<graph_y0) y0_vd2=graph_y0; if (y0_vd2>graph_y0+graph_h-1) y0_vd2=graph_y0+graph_h-1;
        if (y1_vd2<graph_y0) y1_vd2=graph_y0; if (y1_vd2>graph_y0+graph_h-1) y1_vd2=graph_y0+graph_h-1;

        lcdDrawLine(x0,y0_v1,  x1,y1_v1,  COLOR_RED);      // V1
        lcdDrawLine(x0,y0_v2,  x1,y1_v2,  COLOR_GREEN);    // V2
        lcdDrawLine(x0,y0_vd1, x1,y1_vd1, COLOR_BLUE);     // Vd1
        lcdDrawLine(x0,y0_vd2, x1,y1_vd2, COLOR_MAGENTA);  // Vd2
    }

    // 8) legend (chú giải)
    UI_SetText(COLOR_RED, UI_BG);     UI_PrintClipped(graph_x0 + 4,  graph_y0 + 4,  "V1");
    UI_SetText(COLOR_GREEN, UI_BG);   UI_PrintClipped(graph_x0 + 30, graph_y0 + 4,  "V2");
    UI_SetText(COLOR_BLUE, UI_BG);    UI_PrintClipped(graph_x0 + 56, graph_y0 + 4,  "Vd1");
    UI_SetText(COLOR_MAGENTA, UI_BG); UI_PrintClipped(graph_x0 + 90, graph_y0 + 4,  "Vd2");
}


static uint16_t midY;
static float yscale;

void Graph_DrawFrameOnce(void)
{
    // nền graph
    lcdFillRect(graph_x0, graph_y0, graph_w, graph_h, UI_BG);

    // khung
    lcdDrawRect(graph_x0, graph_y0, graph_w, graph_h, UI_FG);

    midY   = graph_y0 + graph_h/2;
    yscale = (float)(graph_h/2 - 2) / YMAX;   // chuẩn ±2

    // lưới ngang 4 ô (đường ở 1/4, 2/4, 3/4)
    uint16_t y_q1 = graph_y0 + graph_h/4;        // +1
    uint16_t y_q2 = graph_y0 + graph_h/2;        // 0
    uint16_t y_q3 = graph_y0 + 3*graph_h/4;      // -1

    lcdDrawLine(graph_x0, y_q1, graph_x0+graph_w-1, y_q1, GRID_COL);
    lcdDrawLine(graph_x0, y_q2, graph_x0+graph_w-1, y_q2, UI_FG);     // trục 0 đậm
    lcdDrawLine(graph_x0, y_q3, graph_x0+graph_w-1, y_q3, GRID_COL);

    // lưới dọc (tuỳ bạn, ví dụ mỗi 50px)
    for (int x = 50; x < graph_w; x += 50) {
        lcdDrawLine(graph_x0+x, graph_y0, graph_x0+x, graph_y0+graph_h-1, GRID_COL);
    }

    // nhãn trục Y: +2 +1 0 -1 -2
    UI_SetText(UI_FG, UI_BG);
    UI_PrintClipped(graph_x0 - 32, graph_y0 + 2,          "+2");
    UI_PrintClipped(graph_x0 - 32, y_q1 - 6,              "+1");
    UI_PrintClipped(graph_x0 - 32, midY - 6,              "0");
    UI_PrintClipped(graph_x0 - 32, y_q3 - 6,              "-1");
    UI_PrintClipped(graph_x0 - 32, graph_y0 + graph_h - 14,"-2");

    // nhãn time (không cần update)
    UI_PrintClipped(graph_x0, graph_y0 + graph_h + 2, "time ->");
}
static uint16_t Graph_BGColorAt(uint16_t x, uint16_t y)
{
    // border
    if (x == graph_x0 || x == graph_x0 + graph_w - 1 ||
        y == graph_y0 || y == graph_y0 + graph_h - 1) return UI_FG;

    // trục 0
    if (y == midY) return UI_FG;

    // lưới ngang
    if (y == (graph_y0 + graph_h/4) || y == (graph_y0 + 3*graph_h/4)) return GRID_COL;

    // lưới dọc (nếu bạn vẽ mỗi 50px)
    if (((x - graph_x0) % 50) == 0) return GRID_COL;

    return UI_BG;
}
void UI_DrawVelocityGraph_CurvesOnly(void)
{
    if (hist_len < 2) return;

    // 1) xoá đường cũ (vẽ lại bằng màu nền đúng)
    if (prev_valid) {
        for (int i = 0; i < (int)hist_len; i++) {
            uint16_t x = graph_x0 + i;

            // mỗi điểm xoá bằng pixel (nhanh và không phá lưới)
            uint16_t c;

            c = Graph_BGColorAt(x, (uint16_t)prev_y_v1[i]);  lcdDrawPixel(x, prev_y_v1[i], c);
            c = Graph_BGColorAt(x, (uint16_t)prev_y_v2[i]);  lcdDrawPixel(x, prev_y_v2[i], c);
            c = Graph_BGColorAt(x, (uint16_t)prev_y_vd1[i]); lcdDrawPixel(x, prev_y_vd1[i], c);
            c = Graph_BGColorAt(x, (uint16_t)prev_y_vd2[i]); lcdDrawPixel(x, prev_y_vd2[i], c);
        }
    }

    // 2) tính y mới theo scale ±2 (0 đúng giữa)
    for (int i = 0; i < (int)hist_len; i++) {
        int y1  = (int)(midY - (V1_hist[i]  * yscale));
        int y2  = (int)(midY - (V2_hist[i]  * yscale));
        int yd1 = (int)(midY - (VD1_hist[i] * yscale));
        int yd2 = (int)(midY - (VD2_hist[i] * yscale));

        // clamp trong khung
        int y_min = graph_y0 + 1;
        int y_max = graph_y0 + graph_h - 2;
        if (y1  < y_min) y1  = y_min; if (y1  > y_max) y1  = y_max;
        if (y2  < y_min) y2  = y_min; if (y2  > y_max) y2  = y_max;
        if (yd1 < y_min) yd1 = y_min; if (yd1 > y_max) yd1 = y_max;
        if (yd2 < y_min) yd2 = y_min; if (yd2 > y_max) yd2 = y_max;

        prev_y_v1[i]  = (int16_t)y1;
        prev_y_v2[i]  = (int16_t)y2;
        prev_y_vd1[i] = (int16_t)yd1;
        prev_y_vd2[i] = (int16_t)yd2;
    }

    // 3) vẽ đường mới (chỉ vẽ pixel / hoặc nối line nếu muốn)
    for (int i = 0; i < (int)hist_len; i++) {
        uint16_t x = graph_x0 + i;
        lcdDrawPixel(x, prev_y_v1[i],  COLOR_RED);
        lcdDrawPixel(x, prev_y_v2[i],  COLOR_GREEN);
        lcdDrawPixel(x, prev_y_vd1[i], COLOR_BLUE);
        lcdDrawPixel(x, prev_y_vd2[i], COLOR_MAGENTA);
    }

    prev_valid = 1;
}
static void Graph_DrawOneFrame(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h,
                               const char* title)
{
    lcdFillRect(x0, y0, w, h, UI_BG);
    lcdDrawRect(x0, y0, w, h, UI_FG);

    uint16_t midY = y0 + h/2;

    // 3 đường mức +1, 0, -1
    uint16_t y_p1 = y0 + h/4;
    uint16_t y_0  = midY;
    uint16_t y_m1 = y0 + 3*h/4;

    lcdDrawLine(x0, y_p1, x0+w-1, y_p1, GRID_COL);
    lcdDrawLine(x0, y_0,  x0+w-1, y_0,  UI_FG);
    lcdDrawLine(x0, y_m1, x0+w-1, y_m1, GRID_COL);

    // lưới dọc (mỗi 50 px)
    for (int dx = 50; dx < w; dx += 50) {
        lcdDrawLine(x0+dx, y0, x0+dx, y0+h-1, GRID_COL);
    }

    // nhãn trục Y (+2 +1 0 -1 -2)
    UI_SetText(UI_FG, UI_BG);
    UI_PrintClipped(x0 - 28, y0 + 2,        "+1");
    UI_PrintClipped(x0 - 28, y_p1 - 6,      "+0.5");
    UI_PrintClipped(x0 - 28, y_0  - 6,      "0");
    UI_PrintClipped(x0 - 28, y_m1 - 6,      "-0.5");
    UI_PrintClipped(x0 - 28, y0 + h - 14,   "-1");

    // title
    UI_PrintClipped(x0 + 2, y0 + 2, title);

    // trục thời gian t (chỉ ghi chữ 1 lần)
    UI_PrintClipped(x0 + w - 40, y0 + h - 14, "t->");
}
void UI_VelGraph_DrawFramesOnce(void)
{
    if (frame_drawn) return;
    frame_drawn = 1;

    // panel của bạn: g_ui.panel (x,y,w,h)
    // chừa trái 35px cho nhãn Y
    uint16_t left_margin = 35;
    uint16_t top_margin  = 10;
    uint16_t gap         = 8;

    uint16_t px = g_ui.panel.x + left_margin;
    uint16_t py = g_ui.panel.y + top_margin;
    uint16_t pw = g_ui.panel.w - left_margin - 5;
    uint16_t ph = g_ui.panel.h - top_margin - 5;

    // chia đôi theo chiều cao
    uint16_t each_h = (ph - gap) / 2;

    g1_x0 = px;         g1_y0 = py;                g1_w = pw; g1_h = each_h;
    g2_x0 = px;         g2_y0 = py + each_h + gap; g2_w = pw; g2_h = each_h;

    g1_midY = g1_y0 + g1_h/2;
    g2_midY = g2_y0 + g2_h/2;

    g1_yscale = (float)(g1_h/2 - 2) / YMAX;
    g2_yscale = (float)(g2_h/2 - 2) / YMAX;

    Graph_DrawOneFrame(g1_x0, g1_y0, g1_w, g1_h, "V1 & Vd1");
    Graph_DrawOneFrame(g2_x0, g2_y0, g2_w, g2_h, "V2 & Vd2");
}

static uint16_t Graph_BGColorAt_One(uint16_t x0,uint16_t y0,uint16_t w,uint16_t h,
                                   uint16_t x,uint16_t y)
{
    uint16_t midY = y0 + h/2;
    if (x == x0 || x == x0+w-1 || y == y0 || y == y0+h-1) return UI_FG;
    if (y == midY) return UI_FG;
    if (y == (y0 + h/4) || y == (y0 + 3*h/4)) return GRID_COL;
    if (((x - x0) % 50) == 0) return GRID_COL;
    return UI_BG;
}
void UI_VelGraph_UpdateCurvesOnly(void)
{
    if (!frame_drawn) return;

    // số điểm vẽ tối đa theo STEP_X
    uint16_t max_pts = g1_w / STEP_X;
    if (max_pts < 2) return;
    if (max_pts > GRAPH_MAX_W) max_pts = GRAPH_MAX_W;

    // số điểm thực sự sẽ vẽ (phụ thuộc dữ liệu đã có)
    uint16_t pts = len;
    if (pts > max_pts) pts = max_pts;
    if (pts < 2) return;
    // Clear sạch nền của 2 graph (giữ border + grid)
    Graph_ClearInner(g1_x0, g1_y0, g1_w, g1_h);
    Graph_ClearInner(g2_x0, g2_y0, g2_w, g2_h);

    // 1) xoá đường cũ (chỉ xoá tại vị trí điểm cũ để không phá lưới)
//    if (prev_valid) {
//        for (uint16_t i=0;i<pts;i++){
//            uint16_t x1 = g1_x0 + i*STEP_X;
//            uint16_t x2 = g2_x0 + i*STEP_X;
//
//            uint16_t c;
//            c = Graph_BGColorAt_One(g1_x0,g1_y0,g1_w,g1_h, x1, (uint16_t)prev_y_v1[i]);
//            lcdDrawPixel(x1, prev_y_v1[i], c);
//            c = Graph_BGColorAt_One(g1_x0,g1_y0,g1_w,g1_h, x1, (uint16_t)prev_y_vd1[i]);
//            lcdDrawPixel(x1, prev_y_vd1[i], c);
//
//            c = Graph_BGColorAt_One(g2_x0,g2_y0,g2_w,g2_h, x2, (uint16_t)prev_y_v2[i]);
//            lcdDrawPixel(x2, prev_y_v2[i], c);
//            c = Graph_BGColorAt_One(g2_x0,g2_y0,g2_w,g2_h, x2, (uint16_t)prev_y_vd2[i]);
//            lcdDrawPixel(x2, prev_y_vd2[i], c);
//        }
//    }

    // 2) tính y mới từ ring-buffer theo cửa sổ mới nhất
    int y_min1 = g1_y0 + 1, y_max1 = g1_y0 + g1_h - 2;
    int y_min2 = g2_y0 + 1, y_max2 = g2_y0 + g2_h - 2;

    // start_x trong "timeline": nếu len>pts thì bỏ bớt phần cũ để chỉ lấy pts mẫu mới nhất
    uint16_t start = (len > pts) ? (uint16_t)(len - pts) : 0;

    for (uint16_t i=0;i<pts;i++)
    {
        // mẫu thứ (start+i) tính theo oldest->newest
        uint16_t k = idx_from_x_runtime((uint16_t)(start + i));

        float v1  = V1_buf[k];
        float vd1 = VD1_buf[k];
        float v2  = V2_buf[k];
        float vd2 = VD2_buf[k];

        int yv1  = (int)(g1_midY - v1  * g1_yscale);
        int yvd1 = (int)(g1_midY - vd1 * g1_yscale);
        int yv2  = (int)(g2_midY - v2  * g2_yscale);
        int yvd2 = (int)(g2_midY - vd2 * g2_yscale);

        if (yv1  < y_min1) yv1  = y_min1; if (yv1  > y_max1) yv1  = y_max1;
        if (yvd1 < y_min1) yvd1 = y_min1; if (yvd1 > y_max1) yvd1 = y_max1;
        if (yv2  < y_min2) yv2  = y_min2; if (yv2  > y_max2) yv2  = y_max2;
        if (yvd2 < y_min2) yvd2 = y_min2; if (yvd2 > y_max2) yvd2 = y_max2;

        prev_y_v1[i]  = (int16_t)yv1;
        prev_y_vd1[i] = (int16_t)yvd1;
        prev_y_v2[i]  = (int16_t)yv2;
        prev_y_vd2[i] = (int16_t)yvd2;
    }

    // 3) vẽ line nối (đẹp + nhìn liên tục)
    for (uint16_t i=1;i<pts;i++){
        uint16_t x1a = g1_x0 + (i-1)*STEP_X, x1b = g1_x0 + i*STEP_X;
        uint16_t x2a = g2_x0 + (i-1)*STEP_X, x2b = g2_x0 + i*STEP_X;

        lcdDrawLine(x1a, prev_y_v1[i-1],  x1b, prev_y_v1[i],  COL_V1);
        lcdDrawLine(x1a, prev_y_vd1[i-1], x1b, prev_y_vd1[i], COL_VD1);

        lcdDrawLine(x2a, prev_y_v2[i-1],  x2b, prev_y_v2[i],  COL_V2);
        lcdDrawLine(x2a, prev_y_vd2[i-1], x2b, prev_y_vd2[i], COL_VD2);
    }

    prev_valid = 1;

}
static uint16_t g_idx = 0;
static uint8_t  g_has_prev = 0;
static int16_t  g_prev_y_v1, g_prev_y_vd1, g_prev_y_v2, g_prev_y_vd2;

static void Graph_ClearColumn(uint16_t x0,uint16_t y0,uint16_t w,uint16_t h, uint16_t x)
{
    for (uint16_t y = y0 + 1; y < y0 + h - 1; y++) {
        uint16_t c = Graph_BGColorAt_One(x0,y0,w,h, x, y);
        lcdDrawPixel(x, y, c);
    }
}
void Graph_ClearInner(uint16_t x0,uint16_t y0,uint16_t w,uint16_t h)
{
    // clear bên trong (chừa border 1px)
    lcdFillRect(x0+1, y0+1, w-2, h-2, UI_BG);

    // vẽ lại lưới ngang + trục 0 (cần)
    uint16_t midY = y0 + h/2;
    uint16_t y_p1 = y0 + h/4;
    uint16_t y_m1 = y0 + 3*h/4;

    lcdDrawLine(x0+1, y_p1, x0+w-2, y_p1, GRID_COL);
    lcdDrawLine(x0+1, midY, x0+w-2, midY, UI_FG);     // trục 0 đậm
    lcdDrawLine(x0+1, y_m1, x0+w-2, y_m1, GRID_COL);

    // nếu bạn cần lưới dọc thì vẽ lại luôn ở đây
    // (nếu thấy nặng thì bỏ lưới dọc cho sạch)
    // for (int dx = 50; dx < w; dx += 50) {
    //     lcdDrawLine(x0+dx, y0+1, x0+dx, y0+h-2, GRID_COL);
    // }
}

void UI_VelGraph_PencilStep(float v1, float vd1, float v2, float vd2)
{
    if (!frame_drawn) return;
    if (hist_len < 2) return;

    // x hiện tại (cột đang vẽ)
    uint16_t x1 = g1_x0 + g_idx;
    uint16_t x2 = g2_x0 + g_idx;

    // 1) xoá cột hiện tại để chuẩn bị vẽ (khi wrap sẽ ghi đè)
    Graph_ClearColumn(g1_x0, g1_y0, g1_w, g1_h, x1);
    Graph_ClearColumn(g2_x0, g2_y0, g2_w, g2_h, x2);

    // 2) tính y mới (clamp)
    int y_min1 = g1_y0 + 1, y_max1 = g1_y0 + g1_h - 2;
    int y_min2 = g2_y0 + 1, y_max2 = g2_y0 + g2_h - 2;

    int yv1  = (int)(g1_midY - v1  * g1_yscale);
    int yvd1 = (int)(g1_midY - vd1 * g1_yscale);
    int yv2  = (int)(g2_midY - v2  * g2_yscale);
    int yvd2 = (int)(g2_midY - vd2 * g2_yscale);

    if (yv1  < y_min1) yv1  = y_min1; if (yv1  > y_max1) yv1  = y_max1;
    if (yvd1 < y_min1) yvd1 = y_min1; if (yvd1 > y_max1) yvd1 = y_max1;
    if (yv2  < y_min2) yv2  = y_min2; if (yv2  > y_max2) yv2  = y_max2;
    if (yvd2 < y_min2) yvd2 = y_min2; if (yvd2 > y_max2) yvd2 = y_max2;

    // 3) vẽ giống “bút”: nối từ idx-1 -> idx (chỉ 1 đoạn)
    if (g_has_prev && g_idx != 0) {

        // V1 & Vd1 (graph trên)
        lcdDrawLine(x1-1, g_prev_y_v1,  x1, (uint16_t)yv1,  COL_V1);
        lcdDrawLine(x1-1, g_prev_y_vd1, x1, (uint16_t)yvd1, COL_VD1);

        // V2 & Vd2 (graph dưới)
        lcdDrawLine(x2-1, g_prev_y_v2,  x2, (uint16_t)yv2,  COL_V2);
        lcdDrawLine(x2-1, g_prev_y_vd2, x2, (uint16_t)yvd2, COL_VD2);

        // 👉 Làm dày thêm 1 pixel trên & dưới
        draw_thick_pixel(x1, (uint16_t)yv1,  COL_V1);
        draw_thick_pixel(x1, (uint16_t)yvd1, COL_VD1);
        draw_thick_pixel(x2, (uint16_t)yv2,  COL_V2);
        draw_thick_pixel(x2, (uint16_t)yvd2, COL_VD2);

    } else {
        // điểm đầu tiên (hoặc lúc wrap)
        draw_thick_pixel(x1, (uint16_t)yv1,  COL_V1);
        draw_thick_pixel(x1, (uint16_t)yvd1, COL_VD1);
        draw_thick_pixel(x2, (uint16_t)yv2,  COL_V2);
        draw_thick_pixel(x2, (uint16_t)yvd2, COL_VD2);
    }


    // lưu prev để lần sau nối line
    g_prev_y_v1  = (int16_t)yv1;
    g_prev_y_vd1 = (int16_t)yvd1;
    g_prev_y_v2  = (int16_t)yv2;
    g_prev_y_vd2 = (int16_t)yvd2;
    g_has_prev = 1;

    // 4) tăng cursor, hết thì quay về trái và reset prev để khỏi nối “từ phải về trái”
    g_idx++;
    if (g_idx >= hist_len) {
    	first = 1;
        g_idx = 0;
        g_has_prev = 0;
        prev_valid = 0;
    }
}


