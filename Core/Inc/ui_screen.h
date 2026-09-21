#ifndef INC_UI_SCREEN_H_
#define INC_UI_SCREEN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
void UI_Graph_Push(float v1, float v2, float vd1, float vd2);
void UI_DrawVelocityGraph(void);

#define ODOM_LABEL_X     20
#define ODOM_VALUE_X     120
void Graph_Push(float v1,float v2,float vd1,float vd2);
void Graph_Reset(void);
void UI_VelGraph_UpdateCurvesOnly(void);
void UI_VelGraph_DrawFramesOnce(void);

#define ODOM_X_Y         40
#define ODOM_Y_Y         80
#define ODOM_THETA_Y     120

typedef struct {
  uint16_t x;
  uint16_t y;
  uint16_t w;
  uint16_t h;
} Rect_t;
void UI_UpdateEncoderPage(int32_t enc1, int32_t enc2,
                          float v1, float v2,
                          float v1d, float v2d,
                          float pwm1, float pwm2);

void UI_UpdateOdomPage(float x, float y, float yaw);
void UI_UpdatePIDPage(void);
void Graph_ClearInner(uint16_t x0,uint16_t y0,uint16_t w,uint16_t h);

typedef enum {
  UI_PAGE_INFO = 0,
  UI_PAGE_1 ,
  UI_PAGE_2,
  UI_PAGE_3,
  UI_PAGE_4,
  UI_PAGE_5,
  UI_PAGE_COUNT
} UI_Page_t;

//static char odom_str[32];
// ====== CONFIG ======
// Bạn đang dùng LANDSCAPE => width=320 height=240
#define UI_LCD_W 320
#define UI_LCD_H 240

// Layout theo hình mới
#define UI_HEADER_H   26
#define UI_LEFT_W     80

// 5 buttons chia đều vùng dưới header
#define UI_BTN_COUNT  5

// màu
#define UI_BG         COLOR_WHITE
#define UI_FG         COLOR_BLACK
#define UI_BTN_ACTIVE COLOR_YELLOW   // nếu colors.h không có COLOR_SILVER -> đổi sang COLOR_GRAY/COLOR_LIGHTGRAY
#define UI_BTN_IDLE   COLOR_WHITE

static int pid_selected = 0;
static int pid_step = 1;
static Rect_t pid_row[3];
static Rect_t pid_btn_left;
static Rect_t pid_btn_mid;
static Rect_t pid_btn_right;


typedef struct {
  Rect_t header;
  Rect_t btn[UI_BTN_COUNT];
  Rect_t panel;
  UI_Page_t active;
  uint8_t inited;
  uint8_t last_pressed;
} UI_State_t;



typedef struct {
  uint16_t x, y;
} UI_TextPos_t;

// Vẽ UI tĩnh 1 lần + show page mặc định
void UI_InitAndDrawOnce(void);

// Gọi liên tục trong task để bắt touch
void UI_OnTouch(uint16_t x, uint16_t y, uint8_t pressed);

// Ép hiển thị trang (clear panel bên phải + vẽ nội dung)
void UI_ShowPage(UI_Page_t page);

// Trang hiện tại
UI_Page_t UI_GetActivePage(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_UI_SCREEN_H_ */
