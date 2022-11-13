#include <util.h>
#include <adb.h>
#include <usb.h>
#include <libopencmsis/core_cm3.h>
#include <libopencm3/cm3/dwt.h>

static const char * banner =
  "   ADB Terminal STM32\r\n(c) 2019 Simon Stapleton\r\n\r\n> ";

static const char * usage = "ADB Line on PA15\r\nCommands :\r\nR        : ADB Bus Reset\r\nF        : ADB Flush\r\ndtr  : Device d Talk Register r\r\ndlrx : Device d Listen Register r Data x\r\nP        : Poll last command until ^C\r\n%%        : Toggle binary output\r\n#        : Toggle hex output\r\n@       : Toggle bit timings\r\n<return> : Repeat last command\r\n\r\n";

char    g_cmd[20] = "2t3\0                ";
uint8_t g_cmd_idx = 0;
bool    g_cmd_poll = false;


static void adb_tester (void) {
  // Set our GPIO to output mode
  gpio_set_mode  (GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
  gpio_clear     (GPIOA, GPIO15);
  usleep         (100);
  gpio_set       (GPIOA, GPIO15);
  usleep         (200);
  gpio_clear     (GPIOA, GPIO15);
  usleep         (300);
  gpio_set       (GPIOA, GPIO15);
  usleep         (400);
  gpio_clear     (GPIOA, GPIO15);
  usleep         (500);
  gpio_set       (GPIOA, GPIO15);
  usleep         (600);
  gpio_clear     (GPIOA, GPIO15);
  usleep         (700);
  gpio_set       (GPIOA, GPIO15);
  usleep         (800);
  gpio_clear     (GPIOA, GPIO15);
  usleep         (100);
  gpio_set       (GPIOA, GPIO15);
  usleep         (200);
  gpio_clear     (GPIOA, GPIO15);
  usleep         (300);
  gpio_set       (GPIOA, GPIO15);
  usleep         (400);
  gpio_clear     (GPIOA, GPIO15);
  usleep         (500);
  gpio_set       (GPIOA, GPIO15);
  usleep         (600);
  gpio_clear     (GPIOA, GPIO15);
  usleep         (700);
  gpio_set       (GPIOA, GPIO15);
  usleep         (800);
  gpio_clear     (GPIOA, GPIO15);
}


static signed char getc (void) {
  while (usb_vcp_avail() == 0) {
    __WFI();
  }
  signed char c = usb_vcp_recv_byte();

  // -1 means dropped connection
  if (-1 == c) return c;

  // Echo the charcter back
  usb_vcp_send_byte (c);
  // Add a newline on carriage return
  if ('\r' == c)
    usb_vcp_send_byte('\n');
  
  return c;
}

static uint8_t htoi (char the_char) {
  if (the_char <= '9') 
    return the_char - '0';
  if (the_char <= 'F')
    return the_char - 'A' + 10;
  else
    return the_char - 'a' + 10;
}

static char itoh (uint8_t i) {
  if (i >= 10)
    return 'a' + i - 10;
  else
    return '0' + i;
}

static bool get_command (void) {
  g_cmd_idx = 0;
  while (1) {
    signed char c = getc();

    // Dropped connection?
    if (-1 == c) return false;
    // User hit return, run the command
    if ('\r' == c) return true;
    // backspace, not sure if this works
    if (('\b' == c)  && (0 < g_cmd_idx)) {
      g_cmd[--g_cmd_idx] = 0;
    } else if ('p' == c) {
      g_cmd_poll = !g_cmd_poll;
    } else {
      g_cmd[g_cmd_idx++] = c;
      g_cmd[g_cmd_idx] = 0;

      // Auto-run command at 19 characters
      if (19 == g_cmd_idx)
	return true;
    }
  }
}   


static void start_command (void) {
  uint8_t cmd = 0;
  switch (g_cmd[0]) {
  case 'h':
  case 'H':
  case '?':
    usb_vcp_printf(usage);
    return;
  default:
    break;
  }
  switch (g_cmd[1]) {
  case 'l':
  case 'L':
    cmd = 0x08;
    break;
  case 't':
  case 'T':
    cmd = 0x0a;
    break;
  case '#':
  case '%':
  case '@':
    usb_vcp_printf(usage);
    return;
  default:
    break;
  }
  cmd |= htoi(g_cmd[0]) << 4;
  cmd |= htoi(g_cmd[2]);

  usb_vcp_printf("send command 0x%x, press a key\r\n", cmd);
  while (usb_vcp_avail() != 0) getc();

  adb_send_command(cmd);

  usb_vcp_printf(" ... sent\r\n", cmd);
  
  g_cmd_idx = 0;
}

int main (void) {

  // Set up clock and usb prescaler.
  rcc_clock_setup_in_hse_8mhz_out_72mhz ();
  rcc_set_usbpre (RCC_CFGR_USBPRE_PLL_CLK_DIV1_5);

  rcc_periph_clock_enable (RCC_TIM2);
  rcc_periph_clock_enable (RCC_GPIOA);
  rcc_periph_clock_enable (RCC_GPIOB);
  rcc_periph_clock_enable (RCC_AFIO);
  rcc_periph_clock_enable (RCC_DMA1);

  dwt_enable_cycle_counter();

  usb_vcp_init();
  adb_common_setup();

  while (1) {
    // Wait for connect
    while (!usb_vcp_is_connected()) __WFI();
    
    usb_vcp_printf(banner);
    while (1) {
      adb_send_command(0x3a);
      usleep(2000);
    }
      
    while (get_command()) {
      start_command();
      //adb_tester();
    }
  }
}
