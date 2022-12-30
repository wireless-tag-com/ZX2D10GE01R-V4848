#pragma once

// RGB SPI interface
#define LCD_SPI_DATA0     41  /*!< for 1-line SPI, this also refered as MOSI */
#define LCD_SPI_CLK       47
#define LCD_SPI_CS        21
#define LCD_SPI_DC        -1
#define LCD_SPI_RST       -1

// RGB interface
#define LCD_PCLK_GPIO     (45)
#define LCD_VSYNC_GPIO    (48)
#define LCD_HSYNC_GPIO    (40)
#define LCD_DE_GPIO       (39)

#define RGB_B0       (47)  // B0
#define RGB_B1       (41)  // B1
#define RGB_B2       (0)   // B2
#define RGB_B3       (42)  // B3
#define RGB_B4       (14)  // B4
#define RGB_G0       (8)   // G0
#define RGB_G1       (13)  // G1
#define RGB_G2       (18)  // G2
#define RGB_G3       (12)  // G3
#define RGB_G4       (11)  // G4
#define RGB_G5      (17)  // G5
#define RGB_R0      (10)  // R0
#define RGB_R1      (16)  // R1
#define RGB_R2      (9)   // R2
#define RGB_R3      (15)  // R3
#define RGB_R4      (46)  // R4

#define LCD_DATA0_GPIO    (RGB_B0)   // B0
#define LCD_DATA1_GPIO    (RGB_B1)   // B1
#define LCD_DATA2_GPIO    (RGB_B2)   // B2
#define LCD_DATA3_GPIO    (RGB_B3)   // B3
#define LCD_DATA4_GPIO    (RGB_B4)   // B4
#define LCD_DATA5_GPIO    (RGB_G0)   // G0
#define LCD_DATA6_GPIO    (RGB_G1)    // G1
#define LCD_DATA7_GPIO    (RGB_G2)   // G2
#define LCD_DATA8_GPIO    (RGB_G3)   // G3
#define LCD_DATA9_GPIO    (RGB_G4)   // G4
#define LCD_DATA10_GPIO   (RGB_G5)   // G5
#define LCD_DATA11_GPIO   (RGB_R0)   // R0
#define LCD_DATA12_GPIO   (RGB_R1)    // R1
#define LCD_DATA13_GPIO   (RGB_R2)    // R2
#define LCD_DATA14_GPIO   (RGB_R3)    // R3
#define LCD_DATA15_GPIO   (RGB_R4)    // R4
#define LCD_DISP_EN_GPIO  (-1)

#define LCD_PIN_BK_LIGHT       38

#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL

#define ENCODER_A_PIN     (5)
#define ENCODER_B_PIN     (6)

#define MOTOR_PIN         (7)
#define LED_PIN           (4)

#define BTN_PIN           (3)

#define EXT_PIN_0         (1)
#define EXT_PIN_1         (2)
#define EXT_PIN_2         (20)
#define EXT_PIN_3         (19)
