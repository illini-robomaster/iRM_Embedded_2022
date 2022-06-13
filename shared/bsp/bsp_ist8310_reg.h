#pragma once

#define IST8310_WHO_AM_I        0x00
#define IST8310_WHO_AM_I_VALUE  0x10 // device ID

#define IST8310_STAT1           0x02

#define IST8310_DATA            0x03 // 0x03 - 0x08 contains the XYZ measurement

#define IST8310_STAT2           0x09

#define IST8310_CNTL1           0x0A
#define IST8310_STANDBY         0x0
#define IST8310_SINGLE_MEAS     0x1

#define IST8310_CNTL2           0x0B
#define IST8310_DREN            (1 << 3)  // data ready interrupt enabled
#define IST8310_DRP             (1 << 2)  // data ready pin polarity
#define IST8310_SRST            (1 << 0)  // power on reset

#define IST8310_TEMP            0x1C // 0x1C - 0x1D contains temperature data

#define IST8310_AVGCNTL         0x41
#define IST8310_Y_AVG_0         (0b000 << 3)
#define IST8310_Y_AVG_2         (0b001 << 3)
#define IST8310_Y_AVG_4         (0b010 << 3)
#define IST8310_Y_AVG_8         (0b011 << 3)
#define IST8310_Y_AVG_16        (0b100 << 3)
#define IST8310_XZ_AVG_0        0b000
#define IST8310_XZ_AVG_2        0b001
#define IST8310_XZ_AVG_4        0b010
#define IST8310_XZ_AVG_8        0b011
#define IST8310_XZ_AVG_16       0b100

#define IST8310_PDCNTL          0x42
#define IST8310_PD_NORMAL       (0b11 << 6) // doc says: please use this setting
#define IST8310_PD_LONG         (0b01 << 6)

#define IST8310_CROSS_AXIS      0x9C // 0x9C - 0xAD contains the cross-axis compensation data
#define IST8310_CROSS_AXIS_C    (1000. / 3.) // cross-axis compensation constants
