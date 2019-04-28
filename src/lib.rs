//! Driver for the Bosch BMI160 accelerometer.
//!
//! <https://www.bosch-sensortec.com/bst/products/all_products/bmi160>
//!
//! > The BMI160 is a small, low power, low noise 16-bit inertial measurement unit designed
//! > for use in mobile applications like augmented reality or indoor navigation which require
//! > highly accurate, real-time sensor data.
//! >
//! > In full operation mode, with both the accelerometer and gyroscope enabled, the current
//! > consumption is typically 950 μA, enabling always-on applications in battery driven devices.
//! > It is available in a compact 14-pin 2.5 x 3.0 x 0.8 mm³ LGA package.

// Buffer to use for I2C messages
pub static mut BUFFER: [u8; 4] = [0; 4];

// Mask definitions
const ACCEL_BW_MASK: u8 = 0x70;
const ACCEL_ODR_MASK: u8 = 0x0F;
const ACCEL_UNDERSAMPLING_MASK: u8 = 0x80;
const ACCEL_RANGE_MASK: u8 = 0x0F;
const GYRO_BW_MASK: u8 = 0x30;
const GYRO_ODR_MASK: u8 = 0x0F;
const GYRO_RANGE_MSK: u8 = 0x07;

// Mask definitions for INT_EN registers */
const ANY_MOTION_X_INT_EN_MASK: u8 = 0x01;
const HIGH_G_X_INT_EN_MASK: u8 = 0x01;
const NO_MOTION_X_INT_EN_MASK: u8 = 0x01;
const ANY_MOTION_Y_INT_EN_MASK: u8 = 0x02;
const HIGH_G_Y_INT_EN_MASK: u8 = 0x02;
const NO_MOTION_Y_INT_EN_MASK: u8 = 0x02;
const ANY_MOTION_Z_INT_EN_MASK: u8 = 0x04;
const HIGH_G_Z_INT_EN_MASK: u8 = 0x04;
const NO_MOTION_Z_INT_EN_MASK: u8 = 0x04;
const SIG_MOTION_INT_EN_MASK: u8 = 0x07;
const ANY_MOTION_ALL_INT_EN_MASK: u8 = 0x07;
const STEP_DETECT_INT_EN_MASK: u8 = 0x08;
const DOUBLE_TAP_INT_EN_MASK: u8 = 0x10;
const SINGLE_TAP_INT_EN_MASK: u8 = 0x20;
const FIFO_FULL_INT_EN_MASK: u8 = 0x20;
const ORIENT_INT_EN_MASK: u8 = 0x40;
const FIFO_WATERMARK_INT_EN_MASK: u8 = 0x40;
const LOW_G_INT_EN_MASK: u8 = 0x08;
const STEP_DETECT_EN_MASK: u8 = 0x08;
const FLAT_INT_EN_MASK: u8 = 0x80;
const DATA_RDY_INT_EN_MASK: u8 = 0x10;

/** PMU status Macros */
const AUX_PMU_SUSPEND: u8 = 0x00;
const AUX_PMU_NORMAL: u8 = 0x01;
const AUX_PMU_LOW_POWER: u8 = 0x02;

const GYRO_PMU_SUSPEND: u8 = 0x00;
const GYRO_PMU_NORMAL: u8 = 0x01;
const GYRO_PMU_FSU: u8 = 0x03;

const ACCEL_PMU_SUSPEND: u8 = 0x00;
const ACCEL_PMU_NORMAL: u8 = 0x01;
const ACCEL_PMU_LOW_POWER: u8 = 0x02;

/** Mask definitions for INT_OUT_CTRL register */
const INT1_EDGE_CTRL_MASK: u8 = 0x01;
const INT1_OUTPUT_MODE_MASK: u8 = 0x04;
const INT1_OUTPUT_TYPE_MASK: u8 = 0x02;
const INT1_OUTPUT_EN_MASK: u8 = 0x08;
const INT2_EDGE_CTRL_MASK: u8 = 0x10;
const INT2_OUTPUT_MODE_MASK: u8 = 0x40;
const INT2_OUTPUT_TYPE_MASK: u8 = 0x20;
const INT2_OUTPUT_EN_MASK: u8 = 0x80;

/** Mask definitions for INT_LATCH register */
const INT1_INPUT_EN_MASK: u8 = 0x10;
const INT2_INPUT_EN_MASK: u8 = 0x20;
const INT_LATCH_MASK: u8 = 0x0F;

/** Mask definitions for INT_MAP register */
const INT1_LOW_G_MASK: u8 = 0x01;
const INT1_HIGH_G_MASK: u8 = 0x02;
const INT1_SLOPE_MASK: u8 = 0x04;
const INT1_NO_MOTION_MASK: u8 = 0x08;
const INT1_DOUBLE_TAP_MASK: u8 = 0x10;
const INT1_SINGLE_TAP_MASK: u8 = 0x20;
const INT1_FIFO_FULL_MASK: u8 = 0x20;
const INT1_FIFO_WM_MASK: u8 = 0x40;
const INT1_ORIENT_MASK: u8 = 0x40;
const INT1_FLAT_MASK: u8 = 0x80;
const INT1_DATA_READY_MASK: u8 = 0x80;
const INT2_LOW_G_MASK: u8 = 0x01;
const INT1_LOW_STEP_DETECT_MASK: u8 = 0x01;
const INT2_LOW_STEP_DETECT_MASK: u8 = 0x01;
const INT2_HIGH_G_MASK: u8 = 0x02;
const INT2_FIFO_FULL_MASK: u8 = 0x02;
const INT2_FIFO_WM_MASK: u8 = 0x04;
const INT2_SLOPE_MASK: u8 = 0x04;
const INT2_DATA_READY_MASK: u8 = 0x08;
const INT2_NO_MOTION_MASK: u8 = 0x08;
const INT2_DOUBLE_TAP_MASK: u8 = 0x10;
const INT2_SINGLE_TAP_MASK: u8 = 0x20;
const INT2_ORIENT_MASK: u8 = 0x40;
const INT2_FLAT_MASK: u8 = 0x80;

/** Mask definitions for INT_DATA register */
const TAP_SRC_INT_MASK: u8 = 0x08;
const LOW_HIGH_SRC_INT_MASK: u8 = 0x80;
const MOTION_SRC_INT_MASK: u8 = 0x80;

/** Mask definitions for INT_MOTION register */
const SLOPE_INT_DUR_MASK: u8 = 0x03;
const NO_MOTION_INT_DUR_MASK: u8 = 0xFC;
const NO_MOTION_SEL_BIT_MASK: u8 = 0x01;

/** Mask definitions for INT_TAP register */
const TAP_DUR_MASK: u8 = 0x07;
const TAP_SHOCK_DUR_MASK: u8 = 0x40;
const TAP_QUIET_DUR_MASK: u8 = 0x80;
const TAP_THRES_MASK: u8 = 0x1F;

/** Mask definitions for INT_FLAT register */
const FLAT_THRES_MASK: u8 = 0x3F;
const FLAT_HOLD_TIME_MASK: u8 = 0x30;
const FLAT_HYST_MASK: u8 = 0x07;

/** Mask definitions for INT_LOWHIGH register */
const LOW_G_HYST_MASK: u8 = 0x03;
const LOW_G_LOW_MODE_MASK: u8 = 0x04;
const HIGH_G_HYST_MASK: u8 = 0xC0;

/** Mask definitions for INT_SIG_MOTION register */
const SIG_MOTION_SEL_MASK: u8 = 0x02;
const SIG_MOTION_SKIP_MASK: u8 = 0x0C;
const SIG_MOTION_PROOF_MASK: u8 = 0x30;

/** Mask definitions for INT_ORIENT register */
const ORIENT_MODE_MASK: u8 = 0x03;
const ORIENT_BLOCK_MASK: u8 = 0x0C;
const ORIENT_HYST_MASK: u8 = 0xF0;
const ORIENT_THETA_MASK: u8 = 0x3F;
const ORIENT_UD_ENABLE: u8 = 0x40;
const AXES_EN_MASK: u8 = 0x80;

/** Mask definitions for FIFO_CONFIG register */
const FIFO_GYRO: u8 = 0x80;
const FIFO_ACCEL: u8 = 0x40;
const FIFO_AUX: u8 = 0x20;
const FIFO_TAG_INT1: u8 = 0x08;
const FIFO_TAG_INT2: u8 = 0x04;
const FIFO_TIME: u8 = 0x02;
const FIFO_HEADER: u8 = 0x10;
const FIFO_CONFIG_1_MASK: u8 = 0xFE;

/** Mask definitions for STEP_CONF register */
const STEP_COUNT_EN_BIT_MASK: u8 = 0x08;
const STEP_DETECT_MIN_THRES_MASK: u8 = 0x18;
const STEP_DETECT_STEPTIME_MIN_MASK: u8 = 0x07;
const STEP_MIN_BUF_MASK: u8 = 0x07;

/** Mask definition for FIFO Header Data Tag */
const FIFO_TAG_INTR_MASK: u8 = 0xFC;

/** Fifo byte counter mask definitions */
const FIFO_BYTE_COUNTER_MASK: u8 = 0x07;

/** Enable/disable bit value */
const ENABLE: u8 = 0x01;
const DISABLE: u8 = 0x00;

/** Latch Duration */
const LATCH_DUR_NONE: u8 = 0x00;
const LATCH_DUR_312_5_MICRO_SEC: u8 = 0x01;
const LATCH_DUR_625_MICRO_SEC: u8 = 0x02;
const LATCH_DUR_1_25_MILLI_SEC: u8 = 0x03;
const LATCH_DUR_2_5_MILLI_SEC: u8 = 0x04;
const LATCH_DUR_5_MILLI_SEC: u8 = 0x05;
const LATCH_DUR_10_MILLI_SEC: u8 = 0x06;
const LATCH_DUR_20_MILLI_SEC: u8 = 0x07;
const LATCH_DUR_40_MILLI_SEC: u8 = 0x08;
const LATCH_DUR_80_MILLI_SEC: u8 = 0x09;
const LATCH_DUR_160_MILLI_SEC: u8 = 0x0A;
const LATCH_DUR_320_MILLI_SEC: u8 = 0x0B;
const LATCH_DUR_640_MILLI_SEC: u8 = 0x0C;
const LATCH_DUR_1_28_SEC: u8 = 0x0D;
const LATCH_DUR_2_56_SEC: u8 = 0x0E;
const LATCHED: u8 = 0x0F;

/** Error code definitions */
const OK: i8 = 0;
const E_NULL_PTR: i8 = -1;
const E_COM_FAIL: i8 = -2;
const E_DEV_NOT_FOUND: i8 = -3;
const E_OUT_OF_RANGE: i8 = -4;
const E_INVALID_INPUT: i8 = -5;
const E_ACCEL_ODR_BW_INVALID: i8 = -6;
const E_GYRO_ODR_BW_INVALID: i8 = -7;
const E_LWP_PRE_FLTR_INT_INVALID: i8 = -8;
const E_LWP_PRE_FLTR_INVALID: i8 = -9;
const E_AUX_NOT_FOUND: i8 = -10;
const FOC_FAILURE: i8 = -11;

/**\name API warning codes */
const W_GYRO_SELF_TEST_FAIL: i8 = 1;
const W_ACCEl_SELF_TEST_FAIL: i8 = 2;

/** BMI160 unique chip identifier */
const CHIP_ID: u8 = 0xD1;

/** Soft reset command */
const SOFT_RESET_CMD: u8 = 0xb6;
const SOFT_RESET_DELAY_MS: u8 = 15;
/** Start FOC command */
const START_FOC_CMD: u8 = 0x03;
/** NVM backup enabling command */
const NVM_BACKUP_EN: u8 = 0xA0;

/* Delay in ms settings */
const ACCEL_DELAY_MS: u8 = 5;
const GYRO_DELAY_MS: u8 = 81;
const ONE_MS_DELAY: u8 = 1;
const AUX_COM_DELAY: u8 = 10;
const GYRO_SELF_TEST_DELAY: u8 = 20;
const ACCEL_SELF_TEST_DELAY: u8 = 50;

/** Self test configurations */
const ACCEL_SELF_TEST_CONFIG: u8 = 0x2C;
const ACCEL_SELF_TEST_POSITIVE_EN: u8 = 0x0D;
const ACCEL_SELF_TEST_NEGATIVE_EN: u8 = 0x09;
const ACCEL_SELF_TEST_LIMIT: u16 = 8192;

/** Power mode settings */
/* Accel power mode */
const ACCEL_NORMAL_MODE: u8 = 0x11;
const ACCEL_LOWPOWER_MODE: u8 = 0x12;
const ACCEL_SUSPEND_MODE: u8 = 0x10;

/* Gyro power mode */
const GYRO_SUSPEND_MODE: u8 = 0x14;
const GYRO_NORMAL_MODE: u8 = 0x15;
const GYRO_FASTSTARTUP_MODE: u8 = 0x17;

/* Aux power mode */
const AUX_SUSPEND_MODE: u8 = 0x18;
const AUX_NORMAL_MODE: u8 = 0x19;
const AUX_LOWPOWER_MODE: u8 = 0x1A;

/** Range settings */
/* Accel Range */
const ACCEL_RANGE_2G: u8 = 0x03;
const ACCEL_RANGE_4G: u8 = 0x05;
const ACCEL_RANGE_8G: u8 = 0x08;
const ACCEL_RANGE_16G: u8 = 0x0C;

/* Gyro Range */
const GYRO_RANGE_2000_DPS: u8 = 0x00;
const GYRO_RANGE_1000_DPS: u8 = 0x01;
const GYRO_RANGE_500_DPS: u8 = 0x02;
const GYRO_RANGE_250_DPS: u8 = 0x03;
const GYRO_RANGE_125_DPS: u8 = 0x04;

/** Bandwidth settings */
/* Accel Bandwidth */
const ACCEL_BW_OSR4_AVG1: u8 = 0x00;
const ACCEL_BW_OSR2_AVG2: u8 = 0x01;
const ACCEL_BW_NORMAL_AVG4: u8 = 0x02;
const ACCEL_BW_RES_AVG8: u8 = 0x03;
const ACCEL_BW_RES_AVG16: u8 = 0x04;
const ACCEL_BW_RES_AVG32: u8 = 0x05;
const ACCEL_BW_RES_AVG64: u8 = 0x06;
const ACCEL_BW_RES_AVG128: u8 = 0x07;

const GYRO_BW_OSR4_MODE: u8 = 0x00;
const GYRO_BW_OSR2_MODE: u8 = 0x01;
const GYRO_BW_NORMAL_MODE: u8 = 0x02;

/* Output Data Rate settings */
/* Accel Output data rate */
const ACCEL_ODR_RESERVED: u8 = 0x00;
const ACCEL_ODR_0_78HZ: u8 = 0x01;
const ACCEL_ODR_1_56HZ: u8 = 0x02;
const ACCEL_ODR_3_12HZ: u8 = 0x03;
const ACCEL_ODR_6_25HZ: u8 = 0x04;
const ACCEL_ODR_12_5HZ: u8 = 0x05;
const ACCEL_ODR_25HZ: u8 = 0x06;
const ACCEL_ODR_50HZ: u8 = 0x07;
const ACCEL_ODR_100HZ: u8 = 0x08;
const ACCEL_ODR_200HZ: u8 = 0x09;
const ACCEL_ODR_400HZ: u8 = 0x0A;
const ACCEL_ODR_800HZ: u8 = 0x0B;
const ACCEL_ODR_1600HZ: u8 = 0x0C;
const ACCEL_ODR_RESERVED0: u8 = 0x0D;
const ACCEL_ODR_RESERVED1: u8 = 0x0E;
const ACCEL_ODR_RESERVED2: u8 = 0x0F;

/* Gyro Output data rate */
const GYRO_ODR_RESERVED: u8 = 0x00;
const GYRO_ODR_25HZ: u8 = 0x06;
const GYRO_ODR_50HZ: u8 = 0x07;
const GYRO_ODR_100HZ: u8 = 0x08;
const GYRO_ODR_200HZ: u8 = 0x09;
const GYRO_ODR_400HZ: u8 = 0x0A;
const GYRO_ODR_800HZ: u8 = 0x0B;
const GYRO_ODR_1600HZ: u8 = 0x0C;
const GYRO_ODR_3200HZ: u8 = 0x0D;

/* Auxiliary sensor Output data rate */
const AUX_ODR_RESERVED: u8 = 0x00;
const AUX_ODR_0_78HZ: u8 = 0x01;
const AUX_ODR_1_56HZ: u8 = 0x02;
const AUX_ODR_3_12HZ: u8 = 0x03;
const AUX_ODR_6_25HZ: u8 = 0x04;
const AUX_ODR_12_5HZ: u8 = 0x05;
const AUX_ODR_25HZ: u8 = 0x06;
const AUX_ODR_50HZ: u8 = 0x07;
const AUX_ODR_100HZ: u8 = 0x08;
const AUX_ODR_200HZ: u8 = 0x09;
const AUX_ODR_400HZ: u8 = 0x0A;
const AUX_ODR_800HZ: u8 = 0x0B;

/* Maximum limits definition */
const ACCEL_ODR_MAX: u8 = 15;
const ACCEL_BW_MAX: u8 = 2;
const ACCEL_RANGE_MAX: u8 = 12;
const GYRO_ODR_MAX: u8 = 13;
const GYRO_BW_MAX: u8 = 2;
const GYRO_RANGE_MAX: u8 = 4;

/** FIFO_CONFIG Definitions */
const FIFO_TIME_ENABLE: u8 = 0x02;
const FIFO_TAG_INT2_ENABLE: u8 = 0x04;
const FIFO_TAG_INT1_ENABLE: u8 = 0x08;
const FIFO_HEAD_ENABLE: u8 = 0x10;
const FIFO_M_ENABLE: u8 = 0x20;
const FIFO_A_ENABLE: u8 = 0x40;
const FIFO_M_A_ENABLE: u8 = 0x60;
const FIFO_G_ENABLE: u8 = 0x80;
const FIFO_M_G_ENABLE: u8 = 0xA0;
const FIFO_G_A_ENABLE: u8 = 0xC0;
const FIFO_M_G_A_ENABLE: u8 = 0xE0;

/* Macro to specify the number of bytes over-read from the
 * FIFO in order to get the sensor time at the end of FIFO */
const FIFO_BYTES_OVERREAD: u8 = 25;

/* Accel, gyro and aux. sensor length and also their combined
 * length definitions in FIFO */
const FIFO_G_LENGTH: u8 = 6;
const FIFO_A_LENGTH: u8 = 6;
const FIFO_M_LENGTH: u8 = 8;
const FIFO_GA_LENGTH: u8 = 12;
const FIFO_MA_LENGTH: u8 = 14;
const FIFO_MG_LENGTH: u8 = 14;
const FIFO_MGA_LENGTH: u8 = 20;

/** FIFO Header Data definitions */
const FIFO_HEAD_SKIP_FRAME: u8 = 0x40;
const FIFO_HEAD_SENSOR_TIME: u8 = 0x44;
const FIFO_HEAD_INPUT_CONFIG: u8 = 0x48;
const FIFO_HEAD_OVER_READ: u8 = 0x80;
const FIFO_HEAD_A: u8 = 0x84;
const FIFO_HEAD_G: u8 = 0x88;
const FIFO_HEAD_G_A: u8 = 0x8C;
const FIFO_HEAD_M: u8 = 0x90;
const FIFO_HEAD_M_A: u8 = 0x94;
const FIFO_HEAD_M_G: u8 = 0x98;
const FIFO_HEAD_M_G_A: u8 = 0x9C;

/** FIFO sensor time length definitions */
const SENSOR_TIME_LENGTH: u8 = 3;

/** FIFO DOWN selection */
/* Accel fifo down-sampling values*/
const ACCEL_FIFO_DOWN_ZERO: u8 = 0x00;
const ACCEL_FIFO_DOWN_ONE: u8 = 0x10;
const ACCEL_FIFO_DOWN_TWO: u8 = 0x20;
const ACCEL_FIFO_DOWN_THREE: u8 = 0x30;
const ACCEL_FIFO_DOWN_FOUR: u8 = 0x40;
const ACCEL_FIFO_DOWN_FIVE: u8 = 0x50;
const ACCEL_FIFO_DOWN_SIX: u8 = 0x60;
const ACCEL_FIFO_DOWN_SEVEN: u8 = 0x70;

/* Gyro fifo down-smapling values*/
const GYRO_FIFO_DOWN_ZERO: u8 = 0x00;
const GYRO_FIFO_DOWN_ONE: u8 = 0x01;
const GYRO_FIFO_DOWN_TWO: u8 = 0x02;
const GYRO_FIFO_DOWN_THREE: u8 = 0x03;
const GYRO_FIFO_DOWN_FOUR: u8 = 0x04;
const GYRO_FIFO_DOWN_FIVE: u8 = 0x05;
const GYRO_FIFO_DOWN_SIX: u8 = 0x06;
const GYRO_FIFO_DOWN_SEVEN: u8 = 0x07;

/* Accel Fifo filter enable*/
const ACCEL_FIFO_FILT_EN: u8 = 0x80;

/* Gyro Fifo filter enable*/
const GYRO_FIFO_FILT_EN: u8 = 0x08;

/** Definitions to check validity of FIFO frames */
const FIFO_CONFIG_MSB_CHECK: u8 = 0x80;
const FIFO_CONFIG_LSB_CHECK: u8 = 0x00;

// BMI160 accel FOC configurations
const FOC_ACCEL_DISABLED: u8 = 0x00;
const FOC_ACCEL_POSITIVE_G: u8 = 0x01;
const FOC_ACCEL_NEGATIVE_G: u8 = 0x02;
const FOC_ACCEL_0G: u8 = 0x03;

// Array Parameter DefinItions
const SENSOR_TIME_LSB_BYTE: u8 = 0;
const SENSOR_TIME_XLSB_BYTE: u8 = 1;
const SENSOR_TIME_MSB_BYTE: u8 = 2;

/** Interface settings */
const SPI_INTF: u8 = 1;
const I2C_INTF: u8 = 0;
const SPI_RD_MASK: u8 = 0x80;
const SPI_WR_MASK: u8 = 0x7F;

/* Sensor & time select definition*/
const ACCEL_SEL: u8 = 0x01;
const GYRO_SEL: u8 = 0x02;
const TIME_SEL: u8 = 0x04;

/* Sensor select mask*/
const SEN_SEL_MASK: u8 = 0x07;

/* Error code mask */
const ERR_REG_MASK: u8 = 0x0F;

/* BMI160 I2C address */
const I2C_ADDR: u8 = 0x68;

/* BMI160 secondary IF address */
const AUX_BMM150_I2C_ADDR: u8 = 0x10;

/** BMI160 Length definitions */
const ONE: u8 = 1;
const TWO: u8 = 2;
const THREE: u8 = 3;
const FOUR: u8 = 4;
const FIVE: u8 = 5;

/** BMI160 fifo level Margin */
const FIFO_LEVEL_MARGIN: u8 = 16;

/** BMI160 fifo flush Command */
const FIFO_FLUSH_VALUE: u8 = 0xB0;

/** BMI160 offset values for xyz axes of accel */
const ACCEL_MIN_OFFSET: i8 = -128;
const ACCEL_MAX_OFFSET: i8 = 127;

/** BMI160 offset values for xyz axes of gyro */
const GYRO_MIN_OFFSET: i16 = -512;
const GYRO_MAX_OFFSET: i16 = 511;

/** BMI160 fifo full interrupt position and mask */
const FIFO_FULL_INT_POS: u8 = 5;
const FIFO_FULL_INT_MSK: u8 = 0x20;
const FIFO_WTM_INT_POS: u8 = 6;
const FIFO_WTM_INT_MSK: u8 = 0x40;

const FIFO_FULL_INT_PIN1_POS: u8 = 5;
const FIFO_FULL_INT_PIN1_MSK: u8 = 0x20;
const FIFO_FULL_INT_PIN2_POS: u8 = 1;
const FIFO_FULL_INT_PIN2_MSK: u8 = 0x02;

const FIFO_WTM_INT_PIN1_POS: u8 = 6;
const FIFO_WTM_INT_PIN1_MSK: u8 = 0x40;
const FIFO_WTM_INT_PIN2_POS: u8 = 2;
const FIFO_WTM_INT_PIN2_MSK: u8 = 0x04;

const MANUAL_MODE_EN_POS: u8 = 7;
const MANUAL_MODE_EN_MSK: u8 = 0x80;
const AUX_READ_BURST_POS: u8 = 0;
const AUX_READ_BURST_MSK: u8 = 0x03;

const GYRO_SELF_TEST_POS: u8 = 4;
const GYRO_SELF_TEST_MSK: u8 = 0x10;
const GYRO_SELF_TEST_STATUS_POS: u8 = 1;
const GYRO_SELF_TEST_STATUS_MSK: u8 = 0x02;

const GYRO_FOC_EN_POS: u8 = 6;
const GYRO_FOC_EN_MSK: u8 = 0x40;

const ACCEL_FOC_X_CONF_POS: u8 = 4;
const ACCEL_FOC_X_CONF_MSK: u8 = 0x30;

const ACCEL_FOC_Y_CONF_POS: u8 = 2;
const ACCEL_FOC_Y_CONF_MSK: u8 = 0x0C;

const ACCEL_FOC_Z_CONF_MSK: u8 = 0x03;

const FOC_STATUS_POS: u8 = 3;
const FOC_STATUS_MSK: u8 = 0x08;

const GYRO_OFFSET_X_MSK: u8 = 0x03;

const GYRO_OFFSET_Y_POS: u8 = 2;
const GYRO_OFFSET_Y_MSK: u8 = 0x0C;

const GYRO_OFFSET_Z_POS: u8 = 4;
const GYRO_OFFSET_Z_MSK: u8 = 0x30;

const GYRO_OFFSET_EN_POS: u8 = 7;
const GYRO_OFFSET_EN_MSK: u8 = 0x80;

const ACCEL_OFFSET_EN_POS: u8 = 6;
const ACCEL_OFFSET_EN_MSK: u8 = 0x40;

const GYRO_OFFSET_POS: u16 = 8;
const GYRO_OFFSET_MSK: u16 = 0x0300;

const NVM_UPDATE_POS: u8 = 1;
const NVM_UPDATE_MSK: u8 = 0x02;

const NVM_STATUS_POS: u8 = 4;
const NVM_STATUS_MSK: u8 = 0x10;

const MAG_POWER_MODE_MSK: u8 = 0x03;

const ACCEL_POWER_MODE_MSK: u8 = 0x30;
const ACCEL_POWER_MODE_POS: u8 = 4;

const GYRO_POWER_MODE_MSK: u8 = 0x0C;
const GYRO_POWER_MODE_POS: u8 = 2;

#[allow(dead_code)]
enum Registers {
    ChipId = 0x00,
    ErrorReg = 0x02,
    PmuStatus = 0x03,
    AuxData = 0x04,
    GyroData = 0x0C,
    AccelData = 0x12,
    Status = 0x1B,
    IntStatus = 0x1C,
    FifoLength = 0x22,
    FifoData = 0x24,
    AccelConfig = 0x40,
    AccelRange = 0x41,
    GyroConfig = 0x42,
    GyroRange = 0x43,
    AuxOdr = 0x44,
    FifoDown = 0x45,
    FifoConfig0 = 0x46,
    FifoConfig1 = 0x47,
    AuxIf0 = 0x4B,
    AuxIf1 = 0x4C,
    AuxIf2 = 0x4D,
    AuxIf3 = 0x4E,
    AuxIf4 = 0x4F,
    InEnable0 = 0x50,
    IntEnable1 = 0x51,
    IntEnable2 = 0x52,
    IntOutCtrl = 0x53,
    IntLatch = 0x54,
    IntMap0 = 0x55,
    IntMap1 = 0x56,
    IntMap2 = 0x57,
    IntData0 = 0x58,
    IntData1 = 0x59,
    IntLowHigh0 = 0x5A,
    IntLowHigh1 = 0x5B,
    IntLowHigh2 = 0x5C,
    IntLowHigh3 = 0x5D,
    IntLowHigh4 = 0x5E,
    IntMotion0 = 0x5F,
    IntMotion1 = 0x60,
    IntMotion2 = 0x61,
    IntMotion3 = 0x62,
    IntTap0 = 0x63,
    IntTap1 = 0x64,
    IntOrient0 = 0x65,
    IntOrient1 = 0x66,
    IntFlat0 = 0x67,
    IntFlat1 = 0x68,
    FocConf = 0x69,
    Conf = 0x6A,
    IfConf = 0x6B,
    SelfTest = 0x6D,
    Offset = 0x71,
    OffsetConf = 0x77,
    IntStepCnt0 = 0x78,
    IntStepConfig0 = 0x7A,
    IntStepConfig1 = 0x7B,
    CommandReg = 0x7E,
    SpiCommTest = 0x7F,
    IntlPullupConf = 0x85,
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
