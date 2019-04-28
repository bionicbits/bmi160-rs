//! ADXL343 register addresses
#![allow(non_camel_case_types, clippy::unreadable_literal)]

//use bitflags::bitflags;

/// Register addresses
/// Taken from the Bosch BMI160 data sheet (Register Map, p.47)
/// <https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMI160-DS000.pdf>
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Register {
    /// CHIPID (Read Only)
    ///
    /// The register contains the chip identification code.
    CHIP_ID = 0x00,

    /// ERR_REG (Read Only)
    ///
    /// Reports sensor error flags. Flags are reset when read.
    /// The register is meant for debug purposes, not for regular verification if
    /// an operation completed successfully.
    ERROR_REG = 0x02,

    /// PMU_STATUS (Read Only)
    ///
    /// Shows the current power mode of the sensor.
    PMU_STATUS = 0x03,

    /// DATA (Read Only)
    ///
    /// Register for accelerometer, gyroscope and magnetometer data.
    DATA = 0x04,

    /// SENSORTIME (Read Only)
    ///
    /// Sensortime is a 24bit counter available in suspend, low power, and normal mode.
    /// The value oftheregister is shadowed when it is read in a burst read with the
    /// data register at the beginning of the operation and the shadowed value is returned.
    /// When the FIFO is read the register is shadowed whenever a new frame is read.
    SENSORTIME = 0x18,

    /// STATUS (Read Only)
    ///
    /// Reports sensor status flags.
    STATUS = 0x1B,

    /// INT_STATUS (Read Only)
    ///
    /// The register contains interrupt status flags.
    INT_STATUS = 0x1C,

    /// TEMPERATURE (Read Only)
    ///
    /// Contains the temperature of the sensor.
    TEMPERATURE = 0x20,

    /// FIFO_LENGTH (Read Only)
    ///
    /// FIFO data readout register.
    FIFO_LENGTH = 0x22,

    /// FIFO_DATA (Read Only)
    ///
    /// FIFO data readout register.
    FIFO_DATA = 0x24,

    /// ACC_CONF (Read/Write)
    ///
    /// Sets the output data rate, the bandwidth, and the read mode of the acceleration sensor.
    ACC_CONF = 0x40,

    /// ACC_RANGE (Read/Write)
    ///
    /// The register allows the selection of the accelerometer g-range.
    ACC_RANGE = 0x41,

    /// GYR_CONF (Read/Write)
    ///
    /// Sets the output data rate, the bandwidth, and the read mode of the gyroscope in the sensor.
    GYR_CONF = 0x42,

    /// GYR_RANGE (Read/Write)
    ///
    /// Defines the BMI160angularrate measurement range.
    GYR_RANGE = 0x43,

    /// MAG_CONF (Read/Write)
    ///
    /// Sets the output data rate of the magnetometer interface in the sensor.
    MAG_CONF = 0x44,

    /// FIFO_DOWNS (Read/Write)
    ///
    /// Used to configure the down sampling ratios of the accel and gyro data for FIFO.
    FIFO_DOWNS = 0x45,

    /// FIFO_CONFIG (Read/Write)
    ///
    /// The Register (0x46-0x47) FIFO_CONFIGis a read/write register and can be used for
    /// reading or setting the current FIFO watermark level. This register can also
    /// be used for setting the different modes of operation of the FIFO,
    /// e.g. which data is going to be stored in it and which format is going to be
    /// used (header or headerlessmode).
    FIFO_CONFIG = 0x46,

    /// MAG_IF (Read/Write)
    ///
    /// Register for indirect addressing of the magnetometer connected to the magnetometer
    /// interface. This register allows read and write operations on the magnetometer
    /// register map. In addition it is used to setup the read loop for the magnetometer
    /// data. Setup and read loop are exclusive to each other, i.e. during the read loop
    /// no registers in the magnetometer may be accessed.
    MAG_IF = 0x4B,

    /// INT_EN (Read/Write)
    ///
    /// Controls whichinterrupt engines are enabled.
    INT_EN = 0x50,

    ///  INT_OUT_CTRL (Read/Write)
    ///
    /// Contains the behavioral configuration (electrical definition of the interrupt pins.
    INT_OUT_CTRL = 0x53,

    /// INT_LATCH (Read/Write)
    ///
    /// Contains the interrupt reset bit and the interrupt mode selection.
    INT_LATCH = 0x54,

    /// INT_MAP (Read/Write)
    ///
    /// Controls which interrupt signals are mapped to the INT1 and INT2 pin.
    INT_MAP = 0x55,

    /// INT_DATA (Read/Write)
    ///
    /// Contains the data source definition for the two interrupt groups.
    INT_DATA = 0x58,

    /// INT_LOWHIGH (Read/Write)
    ///
    /// Contains the configuration for the low g interrupt.
    INT_LOWHIGH = 0x5A,

    /// INT_MOTION (Read/Write)
    ///
    /// Contains the configuration for the anymotion and nomotion interrupts.
    INT_MOTION = 0x5F,

    /// INT_TAP (Read/Write)
    ///
    /// Contains the configuration for the tap interrupts.
    INT_TAP = 0x63,

    /// INT_ORIENT (Read/Write)
    ///
    /// Contains the configuration for the orientation interrupt.
    INT_ORIENT = 0x65,

    /// INT_FLAT (Read/Write)
    ///
    /// Contains the configuration for the flat interrupt.
    INT_FLAT = 0x67,

    /// FOC_CONF (Read/Write)
    ///
    /// Contains configuration settings for the fast offset compensation for the
    /// accelerometer and the gyroscope.
    FOC_CONF = 0x69,

    /// CONF (Read/Write)
    ///
    /// Configuration of the sensor
    CONF = 0x6A,

    /// IF_CONF (Read/Write)
    ///
    /// Contains settings for the digital interface.
    IF_CONF = 0x6B,

    /// PMU_TRIGGER (Read/Write)
    ///
    /// Used to set the trigger conditions to change the gyro power modes.
    PMU_TRIGGER = 0x6C,

    /// SELF_TEST (Read/Write)
    ///
    /// Contains the settings for the sensor self-test configuration and trigger.
    SELF_TEST = 0x6D,

    /// NV_CONF (Read/Write)
    ///
    /// Contains settings for the digital interface.
    NV_CONF = 0x70,

    /// OFFSET (Read/Write)
    ///
    /// Contains the offset compensation values for accelerometer and gyroscope.
    OFFSET = 0x71,

    /// STEP_CNT (Read Only)
    ///
    /// Contains the number of steps.
    STEP_CNT = 0x78,

    /// STEP_CONF (Read/Write)
    ///
    /// Contains configuration of the step detector.
    STEP_CONF = 0x7A,

    /// CMD (Write Only)
    ///
    /// Command register triggers operations like softreset, NVM programming, etc.
    CMD = 0x7E,
}

impl Register {
    /// Get register address
    pub fn addr(&self) -> u8 {
        *self as u8
    }

    /// Is the register read-only?
    pub fn read_only(self) -> bool {
        match self {
            Register::CHIP_ID
            | Register::ERROR_REG
            | Register::PMU_STATUS
            | Register::DATA
            | Register::SENSORTIME
            | Register::STATUS
            | Register::INT_STATUS
            | Register::TEMPERATURE
            | Register::FIFO_LENGTH
            | Register::FIFO_DATA
            | Register::STEP_CNT => true,
            _ => false,
        }
    }
}
/// Commands that can be used passed into CMD Register
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Cmd {
    /// Starts Fast Offset Calibration for the accel and gyro as configured in Register (0x69) 
    /// FOC_CONF and stores the result into the Register (0x71-0x77) OFFSET register.
    START_OFC = 0x03,


    // acc_set_pmu_mode: 0b0001 00nn
    // Sets the PMU mode for the accelerometer. The encoding for ‘nn’ is identical 
    // to acc_pmu_status in Register (0x03) PMU_STATUS.
    /// Sets the PMU mode for the accelerometer to Suspend.
    ACC_SET_PMU_MODE_SUSPEND = 0b00010000,

    /// Sets the PMU mode for the accelerometer to Normal.
    ACC_SET_PMU_MODE_NORMAL = 0b00010001,

    /// Sets the PMU mode for the accelerometer to Low Power.
    ACC_SET_PMU_MODE_LOW_POWER = 0b00010010,

    
}
