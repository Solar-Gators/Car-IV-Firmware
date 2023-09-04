#ifndef BQ76952_HPP_
#define BQ76952_HPP_

#include "main.h"

#define I2C_RES_ADDR                    0x10U
#define I2C_RES_READ_ADDR               (I2C_ADDR + 0x01U)

#define I2C_REG_CTRL_STATUS             0x00U
#define I2C_REG_BATT_STATUS             0x12U
#define I2C_REG_DEVICE_NUMBER           0x0001U
#define I2C_REG_FW_VERSION              0x0002U
#define I2C_REG_HW_VERSION              0x0003U
#define I2C_REG_IROM_SIG                0x0004U
#define I2C_REG_STATIC_CFG_SIG          0x0005U
#define I2C_REG_MANU_DATA               0x0070U


#define CTRL_STATUS_LD_ON_POS           0x0U
#define CTRL_STATUS_LD_ON_MASK          (0x1U << CTRL_STATUS_LD_ON_POS)

#define CTRL_STATUS_LD_TIMEOUT_POS      0x1U
#define CTRL_STATUS_LD_TIMEOUT_MASK     (0x1U << CTRL_STATUS_LD_TIMEOUT_POS)

#define CTRL_STATUS_DEEPSLEEP_POS       0x2U
#define CTRL_STATUS_DEEPSLEEP_MASK      (0x1U << CTRL_STATUS_DEEPSLEEP_POS)

#define BATT_STATUS_CFGUPDATE_POS       0x0U  
#define BATT_STATUS_PCHG_MODE_POS       0x1U  
#define BATT_STATUS_SLEEP_EN_POS        0x2U  
#define BATT_STATUS_POR_POS             0x3U  
#define BATT_STATUS_WD_POS              0x4U  
#define BATT_STATUS_COW_CHK             0x5U  
#define BATT_STATUS_OTPW_POS            0x6U  
#define BATT_STATUS_OTPB_POS            0x7U  
#define BATT_STATUS_SEC0_POS            0x8U  
#define BATT_STATUS_SEC1_POS            0x9U  
#define BATT_STATUS_FUSE_POS            0xAU  
#define BATT_STATUS_SS_POS              0xBU  
#define BATT_STATUS_PF_POS              0xCU  
#define BATT_STATUS_SDM_POS             0xDU  
#define BATT_STATUS_SLEEP_POS           0xFU  


enum BQ76952_State_t {
    BQ76952_SHUTDOWN_STATE,
    BQ76952_DEEPSLEEP_STATE,
    BQ76952_SLEEP_STATE,
    BQ76952_NORMAL_STATE,
    BQ76952_CONFIG_UPDATE
};

class BQ76952 {
    I2C_HandleTypeDef *phi2c_;
    BQ76952_State_t state_;
public:
    BQ76952(I2C_HandleTypeDef *phi2c);

    void SendData(uint8_t register_address, uint8_t data);
    void ReadData(uint8_t register_address, uint8_t *data);
    void SendDataCRC(uint8_t register_address, uint8_t data);
    void ReadDataCRC(uint8_t register_address, uint8_t *data);

    int32_t ToShutdown();
    int32_t ToDeepsleep();
    int32_t ToSleep();
    int32_t ToNormal();
    int32_t ToConfigupdate();
};

#endif  /* BQ76952_HPP_ */