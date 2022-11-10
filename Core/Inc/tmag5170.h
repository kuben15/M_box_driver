/*
*   TMAG51170.h
*
*   Created on: 21.10.2022
*       Author: JS
*
*/


#ifndef _TMAG5170_H_
#define _TMAG5170_H_

//-------------------------Register Address-------------------------

typedef enum{
    TMAG5170_DEVICE_CONFIG_REG = 0x00,
    TMAG5170_SENSOR_CONFIG_REG = 0x01,
    TMAG5170_SYSTEM_CONFIG_REG = 0x02,
    TMAG5170_ALERT_CONFIG_REG = 0x03,
    TMAG5170_X_THRX_CONFIG_REG = 0x04,
    TMAG5170_Y_THRX_CONFIG_REG = 0x05,
    TMAG5170_Z_THRX_CONFIG_REG = 0x06,
    TMAG5170_T_THRX_CONFIG_REG = 0x07,
    TMAG5170_CONV_STATUS_REG = 0x08,
    TMAG5170_X_CH_RESULT_REG = 0x09,
    TMAG5170_Y_CH_RESULT_REG = 0x0A,
    TMAG5170_Z_CH_RESULT_REG = 0x0B,
    TMAG5170_TEMP_RESULT_REG = 0x0C,
    TMAG5170_AFE_STATUS_REG = 0x0D,
    TMAG5170_SYS_STATUS_REG = 0x0E,
    TMAG5170_TEST_CONFIG_REG = 0x0F,
    TMAG5170_OSC_MONITOR_REG = 0x010,
    TMAG5170_MAG_GAIN_CONFIG_REG = 0x11,
    TMAG5170_MAG_OFFSET_CONFIG_REG = 0x12,
    TMAG5170_ANGLE_RESULT_REG = 0x13,
    TMAG5170_MAGNITUDE_RESULT_REG = 0x14,
} TMAG5170_reg_addr_t;

typedef enum{
    TMAG5170_RET_OK = 0,
    TMAG5170_RET_FAIL = -1,
}TMAG5170_return_code_t;


//-------------------------DEVICE_CONFIG-------------------------

typedef enum{
    TMAG5170_CONV_AVG_1x = 0x00,
    TMAG5170_CONV_AVG_2x = 0x01,
    TMAG5170_CONV_AVG_4x = 0x02,
    TMAG5170_CONV_AVG_8x = 0x03,
    TMAG5170_CONV_AVG_16x = 0x04,
    TMAG5170_CONV_AVG_32x = 0x05,
} TMAG5170_CONV_AVG_t;

typedef enum{
    TMAG5170_MAG_TEMPCO_0 = 0x00,
    TMAG5170_MAG_TEMPCO_0R12 = 0x01,
    TMAG5170_MAG_TEMPCO_0R03 = 0x02,
    TMAG5170_MAG_TEMPCO_0R2 = 0x03,
}TMAG5170_MAG_TEMPCO_t;

typedef enum{
    TMAG5170_OPERATING_MODE_ConfigurationMode = 0x00,
    TMAG5170_OPERATING_MODE_StandbyMode = 0x01,
    TMAG5170_OPERATING_MODE_ActiveMeasureMode = 0x02,
    TMAG5170_OPERATING_MODE_ActiveTriggerMode = 0x03,
    TMAG5170_OPERATING_MODE_WakeupAndSleepMode = 0x04,
    TMAG5170_OPERATING_MODE_SleepMode = 0x05,
    TMAG5170_OPERATING_MODE_DeepsleepMode = 0x06,
}TMAG5170_OPERATING_MODE_t;

typedef enum{
    TMAG5170_T_CH_EN_TempChannelDisabled = 0x00,
    TMAG5170_T_CH_EN_TempChannelEnabled = 0x01,
}TMAG5170_T_CH_EN_t;

typedef enum{
    TMAG5170_T_RATE_sameAsOtherSensors = 0x00,
    TMAG5170_T_RATE_oncePerConversionSet = 0x01,
}TMAG5170_T_RATE_t;

typedef enum{
    TMAG5170_T_HLT_EN_tempLimitCheckOff = 0x00,
    TMAG5170_T_HLT_EN_tempLimitCheckOn = 0x01,
}TMAG5170_T_HLT_EN_t;

//-------------------------SENSOR_CONFIG-------------------------

typedef enum{
    TMAG5170_ANGLE_EN_NoAngleCalculation = 0x00,
    TMAG5170_ANGLE_EN_X_Y = 0x01,
    TMAG5170_ANGLE_EN_Y_Z = 0x02,
    TMAG5170_ANGLE_EN_Z_X = 0x03,
}TMAG5170_ANGLE_EN_t;

typedef enum{
    TMAG5170_SLEEPTIME_1ms = 0x00,
    TMAG5170_SLEEPTIME_5ms = 0x01,
    TMAG5170_SLEEPTIME_10ms = 0x02,
    TMAG5170_SLEEPTIME_15ms = 0x03,
    TMAG5170_SLEEPTIME_20ms = 0x04,
    TMAG5170_SLEEPTIME_30ms = 0x05,
    TMAG5170_SLEEPTIME_50ms = 0x06,
    TMAG5170_SLEEPTIME_100ms = 0x07,
    TMAG5170_SLEEPTIME_500ms = 0x08,
    TMAG5170_SLEEPTIME_1000ms = 0x09,
}TMAG5170_SLEEPTIME_t;

typedef enum{
    TMAG5170_MAG_CH_EN_OFF = 0x00,
    TMAG5170_MAG_CH_EN_X = 0x01,
    TMAG5170_MAG_CH_EN_Y = 0x02,
    TMAG5170_MAG_CH_EN_XY = 0x03,
    TMAG5170_MAG_CH_EN_Z = 0x04,
    TMAG5170_MAG_CH_EN_ZX = 0x05,
    TMAG5170_MAG_CH_EN_YZ = 0x06,
    TMAG5170_MAG_CH_EN_XYZ = 0x07,
    TMAG5170_MAG_CH_EN_XYX = 0x08,
    TMAG5170_MAG_CH_EN_YXY = 0x09,
    TMAG5170_MAG_CH_EN_YZY = 0x0A,
    TMAG5170_MAG_CH_EN_ZYZ = 0x0B,
    TMAG5170_MAG_CH_EN_ZXZ = 0x0C,
    TMAG5170_MAG_CH_EN_XZX = 0x0D,
    TMAG5170_MAG_CH_EN_XYZYX = 0x0E,
    TMAG5170_MAG_CH_EN_XYZZYX = 0x0F,
}TMAG5170_MAG_CH_EN_t;

typedef enum{
    TMAG5170_Z_RANGE_50mT = 0x00,
    TMAG5170_Z_RANGE_25mT = 0x01,
    TMAG5170_Z_RANGE_100mT = 0x02,
}TMAG5170_Z_RANGE_t;

typedef enum{
    TMAG5170_Y_RANGE_50mT = 0x00,
    TMAG5170_Y_RANGE_25mT = 0x01,
    TMAG5170_Y_RANGE_100mT = 0x02,
}TMAG5170_Y_RANGE_t;

typedef enum{
    TMAG5170_X_RANGE_50mT = 0x00,
    TMAG5170_X_RANGE_25mT = 0x01,
    TMAG5170_X_RANGE_100mT = 0x02,
}TMAG5170_X_RANGE_t;

//-------------------------SYSTEM_CONFIG-------------------------

typedef enum{
    TMAG5170_DIAG_SEL_AllDataPathTogether = 0x00,
    TMAG5170_DIAG_SEL_EnabledDataPathTogether = 0x01,
    TMAG5170_DIAG_SEL_AllDataPathInSequence = 0x02,
    TMAG5170_DIAG_SEL_EnabledDataPathInSequence = 0x03,
}TMAG5170_DIAG_SEL_t;

typedef enum{
    TMAG5170_TRIGGER_MODE_SPI = 0x00,
    TMAG5170_TRIGGER_MODE_nCSpulse = 0x01,
    TMAG5170_TRIGGER_MODE_nALERTpulse = 0x02,
}TMAG5170_TRIGGER_MODE_t;

typedef enum{
    TMAG5170_DATA_TYPE_32bit = 0x00,
    TMAG5170_DATA_TYPE_12bitXY = 0x01,
    TMAG5170_DATA_TYPE_12bitXZ = 0x02,
    TMAG5170_DATA_TYPE_12bitZY = 0x03,
    TMAG5170_DATA_TYPE_12bitXT = 0x04,
    TMAG5170_DATA_TYPE_12bitYT = 0x05,
    TMAG5170_DATA_TYPE_12bitZT = 0x06,
    TMAG5170_DATA_TYPE_12bitAM = 0x07,
}TMAG5170_DATA_TYPE_t;

typedef enum{
    TMAG5170_DIAG_EN_AFEdiagnosticsDisabled = 0x00,
    TMAG5170_DIAG_EN_AFEdiagnosticsEnabled = 0x00,
}TMAG5170_DIAG_EN_t;

typedef enum{
    TMAG5170_Z_HLT_EN_ZaxisLimitCheckOff = 0x00,
    TMAG5170_Z_HLT_EN_ZaxisLimitCheckOn = 0x01,
}TMAG5170_Z_HLT_EN_t;

typedef enum{
    TMAG5170_Y_HLT_EN_YaxisLimitCheckOff = 0x00,
    TMAG5170_Y_HLT_EN_YaxisLimitCheckOn = 0x01,
}TMAG5170_Y_HLT_EN_t;

typedef enum{
    TMAG5170_X_HLT_EN_XaxisLimitCheckOff = 0x00,
    TMAG5170_X_HLT_EN_XaxisLimitCheckOn = 0x01,
}TMAG5170_X_HLT_EN_t;

//-------------------------ALERT_CONFIG-------------------------

typedef enum{
    TMAG5170_ALERT_LATCH_SourcesNotLached = 0x00,
    TMAG5170_ALERT_LATCH_SourcesLached = 0x01,
}TMAG5170_ALERT_LATCH_t;

typedef enum{
    TMAG5170_ALERT_MODE_InterruptMode = 0x00,
    TMAG5170_ALERT_MODE_SwitchMode = 0x01,
}TMAG5170_ALERT_MODE_t;

typedef enum{
    TMAG5170_STATUS_ALRT_IsNotAsserted = 0x00,
    TMAG5170_STATUS_ALRT_IsAsserted = 0x01,
}TMAG5170_STATUS_ALRT_t;

typedef enum{
    TMAG5170_RSLT_ALRT_IsNotUsed = 0x00,
    TMAG5170_RSLT_ALRT_IsUsed = 0x01,
}TMAG5170_RSLT_ALRT_t;

typedef enum{
    TMAG5170_THRX_COUNT_1_ConversionResult = 0x00,
    TMAG5170_THRX_COUNT_2_ConversionResult = 0x01,
    TMAG5170_THRX_COUNT_3_ConversionResult = 0x02,
    TMAG5170_THRX_COUNT_4_ConversionResult = 0x03,
}TMAG5170_THRX_COUNT_t;

typedef enum{
    TMAG5170_T_THRX_ALRT_IsNotUsed = 0x00,
    TMAG5170_T_THRX_ALRT_IsUsed = 0x01,
}TMAG5170_T_THRX_ALRT_t;

typedef enum{
    TMAG5170_Z_THRX_ALRT_IsNotUsed = 0x00,
    TMAG5170_Z_THRX_ALRT_IsUsed = 0x01,
}TMAG5170_Z_THRX_ALRT_t;

typedef enum{
    TMAG5170_Y_THRX_ALRT_IsNotUsed = 0x00,
    TMAG5170_Y_THRX_ALRT_IsUsed = 0x01,
}TMAG5170_Y_THRX_ALRT_t;

typedef enum{
    TMAG5170_X_THRX_ALRT_IsNotUsed = 0x00,
    TMAG5170_X_THRX_ALRT_IsUsed = 0x01,
}TMAG5170_X_THRX_ALRT_t;

//-------------------------CONV_STATUS-------------------------

typedef enum{
    TMAG5170_RDY_ConversationDataNotValid = 0x00,
    TMAG5170_RDY_ConversationDataValid = 0x01,
}TMAG5170_RDY_t;

typedef enum{
    TMAG5170_A_DataIsNotCurrent = 0x00,
    TMAG5170_A_DataIsCurrent = 0x01,
}TMAG5170_A_t;

typedef enum{
    TMAG5170_T_DataIsNotCurrent = 0x00,
    TMAG5170_T_DataIsCurrent = 0x01,
}TMAG5170_T_t;

typedef enum{
    TMAG5170_Z_DataIsNotCurrent = 0x00,
    TMAG5170_Z_DataIsCurrent = 0x01,
}TMAG5170_Z_t;

typedef enum{
    TMAG5170_Y_DataIsNotCurrent = 0x00,
    TMAG5170_Y_DataIsCurrent = 0x01,
}TMAG5170_Y_t;

typedef enum{
    TMAG5170_X_DataIsNotCurrent = 0x00,
    TMAG5170_X_DataIsCurrent = 0x01,
}TMAG5170_X_t;

typedef enum{
    TMAG5170_ALRT_STATUS_NoALERTconditions = 0x00,
    TMAG5170_ALRT_STATUS_AFEstatusFlagSet = 0x01,
    TMAG5170_ALRT_STATUS_SYSstatusFlagSet = 0x02,
    TMAG5170_ALRT_STATUS_AFE_SYS_statusFlagSet = 0x03,
}TMAG5170_ALRT_STATUS_t;

//-------------------------AFE_STATUS-------------------------

typedef enum{
    TMAG5170_CFG_RESET_AcknowledgedAndCleard = 0x00,
    TMAG5170_CFG_RESET_ExperiencedHardwareReset = 0x01,
}TMAG5170_CFG_RESET_t;

typedef enum{
    TMAG5170_SENS_STAT_NoError = 0x00,
    TMAG5170_SENS_STAT_SensorDiagnosticTestFailed = 0x01,
}TMAG5170_SENS_STAT_t;

typedef enum{
    TMAG5170_TEMP_STAT_NoError = 0x00,
    TMAG5170_TEMP_STAT_SensorDiagnosticTestFailed = 0x01,
}TMAG5170_TEMP_STAT_t;

typedef enum{
    TMAG5170_ZHS_STAT_NoError = 0x00,
    TMAG5170_ZHS_STAT_SensorDiagnosticTestFailed = 0x01,
}TMAG5170_ZHS_STAT_t;

typedef enum{
    TMAG5170_YHS_STAT_NoError = 0x00,
    TMAG5170_YHS_STAT_SensorDiagnosticTestFailed = 0x01,
}TMAG5170_YHS_STAT_t;

typedef enum{
    TMAG5170_XHS_STAT_NoError = 0x00,
    TMAG5170_XHS_STAT_SensorDiagnosticTestFailed = 0x01,
}TMAG5170_XHS_STAT_t;

typedef enum{
    TMAG5170_TRIM_STAT_NoTrimDataError = 0x00,
    TMAG5170_TRIM_STAT_TrimDataError = 0x01,
}TMAG5170_TRIM_STAT_t;

typedef enum{
    TMAG5170_LDO_STAT_NoLDOfault = 0x00,
    TMAG5170_LDO_STAT_LDOfault = 0x01,
}TMAG5170_LDO_STAT_t;

//-------------------------SYS_STATUS-------------------------

typedef enum{
    TMAG5170_ALRT_LVL_ALERTlevelLow = 0x00,
    TMAG5170_ALRT_LVL_ALERTlevelHigh = 0x01,
}TMAG5170_ALRT_LVL_t;

typedef enum{
    TMAG5170_ALRT_DRV_NoALERTerror = 0x00,
    TMAG5170_ALRT_DRV_ALERTerror = 0x01,
}TMAG5170_ALRT_DRV_t;

typedef enum{
    TMAG5170_SDO_DRV_NoSDOerror = 0x00,
    TMAG5170_SDO_DRV_SDOerror = 0x01,
}TMAG5170_SDO_DRV_t;

typedef enum{
    TMAG5170_CRC_STAT_NoCRCerror = 0x00,
    TMAG5170_CRC_STAT_CRCerror = 0x01,
}TMAG5170_CRC_STAT_t;

typedef enum{
    TMAG5170_FRAME_STAT_NoFRAMEerror = 0x00,
    TMAG5170_FRAME_STAT_FRAMEerror = 0x01,
}TMAG5170_FRAME_STAT_t;

typedef enum{
    TMAG5170_OPERATING_STAT_ConfigState = 0x00,
    TMAG5170_OPERATING_STAT_StandbyState = 0x01,
    TMAG5170_OPERATING_STAT_ActiveMeasure = 0x02,
    TMAG5170_OPERATING_STAT_ActiveTriggeredMode = 0x03,
    TMAG5170_OPERATING_STAT_DCMactiveState = 0x04,
    TMAG5170_OPERATING_STAT_DCMsleepState = 0x05,
    TMAG5170_OPERATING_STAT_SleepState = 0x06,
}TMAG5170_OPERATING_STAT_t;

typedef enum{
    TMAG5170_VCC_OV_NoOverVoltage = 0x00,
    TMAG5170_VCC_OV_OverVoltage = 0x01,
}TMAG5170_VCC_OV_t;

typedef enum{
    TMAG5170_VCC_UV_NoOverVoltage = 0x00,
    TMAG5170_VCC_UV_OverVoltage = 0x01,
}TMAG5170_VCC_UV_t;

typedef enum{
    TMAG5170_TEMP_THX_NoThresholdCrossingDetected = 0x00,
    TMAG5170_TEMP_THX_ThresholdCrossingDetected = 0x01,
}TMAG5170_TEMP_THX_t;

typedef enum{
    TMAG5170_ZCH_THX_NoThresholdCrossingDetected = 0x00,
    TMAG5170_ZCH_THX_ThresholdCrossingDetected = 0x01,
}TMAG5170_ZCH_THX_t;

typedef enum{
    TMAG5170_YCH_THX_NoThresholdCrossingDetected = 0x00,
    TMAG5170_YCH_THX_ThresholdCrossingDetected = 0x01,
}TMAG5170_YCH_THX_t;

typedef enum{
    TMAG5170_XCH_THX_NoThresholdCrossingDetected = 0x00,
    TMAG5170_XCH_THX_ThresholdCrossingDetected = 0x01,
}TMAG5170_XCH_THX_t;

//-------------------------TEST_CONFIG-------------------------

typedef enum{
    TMAG5170_VER_A1Rev = 0x00,
    TMAG5170_VER_A2Rev = 0x01,
}TMAG5170_VER_t;

typedef enum{
    TMAG5170_CRC_DIS_CRCenabledInSPI = 0x00,
    TMAG5170_CRC_DIS_CRCdisabledInSPI = 0x01,
}TMAG5170_CRC_DIS_t;

typedef enum{
    TMAG5170_OSC_CNT_CTL_ResetOSC = 0x00,
    TMAG5170_OSC_CNT_CTL_StartOSCdrivenByHFOSC = 0x01,
    TMAG5170_OSC_CNT_CTL_StartOSCdrivenByLFOSC = 0x02,
    TMAG5170_OSC_CNT_CTL_StopOSC = 0x03,
}TMAG5170_OSC_CNT_CTL_t;

//------------------------- MAG_GAIN_CONFIG-------------------------

typedef enum{
    TMAG5170_GAIN_SELECTION_NoAxis = 0x00,
    TMAG5170_GAIN_SELECTION_X_Axis = 0x01,
    TMAG5170_GAIN_SELECTION_Y_Axis = 0x02,
    TMAG5170_GAIN_SELECTION_Z_Axis = 0x03,
}TMAG5170_GAIN_SELECTION_t;





typedef struct{
    TMAG5170_CONV_AVG_t ConvAvg;
    TMAG5170_MAG_TEMPCO_t MagTempCoef;
    TMAG5170_OPERATING_MODE_t OperatingMode;
    TMAG5170_T_CH_EN_t TempChannelEnabled;
    TMAG5170_T_RATE_t TempRate;
    TMAG5170_T_HLT_EN_t TempLimitCheck;
}TMAG5170_DEVICE_CONFIG_settings_t;

typedef struct{
    TMAG5170_ANGLE_EN_t AngleEnabled;
    TMAG5170_SLEEPTIME_t SleepTime;
    TMAG5170_MAG_CH_EN_t MagChannelEnabled;
    TMAG5170_Z_RANGE_t Zrange;
    TMAG5170_Y_RANGE_t Yrange;
    TMAG5170_X_RANGE_t Xrange;
}TMAG5170_SENSOR_CONFIG_settings_t;

typedef struct{
    TMAG5170_DIAG_SEL_t DiagnosticSelect;
    TMAG5170_TRIGGER_MODE_t TriggerMode;
    TMAG5170_DATA_TYPE_t DataType;
    TMAG5170_DIAG_EN_t DiagnosticEnabled;
    TMAG5170_Z_HLT_EN_t ZlimitCheck;
    TMAG5170_Y_HLT_EN_t YlimitCheck;
    TMAG5170_X_HLT_EN_t XlimitCheck;
}TMAG5170_SYSTEM_CONFIG_settings_t;

typedef struct{
    TMAG5170_ALERT_LATCH_t AlertLatch;
    TMAG5170_ALERT_MODE_t AlertMode;
    TMAG5170_STATUS_ALRT_t AlertStatus;
    TMAG5170_RSLT_ALRT_t AlertResponse;
    TMAG5170_THRX_COUNT_t ThresholdCount;
    TMAG5170_T_THRX_ALRT_t TempThrAlert;
    TMAG5170_Z_THRX_ALRT_t ZthrAlert;
    TMAG5170_Y_THRX_ALRT_t YthrAlert;
    TMAG5170_X_THRX_ALRT_t XthrAlert;
}TMAG5170_ALERT_CONFIG_settings_t;


TMAG5170_return_code_t TMAG5170_dev_conf_settings(TMAG5170_DEVICE_CONFIG_settings_t *new_settings);
TMAG5170_return_code_t TMAG5170_sens_conf_settings(TMAG5170_SENSOR_CONFIG_settings_t *new_settings);
TMAG5170_return_code_t TMAG5170_sys_conf_settings(TMAG5170_SYSTEM_CONFIG_settings_t *new_settings);
TMAG5170_return_code_t TMAG5170_al_conf_settings(TMAG5170_ALERT_CONFIG_settings_t *new_settings);
void GetTemp(void);
void DisableCRC(void);
void ReadRegister(void);
void ReadRegister1(void);
void GetmT(void);
void GetMagnitude(void);
float GetAngle();
#endif 