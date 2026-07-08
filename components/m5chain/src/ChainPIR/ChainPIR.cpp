#include "ChainPIR.hpp"
chain_status_t ChainPIR::getIRStatus(uint8_t id, pir_detect_result_t *ir_status, unsigned long timeout)
{
    chain_status_t status = CHAIN_OK;
    if (acquireMutex()) {
        memset(cmdBuffer, 0, cmdBufferSize);
        cmdBufferSize = 0;
        sendPacket(id, CHAIN_PIR_GET_IR_STATUS, cmdBuffer, cmdBufferSize);

        if (waitForData(id, CHAIN_PIR_GET_IR_STATUS, timeout)) {
            if (checkPacket(reinterpret_cast<const uint8_t *>(returnPacket), returnPacketSize)) {
                *ir_status = (pir_detect_result_t)returnPacket[6];
            } else {
                status = CHAIN_RETURN_PACKET_ERROR;
            }
        } else {
            status = CHAIN_TIMEOUT;
        }
        releaseMutex();
    } else {
        status = CHAIN_BUSY;
    }
    return status;
}

chain_status_t ChainPIR::setPIRDetectTriggerMode(uint8_t id, chain_detect_mode_t auto_status, uint8_t *operationStatus,
                                                 unsigned long timeout)
{
    chain_status_t status = CHAIN_OK;
    if (acquireMutex()) {
        sendPacket(id, CHAIN_PIR_SET_AUTO_SEND_IR_STATUS, (uint8_t *)&auto_status, 1);

        if (waitForData(id, CHAIN_PIR_SET_AUTO_SEND_IR_STATUS, timeout)) {
            if (checkPacket(reinterpret_cast<const uint8_t *>(returnPacket), returnPacketSize)) {
                *operationStatus = returnPacket[6];
            } else {
                status = CHAIN_RETURN_PACKET_ERROR;
            }
        } else {
            status = CHAIN_TIMEOUT;
        }
        releaseMutex();
    } else {
        status = CHAIN_BUSY;
    }
    return status;
}

chain_status_t ChainPIR::getPIRDetectTriggerMode(uint8_t id, chain_detect_mode_t *auto_status, unsigned long timeout)
{
    chain_status_t status = CHAIN_OK;
    if (acquireMutex()) {
        memset(cmdBuffer, 0, cmdBufferSize);
        cmdBufferSize = 0;
        sendPacket(id, CHAIN_PIR_GET_AUTO_SEND_IR_STATUS, cmdBuffer, cmdBufferSize);

        if (waitForData(id, CHAIN_PIR_GET_AUTO_SEND_IR_STATUS, timeout)) {
            if (checkPacket(reinterpret_cast<const uint8_t *>(returnPacket), returnPacketSize)) {
                *auto_status = (chain_detect_mode_t)returnPacket[6];
            } else {
                status = CHAIN_RETURN_PACKET_ERROR;
            }
        } else {
            status = CHAIN_TIMEOUT;
        }
        releaseMutex();
    } else {
        status = CHAIN_BUSY;
    }
    return status;
}
uint16_t ChainPIR::getPIRTypeCode(void)
{
    return CHAIN_PIR_DEVICE_TYPE_CODE;
}

bool ChainPIR::getPIRDetectTrigger(uint8_t id, pir_detect_report_t *triggerStatus)
{
    processIncomingData();
    bool findStatus = 0;
    record_info_t result;
    findStatus = findRecord(&recordList, id, &result);
    if (findStatus == true) {
        *triggerStatus = (pir_detect_report_t)result.type;
        return true;
    }
    return false;
}

chain_status_t ChainPIR::setPIRLeaveTriggerKeepSeconds(uint8_t id, uint8_t keepSeconds, uint8_t *operationStatus,
                                                       uint8_t saveToFlash, unsigned long timeout)
{
    if (keepSeconds > 255) {
        return CHAIN_PARAMETER_ERROR;
    }

    chain_status_t status = CHAIN_OK;
    if (acquireMutex()) {
        memset(cmdBuffer, 0, cmdBufferSize);
        cmdBufferSize              = 0;
        cmdBuffer[cmdBufferSize++] = keepSeconds;
        cmdBuffer[cmdBufferSize++] = saveToFlash;
        sendPacket(id, CHAIN_SET_TRIGGER_KEEP_SECONDS, (uint8_t *)&cmdBuffer, cmdBufferSize);

        if (waitForData(id, CHAIN_SET_TRIGGER_KEEP_SECONDS, timeout)) {
            if (checkPacket(reinterpret_cast<const uint8_t *>(returnPacket), returnPacketSize)) {
                *operationStatus = returnPacket[6];
            } else {
                status = CHAIN_RETURN_PACKET_ERROR;
            }
        } else {
            status = CHAIN_TIMEOUT;
        }
        releaseMutex();
    } else {
        status = CHAIN_BUSY;
    }
    return status;
}

chain_status_t ChainPIR::getPIRComeTriggerKeepSeconds(uint8_t id, uint8_t *keepSeconds, unsigned long timeout)
{
    chain_status_t status = CHAIN_OK;
    if (acquireMutex()) {
        memset(cmdBuffer, 0, cmdBufferSize);
        cmdBufferSize = 0;
        sendPacket(id, CHAIN_GET_TRIGGER_KEEP_SECONDS, cmdBuffer, cmdBufferSize);

        if (waitForData(id, CHAIN_GET_TRIGGER_KEEP_SECONDS, timeout)) {
            if (checkPacket(reinterpret_cast<const uint8_t *>(returnPacket), returnPacketSize)) {
                *keepSeconds = returnPacket[6];
            } else {
                status = CHAIN_RETURN_PACKET_ERROR;
            }
        } else {
            status = CHAIN_TIMEOUT;
        }
        releaseMutex();
    } else {
        status = CHAIN_BUSY;
    }
    return status;
}