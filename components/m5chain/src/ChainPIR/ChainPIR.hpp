/*
 *SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

#ifndef CHAIN_PIR_HPP_
#define CHAIN_PIR_HPP_

#include <ChainCommon.hpp>

/**
 * @brief CHAIN_PIIR device type code.
 */
#define CHAIN_PIR_DEVICE_TYPE_CODE (0x0009)

typedef enum {
    CHAIN_PIR_NO_PERSON = 0x00, /**< Status indicating the PIR is not detecting any person. */
    CHAIN_PIR_PERSON    = 0x01, /**< Status indicating the PIR is detecting a person. */
} pir_detect_result_t;          /**< Enumeration for recording the PIR's current detect result of the pir device. */

typedef enum {
    CHAIN_PIR_REPORT_PERSON_LEAVE = 0x0500, /**< Status indicating the PIR is detecting a person leave. */
    CHAIN_PIR_REPORT_PERSON_COME  = 0x0501, /**< Status indicating the PIR is detecting a person come. */
} pir_detect_report_t; /**< Enumeration for recording the PIR's current detect result of the pir device at Report Mode.
                        */

/**
 * @brief Chain PIR detect mode enumeration.
 */
typedef enum {
    CHAIN_DETECT_NONE_REPORT_MODE = 0x00, /**< None report mode. */
    CHAIN_DETECT_REPORT_MODE      = 0x01, /**< Report mode. */
} chain_detect_mode_t;

/**
 * @brief Enumeration for CHAIN_SWITCH device commands.
 *
 * This enumeration defines command codes for various operations of the CHAIN_SWITCH device.
 */
typedef enum {
    CHAIN_PIR_GET_IR_STATUS           = 0x37, /**< Command to get the current IR induction status */
    CHAIN_PIR_SET_AUTO_SEND_IR_STATUS = 0xE1, /**< Command to set the auto-send ir status */
    CHAIN_PIR_GET_AUTO_SEND_IR_STATUS = 0xE2, /**< Command to get the auto-send ir status */
    CHAIN_SET_TRIGGER_KEEP_SECONDS    = 0xE3, /**< Command to set kepp seconds when Person-Leave Status trigger */
    CHAIN_GET_TRIGGER_KEEP_SECONDS    = 0xE4, /**< Command to get kepp seconds when Person-Leave Status trigger */
} CHAIN_PIR_CMD_T;                            /**< Command types for Chain_PIR device operations */

class ChainPIR : virtual public ChainCommon {
public:
    /**
     * @brief Retrieves the current slip's switch status of the Switch device at the specified position in
     * the chain.
     *
     * This function gets the current slip direction change status of the Switch device. It determines whether the
     * rotation direction is increasing or decreasing in the down-to-up direction.
     *
     * @param id The position of the Switch device in the chain (starting from 1).
     * @param ir_status Pointer to store the current slip's switch status. (1 for open, 0 for close).
     * @param timeout The timeout duration for the operation in milliseconds (default is 100ms).
     *
     * @return The operation status (e.g., `CHAIN_OK`, `CHAIN_TIMEOUT`, etc.).
     */
    chain_status_t getIRStatus(uint8_t id, pir_detect_result_t* ir_status, unsigned long timeout = 100);

    /**
     * @brief Sets the PIR detect trigger mode for the PIR device at the specified position in the chain.
     *
     * This function sets the PIR detect trigger mode for the PIR device. The mode can either be
     * 'CHAIN_DETECT_NONE_REPORT_MODE' or 'CHAIN_DETECT_REPORT_MODE'.
     *
     * @param id The position of the PIR device in the chain (starting from 1).
     * @param auto_status The desired trigger mode (CHAIN_DETECT_REPORT_MODE or CHAIN_DETECT_NONE_REPORT_MODE).
     * @param operationStatus Pointer to store the operation status.
     * @param timeout The timeout duration for the operation in milliseconds (default is 100ms).
     *
     * @return Operation status (e.g., CHAIN_OK, CHAIN_BUSY, etc.).
     */
    chain_status_t setPIRDetectTriggerMode(uint8_t id, chain_detect_mode_t auto_status, uint8_t* operationStatus,
                                           unsigned long timeout = 100);

    /**
     * @brief Gets the PIR detect trigger mode for the PIR device at the specified position in the chain.
     *
     * This function retrieves the current PIR detect trigger mode of the PIR device.
     *
     * @param id The position of the PIR device in the chain (starting from 1).
     * @param auto_status Pointer to store the current trigger mode (CHAIN_DETECT_REPORT_MODE or
     * CHAIN_DETECT_NONE_REPORT_MODE).
     * @param timeout The timeout duration for the operation in milliseconds (default is 100ms).
     *
     * @return Operation status (e.g., CHAIN_OK, CHAIN_BUSY, etc.).
     */
    chain_status_t getPIRDetectTriggerMode(uint8_t id, chain_detect_mode_t* auto_status, unsigned long timeout = 100);

    /**
     * @brief Gets the Chain_Switch type code.
     *
     * This function returns the type code for the Chain_Switch device, used to identify the device type.
     *
     * @return Returns the Chain_Switch device type code.
     */
    uint16_t getPIRTypeCode(void);

    /**
     * @brief Gets the PIR detect report status.
     *
     * This function retrieves the PIR detect report status, indicating whether the PIR is detecting a person or not.
     *
     * @param id The position of the PIR device in the chain (starting from 1).
     * @param triggerStatus Pointer to store the PIR detect report status (CHAIN_PIR_REPORT_NO_PERSON or
     * CHAIN_PIR_REPORT_PERSON).
     *
     * @return Returns true if the operation was successful, otherwise returns false.
     */
    bool getPIRDetectTrigger(uint8_t id, pir_detect_report_t* triggerStatus);

    /**
     * @brief Sets keep seconds of the PIR detecting no-moving trigger and IRStatus changed in the chain.
     *
     * This function sets keep seconds of the PIR detecting no-moving trigger and IRStatus changed for the PIR device.
     * The times is limited to 60 seconds, default is 5 seconds.
     *
     * @param id The position of the PIR device in the chain (starting from 1).
     * @param keepSeconds Seconds to keep Tirgger status and IRStatus value. limited to 0~255 seconds.
     * @param operationStatus Pointer to store the operation status.
     * @param timeout The timeout duration for the operation in milliseconds (default is 100ms).
     *
     * @return Operation status (e.g., CHAIN_OK, CHAIN_BUSY, etc.).
     */
    chain_status_t setPIRLeaveTriggerKeepSeconds(uint8_t id, uint8_t keepSeconds, uint8_t* operationStatus,
                                                 uint8_t saveToFlash = 0, unsigned long timeout = 100);

    /**
     * @brief Gets keep seconds of the PIR detecting person-come trigger and IRStatus changed in the chain.
     *
     * This function gets keep seconds of the PIR detecting person-come trigger and IRStatus changed for the PIR device.
     * The times is limited to 255 seconds, default is 5 seconds.
     *
     * @param id The position of the PIR device in the chain (starting from 1).
     * @param keepSeconds Pointer to store the keep seconds.
     * @param timeout The timeout duration for the operation in milliseconds (default is 100ms).
     *
     * @return Operation status (e.g., CHAIN_OK, CHAIN_BUSY, etc.).
     */
    chain_status_t getPIRComeTriggerKeepSeconds(uint8_t id, uint8_t* keepSeconds, unsigned long timeout = 100);

private:
};

#endif  // CHAIN_PIR_HPP_