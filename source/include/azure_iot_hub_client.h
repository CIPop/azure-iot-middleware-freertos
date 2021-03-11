/* Copyright (c) Microsoft Corporation. All rights reserved. */
/* SPDX-License-Identifier: MIT */

/**
 * @file azure_iot_hub_client.h
 *
 */

#ifndef AZURE_IOT_HUB_CLIENT_H
#define AZURE_IOT_HUB_CLIENT_H

/* Transport interface include. */
#include "FreeRTOS.h"

#include "azure_iot.h"

#include "azure_iot_mqtt.h"
#include "azure_iot_mqtt_port.h"

typedef struct AzureIoTHubClient * AzureIoTHubClientHandle_t;

typedef enum AzureIoTHubMessageType
{
    AZURE_IOT_HUB_CLOUD_MESSAGE = 1,              /*/< The message is a Cloud message. */
    AZURE_IOT_HUB_DIRECT_METHOD_MESSAGE,          /*/< The message is a direct method message. */
    AZURE_IOT_HUB_TWIN_GET_MESSAGE,               /*/< The message is a twin get response (payload contains the twin document). */
    AZURE_IOT_HUB_TWIN_REPORTED_RESPONSE_MESSAGE, /*/< The message is a twin reported property status response. */
    AZURE_IOT_HUB_TWIN_DESIRED_PROPERTY_MESSAGE,  /*/< The message is a twin desired property message (incoming from the service). */
} AzureIoTHubMessageType_t;

typedef enum AzureIoTHubClientError
{
    AZURE_IOT_HUB_CLIENT_SUCCESS = 0,         /*/< Success. */
    AZURE_IOT_HUB_CLIENT_INVALID_ARGUMENT,    /*/< Input argument does not comply with the expected range of values. */
    AZURE_IOT_HUB_CLIENT_PENDING,             /*/< The status of the operation is pending. */
    AZURE_IOT_HUB_CLIENT_OUT_OF_MEMORY,       /*/< The system is out of memory. */
    AZURE_IOT_HUB_CLIENT_INIT_FAILED,         /*/< The initialization failed. */
    AZURE_IOT_HUB_CLIENT_SUBACK_WAIT_TIMEOUT, /*/< There was timeout while waiting for SUBACK. */
    AZURE_IOT_HUB_CLIENT_FAILED,              /*/< There was a failure. */
} AzureIoTHubClientResult_t;

typedef enum AzureIoTHubMessageStatus
{
    /* Default, unset value */
    AZURE_IOT_STATUS_UNKNOWN = 0,

    /* Service success codes */
    AZURE_IOT_STATUS_OK = 200,
    AZURE_IOT_STATUS_ACCEPTED = 202,
    AZURE_IOT_STATUS_NO_CONTENT = 204,

    /* Service error codes */
    AZURE_IOT_STATUS_BAD_REQUEST = 400,
    AZURE_IOT_STATUS_UNAUTHORIZED = 401,
    AZURE_IOT_STATUS_FORBIDDEN = 403,
    AZURE_IOT_STATUS_NOT_FOUND = 404,
    AZURE_IOT_STATUS_NOT_ALLOWED = 405,
    AZURE_IOT_STATUS_NOT_CONFLICT = 409,
    AZURE_IOT_STATUS_PRECONDITION_FAILED = 412,
    AZURE_IOT_STATUS_REQUEST_TOO_LARGE = 413,
    AZURE_IOT_STATUS_UNSUPPORTED_TYPE = 415,
    AZURE_IOT_STATUS_THROTTLED = 429,
    AZURE_IOT_STATUS_CLIENT_CLOSED = 499,
    AZURE_IOT_STATUS_SERVER_ERROR = 500,
    AZURE_IOT_STATUS_BAD_GATEWAY = 502,
    AZURE_IOT_STATUS_SERVICE_UNAVAILABLE = 503,
    AZURE_IOT_STATUS_TIMEOUT = 504,
} AzureIoTHubMessageStatus_t;

/*
 *  Cloud Message STRUCTS
 */
typedef struct AzureIoTHubClientCloudMessageRequest
{
    const void * messagePayload;            /*/< The pointer to the message payload. */
    size_t payloadLength;                   /*/< The length of the message payload. */

    AzureIoTMessageProperties_t properties; /*/< The bag of properties received with the message. */
} AzureIoTHubClientCloudMessageRequest_t;

/*
 *  Method STRUCTS
 */
typedef struct AzureIoTHubClientMethodRequest
{
    const void * messagePayload; /*/< The pointer to the message payload. */
    size_t payloadLength;        /*/< The length of the message payload. */

    const uint8_t * requestId;   /*/< The pointer to the request id. */
    size_t requestIdLength;      /*/< The length of the request id. */

    const uint8_t * methodName;  /*/< The name of the method to invoke. */
    size_t methodNameLength;     /*/< The length of the method name. */
} AzureIoTHubClientMethodRequest_t;

/*
 *  Twin STRUCTS
 */
typedef struct AzureIoTHubClientTwinResponse
{
    AzureIoTHubMessageType_t messageType;     /*/< The type of message received. */

    const void * messagePayload;              /*/< The pointer to the message payload. */
    size_t payloadLength;                     /*/< The length of the message payload. */

    uint32_t requestId;                       /*/< request id. */

    AzureIoTHubMessageStatus_t messageStatus; /*/< The operation status. */

    const uint8_t * version;                  /*/< The pointer to the twin document version. */
    size_t versionLength;                     /*/< The length of the twin document version. */
} AzureIoTHubClientTwinResponse_t;

/* Typedef for the CloudMessage callback */
typedef void ( * AzureIoTHubClientCloudMessageCallback_t ) ( struct AzureIoTHubClientCloudMessageRequest * pxMessage,
                                                             void * pvContext );

/* Typedef for the method callback */
typedef void ( * AzureIoTHubClientMethodCallback_t ) ( struct AzureIoTHubClientMethodRequest * pxMessage,
                                                       void * pvContext );

/* Typedef for the twin callback */
typedef void ( * AzureIoTHubClientTwinCallback_t ) ( struct AzureIoTHubClientTwinResponse * pxMessage,
                                                     void * pvContext );

/* Receive context to be used internally to the processing of messages */
typedef struct AzureIoTHubClientReceiveContext
{
    struct
    {
        uint16_t state;
        uint16_t mqttSubPacketId;
        uint32_t ( * process_function )( struct AzureIoTHubClientReceiveContext * pxContext,
                                         AzureIoTHubClientHandle_t xAzureIoTHubClientHandle,
                                         void * pvPublishInfo );

        void * callback_context;
        union
        {
            AzureIoTHubClientCloudMessageCallback_t cloudMessageCallback;
            AzureIoTHubClientMethodCallback_t methodCallback;
            AzureIoTHubClientTwinCallback_t twinCallback;
        } callbacks;
    } _internal;
} AzureIoTHubClientReceiveContext_t;

typedef struct AzureIoTHubClientOptions
{
    const uint8_t * pModuleId;  /*/ The module id to use for this device. */
    uint32_t moduleIdLength;    /*/ The length of the module id. */

    const uint8_t * pModelId;   /*/ The model ID used to identify the capabilities of a device based on the Digital Twin document. */
    uint32_t modelIdLength;     /*/ The length of the model id. */

    const uint8_t * pUserAgent; /*/ The user agent to use for this device. */
    uint32_t userAgentLength;   /*/ The length of the user agent. */
} AzureIoTHubClientOptions_t;

typedef struct AzureIoTHubClient
{
    struct
    {
        AzureIoTMQTT_t xMQTTContext;

        uint8_t * iot_hub_client_scratch_buffer;
        uint32_t iot_hub_client_scratch_buffer_length;
        az_iot_hub_client iot_hub_client_core;

        const uint8_t * hostname;
        uint32_t hostnameLength;
        const uint8_t * deviceId;
        uint32_t deviceIdLength;
        const uint8_t * azure_iot_hub_client_symmetric_key;
        uint32_t azure_iot_hub_client_symmetric_key_length;

        uint32_t ( * azure_iot_hub_client_token_refresh )( struct AzureIoTHubClient * xAzureIoTHubClientHandle,
                                                           uint64_t ullExpiryTimeSecs,
                                                           const uint8_t * ucKey,
                                                           uint32_t ulKeyLen,
                                                           uint8_t * pucSASBuffer,
                                                           uint32_t ulSasBufferLen,
                                                           uint32_t * pulSaSLength );
        AzureIoTGetHMACFunc_t azure_iot_hub_client_hmac_function;
        AzureIoTGetCurrentTimeFunc_t azure_iot_hub_client_time_function;

        uint32_t currentRequestId;

        AzureIoTHubClientReceiveContext_t xReceiveContext[ 3 ];
    } _internal;
} AzureIoTHubClient_t;

/**
 * @brief Initialize the Azure IoT Hub Options with default values.
 *
 * @param[out] pxHubClientOptions The #AzureIoTHubClientOptions_t instance to set with default values.
 * @return An #AzureIoTHubClientResult_t with the result of the operation.
 */
AzureIoTHubClientResult_t AzureIoTHubClient_OptionsInit( AzureIoTHubClientOptions_t * pxHubClientOptions );

/**
 * @brief Initialize the Azure IoT Hub Client.
 *
 * @param[out] xAzureIoTHubClientHandle The #AzureIoTHubClientHandle_t to use for this call.
 * @param[in] pucHostname The IoT Hub Hostname.
 * @param[in] ulHostnameLength The length of the IoT Hub Hostname.
 * @param[in] pucDeviceId The Device ID. If the ID contains any of the following characters, they must
 * be percent-encoded as follows:
 *         - `/` : `%2F`
 *         - `%` : `%25`
 *         - `#` : `%23`
 *         - `&` : `%26`
 * @param[in] ulDeviceIdLength The length of the device id.
 * @param[in] pxHubClientOptions The #AzureIoTHubClientOptions_t for the IoT Hub client instance.
 * @param[in] pucBuffer The buffer to use for MQTT messages.
 * @param[in] ulBufferLength The length of the \p pucBuffer.
 * @param[in] xGetTimeFunction A function pointer to a function which gives the current epoch time.
 * @param[in] pxTransportInterface The transport interface to use for the MQTT library.
 * @return An #AzureIoTHubClientResult_t with the result of the operation.
 */
AzureIoTHubClientResult_t AzureIoTHubClient_Init( AzureIoTHubClientHandle_t xAzureIoTHubClientHandle,
                                                  const uint8_t * pucHostname,
                                                  uint32_t ulHostnameLength,
                                                  const uint8_t * pucDeviceId,
                                                  uint32_t ulDeviceIdLength,
                                                  AzureIoTHubClientOptions_t * pxHubClientOptions,
                                                  uint8_t * pucBuffer,
                                                  uint32_t ulBufferLength,
                                                  AzureIoTGetCurrentTimeFunc_t xGetTimeFunction,
                                                  const AzureIoTTransportInterface_t * pxTransportInterface );

/**
 * @brief Deinitialize the Azure IoT Hub Client.
 *
 * @param xAzureIoTHubClientHandle The #AzureIoTHubClientHandle_t to use for this call.
 */
void AzureIoTHubClient_Deinit( AzureIoTHubClientHandle_t xAzureIoTHubClientHandle );

/**
 * @brief Set the symmetric key to use for authentication.
 *
 * @param[in] xAzureIoTHubClientHandle The #AzureIoTHubClientHandle_t to use for this call.
 * @param[in] pucSymmetricKey The symmetric key to use for the connection.
 * @param[in] ulSymmetricKeyLength The length of the \p pucSymmetricKey.
 * @param[in] xHmacFunction The #AzureIoTGetHMACFunc_t function pointer to a function which computes the HMAC256 over a set of bytes.
 * @return An #AzureIoTHubClientResult_t with the result of the operation.
 */
AzureIoTHubClientResult_t AzureIoTHubClient_SymmetricKeySet( AzureIoTHubClientHandle_t xAzureIoTHubClientHandle,
                                                             const uint8_t * pucSymmetricKey,
                                                             uint32_t ulSymmetricKeyLength,
                                                             AzureIoTGetHMACFunc_t xHmacFunction );

/**
 * @brief Connect via MQTT to the IoT Hub endpoint.
 *
 * @param[in] xAzureIoTHubClientHandle The #AzureIoTHubClientHandle_t to use for this call.
 * @param[in] cleanSession A boolean dictating whether to connect with a clean session or not.
 * @param[in] ulTimeoutMilliseconds The maximum time in milliseconds to wait for a CONNACK.
 * @return An #AzureIoTHubClientResult_t with the result of the operation.
 */
AzureIoTHubClientResult_t AzureIoTHubClient_Connect( AzureIoTHubClientHandle_t xAzureIoTHubClientHandle,
                                                     bool cleanSession,
                                                     uint32_t ulTimeoutMilliseconds );

/**
 * @brief Disconnect from the IoT Hub endpoint
 *
 * @param[in] xAzureIoTHubClientHandle The #AzureIoTHubClientHandle_t to use for this call.
 * @return An #AzureIoTHubClientResult_t with the result of the operation.
 */
AzureIoTHubClientResult_t AzureIoTHubClient_Disconnect( AzureIoTHubClientHandle_t xAzureIoTHubClientHandle );

/**
 * @brief Send telemetry data to IoT Hub.
 *
 * @param[in] xAzureIoTHubClientHandle The #AzureIoTHubClientHandle_t to use for this call.
 * @param[in] pucTelemetryData The pointer to the buffer of telemetry data.
 * @param[in] ulTelemetryDataLength The length of the buffer to send as telemetry.
 * @param[in] pxProperties The property bag to send with the message.
 * @return An #AzureIoTHubClientResult_t with the result of the operation.
 */
AzureIoTHubClientResult_t AzureIoTHubClient_TelemetrySend( AzureIoTHubClientHandle_t xAzureIoTHubClientHandle,
                                                           const uint8_t * pucTelemetryData,
                                                           uint32_t ulTelemetryDataLength,
                                                           AzureIoTMessageProperties_t * pxProperties );

/**
 * @brief Receive any incoming MQTT messages from and manage the MQTT connection to IoT Hub.
 *
 * @note This API will receive any messages sent to the device and manage the connection such as send
 * `PING` messages.
 *
 * @param[in] xAzureIoTHubClientHandle The #AzureIoTHubClientHandle_t to use for this call.
 * @param[in] ulTimeoutMilliseconds Minimum time (in millisecond) for the loop to run. If `0` is passed, it will only run once.
 * @return An #AzureIoTHubClientResult_t with the result of the operation.
 */
AzureIoTHubClientResult_t AzureIoTHubClient_ProcessLoop( AzureIoTHubClientHandle_t xAzureIoTHubClientHandle,
                                                         uint32_t ulTimeoutMilliseconds );

/**
 * @brief Subscribe to cloud to device messages.
 *
 * @param[in] xAzureIoTHubClientHandle The #AzureIoTHubClientHandle_t to use for this call.
 * @param[in] xCloudMessageCallback The #AzureIoTHubClientCloudMessageCallback_t to invoke when a CloudMessage messages arrive.
 * @param[in] prvCallbackContext A pointer to a context to pass to the callback.
 * @param[in] ulTimeoutMilliseconds Timeout in millisecond for subscribe operation to complete.
 * @return An #AzureIoTHubClientResult_t with the result of the operation.
 */
AzureIoTHubClientResult_t AzureIoTHubClient_CloudMessageSubscribe( AzureIoTHubClientHandle_t xAzureIoTHubClientHandle,
                                                                   AzureIoTHubClientCloudMessageCallback_t xCloudMessageCallback,
                                                                   void * prvCallbackContext,
                                                                   uint32_t ulTimeoutMilliseconds );

/**
 * @brief Subscribe to direct methods.
 *
 * @param[in] xAzureIoTHubClientHandle The #AzureIoTHubClientHandle_t to use for this call.
 * @param[in] xMethodCallback The #AzureIoTHubClientMethodCallback_t to invoke when direct method messages arrive.
 * @param[in] prvCallbackContext A pointer to a context to pass to the callback.
 * @param[in] ulTimeoutMilliseconds Timeout in millisecond for Subscribe operation to complete.
 * @return An #AzureIoTHubClientResult_t with the result of the operation.
 */
AzureIoTHubClientResult_t AzureIoTHubClient_DirectMethodSubscribe( AzureIoTHubClientHandle_t xAzureIoTHubClientHandle,
                                                                   AzureIoTHubClientMethodCallback_t xMethodCallback,
                                                                   void * prvCallbackContext,
                                                                   uint32_t ulTimeoutMilliseconds );

/**
 * @brief Subscribe to device twin.
 *
 * @param[in] xAzureIoTHubClientHandle The #AzureIoTHubClientHandle_t to use for this call.
 * @param[in] xTwinCallback The #AzureIoTHubClientTwinCallback_t to invoke when device twin messages arrive.
 * @param[in] prvCallbackContext A pointer to a context to pass to the callback.
 * @param[in] ulTimeoutMilliseconds Timeout in millisecond for Subscribe operation to complete.
 * @return An #AzureIoTHubClientResult_t with the result of the operation.
 */
AzureIoTHubClientResult_t AzureIoTHubClient_DeviceTwinSubscribe( AzureIoTHubClientHandle_t xAzureIoTHubClientHandle,
                                                                 AzureIoTHubClientTwinCallback_t xTwinCallback,
                                                                 void * prvCallbackContext,
                                                                 uint32_t ulTimeoutMilliseconds );

/**
 * @brief Unsubscribe from cloud to device messages.
 *
 * @param[in] xAzureIoTHubClientHandle The #AzureIoTHubClientHandle_t to use for this call.
 * @return An #AzureIoTHubClientResult_t with the result of the operation.
 */
AzureIoTHubClientResult_t AzureIoTHubClient_CloudMessageUnsubscribe( AzureIoTHubClientHandle_t xAzureIoTHubClientHandle );

/**
 * @brief Unsubscribe from direct methods.
 *
 * @param[in] xAzureIoTHubClientHandle The #AzureIoTHubClientHandle_t to use for this call.
 * @return An #AzureIoTHubClientResult_t with the result of the operation.
 */
AzureIoTHubClientResult_t AzureIoTHubClient_DirectMethodUnsubscribe( AzureIoTHubClientHandle_t xAzureIoTHubClientHandle );

/**
 * @brief Unsubscribe from device twin.
 *
 * @param[in] xAzureIoTHubClientHandle The #AzureIoTHubClientHandle_t to use for this call.
 * @return An #AzureIoTHubClientResult_t with the result of the operation.
 */
AzureIoTHubClientResult_t AzureIoTHubClient_DeviceTwinUnsubscribe( AzureIoTHubClientHandle_t xAzureIoTHubClientHandle );

/**
 * @brief Send a response to a received direct method message.
 *
 * @param[in] xAzureIoTHubClientHandle The #AzureIoTHubClientHandle_t to use for this call.
 * @param[in] pxMessage The pointer to the #AzureIoTHubClientMethodRequest_t to which a response is being sent.
 * @param[in] ulStatus A code that indicates the result of the method, as defined by the user.
 * @param[in] pucMethodPayload __[nullable]__ An optional method response payload.
 * @param[in] ulMethodPayloadLength The length of the method response payload.
 * @return An #AzureIoTHubClientResult_t with the result of the operation.
 */
AzureIoTHubClientResult_t AzureIoTHubClient_SendMethodResponse( AzureIoTHubClientHandle_t xAzureIoTHubClientHandle,
                                                                const AzureIoTHubClientMethodRequest_t * pxMessage,
                                                                uint32_t ulStatus,
                                                                const uint8_t * pucMethodPayload,
                                                                uint32_t ulMethodPayloadLength );

/**
 * @brief Send reported device twin properties to Azure IoT Hub.
 *
 * @param[in] xAzureIoTHubClientHandle The #AzureIoTHubClientHandle_t to use for this call.
 * @param[in] pucReportedPayload The payload of properly formatted, reported properties.
 * @param[in] ulReportedPayloadLength The length of the reported property payload.
 * @param[out] pulRequestId Pointer to request Id used to send the twin reported property.
 * @return An #AzureIoTHubClientResult_t with the result of the operation.
 */
AzureIoTHubClientResult_t AzureIoTHubClient_DeviceTwinReportedSend( AzureIoTHubClientHandle_t xAzureIoTHubClientHandle,
                                                                    const uint8_t * pucReportedPayload,
                                                                    uint32_t ulReportedPayloadLength,
                                                                    uint32_t * pulRequestId );

/**
 * @brief Request to get the device twin document.
 *
 * The answer to the request will be returned via the #AzureIoTHubClientTwinCallback_t which was passed
 * in the AzureIoTHubClient_DeviceTwinSubscribe() call. The type of message will be #AZURE_IOT_HUB_TWIN_GET_MESSAGE
 * and the payload (on success) will be the twin document.
 *
 * @param[in] xAzureIoTHubClientHandle The #AzureIoTHubClientHandle_t to use for this call.
 * @return An #AzureIoTHubClientResult_t with the result of the operation.
 */
AzureIoTHubClientResult_t AzureIoTHubClient_DeviceTwinGet( AzureIoTHubClientHandle_t xAzureIoTHubClientHandle );

#endif /* AZURE_IOT_HUB_CLIENT_H */
