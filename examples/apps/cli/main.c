/*
 *  Copyright (c) 2016, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/***************************************************************************************************
 * @section Includes
 **************************************************************************************************/

#include <assert.h>
#include <openthread-core-config.h>
#include <openthread/config.h>
#include <openthread/cli.h>
#include <openthread/diag.h>
#include <openthread/tasklet.h>
#include <openthread/platform/logging.h>
#include "openthread-system.h"
#include "cli/cli_config.h"
#include "common/code_utils.hpp"
#include <openthread/instance.h>
#include <openthread/thread.h>
#include <openthread/thread_ftd.h>
#include <string.h>
#include <stdio.h>
#include <openthread/message.h>
#include <openthread/udp.h>
#include "utils/code_utils.h"
#include <openthread/dataset_ftd.h>
#include <openthread/ip6.h>
#include <openthread/platform/time.h>
#include <openthread/ping_sender.h>

/***************************************************************************************************
 * @section Declarations
 **************************************************************************************************/

#define UDP_COMMAND_PORT 1212
#define UDP_DATA_PORT 1213
#define MAX_UDP_PARAMETER_LEN 64

static char UDP_MULTICAST_ADDR[] = "ff03::1";
int32_t udpCounter = 0;

void handleNetifStateChanged(uint32_t aFlags, void *aContext);
void initNetworkConfiguration(otInstance *aInstance, char *aNetworkName, int channel, otPanId aOtPanID, uint8_t aKey[OT_NETWORK_KEY_SIZE]);
static void thread_customCommands_init(void);
static void initUdp(otInstance *aInstance);
void sendUdp(otInstance *aInstance, char* ipDestination,  char *payload, int port);
static void handleButtonInterrupt(otInstance *aInstance);
void handleUdpReceive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo);
static otUdpSocket sUdpSocket;
extern void otAppCliInit(otInstance *aInstance);
static void leaveNetwork(void *aContext, uint8_t aArgsLength, char *aArgs[]);
static void setChild(void *aContext, uint8_t aArgsLength, char *aArgs[]);
static void setRouter(void *aContext, uint8_t aArgsLength, char *aArgs[]);
static void initThreadCustomCommands(void *aContext);
void parsePayload(otMessage *aMessage, char *destination, char *command, char *argument);
void sendCommandUDP(otInstance *aInstance, char* ipDestination,  char *euiDestination, char *command, char *argument);
void sendDataUDP(otInstance *aInstance, char* ipDestination, char* aCommand, char* aMessage);
void getEuidEnd(otInstance *aContext, char aEuid[2]);
void HandlePingStatistics(const otPingSenderStatistics *aStatistics, void *aContext);



/***************************************************************************************************
 * @section misc
 **************************************************************************************************/

/*
 * Provide, if required an "otPlatLog()" function
 */
#if OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_APP
void otPlatLog(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aFormat, ...)
{
    va_list ap;
    va_start(ap, aFormat);
    otCliPlatLogv(aLogLevel, aLogRegion, aFormat, ap);
    va_end(ap);
}

void otPlatLogLine(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aLogLine)
{
    otCliPlatLogLine(aLogLevel, aLogRegion, aLogLine);
}

#endif

#if OPENTHREAD_EXAMPLES_SIMULATION
#include <setjmp.h>
#include <unistd.h>

jmp_buf gResetJump;

void __gcov_flush();
#endif

#ifndef OPENTHREAD_ENABLE_COVERAGE
#define OPENTHREAD_ENABLE_COVERAGE 0
#endif

#if OPENTHREAD_CONFIG_HEAP_EXTERNAL_ENABLE
void *otPlatCAlloc(size_t aNum, size_t aSize)
{
    return calloc(aNum, aSize);
}

void otPlatFree(void *aPtr)
{
    free(aPtr);
}
#endif

void otTaskletsSignalPending(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
}

#if OPENTHREAD_POSIX && !defined(FUZZING_BUILD_MODE_UNSAFE_FOR_PRODUCTION)
static void ProcessExit(void *aContext, uint8_t aArgsLength, char *aArgs[])
{
    OT_UNUSED_VARIABLE(aContext);
    OT_UNUSED_VARIABLE(aArgsLength);
    OT_UNUSED_VARIABLE(aArgs);

    exit(EXIT_SUCCESS);
}

#endif

/***************************************************************************************************
 * @section Main
 **************************************************************************************************/

int main(int argc, char *argv[])
{
    otInstance *instance;

#if OPENTHREAD_EXAMPLES_SIMULATION
    if (setjmp(gResetJump))
    {
        alarm(0);
#if OPENTHREAD_ENABLE_COVERAGE
        __gcov_flush();
#endif
        execvp(argv[0], argv);
    }
#endif

#if OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE
    size_t   otInstanceBufferLength = 0;
    uint8_t *otInstanceBuffer       = NULL;
#endif

pseudo_reset:

    otSysInit(argc, argv);

#if OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE
    // Call to query the buffer size
    (void)otInstanceInit(NULL, &otInstanceBufferLength);

    // Call to allocate the buffer
    otInstanceBuffer = (uint8_t *)malloc(otInstanceBufferLength);
    assert(otInstanceBuffer);

    // Initialize OpenThread with the buffer
    instance = otInstanceInit(otInstanceBuffer, &otInstanceBufferLength);
#else
    instance = otInstanceInitSingle();
#endif
    assert(instance);

    otAppCliInit(instance);

    //ADD INITS BELOW

    /* Register Thread state change handler */
    otSetStateChangedCallback(instance, handleNetifStateChanged, instance);

    /* Override default network credentials */
    uint8_t key[OT_NETWORK_KEY_SIZE] = {0x12, 0x34, 0xC0, 0xDE, 0x1A, 0xB5, 0x12, 0x34, 0xC0, 0xDE, 0x1A, 0xB5, 0x12, 0x34, 0xC0, 0xDE};
    initNetworkConfiguration(instance, "OTCodelab", 15, 0x2222, key);

    /* init GPIO LEDs and button */
    otSysLedInit();
    otSysButtonInit(handleButtonInterrupt); 

    initThreadCustomCommands(instance);

    /* Start the Thread network interface (CLI cmd > ifconfig up) */
    otIp6SetEnabled(instance, true);

    /* Start the Thread stack (CLI cmd > thread start) */
    otThreadSetEnabled(instance, true);

    initUdp(instance);

    

    //ADD INITS ABOVE

#if OPENTHREAD_POSIX && !defined(FUZZING_BUILD_MODE_UNSAFE_FOR_PRODUCTION)
    otCliSetUserCommands(kCommands, OT_ARRAY_LENGTH(kCommands), instance);
#endif

    while (!otSysPseudoResetWasRequested())
    {
        otTaskletsProcess(instance);
        otSysProcessDrivers(instance);
        otSysButtonProcess(instance);
    }

    otInstanceFinalize(instance);
#if OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE
    free(otInstanceBuffer);
#endif

    goto pseudo_reset;

    return 0;
}

/***************************************************************************************************
 * @section Helpers
 **************************************************************************************************/

/**
 * @brief Get last two characters of EUID
 */
void getEuidEnd(otInstance *aContext, char aEuid[2]){
    otExtAddress aEuid64;
    otLinkGetFactoryAssignedIeeeEui64(aContext, &aEuid64);
    sprintf(aEuid,"%02x", aEuid64.m8[7]);
}

/**
 * @brief Send a UDP remote command datagram
 */
void sendCommandUDP(otInstance *aInstance, char* ipDestination,  char *euiDestination, char *command, char *argument){
    char str[MAX_UDP_PARAMETER_LEN];
    sprintf(str, "%s-%s-%s", euiDestination, command, argument);
    sendUdp(aInstance, ipDestination, str, UDP_COMMAND_PORT);
}

/**
 * @brief Send a UDP data datagram
 */
void sendDataUDP(otInstance *aInstance, char* ipDestination, char* aCommand,  char* aMessage){
    char str[MAX_UDP_PARAMETER_LEN];
    char aEuid[2];
    uint32_t aTime = otPlatTimeGet(); 
    getEuidEnd(aInstance, aEuid);
    sprintf(str, "%s, %d, %s, %d, %s*", aEuid, udpCounter, aCommand, aTime, aMessage);
    sendUdp(aInstance, ipDestination, str, UDP_DATA_PORT);
    udpCounter++;
}


/**
 * @brief Send a UDP datagram
 */
void sendUdp(otInstance *aInstance, char* ipDestination,  char* payload, int port)
{
    otError       error = OT_ERROR_NONE;
    otMessage *   message;
    otMessageInfo messageInfo;
    otIp6Address  destinationAddr;
    
    memset(&messageInfo, 0, sizeof(messageInfo));

    otIp6AddressFromString(ipDestination, &destinationAddr);
    messageInfo.mPeerAddr    = destinationAddr;
    messageInfo.mPeerPort    = port;

    message = otUdpNewMessage(aInstance, NULL);
    otEXPECT_ACTION(message != NULL, error = OT_ERROR_NO_BUFS);

    error = otMessageAppend(message, payload, MAX_UDP_PARAMETER_LEN);
    otEXPECT(error == OT_ERROR_NONE);

    error = otUdpSend(aInstance, &sUdpSocket, message, &messageInfo);

 exit:
    if (error != OT_ERROR_NONE && message != NULL)
    {
        otMessageFree(message);
    }
}

/**
 * @brief read a UDP message output the three arguments
 */
void parsePayload(otMessage *aMessage, char *destination, char *command, char *argument){

    char str[MAX_UDP_PARAMETER_LEN];
    int  length;
    const char delimiter[2] = "-";
    char *token;

    length = otMessageRead(aMessage, otMessageGetOffset(aMessage), str, sizeof(str) - 1);
    str[length] = '\0';

    token = strtok(str, delimiter);
    strcpy(destination, token);
    token = strtok(NULL, delimiter);
    strcpy(command, token);
    token = strtok(NULL, delimiter);
    strcpy(argument, token);

    
}



/***************************************************************************************************
 * @section Handlers
 **************************************************************************************************/

/**
 * @brief Function to handle button push event
 */
void handleButtonInterrupt(otInstance *aInstance)
{
    otCliOutputFormat("Sending UDP multicast\n\r");
    sendDataUDP(aInstance, UDP_MULTICAST_ADDR, "button", "1");
    sendCommandUDP(aInstance, UDP_MULTICAST_ADDR, "ff", "test", "syn");
}

/**
 * @brief Function to handle UDP datagrams received on the listening socket
 */
void handleUdpReceive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    char destination[2];
    char command[16];
    char argument[16];
    char aEuid[2];
    char returnAddressString[64];
    char str[64];
    otError error = OT_ERROR_NONE;

    getEuidEnd(aContext, aEuid);
    parsePayload(aMessage, destination, command, argument);

    if(strcmp(destination, aEuid) != 0 && strcmp(destination, "ff") != 0){
        return;
    }
    const otIp6Address *aAddress = &aMessageInfo->mPeerAddr;
    otIp6AddressToString(aAddress, returnAddressString, OT_IP6_ADDRESS_STRING_SIZE);
    otCliOutputFormat("%d bytes from ", otMessageGetLength(aMessage) - otMessageGetOffset(aMessage));
    otCliOutputFormat("%s\n\r", returnAddressString);
    otCliOutputFormat("%s-%s-%s\n\r", destination, command, argument);

    if(strcmp(command, "test" ) == 0 && strcmp(argument, "syn") == 0){
        sendCommandUDP(aContext, returnAddressString, "ff", "test", "ack");
    }

    else if(strcmp(command, "txpower") == 0 && strcmp(argument, "get") == 0){
        int8_t aPower;
        otPlatRadioGetTransmitPower(aContext, &aPower);
        sprintf(str, "%d", aPower);
        sendDataUDP(aContext, returnAddressString, command, str);
    }

    else if(strcmp(command, "ip") == 0 && strcmp(argument, "get") == 0){
        const otIp6Address *aRloc = otThreadGetRloc(aContext);
        char aRlocString[16];
        otIp6AddressToString(aRloc, aRlocString, OT_IP6_ADDRESS_STRING_SIZE);
        sendDataUDP(aContext, returnAddressString, command, aRlocString);
    }

    else if(strcmp(command, "ping") == 0){
        otPingSenderConfig config;
        memset(&config, 0, sizeof(config));
        otIp6Address aDestination;

        otIp6AddressFromString(argument, &aDestination);

        config.mDestination         = aDestination;
        memcpy(&config.mSource, otThreadGetRloc(aContext), 16);
        config.mStatisticsCallback  = HandlePingStatistics;
        config.mCallbackContext     = aContext;
 
        error = otPingSenderPing(aContext, &config);

        otCliOutputFormat("%s\r\n",otThreadErrorToString(error));
    }

    else if(strcmp(command, "macfilteradd") ==0 ){
        sprintf(str, "macfilter addr denylist\0");
        otCliInputLine(str);
        sprintf(str, "macfilter addr add %s\0", argument);
        otCliInputLine(str);
        otCliOutputFormat("added filter for %s\n\r", argument);
    }

    else if(strcmp(command, "macfilterremove") ==0 ){
        sprintf(str, "macfilter addr remove %s\0", argument);
        otCliInputLine(str);
        otCliOutputFormat("removed filter for %s\n\r", argument);
    }

}

void HandlePingStatistics(const otPingSenderStatistics *aStatistics, void *aContext){
    otCliOutputFormat("%u packets transmitted, %u packets received.", aStatistics->mSentCount, aStatistics->mReceivedCount);
}

/**
 * @brief handler for network state changes
 */
void handleNetifStateChanged(uint32_t aFlags, void *aContext)
{
   if ((aFlags & OT_CHANGED_THREAD_ROLE) != 0)
   {
       otDeviceRole changedRole = otThreadGetDeviceRole(aContext);

       switch (changedRole)
       {
       case OT_DEVICE_ROLE_LEADER:
           otSysLedSet(1, true);
           otSysLedSet(2, false);
           otSysLedSet(3, false);
           break;

       case OT_DEVICE_ROLE_ROUTER:
           otSysLedSet(1, false);
           otSysLedSet(2, true);
           otSysLedSet(3, false);
           break;

       case OT_DEVICE_ROLE_CHILD:
           otSysLedSet(1, false);
           otSysLedSet(2, false);
           otSysLedSet(3, true);
           break;

       case OT_DEVICE_ROLE_DETACHED:
       case OT_DEVICE_ROLE_DISABLED:
           otSysLedSet(1, false);
           otSysLedSet(2, false);
           otSysLedSet(3, false);
           break;
        }
    }
}

/***************************************************************************************************
 * @section Initialization
 **************************************************************************************************/

/**
 * @brief Initialize UDP socket
 */
void initUdp(otInstance *aInstance)
{
    otSockAddr  listenSockAddr;
    otNetifIdentifier netif = OT_NETIF_THREAD;

    memset(&sUdpSocket, 0, sizeof(sUdpSocket));
    memset(&listenSockAddr, 0, sizeof(listenSockAddr));

    listenSockAddr.mPort    = UDP_COMMAND_PORT;

    otUdpOpen(aInstance, &sUdpSocket, handleUdpReceive, aInstance);
    otUdpBind(aInstance, &sUdpSocket, &listenSockAddr, netif);
}

/**
 * @brief Override default network settings, such as panid, so the devices can join a network
 */
void initNetworkConfiguration(otInstance *aInstance, char *aNetworkName, int aChannel, otPanId aOtPanID, uint8_t aKey[OT_NETWORK_KEY_SIZE])
{
    otOperationalDataset aDataset;

    memset(&aDataset, 0, sizeof(otOperationalDataset));

    /*
     * Fields that can be configured in otOperationDataset to override defaults:
     *     Network Name, Mesh Local Prefix, Extended PAN ID, PAN ID, Delay Timer,
     *     Channel, Channel Mask Page 0, Network Key, PSKc, Security Policy
     */
    aDataset.mActiveTimestamp                      = 1;
    aDataset.mComponents.mIsActiveTimestampPresent = true;

    /* Set Channel to 15 */
    aDataset.mChannel                      = aChannel;
    aDataset.mComponents.mIsChannelPresent = true;

    /* Set Pan ID to 2222 */
    aDataset.mPanId                      = aOtPanID;
    aDataset.mComponents.mIsPanIdPresent = true;

    /* Set Extended Pan ID to C0DE1AB5C0DE1AB5 */
    uint8_t extPanId[OT_EXT_PAN_ID_SIZE] = {0xC0, 0xDE, 0x1A, 0xB5, 0xC0, 0xDE, 0x1A, 0xB5};
    memcpy(aDataset.mExtendedPanId.m8, extPanId, sizeof(aDataset.mExtendedPanId));
    aDataset.mComponents.mIsExtendedPanIdPresent = true;

    /* Set network key to 1234C0DE1AB51234C0DE1AB51234C0DE */
    memcpy(aDataset.mNetworkKey.m8, aKey, sizeof(aDataset.mNetworkKey));
    aDataset.mComponents.mIsNetworkKeyPresent = true;

    /* Set Network Name to OTCodelab */
    size_t length = strlen(aNetworkName);
    assert(length <= OT_NETWORK_NAME_MAX_SIZE);
    memcpy(aDataset.mNetworkName.m8, aNetworkName, length);
    aDataset.mComponents.mIsNetworkNamePresent = true;

    /* Set Mesh Local Prefix to fd33:3333:3344:0::/64 */
    static const otIp6Address prefix = {{{0xfd, 0x33, 0x33, 0x33, 0x33, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}}};
    memcpy(aDataset.mMeshLocalPrefix.m8, prefix.mFields.m8, sizeof(prefix));
    aDataset.mComponents.mIsMeshLocalPrefixPresent = true;


    otDatasetSetActive(aInstance, &aDataset);
    /* Set the router selection jitter to override the 2 minute default.
       CLI cmd > routerselectionjitter 20
       Warning: For demo purposes only - not to be used in a real product */
    uint8_t jitterValue = 20;
    otThreadSetRouterSelectionJitter(aInstance, jitterValue);
}

/**
 * @brief Function for initializing custom commands
 */
static void initThreadCustomCommands(void *aContext){
    static const otCliCommand customCommands[] = {{"leave", leaveNetwork}, {"setchild", setChild}, {"setrouter", setRouter}};
    otCliSetUserCommands(customCommands, OT_ARRAY_LENGTH(customCommands), aContext);
}

/***************************************************************************************************
 * @section CLI Commands
 **************************************************************************************************/

/**
 * @brief Releases router address from partition and then turns itself off. If its a child then it becomes a router then releases. Doesnt work with leader.
 * 
 */
static void leaveNetwork(void *aContext, uint8_t aArgsLength, char *aArgs[]){
    OT_UNUSED_VARIABLE(aArgsLength);
    OT_UNUSED_VARIABLE(aArgs);

    otIp6Address aLeaderRloc;

    if (otThreadGetDeviceRole(aContext) == OT_DEVICE_ROLE_CHILD){
        setRouter(aContext, 0, aArgs);
    }

    otThreadRemoveNeighbor(aContext);

    while(otThreadGetLeaderRloc(aContext, &aLeaderRloc) != OT_ERROR_DETACHED){
        otTaskletsProcess(aContext);
        otSysProcessDrivers(aContext);
        otSysButtonProcess(aContext);
    }

    otThreadSetEnabled(aContext, false);
}

static void setChild(void *aContext, uint8_t aArgsLength, char *aArgs[]){
    OT_UNUSED_VARIABLE(aArgsLength);
    OT_UNUSED_VARIABLE(aArgs);
    otThreadSetRouterEligible(aContext, false);
}

static void setRouter(void *aContext, uint8_t aArgsLength, char *aArgs[]){
    OT_UNUSED_VARIABLE(aArgsLength);
    OT_UNUSED_VARIABLE(aArgs);
    otThreadSetRouterEligible(aContext, true);
    otThreadBecomeRouter(aContext);
}