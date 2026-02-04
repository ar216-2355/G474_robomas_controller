#pragma once
#include "usb_packet.hpp"

// 関数宣言
void Parse_USB_Packet(const USBCtrlPacket* pkt);
void Prepare_Motor_Packet(USBFeedbackPacket* pkt);
