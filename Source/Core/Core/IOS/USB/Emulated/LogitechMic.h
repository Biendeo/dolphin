// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

// As I've already stated, I copied the Wii Speak emulation code and I tried to modify it to fit the Logitech USB Microphone

#pragma once

#include <atomic>
#include <memory>
#include <vector>

#include "Common/CommonTypes.h"
#include "Core/IOS/USB/Common.h"
#include "Core/IOS/USB/Emulated/Microphone-Logitech.h"

namespace IOS::HLE::USB
{
struct LogitechMicState
{
  // Use atomic for members concurrently used by the data callback
  std::atomic<bool> sample_on;
  std::atomic<bool> mute;
  std::array<u8, 2> vol;
  std::array<u32, 2> srate;
  int freq;
  int gain;
  bool ec_reset;
  bool sp_on;

  static constexpr u32 DEFAULT_SAMPLING_RATE = 48000;
};

class LogitechMic final : public Device
{
public:
  LogitechMic();
  ~LogitechMic() override;

  DeviceDescriptor GetDeviceDescriptor() const override;
  std::vector<ConfigDescriptor> GetConfigurations() const override;
  std::vector<InterfaceDescriptor> GetInterfaces(u8 config) const override;
  std::vector<EndpointDescriptor> GetEndpoints(u8 config, u8 interface, u8 alt) const override;
  bool Attach() override; 
  bool AttachAndChangeInterface(u8 interface) override;
  int CancelTransfer(u8 endpoint) override;
  int ChangeInterface(u8 interface) override;
  int GetNumberOfAltSettings(u8 interface) override;
  int SetAltSetting(u8 alt_setting) override;
  int SubmitTransfer(std::unique_ptr<CtrlMessage> message) override;
  int SubmitTransfer(std::unique_ptr<BulkMessage> message) override;
  int SubmitTransfer(std::unique_ptr<IntrMessage> message) override;
  int SubmitTransfer(std::unique_ptr<IsoMessage> message) override;

private:
  LogitechMicState m_sampler{};

  int GetAudioControl(std::unique_ptr<CtrlMessage>& cmd);
  int SetAudioControl(std::unique_ptr<CtrlMessage>& cmd);
  int EndpointAudioControl(std::unique_ptr<CtrlMessage>& cmd);

  const u16 m_vid = 0x046d;
  const u16 m_pid = 0x0a03;
  u8 m_active_interface = 0;
  bool m_device_attached = false;
  std::unique_ptr<MicrophoneLogitech> m_microphone{};

    //const DeviceDescriptor m_device_descriptor{0xC0,  0xB3,   0x9F,   0x4E, 0x43, 0x93, 0xFF,
    //                                         0x46D, 0x0A03, 0x0102, 0x1,  0x2,  0x0,  0x1};
  //TODO: I can't explain but the idProduct is 0x0000 in PCSX2, but it seems to want to be 0x0a03 in Dolphin. Not sure where it's coming from.
  const DeviceDescriptor m_device_descriptor{0x12,   0x01,   0x0110, 0x00, 0x00, 0x00, 0x08,
                                             0x046d, 0x0a03, 0x0001, 0x01, 0x02, 0x00, 0x01};
  const std::vector<ConfigDescriptor> m_config_descriptor{
    ConfigDescriptor{0x09, 0x02, 0x00b1, 0x02, 0x01, 0x00, 0x80, 0x2d},
  };
  const std::vector<std::vector<InterfaceDescriptor>> m_interface_descriptor{
    {
      InterfaceDescriptor{0x09, 0x04, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00},
    },
    {
      InterfaceDescriptor{0x09, 0x04, 0x01, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00},
      InterfaceDescriptor{0x09, 0x04, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00},
      InterfaceDescriptor{0x09, 0x04, 0x01, 0x02, 0x01, 0x01, 0x02, 0x00, 0x00}
    }
  };
  static constexpr u8 ENDPOINT_AUDIO_IN = 0x81;
  const std::vector<std::vector<EndpointDescriptor>> m_endpoint_descriptor{
    {
      EndpointDescriptor{0x09, 0x05, 0x81, 0x05, 0x0064, 0x01},
    },
    {
      EndpointDescriptor{0x09, 0x05, 0x81, 0x05, 0x00c8, 0x01}
    }
  };

  static constexpr std::array<u8, 121> the_old_config_descriptor = {
      /* Configuration 1 */
      0x09, 0x02, 0x79, 0x00, 0x02, 0x01, 0x03, 0x80, 0x1E,

      /* Interface 0, Alternate Setting 0, Audio Control */
      0x09, 0x04, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,

      /* Audio Control Interface */
      0x09, 0x24, 0x01, 0x00, 0x01, 0x27, 0x00, 0x01, 0x01,

      /* Audio Input Terminal */
      0x0C, 0x24, 0x02, 0x0D, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,

      /* Audio Output Terminal */
      0x09, 0x24, 0x06, 0x02, 0x0D, 0x01, 0x03, 0x00, 0x00,

      /* Audio Feature Unit */
      0x09, 0x24, 0x03, 0x0A, 0x01, 0x01, 0x00, 0x02, 0x00,

      /* Interface 1, Alternate Setting 0, Audio Streaming - Zero Bandwith */
      0x09, 0x04, 0x01, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00,

      /* Interface 1, Alternate Setting 1, Audio Streaming - 1 channel */
      0x09, 0x04, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00,

      /* Audio Streaming Interface */
      0x07, 0x24, 0x01, 0x0A, 0x00, 0x01, 0x00,

      /* Audio Type I Format */
      0x17, 0x24, 0x02, 0x01, 0x01, 0x02, 0x10, 0x05, 0x40, 0x1F, 0x00, 0x11, 0x2B, 0x00, 0x22,
      0x56, 0x00, 0x44, 0xAC, 0x00, 0x80, 0xBB, 0x00,

      /* Endpoint - Standard Descriptor */
      0x09, 0x05, 0x84, 0x0D, 0x60, 0x00, 0x01, 0x00, 0x00,

      /* Endpoint - Audio Streaming */
      0x07, 0x25, 0x01, 0x01, 0x02, 0x01, 0x00};

  static constexpr std::array<u8, 178> m_full_descriptor = {
      /* Configuration 1 */
      0x09,       /* bLength */
      0x02,       /* bDescriptorType */
      0xb1, 0x00, /* wTotalLength */
      0x02,       /* bNumInterfaces */
      0x01,       /* bConfigurationValue */
      0x00,       /* iConfiguration */
      0x80,       /* bmAttributes */
      0x2d,       /* bMaxPower */

      /* Interface 0, Alternate Setting 0, Audio Control */
      0x09, /* bLength */
      0x04, /* bDescriptorType */
      0x00, /* bInterfaceNumber */
      0x00, /* bAlternateSetting */
      0x00, /* bNumEndpoints */
      0x01, /* bInterfaceClass */
      0x01, /* bInterfaceSubClass */
      0x00, /* bInterfaceProtocol */
      0x00, /* iInterface */

      /* Audio Control Interface */
      0x09,       /* bLength */
      0x24,       /* bDescriptorType */
      0x01,       /* bDescriptorSubtype */
      0x00, 0x01, /* bcdADC */
      0x28, 0x00, /* wTotalLength */
      0x01,       /* bInCollection */
      0x01,       /* baInterfaceNr */

      /* Audio Input Terminal */
      0x0c,       /* bLength */
      0x24,       /* bDescriptorType */
      0x02,       /* bDescriptorSubtype */
      0x01,       /* bTerminalID */
      0x01, 0x02, /* wTerminalType */
      0x02,       /* bAssocTerminal */
      0x02,       /* bNrChannels */
      0x03, 0x00, /* wChannelConfig */
      0x00,       /* iChannelNames */
      0x00,       /* iTerminal */

      /* Audio Output Terminal */
      0x09,       /* bLength */
      0x24,       /* bDescriptorType */
      0x03,       /* bDescriptorSubtype */
      0x02,       /* bTerminalID */
      0x01, 0x01, /* wTerminalType */
      0x01,       /* bAssocTerminal */
      0x03,       /* bSourceID */
      0x00,       /* iTerminal */

      /* Audio Feature Unit */
      0x0a, /* bLength */
      0x24, /* bDescriptorType */
      0x06, /* bDescriptorSubtype */
      0x03, /* bUnitID */
      0x01, /* bSourceID */
      0x01, /* bControlSize */
      0x01, /* bmaControls(0) */
      0x02, /* bmaControls(1) */
      0x02, /* bmaControls(2) */
      0x00, /* iTerminal */

      /* Interface 1, Alternate Setting 0, Audio Streaming - Zero Bandwith */
      0x09, /* bLength */
      0x04, /* bDescriptorType */
      0x01, /* bInterfaceNumber */
      0x00, /* bAlternateSetting */
      0x00, /* bNumEndpoints */
      0x01, /* bInterfaceClass */
      0x02, /* bInterfaceSubClass */
      0x00, /* bInterfaceProtocol */
      0x00, /* iInterface */

      /* Interface 1, Alternate Setting 1, Audio Streaming - 1 channel */
      0x09, /* bLength */
      0x04, /* bDescriptorType */
      0x01, /* bInterfaceNumber */
      0x01, /* bAlternateSetting */
      0x01, /* bNumEndpoints */
      0x01, /* bInterfaceClass */
      0x02, /* bInterfaceSubClass */
      0x00, /* bInterfaceProtocol */
      0x00, /* iInterface */

      /* Audio Streaming Interface */
      0x07,       /* bLength */
      0x24,       /* bDescriptorType */
      0x01,       /* bDescriptorSubtype */
      0x02,       /* bTerminalLink */
      0x01,       /* bDelay */
      0x01, 0x00, /* wFormatTag */

      /* Audio Type I Format */
      0x17,             /* bLength */
      0x24,             /* bDescriptorType */
      0x02,             /* bDescriptorSubtype */
      0x01,             /* bFormatType */
      0x01,             /* bNrChannels */
      0x02,             /* bSubFrameSize */
      0x10,             /* bBitResolution */
      0x05,             /* bSamFreqType */
      0x40, 0x1f, 0x00, /* tSamFreq 1 */
      0x11, 0x2b, 0x00, /* tSamFreq 2 */
      0x22, 0x56, 0x00, /* tSamFreq 3 */
      0x44, 0xac, 0x00, /* tSamFreq 4 */
      0x80, 0xbb, 0x00, /* tSamFreq 5 */

      /* Endpoint - Standard Descriptor */
      0x09,       /* bLength */
      0x05,       /* bDescriptorType */
      0x81,       /* bEndpointAddress */
      0x05,       /* bmAttributes */
      0x64, 0x00, /* wMaxPacketSize */
      0x01,       /* bInterval */
      0x00,       /* bRefresh */
      0x00,       /* bSynchAddress */

      /* Endpoint - Audio Streaming */
      0x07,       /* bLength */
      0x25,       /* bDescriptorType */
      0x01,       /* bDescriptor */
      0x01,       /* bmAttributes */
      0x00,       /* bLockDelayUnits */
      0x00, 0x00, /* wLockDelay */

      /* Interface 1, Alternate Setting 2, Audio Streaming - 2 channels */
      0x09, /* bLength */
      0x04, /* bDescriptorType */
      0x01, /* bInterfaceNumber */
      0x02, /* bAlternateSetting */
      0x01, /* bNumEndpoints */
      0x01, /* bInterfaceClass */
      0x02, /* bInterfaceSubClass */
      0x00, /* bInterfaceProtocol */
      0x00, /* iInterface */

      /* Audio Streaming Interface */
      0x07,       /* bLength */
      0x24,       /* bDescriptorType */
      0x01,       /* bDescriptorSubtype */
      0x02,       /* bTerminalLink */
      0x01,       /* bDelay */
      0x01, 0x00, /* wFormatTag */

      /* Audio Type I Format */
      0x17,             /* bLength */
      0x24,             /* bDescriptorType */
      0x02,             /* bDescriptorSubtype */
      0x01,             /* bFormatType */
      0x02,             /* bNrChannels */
      0x02,             /* bSubFrameSize */
      0x10,             /* bBitResolution */
      0x05,             /* bSamFreqType */
      0x40, 0x1f, 0x00, /* tSamFreq 1 */
      0x11, 0x2b, 0x00, /* tSamFreq 2 */
      0x22, 0x56, 0x00, /* tSamFreq 3 */
      0x44, 0xac, 0x00, /* tSamFreq 4 */
      0x80, 0xbb, 0x00, /* tSamFreq 5 */

      /* Endpoint - Standard Descriptor */
      0x09,       /* bLength */
      0x05,       /* bDescriptorType */
      0x81,       /* bEndpointAddress */
      0x05,       /* bmAttributes */
      0xc8, 0x00, /* wMaxPacketSize */
      0x01,       /* bInterval */
      0x00,       /* bRefresh */
      0x00,       /* bSynchAddress */

      /* Endpoint - Audio Streaming */
      0x07,       /* bLength */
      0x25,       /* bDescriptorType */
      0x01,       /* bDescriptor */
      0x01,       /* bmAttributes */
      0x00,       /* bLockDelayUnits */
      0x00, 0x00, /* wLockDelay */

      /* Terminator */
      0 /* bLength */
  };

};
}  // namespace IOS::HLE::USB

