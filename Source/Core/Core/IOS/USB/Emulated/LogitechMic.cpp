// Copyright 2025 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/IOS/USB/Emulated/LogitechMic.h"

#include <algorithm>

#include "Core/HW/Memmap.h"
#include "Core/System.h"

namespace IOS::HLE::USB
{
LogitechMic::LogitechMic()
{
  m_id = u64(m_vid) << 32 | u64(m_pid) << 16 | u64(9) << 8 | u64(1);
}

LogitechMic::~LogitechMic() = default;

DeviceDescriptor LogitechMic::GetDeviceDescriptor() const
{
  return m_device_descriptor;
}

std::vector<ConfigDescriptor> LogitechMic::GetConfigurations() const
{
  return m_config_descriptor;
}

std::vector<InterfaceDescriptor> LogitechMic::GetInterfaces(u8 config) const
{
  return m_interface_descriptor[m_active_interface];
}

std::vector<EndpointDescriptor> LogitechMic::GetEndpoints(u8 config, u8 interface, u8 alt) const
{
  return m_endpoint_descriptor[m_active_interface];
}

bool LogitechMic::Attach()
{
  if (m_device_attached)
    return true;

  DEBUG_LOG_FMT(IOS_USB, "[{:04x}:{:04x}] Opening device", m_vid, m_pid);
  if (!m_microphone)
    m_microphone = std::make_unique<MicrophoneLogitech>(m_sampler);
  m_device_attached = true;
  return true;
}

bool LogitechMic::AttachAndChangeInterface(const u8 interface)
{
  if (!Attach())
    return false;

  if (interface != m_active_interface)
    return ChangeInterface(interface) == 0;

  return true;
}

int LogitechMic::CancelTransfer(const u8 endpoint)
{
  INFO_LOG_FMT(IOS_USB, "[{:04x}:{:04x} {}] Cancelling transfers (endpoint {:#x})", m_vid, m_pid,
               m_active_interface, endpoint);

  return IPC_SUCCESS;
}

int LogitechMic::ChangeInterface(const u8 interface)
{
  DEBUG_LOG_FMT(IOS_USB, "[{:04x}:{:04x} {}] Changing interface to {}", m_vid, m_pid,
                m_active_interface, interface);
  m_active_interface = interface;
  return 0;
}

int LogitechMic::GetNumberOfAltSettings(u8 interface)
{
  return 0;
}

int LogitechMic::SetAltSetting(u8 alt_setting)
{
  return 0;
}

constexpr u32 USBGETAID(u8 cs, u8 request, u16 index)
{
  return static_cast<u32>((cs << 24) | (request << 16) | index);
}

int LogitechMic::GetAudioControl(std::unique_ptr<CtrlMessage>& cmd)
{
  /*
    PCSX2's code is based entirely on QEMU https://gitlab.com/qemu-project/qemu/-/blob/master/hw/usb/dev-audio.c#L714.
  */
  auto& system = cmd->GetEmulationKernel().GetSystem();
  auto& memory = system.GetMemory();
  const u8 cs = static_cast<u8>(cmd->value >> 8);
  const u8 cn = static_cast<u8>(cmd->value - 1);
  int ret = IPC_STALL;
  INFO_LOG_FMT(IOS_USB,
               "GetAudioControl: bCs={:02x} bCn={:02x} bRequestType={:02x} bRequest={:02x} bIndex={:02x} aid={:08x}",
               cs, cn, cmd->request_type, cmd->request, cmd->index,
               USBGETAID(cs, cmd->request, cmd->index));
  switch (USBGETAID(cs, cmd->request, cmd->index))
  {
  case USBGETAID(0, REQUEST_GET_CUR, 0x0300):
  {
    memory.Write_U8(m_sampler.mute ? 1 : 0, cmd->data_address);
    ret = 1;
    break; 
  }
  case USBGETAID(2, REQUEST_GET_CUR, 0x0300):
  {
    if (cn < 2 || cn == 0xff)
    {
      const uint16_t vol = (m_sampler.vol[cn == 1 ? 1 : 0] * 0x8800 + 127) / 255 + 0x8000;
      memory.Write_U8((uint8_t)(vol & 0xFF), cmd->data_address);
      memory.Write_U8(vol >> 8, cmd->data_address + 1);
      ret = 2;
    }
    break;
  }
  case USBGETAID(2, REQUEST_GET_MIN, 0x0300):
  {
    if (cn < 2 || cn == 0xff)
    {
      memory.Write_U16(0x8001, cmd->data_address);
      ret = 2;
    }
    break;
  }
  case USBGETAID(2, REQUEST_GET_MAX, 0x0300):
  {
    if (cn < 2 || cn == 0xff)
    {
      memory.Write_U16(0x0800, cmd->data_address);
      ret = 2;
    }
    break;
  }
  case USBGETAID(2, REQUEST_GET_RES, 0x0300):
  {
    if (cn < 2 || cn == 0xff)
    {
      memory.Write_U16(0x0088, cmd->data_address);
      ret = 2;
    }
    break;
  }
  }
  return ret;
}

int LogitechMic::SetAudioControl(std::unique_ptr<CtrlMessage>& cmd)
{
  auto& system = cmd->GetEmulationKernel().GetSystem();
  auto& memory = system.GetMemory();
  const u8 cs = static_cast<u8>(cmd->value >> 8);
  const u8 cn = static_cast<u8>(cmd->value - 1);
  int ret = IPC_STALL;
  INFO_LOG_FMT(
      IOS_USB,
      "SetAudioControl: bCs={:02x} bCn={:02x} bRequestType={:02x} bRequest={:02x} bIndex={:02x} aid={:08x}",
               cs, cn, cmd->request_type, cmd->request, cmd->index,
               USBGETAID(cs, cmd->request, cmd->index));
  switch (USBGETAID(cs, cmd->request, cmd->index))
  {
  case USBGETAID(1, REQUEST_SET_CUR, 0x0300):
  {
    m_sampler.mute = memory.Read_U8(cmd->data_address) & 0x01;
    ret = 0;
    break;
  }
  case USBGETAID(2, REQUEST_SET_CUR, 0x0300):
  {
    if (cn < 2 || cn == 0xff)
    {
      uint16_t vol = memory.Read_U16(cmd->data_address);

      vol -= 0x8000;
      vol = (vol * 255 + 0x4400) / 0x8800;
      if (vol > 255)
        vol = 255;

      if (cn == 0xff)
      {
        if (m_sampler.vol[0] != vol)
          m_sampler.vol[0] = static_cast<u8>(vol);
        if (m_sampler.vol[1] != vol)
          m_sampler.vol[1] = static_cast<u8>(vol);
      }
      else
      {
        if (m_sampler.vol[cn] != vol)
          m_sampler.vol[cn] = static_cast<u8>(vol);
      }

      ret = 0;
    }
    break;
  }
  case USBGETAID(7, REQUEST_SET_CUR, 0x0300):
  {
    ret = 0;
    break;
  }
  }
  return ret;
}

int LogitechMic::EndpointAudioControl(std::unique_ptr<CtrlMessage>& cmd)
{
  auto& system = cmd->GetEmulationKernel().GetSystem();
  auto& memory = system.GetMemory();
  const u8 cs = static_cast<u8>(cmd->value >> 8);
  const u8 cn = static_cast<u8>(cmd->value - 1);
  int ret = IPC_STALL;
  INFO_LOG_FMT(
      IOS_USB,
      "EndpointAudioControl: bCs={:02x} bCn={:02x} bRequestType={:02x} bRequest={:02x} bIndex={:02x} aid:{:08x}",
               cs, cn, cmd->request_type, cmd->request, cmd->index,
               USBGETAID(cs, cmd->request, cmd->index));
  switch (USBGETAID(cs, cmd->request, cmd->index))
  {
  case USBGETAID(1, REQUEST_SET_CUR, 0x0081):
  {
    if (cn == 0xff)
    {
      const uint32_t sr =
          memory.Read_U8(cmd->data_address) | (memory.Read_U8(cmd->data_address + 1) << 8) | (memory.Read_U8(cmd->data_address + 2) << 16);
      if (m_sampler.srate[0] != sr)
      {
        m_sampler.srate[0] = sr;
        if (m_microphone != nullptr)
        {
          INFO_LOG_FMT(IOS_USB,
                       "EndpointAudioControl: Setting sampling rate to {:d}, [0] {:02x} [1] {:02x} [2] {:02x}",
                       sr, memory.Read_U8(cmd->data_address), memory.Read_U8(cmd->data_address + 1),
                       memory.Read_U8(cmd->data_address + 2));
          m_microphone->SetSamplingRate(sr);
        }
      }
    }
    else if (cn < 2)
    {
			//TODO: Handle this case.
      WARN_LOG_FMT(IOS_USB, "EndpointAudioControl: Got that weird case where cn is {:d}", cn);
    }
    ret = 0;
    break;
  }
  case USBGETAID(1, REQUEST_GET_CUR, 0x0081):
  {
    memory.Write_U8(m_sampler.srate[0] & 0xff, cmd->data_address);
    memory.Write_U8((m_sampler.srate[0] >> 8) & 0xff, cmd->data_address + 1);
    memory.Write_U8((m_sampler.srate[0] >> 16) & 0xff, cmd->data_address + 2);
    ret = 3;
    break;
  }
  }
  return ret;
}

int LogitechMic::SubmitTransfer(std::unique_ptr<CtrlMessage> cmd)
{
  INFO_LOG_FMT(IOS_USB,
                "[{:04x}:{:04x} {}] Control: bRequestType={:02x} bRequest={:02x} wValue={:04x}"
                " wIndex={:04x} wLength={:04x}",
                m_vid, m_pid, m_active_interface, cmd->request_type, cmd->request, cmd->value,
                cmd->index, cmd->length);
  switch (cmd->request_type << 8 | cmd->request)
  {
  case USBHDR(DIR_DEVICE2HOST, TYPE_STANDARD, REC_DEVICE, REQUEST_GET_DESCRIPTOR):
  {
		// It seems every game always pokes this at first twice; one with a length of 9 which gets the config, and then another with the length provided by the config.
    INFO_LOG_FMT(IOS_USB, "[{:04x}:{:04x} {}] REQUEST_GET_DESCRIPTOR index={:04x} value={:04x}",
                 m_vid, m_pid, m_active_interface, cmd->index, cmd->value);
    cmd->FillBuffer(m_full_descriptor.data(),
                    std::min<size_t>(cmd->length, m_full_descriptor.size()));
    cmd->GetEmulationKernel().EnqueueIPCReply(cmd->ios_request, IPC_SUCCESS);
    break;
  }
  case USBHDR(DIR_HOST2DEVICE, TYPE_STANDARD, REC_INTERFACE, REQUEST_SET_INTERFACE):
  {
    INFO_LOG_FMT(IOS_USB, "[{:04x}:{:04x} {}] REQUEST_SET_INTERFACE index={:04x} value={:04x}",
                 m_vid, m_pid, m_active_interface, cmd->index, cmd->value);
    if (static_cast<u8>(cmd->index) != m_active_interface)
    {
      const int ret = ChangeInterface(static_cast<u8>(cmd->index));
      if (ret < 0)
      {
        ERROR_LOG_FMT(IOS_USB, "[{:04x}:{:04x} {}] Failed to change interface to {}", m_vid, m_pid,
                      m_active_interface, cmd->index);
        return ret;
      }
    }
    const int ret = SetAltSetting(static_cast<u8>(cmd->value));
    if (ret == 0)
      cmd->GetEmulationKernel().EnqueueIPCReply(cmd->ios_request, cmd->length);
    return ret;
  }
  case USBHDR(DIR_DEVICE2HOST, TYPE_CLASS, REC_INTERFACE, REQUEST_GET_CUR):
  case USBHDR(DIR_DEVICE2HOST, TYPE_CLASS, REC_INTERFACE, REQUEST_GET_MIN):
  case USBHDR(DIR_DEVICE2HOST, TYPE_CLASS, REC_INTERFACE, REQUEST_GET_MAX):
  case USBHDR(DIR_DEVICE2HOST, TYPE_CLASS, REC_INTERFACE, REQUEST_GET_RES):
  {
    INFO_LOG_FMT(IOS_USB, "[{:04x}:{:04x} {}] Get Control index={:04x} value={:04x}", m_vid, m_pid,
                 m_active_interface, cmd->index, cmd->value);
    int ret = GetAudioControl(cmd);
    if (ret < 0)
    {
      ERROR_LOG_FMT(IOS_USB,
                    "[{:04x}:{:04x} {}] Get Control Failed index={:04x} value={:04x} ret={}", m_vid,
                    m_pid, m_active_interface, cmd->index, cmd->value, ret);
      goto fail;
    }
    cmd->GetEmulationKernel().EnqueueIPCReply(cmd->ios_request, ret);
    break;
  }
  case USBHDR(DIR_DEVICE2HOST, TYPE_CLASS, REC_INTERFACE, REQUEST_SET_CUR):
  case USBHDR(DIR_DEVICE2HOST, TYPE_CLASS, REC_INTERFACE, REQUEST_SET_MIN):
  case USBHDR(DIR_DEVICE2HOST, TYPE_CLASS, REC_INTERFACE, REQUEST_SET_MAX):
  case USBHDR(DIR_DEVICE2HOST, TYPE_CLASS, REC_INTERFACE, REQUEST_SET_RES):
  {
    INFO_LOG_FMT(IOS_USB, "[{:04x}:{:04x} {}] Set Control index={:04x} value={:04x}", m_vid, m_pid,
                 m_active_interface, cmd->index, cmd->value);
    int ret = SetAudioControl(cmd);
    if (ret < 0)
    {
      ERROR_LOG_FMT(IOS_USB,
                    "[{:04x}:{:04x} {}] Set Control Failed index={:04x} value={:04x} ret={}", m_vid,
                    m_pid, m_active_interface, cmd->index, cmd->value, ret);
      goto fail;
    }
    cmd->GetEmulationKernel().EnqueueIPCReply(cmd->ios_request, ret);
    break;
  }
  case USBHDR(DIR_HOST2DEVICE, TYPE_CLASS, REC_ENDPOINT, REQUEST_GET_CUR):
  case USBHDR(DIR_HOST2DEVICE, TYPE_CLASS, REC_ENDPOINT, REQUEST_GET_MIN):
  case USBHDR(DIR_HOST2DEVICE, TYPE_CLASS, REC_ENDPOINT, REQUEST_GET_MAX):
  case USBHDR(DIR_HOST2DEVICE, TYPE_CLASS, REC_ENDPOINT, REQUEST_GET_RES):
  case USBHDR(DIR_HOST2DEVICE, TYPE_CLASS, REC_ENDPOINT, REQUEST_SET_CUR):
  case USBHDR(DIR_HOST2DEVICE, TYPE_CLASS, REC_ENDPOINT, REQUEST_SET_MIN):
  case USBHDR(DIR_HOST2DEVICE, TYPE_CLASS, REC_ENDPOINT, REQUEST_SET_MAX):
  case USBHDR(DIR_HOST2DEVICE, TYPE_CLASS, REC_ENDPOINT, REQUEST_SET_RES):
  {
    INFO_LOG_FMT(IOS_USB, "[{:04x}:{:04x} {}] REC_ENDPOINT index={:04x} value={:04x}",
                 m_vid, m_pid, m_active_interface, cmd->index, cmd->value);
    int ret = EndpointAudioControl(cmd);
    if (ret < 0)
    {
      ERROR_LOG_FMT(IOS_USB,
                    "[{:04x}:{:04x} {}] Enndpoint Control Failed index={:04x} value={:04x} ret={}", m_vid,
                    m_pid, m_active_interface, cmd->index, cmd->value, ret);
      goto fail;
    }
    cmd->GetEmulationKernel().EnqueueIPCReply(cmd->ios_request, ret);
    break;
  }
  case USBHDR(DIR_HOST2DEVICE, TYPE_CLASS, REC_INTERFACE, REQUEST_SET_CUR):
  case USBHDR(DIR_HOST2DEVICE, TYPE_CLASS, REC_INTERFACE, REQUEST_SET_MIN):
  case USBHDR(DIR_HOST2DEVICE, TYPE_CLASS, REC_INTERFACE, REQUEST_SET_MAX):
  case USBHDR(DIR_HOST2DEVICE, TYPE_CLASS, REC_INTERFACE, REQUEST_SET_RES):
  {
    INFO_LOG_FMT(IOS_USB, "[{:04x}:{:04x} {}] Set Control HOST2DEVICE index={:04x} value={:04x}", m_vid, m_pid,
                 m_active_interface, cmd->index, cmd->value);
    int ret = SetAudioControl(cmd);
    if (ret < 0)
    {
      ERROR_LOG_FMT(IOS_USB,
                    "[{:04x}:{:04x} {}] Set Control HOST2DEVICE Failed index={:04x} value={:04x} ret={}", m_vid,
                    m_pid, m_active_interface, cmd->index, cmd->value, ret);
      goto fail;
    }
    cmd->GetEmulationKernel().EnqueueIPCReply(cmd->ios_request, ret);
    break;
  }
  default:
  fail:
    NOTICE_LOG_FMT(IOS_USB, "Unknown command");
    cmd->GetEmulationKernel().EnqueueIPCReply(cmd->ios_request, IPC_STALL);
  }

  return IPC_SUCCESS;
}


int LogitechMic::SubmitTransfer(std::unique_ptr<BulkMessage> cmd)
{
  cmd->GetEmulationKernel().EnqueueIPCReply(cmd->ios_request, IPC_SUCCESS);
  return IPC_SUCCESS;
}

int LogitechMic::SubmitTransfer(std::unique_ptr<IntrMessage> cmd)
{
  cmd->GetEmulationKernel().EnqueueIPCReply(cmd->ios_request, IPC_SUCCESS);
  return IPC_SUCCESS;
}

constexpr std::array<u8, 800> dummyData()
{
	std::array<u8, 800> a;
  for (std::size_t i = 0; i < 800; ++i)
  {
    a[i] = (i / 100) % 2 ? 0xff : 0x00;
  }
  return a;
}

int LogitechMic::SubmitTransfer(std::unique_ptr<IsoMessage> cmd)
{
  auto& system = cmd->GetEmulationKernel().GetSystem();
  auto& memory = system.GetMemory();

  u8* packets = memory.GetPointerForRange(cmd->data_address, cmd->length);
  if (packets == nullptr)
  {
    ERROR_LOG_FMT(IOS_USB, "Logitech USB Microphone command invalid");
    return IPC_EINVAL;
  }

  switch (cmd->endpoint)
  {
  case ENDPOINT_AUDIO_IN:
  {
    //TODO: This log fires multiple times a frame; I know it gets called, the output data doesn't seem right yet still.
    //INFO_LOG_FMT(
    //    IOS_USB,
    //    "Logitech Mic isochronous transfer, ENDPOINT_AUDIO_IN: length={:04x} endpoint={:02x} num_packets={:02x}",
    //    cmd->length, cmd->endpoint, cmd->num_packets);
    memcpy(packets, dummyData().data(), cmd->length);

    u16 size = 0;
    //TODO: After getting a known sound wave to transmit through, then we can get the microphone wired in.
    //if (m_microphone && m_microphone->HasData(cmd->length / sizeof(s16)))
    //  size = m_microphone->ReadIntoBuffer(packets, cmd->length);
    for (std::size_t i = 0; i < cmd->num_packets; i++)
    {
      cmd->SetPacketReturnValue(i, std::min(size, cmd->packet_sizes[i]));
      size = (size > cmd->packet_sizes[i]) ? (size - cmd->packet_sizes[i]) : 0;
    }
    break;
  }
  default:
  {
    INFO_LOG_FMT(
        IOS_USB,
        "Logitech Mic isochronous transfer, unknown endpoint?: length={:04x} endpoint={:02x} num_packets={:02x}",
        cmd->length, cmd->endpoint, cmd->num_packets);
  }
  }

  cmd->FillBuffer(packets, cmd->length);
  cmd->ScheduleTransferCompletion(cmd->length, 100);
  return IPC_SUCCESS;
}
} // namespace IOS::HLE::USB
