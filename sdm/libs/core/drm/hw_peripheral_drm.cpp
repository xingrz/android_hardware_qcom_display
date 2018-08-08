/*
Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <utils/debug.h>

#include "hw_peripheral_drm.h"

#define __CLASS__ "HWPeripheralDRM"

using sde_drm::DRMDisplayType;
using sde_drm::DRMOps;
using sde_drm::DRMPowerMode;

namespace sdm {

HWPeripheralDRM::HWPeripheralDRM(int32_t display_id, BufferSyncHandler *buffer_sync_handler,
                                 BufferAllocator *buffer_allocator,
                                 HWInfoInterface *hw_info_intf)
  : HWDeviceDRM(buffer_sync_handler, buffer_allocator, hw_info_intf) {
  disp_type_ = DRMDisplayType::PERIPHERAL;
  device_name_ = "Peripheral Display";
  display_id_ = display_id;
}

DisplayError HWPeripheralDRM::Init() {
  DisplayError ret = HWDeviceDRM::Init();
  if (ret != kErrorNone) {
    DLOGE("Init failed for %s", device_name_);
    return ret;
  }

  scalar_data_.resize(hw_resource_.hw_dest_scalar_info.count);

  return kErrorNone;
}

DisplayError HWPeripheralDRM::Validate(HWLayers *hw_layers) {
  HWLayersInfo &hw_layer_info = hw_layers->info;
  SetDestScalarData(hw_layer_info);
  SetIdlePCState();

  return HWDeviceDRM::Validate(hw_layers);
}

DisplayError HWPeripheralDRM::Commit(HWLayers *hw_layers) {
  HWLayersInfo &hw_layer_info = hw_layers->info;
  SetDestScalarData(hw_layer_info);
  SetIdlePCState();

  DisplayError error = HWDeviceDRM::Commit(hw_layers);
  if (error != kErrorNone) {
    return error;
  }

  // Initialize to default after successful commit
  synchronous_commit_ = false;

  return error;
}

void HWPeripheralDRM::ResetDisplayParams() {
  sde_dest_scalar_data_ = {};
  for (uint32_t j = 0; j < scalar_data_.size(); j++) {
    scalar_data_[j] = {};
  }
}

void HWPeripheralDRM::SetDestScalarData(HWLayersInfo hw_layer_info) {
  if (!hw_resource_.hw_dest_scalar_info.count) {
    return;
  }

  uint32_t index = 0;
  for (uint32_t i = 0; i < hw_resource_.hw_dest_scalar_info.count; i++) {
    DestScaleInfoMap::iterator it = hw_layer_info.dest_scale_info_map.find(i);

    if (it == hw_layer_info.dest_scale_info_map.end()) {
      continue;
    }

    HWDestScaleInfo *dest_scale_info = it->second;
    SDEScaler *scale = &scalar_data_[index];
    hw_scale_->SetScaler(dest_scale_info->scale_data, scale);
    sde_drm_dest_scaler_cfg *dest_scalar_data = &sde_dest_scalar_data_.ds_cfg[index];
    dest_scalar_data->flags = 0;
    if (scale->scaler_v2.enable) {
      dest_scalar_data->flags |= SDE_DRM_DESTSCALER_ENABLE;
    }
    if (scale->scaler_v2.de.enable) {
      dest_scalar_data->flags |= SDE_DRM_DESTSCALER_ENHANCER_UPDATE;
    }
    if (dest_scale_info->scale_update) {
      dest_scalar_data->flags |= SDE_DRM_DESTSCALER_SCALE_UPDATE;
    }
    dest_scalar_data->index = i;
    dest_scalar_data->lm_width = dest_scale_info->mixer_width;
    dest_scalar_data->lm_height = dest_scale_info->mixer_height;
    dest_scalar_data->scaler_cfg = reinterpret_cast<uint64_t>(&scale->scaler_v2);
    if (hw_panel_info_.partial_update) {
      dest_scalar_data->flags |= SDE_DRM_DESTSCALER_PU_ENABLE;
    }
    index++;
  }
  sde_dest_scalar_data_.num_dest_scaler = UINT32(hw_layer_info.dest_scale_info_map.size());
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_DEST_SCALER_CONFIG, token_.crtc_id,
                            reinterpret_cast<uint64_t>(&sde_dest_scalar_data_));
}

DisplayError HWPeripheralDRM::Flush(HWLayers *hw_layers) {
  DisplayError err = HWDeviceDRM::Flush(hw_layers);
  if (err != kErrorNone) {
    return err;
  }

  ResetDisplayParams();
  return kErrorNone;
}

DisplayError HWPeripheralDRM::ControlIdlePowerCollapse(bool enable, bool synchronous) {
  sde_drm::DRMIdlePCState idle_pc_state =
    enable ? sde_drm::DRMIdlePCState::ENABLE : sde_drm::DRMIdlePCState::DISABLE;
  if (idle_pc_state == idle_pc_state_) {
    return kErrorNone;
  }
  // As idle PC is disabled after subsequent commit, Make sure to have synchrounous commit and
  // ensure TA accesses the display_cc registers after idle PC is disabled.
  idle_pc_state_ = idle_pc_state;
  synchronous_commit_ = !enable ? synchronous : false;
  return kErrorNone;
}

DisplayError HWPeripheralDRM::PowerOn(const HWQosData &qos_data, int *release_fence) {
  DTRACE_SCOPED();
  if (!drm_atomic_intf_) {
    DLOGE("DRM Atomic Interface is null!");
    return kErrorUndefined;
  }

  if (first_cycle_) {
    if (!hw_panel_info_.is_primary_panel && (disp_type_ == DRMDisplayType::PERIPHERAL)
        && (!builtin_mirroring_enabled_)) {
      drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, token_.conn_id, token_.crtc_id);
      drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
      drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &current_mode);
      DLOGI("Allowing poweron without commit");
    } else {
      return kErrorNone;
    }
  }
  drm_atomic_intf_->Perform(sde_drm::DRMOps::CRTC_SET_IDLE_PC_STATE, token_.crtc_id,
                            sde_drm::DRMIdlePCState::ENABLE);
  DisplayError err = HWDeviceDRM::PowerOn(qos_data, release_fence);
  if (err != kErrorNone) {
    return err;
  }
  idle_pc_state_ = sde_drm::DRMIdlePCState::ENABLE;

  return kErrorNone;
}

}  // namespace sdm
