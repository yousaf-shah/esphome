#ifdef USE_ESP32_VARIANT_ESP32P4

#include <utility>
#include "mipi_dsi.h"

namespace esphome {
namespace mipi_dsi {

// ----------------------------------------------------------------------------------
// Internal: callback to signal a draw transaction finished
// ----------------------------------------------------------------------------------
static bool notify_refresh_ready(esp_lcd_panel_handle_t /*panel*/,
                                 esp_lcd_dpi_panel_event_data_t * /*edata*/,
                                 void *user_ctx) {
  auto *sem = static_cast<SemaphoreHandle_t *>(user_ctx);
  BaseType_t need_yield = pdFALSE;
  xSemaphoreGiveFromISR(sem, &need_yield);
  return (need_yield == pdTRUE);
}

// ----------------------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------------------
void MIPI_DSI::setup() {
  ESP_LOGCONFIG(TAG, "Running Setup");

  // Optional enable pins (power rails / bridges on some boards)
  if (!this->enable_pins_.empty()) {
    for (auto *pin : this->enable_pins_) {
      pin->setup();
      pin->digital_write(true);
    }
    delay(10);
  }

  // --- DSI host (DPHY) ---
  esp_lcd_dsi_bus_config_t bus_config = {
      .bus_id = 0,
      .num_data_lanes = this->lanes_,
      .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
      .lane_bit_rate_mbps = this->lane_bit_rate_,  // e.g. 500/750/1000
  };
  auto err = esp_lcd_new_dsi_bus(&bus_config, &this->bus_handle_);
  if (err != ESP_OK) {
    this->smark_failed("lcd_new_dsi_bus failed", err);
    return;
  }

  // --- DCS/DBI command path (for init sequences / DCS writes) ---
  esp_lcd_dbi_io_config_t dbi_config = {
      .virtual_channel = 0,
      .lcd_cmd_bits = 8,
      .lcd_param_bits = 8,
  };
  err = esp_lcd_new_panel_io_dbi(this->bus_handle_, &dbi_config, &this->io_handle_);
  if (err != ESP_OK) {
    this->smark_failed("new_panel_io_dbi failed", err);
    return;
  }

  // --- DPI (video) panel configuration ---
  auto pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
  if (this->color_depth_ == display::COLOR_BITNESS_888) {
    pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB888;
  }

  ESP_LOGI(TAG,
           "Timings: %dx%d, hsw=%d hbp=%d hfp=%d, vsw=%d vbp=%d vfp=%d, "
           "pclk=%dMHz, lanes=%d, lane_rate=%dMbps, fmt=%s",
           this->width_, this->height_,
           this->hsync_pulse_width_, this->hsync_back_porch_, this->hsync_front_porch_,
           this->vsync_pulse_width_, this->vsync_back_porch_, this->vsync_front_porch_,
           this->pclk_frequency_, this->lanes_, this->lane_bit_rate_,
           (pixel_format == LCD_COLOR_PIXEL_FORMAT_RGB888 ? "RGB888" : "RGB565"));

  esp_lcd_dpi_panel_config_t dpi_config = {
      .virtual_channel = 0,
      .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
      .dpi_clock_freq_mhz = this->pclk_frequency_,
      .pixel_format = pixel_format,
      .num_fbs = 1,
      .video_timing =
          {
              .h_size = this->width_,
              .v_size = this->height_,
              .hsync_pulse_width = this->hsync_pulse_width_,
              .hsync_back_porch = this->hsync_back_porch_,
              .hsync_front_porch = this->hsync_front_porch_,
              .vsync_pulse_width = this->vsync_pulse_width_,
              .vsync_back_porch = this->vsync_back_porch_,
              .vsync_front_porch = this->vsync_front_porch_,
          },
      .flags = {.use_dma2d = true},
  };

  err = esp_lcd_new_panel_dpi(this->bus_handle_, &dpi_config, &this->handle_);
  if (err != ESP_OK) {
    this->smark_failed("esp_lcd_new_panel_dpi failed", err);
    return;
  }

  // Optional hardware reset if provided; otherwise send SW RESET
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(true);
    delay(5);
    this->reset_pin_->digital_write(false);
    delay(5);
    this->reset_pin_->digital_write(true);
  } else {
    esp_lcd_panel_io_tx_param(this->io_handle_, SW_RESET_CMD, nullptr, 0);
  }

  // The ILI9881C needs ~120ms before SLPOUT is accepted reliably after reset
  auto when = millis() + 120;

  err = esp_lcd_panel_init(this->handle_);
  if (err != ESP_OK) {
    this->smark_failed("esp_lcd_init failed", err);
    return;
  }

  // --- Run optional per-model init sequence ---
  size_t index = 0;
  auto &vec = this->init_sequence_;
  while (index != vec.size()) {
    if (vec.size() - index < 2) {
      this->mark_failed("Malformed init sequence");
      return;
    }
    uint8_t cmd = vec[index++];
    uint8_t x = vec[index++];

    if (x == DELAY_FLAG) {
      ESP_LOGD(TAG, "Delay %ums", cmd);
      delay(cmd);
      continue;
    }

    uint8_t num_args = (x & 0x7F);
    if (vec.size() - index < num_args) {
      this->mark_failed("Malformed init sequence (args overflow)");
      return;
    }

    // Special case: SLPOUT should not be sent earlier than 120ms after reset
    if (cmd == SLEEP_OUT) {
      int duration = static_cast<int>(when - millis());
      if (duration > 0) delay(duration);
    }

    const uint8_t *ptr = vec.data() + index;
    ESP_LOGVV(TAG, "DCS %02X, len %u, %s", cmd, num_args,
              format_hex_pretty(ptr, num_args, '.', false).c_str());
    err = esp_lcd_panel_io_tx_param(this->io_handle_, cmd, ptr, num_args);
    if (err != ESP_OK) {
      this->smark_failed("lcd_panel_io_tx_param failed", err);
      return;
    }
    index += num_args;

    if (cmd == SLEEP_OUT) delay(10);
  }

  // Draw completion callback semaphore
  this->io_lock_ = xSemaphoreCreateBinary();
  esp_lcd_dpi_panel_event_callbacks_t cbs = {
      .on_color_trans_done = notify_refresh_ready,
  };
  err = esp_lcd_dpi_panel_register_event_callbacks(this->handle_, &cbs, this->io_lock_);
  if (err != ESP_OK) {
    this->smark_failed("Failed to register callbacks", err);
    return;
  }

  ESP_LOGCONFIG(TAG, "MIPI DSI setup complete");
}

// ----------------------------------------------------------------------------------
// Periodic update / display writer plumbing (unchanged from upstream)
// ----------------------------------------------------------------------------------
void MIPI_DSI::update() {
  if (this->auto_clear_enabled_) this->clear();

  if (this->show_test_card_) {
    this->test_card();
  } else if (this->page_ != nullptr) {
    this->page_->get_writer()(*this);
  } else if (this->writer_.has_value()) {
    (*this->writer_)(*this);
  } else {
    this->stop_poller();
  }

  if (this->buffer_ == nullptr || this->x_low_ > this->x_high_ || this->y_low_ > this->y_high_) return;

  ESP_LOGV(TAG, "x_low %d, y_low %d, x_high %d, y_high %d", this->x_low_, this->y_low_, this->x_high_, this->y_high_);
  int w = this->x_high_ - this->x_low_ + 1;
  int h = this->y_high_ - this->y_low_ + 1;
  this->write_to_display_(this->x_low_, this->y_low_, w, h, this->buffer_, this->x_low_, this->y_low_,
                          this->width_ - w - this->x_low_);
  // Invalidate watermarks
  this->x_low_ = this->width_;
  this->y_low_ = this->height_;
  this->x_high_ = 0;
  this->y_high_ = 0;
}

void MIPI_DSI::draw_pixels_at(int x_start, int y_start, int w, int h, const uint8_t *ptr,
                              display::ColorOrder order, display::ColorBitness bitness,
                              bool big_endian, int x_offset, int y_offset, int x_pad) {
  if (w <= 0 || h <= 0) return;

  // If color mapping is required, let Display core handle it
  if (bitness != this->color_depth_) {
    display::Display::draw_pixels_at(x_start, y_start, w, h, ptr, order, bitness, big_endian, x_offset, y_offset, x_pad);
  }
  this->write_to_display_(x_start, y_start, w, h, ptr, x_offset, y_offset, x_pad);
}

void MIPI_DSI::write_to_display_(int x_start, int y_start, int w, int h, const uint8_t *ptr,
                                 int x_offset, int y_offset, int x_pad) {
  esp_err_t err = ESP_OK;
  auto bytes_per_pixel = 3 - this->color_depth_;
  auto stride = (x_offset + w + x_pad) * bytes_per_pixel;
  ptr += y_offset * stride + x_offset * bytes_per_pixel;

  if (x_offset == 0 && x_pad == 0) {
    err = esp_lcd_panel_draw_bitmap(this->handle_, x_start, y_start, x_start + w, y_start + h, ptr);
    xSemaphoreTake(this->io_lock_, portMAX_DELAY);
  } else {
    for (int y = 0; y != h; y++) {
      err = esp_lcd_panel_draw_bitmap(this->handle_, x_start, y + y_start, x_start + w, y + y_start + 1, ptr);
      if (err != ESP_OK) break;
      ptr += stride;
      xSemaphoreTake(this->io_lock_, portMAX_DELAY);
    }
  }
  if (err != ESP_OK) ESP_LOGE(TAG, "esp_lcd_panel_draw_bitmap failed: %s", esp_err_to_name(err));
}

bool MIPI_DSI::check_buffer_() {
  if (this->is_failed()) return false;
  if (this->buffer_ != nullptr) return true;

  auto bytes_per_pixel = 3 - this->color_depth_;
  RAMAllocator<uint8_t> allocator;
  this->buffer_ = allocator.allocate(this->height_ * this->width_ * bytes_per_pixel);
  if (this->buffer_ == nullptr) {
    this->mark_failed("Could not allocate buffer for display!");
    return false;
  }
  return true;
}

// ----------------------------------------------------------------------------------
// NEW: DCS helper methods (so you can call them from YAML lambdas)
// ----------------------------------------------------------------------------------
void MIPI_DSI::send_dcs(uint8_t cmd) {
  if (this->io_handle_ == nullptr) return;
  esp_lcd_panel_io_tx_param(this->io_handle_, cmd, nullptr, 0);
}

void MIPI_DSI::send_dcs(uint8_t cmd, uint8_t param) {
  if (this->io_handle_ == nullptr) return;
  esp_lcd_panel_io_tx_param(this->io_handle_, cmd, &param, 1);
}

void MIPI_DSI::send_dcs(uint8_t cmd, const std::vector<uint8_t> &params) {
  if (this->io_handle_ == nullptr) return;
  const uint8_t *ptr = params.empty() ? nullptr : params.data();
  size_t len = params.size();
  esp_lcd_panel_io_tx_param(this->io_handle_, cmd, ptr, len);
}

}  // namespace mipi_dsi
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
