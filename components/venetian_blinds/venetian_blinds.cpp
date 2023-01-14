#include "venetian_blinds.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
	namespace venetian_blinds {

		static const char* TAG = "venetian_blinds.cover";
		static const int WaitTimeDefault = 180;// waiting for movement start, when little tilt change

		using namespace esphome::cover;

		void VenetianBlinds::dump_config() {
			LOG_COVER("", "Venetian Blinds", this);
			ESP_LOGCONFIG(TAG, "  Open Duration: %.1fs", this->open_duration / 1e3f);
			ESP_LOGCONFIG(TAG, "  Close Duration: %.1fs", this->close_duration / 1e3f);
			ESP_LOGCONFIG(TAG, "  Tilt Duration: %.1fs", this->tilt_duration / 1e3f);
		}

		void VenetianBlinds::setup() {
			auto restore = this->restore_state_();
			if (restore.has_value()) {
				restore->apply(this);
			}
			else {
				this->position = 0.0;
				this->tilt = 0.0;
			}

			exact_pos = this->position * this->close_duration;
			exact_tilt = this->tilt * this->tilt_duration;

			ESP_LOGCONFIG(TAG, "Initial position: %.1f", this->position * 100);
			ESP_LOGCONFIG(TAG, "Initial tilt: %.1f", this->tilt * 100);
		}

		CoverTraits VenetianBlinds::get_traits() {
			auto traits = CoverTraits();
			traits.set_supports_position(true);
			traits.set_supports_tilt(true);
			traits.set_is_assumed_state(this->assumed_state);
			return traits;
		}

		void VenetianBlinds::control(const CoverCall& call) {
			if (call.get_position().has_value()) {
				starting_pos = exact_pos;
				starting_tilt = exact_tilt;
				int new_pos = *call.get_position() * this->close_duration;
				change_pos = exact_pos - new_pos;
				rest_pos = change_pos;
				change_tilt = 0;
				rest_tilt = 0;
			}

			if (call.get_tilt().has_value()) {
				starting_pos = exact_pos;
				starting_tilt = exact_tilt;
				int new_tilt = *call.get_tilt() * this->tilt_duration;
				change_tilt = exact_tilt - new_tilt;
				rest_tilt = change_tilt;
				change_pos = 0;
				rest_pos = 0;
			}

			if (call.get_position().has_value() || call.get_tilt().has_value()) {
				starting_time = millis();
				publishingDelay = 0;
			}

			if (call.get_stop()) {
				rest_pos = 0;
				rest_tilt = 0;
				change_pos = 0;
				change_tilt = 0;
				this->stop_trigger->trigger();
				this->current_action = COVER_OPERATION_IDLE;
				this->publishCoverState();
			}
		}

		void VenetianBlinds::loop() {
			if (wait_time > 0) {
				if (wait_time > (millis() - starting_time))
					return;
				else {
					wait_time = 0;
					starting_time = millis();
				}
			}

			if (rest_pos > 0 || rest_tilt < 0) {
				if (this->current_action != COVER_OPERATION_CLOSING) {
					this->close_trigger->trigger();
					this->current_action = COVER_OPERATION_CLOSING;
					wait_time = WaitTimeDefault;
					return;
				}

				uint32_t current_time = millis();
				int delta_time = current_time - starting_time;
				publishingDelay++;

				rest_tilt = clamp(change_tilt + delta_time, -1 * this->tilt_duration, 0);
				exact_tilt = clamp(starting_tilt + delta_time, 0, this->tilt_duration);

				rest_pos = clamp(change_pos - delta_time, 0, this->close_duration);
				exact_pos = clamp(starting_pos - delta_time, 0, this->close_duration);

				if (rest_pos <= 0 && rest_tilt >= 0) {
					this->stop_trigger->trigger();
					this->current_action = COVER_OPERATION_IDLE;
					this->publishCoverState();
				}
				else if (publishingDelay % 100 == 0) {
					this->publishCoverState();
				}
			}
			else if (rest_pos < 0 || rest_tilt > 0) {
				if (this->current_action != COVER_OPERATION_OPENING) {
					this->open_trigger->trigger();
					this->current_action = COVER_OPERATION_OPENING;
					wait_time = WaitTimeDefault;
					return;
				}

				uint32_t current_time = millis();
				int delta_time = current_time - starting_time;
				publishingDelay++;

				rest_tilt = clamp(change_tilt - delta_time, 0, this->tilt_duration);
				exact_tilt = clamp(starting_tilt - delta_time, 0, this->tilt_duration);

				rest_pos = clamp(change_pos + delta_time, -1 * this->close_duration, 0);
				exact_pos = clamp(starting_pos + delta_time, 0, this->close_duration);

				if (rest_pos >= 0 && rest_tilt <= 0) {
					this->stop_trigger->trigger();
					this->current_action = COVER_OPERATION_IDLE;
					this->publishCoverState();
				}
				else if (publishingDelay % 100 == 0) {
					this->publishCoverState();
				}
			}
		}

		void VenetianBlinds::publishCoverState() {
			this->position = exact_pos / (float)this->close_duration;
			this->tilt = exact_tilt / (float)this->tilt_duration;
			this->publish_state();
		};

	}
}