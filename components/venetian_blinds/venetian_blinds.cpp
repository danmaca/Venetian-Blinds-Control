#include "venetian_blinds.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
	namespace venetian_blinds {

		static const char* TAG = "venetian_blinds.cover";
		static const int ButtonHoldingIterationWaitTime = 600;// wait among tilt steps when holding buttons
		static const bool IsTestingMode = false;
		static const bool IsMaxButtonOpenRangeRestricted = false;

		using namespace esphome::cover;

		void VenetianBlinds::dump_config() {
			LOG_COVER("", "Venetian Blinds", this);
			ESP_LOGCONFIG(TAG, "  Open Duration: %.0fms", this->_open_duration / 1e0f);
			ESP_LOGCONFIG(TAG, "  Close Duration: %.0fms", this->_close_duration / 1e0f);
			ESP_LOGCONFIG(TAG, "  Tilt Duration: %.0fms", this->_tilt_duration / 1e0f);
			ESP_LOGCONFIG(TAG, "  Motor Warmup Delay: %.0fms", this->_motor_warmup_delay / 1e0f);
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

			_exact_pos = this->position * this->_close_duration;
			_exact_tilt = this->tilt * this->_tilt_duration;

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
				_starting_pos = _exact_pos;
				_starting_tilt = _exact_tilt;
				int new_pos = *call.get_position() * this->_close_duration;
				_change_pos = _exact_pos - new_pos;
				_rest_pos = _change_pos;
				_change_tilt = 0;
				_rest_tilt = 0;

				if (_rest_pos == 0)
					this->ProcessDeferredTilts();
			}

			if (call.get_tilt().has_value()) {
				_starting_pos = _exact_pos;
				_starting_tilt = _exact_tilt;
				int new_tilt = *call.get_tilt() * this->_tilt_duration;
				_change_tilt = _exact_tilt - new_tilt;
				_rest_tilt = _change_tilt;
				_change_pos = 0;
				_rest_pos = 0;
			}

			if (call.get_position().has_value() || call.get_tilt().has_value()) {
				_starting_time = millis();
				_publishingDelay = 0;
			}

			if (call.get_stop()) {
				_rest_pos = 0;
				_rest_tilt = 0;
				_change_pos = 0;
				_change_tilt = 0;
				_buttonHoldingDirection = 0;
				this->stop_trigger->trigger();
				this->_current_action = COVER_OPERATION_IDLE;
				this->_deferred_tilt.reset();
				this->PublishCoverState();
			}
		}

		void VenetianBlinds::loop() {
			if (_wait_time > 0) {
				if (_wait_time > (millis() - _starting_time))
					return;
				else {
					_wait_time = 0;
					_starting_time = millis();
					
					if (this->ProcessHoldedButton(false))
						return;
				}
			}

			if (_rest_pos > 0 || _rest_tilt < 0) {
				if (this->_current_action != COVER_OPERATION_CLOSING) {
					if (IsTestingMode == false)
						this->close_trigger->trigger();
					ESP_LOGD(TAG, "Current Operation: Closing");
					this->_current_action = COVER_OPERATION_CLOSING;
					_wait_time = this->_motor_warmup_delay;
					return;
				}

				uint32_t current_time = millis();
				int delta_time = current_time - _starting_time;
				_publishingDelay++;

				_rest_tilt = clamp(_change_tilt + delta_time, -1 * this->_tilt_duration, 0);
				_exact_tilt = clamp(_starting_tilt + delta_time, 0, this->_tilt_duration);

				_rest_pos = clamp(_change_pos - delta_time, 0, this->_close_duration);
				_exact_pos = clamp(_starting_pos - delta_time, 0, this->_close_duration);

				if (_rest_pos <= 0 && _rest_tilt >= 0) {
					this->stop_trigger->trigger();
					this->_current_action = COVER_OPERATION_IDLE;
					this->PublishCoverState();
					if (this->ProcessHoldedButton(true) == false)
						this->ProcessDeferredTilts();
					this->TryEndAutoProcessing();
				}
				else if (_publishingDelay % 100 == 0) {
					this->PublishCoverState();
				}
			}
			else if (_rest_pos < 0 || _rest_tilt > 0) {
				if (this->_current_action != COVER_OPERATION_OPENING) {
					if (IsTestingMode == false)
						this->open_trigger->trigger();
					ESP_LOGD(TAG, "Current Operation: Opening");
					this->_current_action = COVER_OPERATION_OPENING;
					_wait_time = this->_motor_warmup_delay;
					return;
				}

				uint32_t current_time = millis();
				int delta_time = current_time - _starting_time;
				_publishingDelay++;

				_rest_tilt = clamp(_change_tilt - delta_time, 0, this->_tilt_duration);
				_exact_tilt = clamp(_starting_tilt - delta_time, 0, this->_tilt_duration);

				_rest_pos = clamp(_change_pos + delta_time, -1 * this->_close_duration, 0);
				_exact_pos = clamp(_starting_pos + delta_time, 0, this->_close_duration);

				if (_rest_pos >= 0 && _rest_tilt <= 0) {
					this->stop_trigger->trigger();
					this->_current_action = COVER_OPERATION_IDLE;
					this->PublishCoverState();
					if (this->ProcessHoldedButton(true) == false)
						this->ProcessDeferredTilts();
					this->TryEndAutoProcessing();
				}
				else if (_publishingDelay % 100 == 0) {
					this->PublishCoverState();
				}
			}
		}

		void VenetianBlinds::PublishCoverState() {
			this->position = _exact_pos / (float)this->_close_duration;
			this->tilt = _exact_tilt / (float)this->_tilt_duration;
			this->publish_state();
		}

		void VenetianBlinds::ProcessDeferredTilts() {
			if (this->_deferred_tilt.has_value()) {
				ESP_LOGD(TAG, "processing _deferred_tilt= %.1f", this->_deferred_tilt.value() / 1.0);
				_wait_time = 400;// motor required some time when direction of movement change (cover down, stop, wait, open tilt)

				auto call = this->make_call();
				call.set_tilt(this->_deferred_tilt.value() / 100.0);
				this->_deferred_tilt.reset();
				call.perform();
			}
		}

		bool VenetianBlinds::ProcessHoldedButton(bool justProceeded) {
			if (_buttonHoldingDirection != 0) {
				if (justProceeded) {
					_wait_time = ButtonHoldingIterationWaitTime;
				}
				else {
					int exactTiltPerc = _exact_tilt / (float)this->_tilt_duration * 100;
					int requestedTiltPerc = clamp(exactTiltPerc + (_buttonHoldingDirection * 9), 0, 100);

					if (exactTiltPerc == requestedTiltPerc) {
						_buttonHoldingDirection = 0;
					}
					else {
						ESP_LOGD(TAG, "ProcessHoldedButton requestedTiltPerc= %.1f", requestedTiltPerc / 1.0);

						auto call = this->make_call();
						call.set_tilt(requestedTiltPerc / 100.0);
						this->_deferred_tilt.reset();
						call.perform();
					}
				}
				return true;
			}
			return false;
		}

		void VenetianBlinds::StartCalibration() {
			int exactPosPerc = _exact_pos / (float)this->_close_duration * 100;
			if (exactPosPerc <= 10) {
				_exact_pos = this->_close_duration + 1000;
				auto call = this->make_call();
				call.set_position(0.0);
				call.perform();
			}
			else {
				_exact_pos = -1000;
				auto call = this->make_call();
				call.set_position(1.0);
				call.perform();
			}
		}

		void VenetianBlinds::ProcessButton(std::string buttonType, std::string pressMode) {
			int exactPosPerc = _exact_pos / (float)this->_close_duration * 100;
			int exactTiltPerc = _exact_tilt / (float)this->_tilt_duration * 100;

			float requestedPosPerc = -1;
			float requestedTiltPerc = -1;
			bool requestedStop = false;

			if (buttonType == "up") {
				if (pressMode == "single")
				{
					if (this->_current_action == COVER_OPERATION_OPENING) {
						requestedStop = true;
					}
					else if (exactPosPerc < 3 && exactTiltPerc > 5) {
						requestedPosPerc = 0;
						requestedTiltPerc = 0;

						// nastaveni auto tilt a návrat do AutoMode
						if (_isAutoMode == false && _autoTiltPerc >= 0 && _autoTiltPerc < 90) {
							ESP_LOGD(TAG, "Restoring AutoMode, autoTiltPerc= %.1f", _autoTiltPerc / 1.0);
							requestedTiltPerc = _autoTiltPerc;
							_isAutoMode = true;
							_isAutoProcessing = true;
						}
					}
					else if (exactPosPerc < (IsMaxButtonOpenRangeRestricted ? 10 : 100)) {
						requestedPosPerc = (IsMaxButtonOpenRangeRestricted ? 10 : 100);
						requestedTiltPerc = 0;
					}
				}
				else if (pressMode == "double") {
					requestedPosPerc = (IsMaxButtonOpenRangeRestricted ? 10 : 100);
					requestedTiltPerc = 0;
				}
				else if (pressMode == "hold") {
					_buttonHoldingDirection = -1;
					this->ProcessHoldedButton(false);
				}
			}
			else if (buttonType == "down") {
				if (pressMode == "single") {
					if (this->_current_action == COVER_OPERATION_CLOSING) {
						requestedStop = true;
					}
					else if (exactPosPerc > 0 || exactTiltPerc < 100) {
						requestedPosPerc = 0;
						requestedTiltPerc = 100;

						// nastaveni auto tilt a návrat do AutoMode
						if (_isAutoMode == false && _autoTiltPerc >= 0) {
							ESP_LOGD(TAG, "Restoring AutoMode, autoTiltPerc= %.1f", _autoTiltPerc / 1.0);
							requestedTiltPerc = _autoTiltPerc;
							_isAutoMode = true;
							_isAutoProcessing = true;
						}
					}
				}
				else if (pressMode == "double") {
					if (exactPosPerc > 3 || exactTiltPerc > 0) {
						requestedPosPerc = 0;
						requestedTiltPerc = 0;
					}
				}
				else if (pressMode == "hold") {
					_buttonHoldingDirection = 1;
					this->ProcessHoldedButton(false);
				}
			}

			if (pressMode == "release") {
				_buttonHoldingDirection = 0;
			}

			this->_deferred_tilt.reset();

			if (requestedStop) {
				auto call = this->make_call();
				call.set_command_stop();
				call.perform();
			}
			else {
				this->MoveToPosition(requestedPosPerc, requestedTiltPerc);
			}
		};

		void VenetianBlinds::SetAutoPosition(float positionPerc, float tiltPerc) {
			ESP_LOGD(TAG, "SetAuto positionPerc= %.1f", positionPerc / 1.0);
			ESP_LOGD(TAG, "SetAuto tiltPerc= %.1f", tiltPerc / 1.0);
			if (positionPerc >= 0)
				_autoPositionPerc = positionPerc;
			if (tiltPerc >= 0)
				_autoTiltPerc = tiltPerc;

			if (_autoPositionPerc < 0)
				_autoPositionPerc = 0;
			if (_autoTiltPerc < 0)
				_autoTiltPerc = 0;

			if (_isAutoMode) {
				_isAutoProcessing = true;
				this->MoveToPosition(positionPerc, tiltPerc);
			}
		};

		void VenetianBlinds::SetAutoMode(bool isAutoMode) {
			if (isAutoMode)
				ESP_LOGD(TAG, "Set Mode: Auto");
			else
				ESP_LOGD(TAG, "Set Mode: Manual");

			if (isAutoMode && _isAutoMode == false)
				this->RestoreToAutoMode();
			_isAutoMode = isAutoMode;
		}

		void VenetianBlinds::MoveToPosition(float positionPerc, float tiltPerc) {
			ESP_LOGD(TAG, "MoveTo positionPerc= %.1f", positionPerc / 1.0);
			ESP_LOGD(TAG, "MoveTo tiltPerc= %.1f", tiltPerc / 1.0);
			if (positionPerc >= 0) {
				if (tiltPerc >= 0) {
					ESP_LOGD(TAG, "set _deferred_tilt= %.1f", tiltPerc / 1.0);
					this->_deferred_tilt = tiltPerc / 1.0;
				}

				auto call = this->make_call();
				call.set_position(positionPerc / 100.0);
				call.perform();
			}
			else if (tiltPerc >= 0) {
				auto call = this->make_call();
				call.set_tilt(tiltPerc / 100.0);
				call.perform();
			}
		};

		void VenetianBlinds::RestoreToAutoMode() {
			ESP_LOGD(TAG, "RestoreToAutoMode");
			_isAutoMode = true;
			_isAutoProcessing = true;
			if (_autoPositionPerc >= 0 || _autoTiltPerc >= 0)
				this->MoveToPosition(_autoPositionPerc, _autoTiltPerc);
		}

		void VenetianBlinds::TryEndAutoProcessing() {
			ESP_LOGD(TAG, "TryEndAutoProcessing: Check");
			if (this->_deferred_tilt.has_value() == false)
			{
				if (_isAutoProcessing) {
					ESP_LOGD(TAG, "TryEndAutoProcessing: Ended Auto Processing");
					_isAutoProcessing = false;
				}
				else if (_isAutoMode) {
					ESP_LOGD(TAG, "TryEndAutoProcessing: Manual Set");
					_isAutoMode = false;
				}
			}

			if (_isAutoMode)
				ESP_LOGD(TAG, "Current Mode: Auto");
			else
				ESP_LOGD(TAG, "Current Mode: Manual");
		}
	}
}