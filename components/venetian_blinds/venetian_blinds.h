#pragma once
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/cover/cover.h"

namespace esphome {
	namespace venetian_blinds {

		class VenetianBlinds : public Component, public cover::Cover {
		public:
			void setup() override;
			void loop() override;
			void dump_config() override;
			cover::CoverTraits get_traits() override;
			void control(const cover::CoverCall& call) override;
			Trigger<>* get_open_trigger() const { return this->open_trigger; }
			Trigger<>* get_close_trigger() const { return this->close_trigger; }
			Trigger<>* get_stop_trigger() const { return this->stop_trigger; }
			void set_open_duration(int open) { this->_open_duration = open; }
			void set_close_duration(int close) { this->_close_duration = close; }
			void set_tilt_duration(int tilt) { this->_tilt_duration = tilt; }
			void set_assumed_state(bool value) { this->assumed_state = value; }
			void set_motor_warmup_delay(int delay) { this->_motor_warmup_delay = delay; }
			void StartCalibration();
			void ProcessButton(std::string buttonType, std::string pressMode);
		private:
			int _starting_time{ 0 };
			int _wait_time{ 0 };
			int _exact_pos{ 0 };
			int _starting_pos{ 0 };
			int _rest_pos{ 0 };
			int _change_pos{ 0 };
			int _exact_tilt{ 0 };
			int _starting_tilt{ 0 };
			int _rest_tilt{ 0 };
			int _change_tilt{ 0 };
			int _publishingDelay{ 0 };
			optional<float> _deferred_tilt{};
			cover::CoverOperation _current_action{ cover::COVER_OPERATION_IDLE };
			int _buttonHoldingDirection{ 0 };
			void publishCoverState();
			void processDeferredTilts();
			bool processHoldedButton(bool justProceeded);
		protected:
			Trigger<>* open_trigger{ new Trigger<>() };
			Trigger<>* close_trigger{ new Trigger<>() };
			Trigger<>* stop_trigger{ new Trigger<>() };
			int _open_duration;
			int _close_duration;
			int _tilt_duration;
			bool assumed_state{ false };
			int _motor_warmup_delay{ 20 };// waiting for movement start, when little tilt change
		};
	}
}