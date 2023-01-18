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
			void set_open_duration(int open) { this->open_duration = open; }
			void set_close_duration(int close) { this->close_duration = close; }
			void set_tilt_duration(int tilt) { this->tilt_duration = tilt; }
			void set_assumed_state(bool value) { this->assumed_state = value; }
			void StartCalibration();
			void ProcessButton(std::string buttonType, std::string pressMode);
		private:
			int starting_time{ 0 };
			int wait_time{ 0 };
			int exact_pos{ 0 };
			int starting_pos{ 0 };
			int rest_pos{ 0 };
			int change_pos{ 0 };
			int exact_tilt{ 0 };
			int starting_tilt{ 0 };
			int rest_tilt{ 0 };
			int change_tilt{ 0 };
			int publishingDelay{ 0 };
			optional<float> deferred_tilt{};
			cover::CoverOperation current_action{ cover::COVER_OPERATION_IDLE };
			void publishCoverState();
			void processDeferredTilts();
		protected:
			Trigger<>* open_trigger{ new Trigger<>() };
			Trigger<>* close_trigger{ new Trigger<>() };
			Trigger<>* stop_trigger{ new Trigger<>() };
			int open_duration;
			int close_duration;
			int tilt_duration;
			bool assumed_state{ false };
		};

	}
}