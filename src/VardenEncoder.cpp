#include "WPILib.h"
#include <algorithm>
#include <queue>

using namespace std;

struct EncoderSample {
	int ticks;
	double time;
	EncoderSample(int tic, double tim):
		ticks(tic),
		time(tim)
	{
	}
};

#define LT_WINDOW 3.0

class VardenEncoder {
	Encoder target;
	double window, distance_per_tick;
	queue<EncoderSample> sample_train;
	EncoderSample current_anchor;
	double last_rate;
	queue<EncoderSample> lt_sample_train;
	EncoderSample lt_current_anchor;
	double lt_last_rate;


public:
	VardenEncoder(int p1, int p2, bool reversed, Encoder::EncodingType enc_type, double wndw, double dist_per_tick):
		target(p1, p2, reversed, enc_type),
		window(wndw),
		distance_per_tick(dist_per_tick),
		sample_train(),
		current_anchor(0, Timer::GetFPGATimestamp()),
		last_rate(0),
		lt_sample_train(),
		lt_current_anchor(0, Timer::GetFPGATimestamp()),
		lt_last_rate(0)
	{
	}

	void tick() {
		EncoderSample cur(this->target.Get(), Timer::GetFPGATimestamp());
		this->sample_train.push(cur);
		while(!this->sample_train.empty() && this->sample_train.front().time < (cur.time-this->window)){
			this->current_anchor = this->sample_train.front();
			this->sample_train.pop();
		}
		double time_delta = max(this->window, cur.time - this->current_anchor.time);
		double tick_delta = cur.ticks - this->current_anchor.ticks;
		this->last_rate = (tick_delta/time_delta)*this->distance_per_tick;

		// Longer term version of the averaging code. Used for detecting encoder failure.
		this->lt_sample_train.push(cur);
		while(!this->lt_sample_train.empty() && this->lt_sample_train.front().time < (cur.time-LT_WINDOW)){
			this->lt_current_anchor = this->lt_sample_train.front();
			this->lt_sample_train.pop();
		}
		double lt_time_delta = max(LT_WINDOW, cur.time - this->lt_current_anchor.time);
		double lt_tick_delta = cur.ticks - this->lt_current_anchor.ticks;
		this->lt_last_rate = (lt_tick_delta/lt_time_delta)*this->distance_per_tick;
	}

	double GetRate() {
		return this->last_rate;
	}

	double GetLongTermRate() {
		return this->lt_last_rate;
	}
};
