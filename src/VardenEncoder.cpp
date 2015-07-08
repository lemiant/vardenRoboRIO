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

class VardenEncoder {
	Encoder target;
	double window, distance_per_tick;
	queue<EncoderSample> sample_train;
	EncoderSample current_anchor;
	double last_rate;

public:
	VardenEncoder(int p1, int p2, bool reversed, Encoder::EncodingType enc_type, double wndw, double dist_per_tick):
		target(p1, p2, reversed, enc_type),
		window(wndw),
		distance_per_tick(dist_per_tick),
		current_anchor(0, Timer::GetFPGATimestamp()),
		last_rate(0)
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
	}

	double GetRate() {
		return this->last_rate;
	}
};
