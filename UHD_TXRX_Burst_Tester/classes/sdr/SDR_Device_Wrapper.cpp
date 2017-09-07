#include "SDR_Device_Wrapper.h"

#include <iostream>

#include <boost/format.hpp>

#include "../utils/utils.h"

using namespace std;
using namespace utils;

namespace sdr
{
	SDR_Device_Wrapper::SDR_Device_Wrapper(SDR_Device_Config::sptr_t device_cfg)
	{
		this->device_cfg = device_cfg;

		msg("sdr: Trying to initialize SDR Device...");

		bool init_successfull = false;

		do
		{
			try
			{
				if (device_cfg->debug_settings)
					msg("sdr: Creating the SDR device with: args=" + device_cfg->args);
				msg("", INFO, false, false);
				device = uhd::usrp::multi_usrp::make(device_cfg->args);
				msg("", INFO, false, false);

				if (device.get() == nullptr)
					throw(std::runtime_error("Unable to create SDR device!"));
				else if (device_cfg->debug_settings)
					msg("SDR device successfully created!");

				if (device_cfg->clock_source != "")
				{
					//Set clock source
					if (device_cfg->debug_settings)
						msg("sdr: Trying to set the following clock source: " + device_cfg->clock_source);
					device->set_clock_source(device_cfg->clock_source);
					if (device_cfg->debug_settings)
						msg("sdr: Actual clock source: " + device->get_clock_source(0));
				}

				if (device_cfg->time_source != "")
				{
					//Set time source
					if (device_cfg->debug_settings)
						msg("sdr: Trying to set the following time source: " + device_cfg->time_source);
					device->set_time_source(device_cfg->time_source);
					if (device_cfg->debug_settings)
						msg("sdr: Actual time source: " + device->get_time_source(0));
				}

				//set the master clock rate
				if (device_cfg->debug_settings)
					msg("sdr: Trying to set the following master clock rate: " + to_string(device_cfg->f_clk/1e6) + " [MHz]");
				msg("", INFO, false, false);
				device->set_master_clock_rate(device_cfg->f_clk);
				if (device_cfg->debug_settings)
					msg("sdr: Actual master clock rate: " + to_string(device->get_master_clock_rate()/1e6) + " [MHz]");

				if (device_cfg->tx_active)
				{
					//set the tx sample rate
					if (device_cfg->debug_settings)
						msg("sdr: Trying to set the following TX sample rate on channel " + to_string(device_cfg->channel_tx) + ": " + to_string((device_cfg->f_clk / (double)device_cfg->D_tx)/1e6) + " [MHz]");
					device->set_tx_rate(device_cfg->f_clk / (double)device_cfg->D_tx, device_cfg->channel_tx);
					if (device_cfg->debug_settings)
						msg("sdr: Actual TX sample rate on channel " + to_string(device_cfg->channel_tx) + ": " + to_string(device->get_tx_rate(device_cfg->channel_tx)/1e6) + " [MHz]");
				}

				if (device_cfg->rx_active)
				{
					//set the rx sample rate
					if (device_cfg->debug_settings)
						msg("sdr: Trying to set the following RX sample rate on channel " + to_string(device_cfg->channel_rx) + ": " + to_string((device_cfg->f_clk / (double)device_cfg->D_rx)/1e6) + " [MHz]");
					device->set_rx_rate(device_cfg->f_clk / (double)device_cfg->D_rx, device_cfg->channel_rx);
					if (device_cfg->debug_settings)
						msg("sdr: Actual RX sample rate on channel " + to_string(device_cfg->channel_rx) + ": " + to_string(device->get_rx_rate(device_cfg->channel_rx)/1e6) + " [MHz]");
				}

				if ((device_cfg->tx_active) && (device_cfg->B_tx > 0))
				{
					//set the tx bandwidth
					if (device_cfg->debug_settings)
						msg("sdr: Trying to set the following TX filter bandwidth on channel " + to_string(device_cfg->channel_tx) + ": " + to_string(device_cfg->B_tx / 1e6) + " [MHz]");
					device->set_tx_bandwidth(device_cfg->B_tx, device_cfg->channel_tx);
					if (device_cfg->debug_settings)
						msg("sdr: Actual TX bandwidth on channel " + to_string(device_cfg->channel_tx) + ": " + to_string(device->get_tx_bandwidth(device_cfg->channel_tx)/1e6) + " [MHz]");
				}

				if ((device_cfg->rx_active) && (device_cfg->B_rx > 0))
				{
					//set the tx bandwidth
					if (device_cfg->debug_settings)
						msg("sdr: Trying to set the following RX filter bandwidth on channel " + to_string(device_cfg->channel_rx) + ": " + to_string(device_cfg->B_rx / 1e6) + " [MHz]");
					device->set_rx_bandwidth(device_cfg->B_rx, device_cfg->channel_rx);
					if (device_cfg->debug_settings)
						msg("sdr: Actual RX bandwidth on channel " + to_string(device_cfg->channel_rx) + ": " + to_string(device->get_rx_bandwidth(device_cfg->channel_rx)/1e6) + " [MHz]");
				}

				if (device_cfg->tx_active)
				{
					//set the tx RF chain gain
					if (device_cfg->debug_settings)
						msg("sdr: Trying to set the following TX gain on channel " + to_string(device_cfg->channel_tx) + ": " + to_string(device_cfg->G_tx) + " [dB]");
					device->set_tx_gain(device_cfg->G_tx, device_cfg->channel_tx);
					if (device_cfg->debug_settings)
						msg("sdr: Actual TX gain on channel " + to_string(device_cfg->channel_tx) + ": " + to_string(device->get_tx_gain(device_cfg->channel_tx)) + " [dB]");
				}

				if (device_cfg->rx_active)
				{
					//set the rx RF chain gain
					if (device_cfg->debug_settings)
						msg("sdr: Trying to set the following RX gain on channel " + to_string(device_cfg->channel_rx) + ": " + to_string(device_cfg->G_rx) + " [dB]");
					device->set_rx_gain(device_cfg->G_rx, device_cfg->channel_rx);
					if (device_cfg->debug_settings)
						msg("sdr: Actual RX gain on channel " + to_string(device_cfg->channel_rx) + ": " + to_string(device->get_rx_gain(device_cfg->channel_rx)) + " [dB]");
				}

				if (device_cfg->tx_active)
				{
					//set the tx antenna
					if (device_cfg->debug_settings)
						msg("sdr: Trying to set the following TX antenna on channel " + to_string(device_cfg->channel_tx) + ": " + device_cfg->antenna_tx);
					device->set_tx_antenna(device_cfg->antenna_tx, device_cfg->channel_tx);
					if (device_cfg->debug_settings)
						msg("sdr: Actual TX antenna on channel " + to_string(device_cfg->channel_tx) + ": " + device->get_tx_antenna(device_cfg->channel_tx));
				}

				if (device_cfg->rx_active)
				{
					//set the rx antenna
					if (device_cfg->debug_settings)
						msg("sdr: Trying to set the following RX antenna on channel " + to_string(device_cfg->channel_rx) + ": " + device_cfg->antenna_rx);
					device->set_rx_antenna(device_cfg->antenna_rx, device_cfg->channel_rx);
					if (device_cfg->debug_settings)
						msg("sdr: Actual RX antenna on channel " + to_string(device_cfg->channel_rx) + ": " + device->get_rx_antenna(device_cfg->channel_rx));
				}

				if (device_cfg->tx_active)
				{
					//Set tx frequency
					if (device_cfg->debug_settings)
						msg("sdr: Trying to set the following TX frequency on channel " + to_string(device_cfg->channel_tx) + ": " + to_string(device_cfg->f_c_tx/1e6) + " [MHz]");

					//prepare tx and rx tune requests using Int-N tuning
					uhd::tune_request_t tx_tune_req(device_cfg->f_c_tx);
					tx_tune_req.args = uhd::device_addr_t("mode_n=integer");
					device->set_tx_freq(tx_tune_req, device_cfg->channel_tx);

					//Check TX LO lock...
					msg("sdr: Waiting for TX LO lock on channel " + to_string(device_cfg->channel_tx) + "...");

					bool tx_lo_locked = false;

					while (!(stop || tx_lo_locked))
					{
						tx_lo_locked = device->get_tx_sensor("lo_locked", device_cfg->channel_tx).to_bool();

						usleep(100);
						signal_handler.get_io_service().poll();
					}

					msg("sdr: TX LO lock detected on channel " + to_string(device_cfg->channel_tx) + "!");

					if (device_cfg->debug_settings)
						msg("sdr: Actual TX frequency on channel " + to_string(device_cfg->channel_tx) + ": " + to_string(device->get_tx_freq(device_cfg->channel_tx)/1e6) + " [MHz]");
				}

				if (device_cfg->rx_active)
				{
					//Set rx frequency
					if (device_cfg->debug_settings)
						msg("sdr: Trying to set the following RX frequency on channel " + to_string(device_cfg->channel_rx) + ": " + to_string(device_cfg->f_c_rx/1e6) + " [MHz]");

					//prepare tx and rx tune requests using Int-N tuning
					uhd::tune_request_t rx_tune_req(device_cfg->f_c_rx);
					rx_tune_req.args = uhd::device_addr_t("mode_n=integer");
					device->set_rx_freq(rx_tune_req, device_cfg->channel_rx);

					//Check RX LO lock...
					msg("sdr: Waiting for RX LO lock on channel " + to_string(device_cfg->channel_rx) + "...");

					bool rx_lo_locked = false;

					while (!(stop || rx_lo_locked))
					{
						rx_lo_locked = device->get_rx_sensor("lo_locked", device_cfg->channel_rx).to_bool();

						usleep(100);
						signal_handler.get_io_service().poll();
					}

					msg("sdr: RX LO lock detected on channel " + to_string(device_cfg->channel_rx) + "!");

					if (device_cfg->debug_settings)
						msg("sdr: Actual RX frequency on channel " + to_string(device_cfg->channel_rx) + ": " + to_string(device->get_rx_freq(device_cfg->channel_rx)/1e6) + " [MHz]");
				}

				if (device_cfg->tx_active)
				{
					//setup TX stream
					if (device_cfg->debug_settings)
						msg("sdr: Trying to setup TX stream...");

					uhd::stream_args_t stream_args_tx("fc32"); //complex floats
					stream_args_tx.channels.push_back(device_cfg->channel_tx);

					tx_stream = device->get_tx_stream(stream_args_tx);

					if (tx_stream.get() == nullptr)
						throw(std::runtime_error("Unable to setup TX stream!"));
					else if (device_cfg->debug_settings)
						msg("sdr: TX stream has been successfully set up!");
				}

				if (device_cfg->rx_active)
				{
					//setup RX stream
					if (device_cfg->debug_settings)
						msg("sdr: Trying to setup RX stream...");

					uhd::stream_args_t stream_args_rx("fc32"); //complex floats
					stream_args_rx.channels.push_back(device_cfg->channel_rx);

					rx_stream = device->get_rx_stream(stream_args_rx);

					if (rx_stream.get() == nullptr)
						throw(std::runtime_error("Unable to setup RX stream!"));
					else if (device_cfg->debug_settings)
						msg("sdr: RX stream has been successfully set up!");
				}

				//no "activation" calls present in UHD

				int mtu_tx = tx_stream->get_max_num_samps();
				int mtu_rx = rx_stream->get_max_num_samps();

				msg("sdr: mtu_tx=" + to_string(mtu_tx) + " [Sa], mtu_rx=" + to_string(mtu_rx) + " [Sa]");

				init_successfull = true;

				sleep(1);
			}
			catch (const std::exception& e)
			{
				msg("sdr: Following exception occurred during initialization of the SDR device: " + string(e.what()), ERROR);
				sleep(1);
			}
			catch (...)
			{
				msg("sdr: Unexpected exception was caught during initialization of the SDR device", ERROR);
				sleep(1);
			}

			signal_handler.get_io_service().poll();
		}
		while ((!stop) && (!init_successfull));
	}

	SDR_Device_Wrapper::~SDR_Device_Wrapper()
	{
		//no special cleaning needed in UHD...
	}

	bool SDR_Device_Wrapper::send_samples(const int64_t tick, const std::complex<float>* samples, int no_of_requested_samples, const bool ack)
	{
		bool res = false;

		std::vector<std::complex<float>> buf(1, {0, 0});

		if (no_of_requested_samples <= 0)
		{
			samples = &buf[0];
			no_of_requested_samples = buf.size();
		}

		string tx_verbose_msg = "[TX] ";

		int64_t burst_time = 1e9 * uhd::time_spec_t::from_ticks(tick, device_cfg->f_clk).get_real_secs();
		uhd::time_spec_t current_time_spec = device->get_time_now();
		int64_t current_time = 1e9*current_time_spec.get_real_secs();
		//int64_t current_tick = current_time_spec.to_ticks(device_cfg->f_clk);
		tx_verbose_msg += " current_time=" + to_string(current_time) + ", burst_time: " + to_string(burst_time);

		uhd::tx_metadata_t tx_md;

		tx_md.start_of_burst = true;
		tx_md.end_of_burst = true;
		tx_md.has_time_spec = true;
		tx_md.time_spec = uhd::time_spec_t::from_ticks(tick, device_cfg->f_clk);

		int no_of_transmitted_samples = tx_stream->send
		(
			samples,
			no_of_requested_samples,
			tx_md,
			device_cfg->T_timeout
		);

		if (!ack)
		{
			tx_verbose_msg += (boost::format(", no_of_transmitted_samples: %u/%u") % no_of_transmitted_samples % no_of_requested_samples).str();
			msg(tx_verbose_msg);
			return true;
		}
		else
		{

			uhd::async_metadata_t async_md;

			if (ack)
			{
				tx_verbose_msg += ", ack_code: ";

				if (!tx_stream->recv_async_msg(async_md, device_cfg->T_timeout))
				{
					tx_verbose_msg += "ERROR_CODE_TIMEOUT";
				}
				else
				{
					switch(async_md.event_code)
					{
						case uhd::async_metadata_t::EVENT_CODE_BURST_ACK:
							tx_verbose_msg += "EVENT_CODE_BURST_ACK";
							break;
						case uhd::async_metadata_t::EVENT_CODE_TIME_ERROR:
							tx_verbose_msg += "EVENT_CODE_TIME_ERROR";
							break;
						case uhd::async_metadata_t::EVENT_CODE_UNDERFLOW:
							tx_verbose_msg += "EVENT_CODE_UNDERFLOW";
							break;
						default:
							tx_verbose_msg += (boost::format("UNEXPECTED_EVENT_CODE: 0x%i") % async_md.event_code).str();
							break;
					}
				}
			}

			tx_verbose_msg += (boost::format(", no_of_transmitted_samples: %u/%u") % no_of_transmitted_samples % no_of_requested_samples).str();


			if ((async_md.event_code == uhd::async_metadata_t::EVENT_CODE_BURST_ACK) && (no_of_transmitted_samples == no_of_requested_samples))
			{
				msg("sdr: " + tx_verbose_msg);
				res = true;
			}
			else
			{
				msg("sdr: " + tx_verbose_msg, WARNING);
				res = false;
			}
		}

		return res;
	}

	void SDR_Device_Wrapper::send_samples_void(bool& success, const int64_t tick, const std::complex<float>* samples, int no_of_requested_samples, const bool ack)
	{
		success = send_samples(tick, samples, no_of_requested_samples, ack);
	}

	bool SDR_Device_Wrapper::receive_samples(const int64_t tick, std::complex<float>* samples, int no_of_requested_samples)
	{
		bool res = false;

		std::vector<std::complex<float>> buf(1, {0, 0});

		if (no_of_requested_samples <= 0)
		{
			samples = &buf[0];
			no_of_requested_samples = buf.size();
		}

		string rx_verbose_msg = "[RX] ";

		int64_t burst_time = 1e9 * uhd::time_spec_t::from_ticks(tick, device_cfg->f_clk).get_real_secs();
		uhd::time_spec_t current_time_spec = device->get_time_now();
		int64_t current_time = 1e9*current_time_spec.get_real_secs();
		//int64_t current_tick = current_time_spec.to_ticks(device_cfg->f_clk);
		rx_verbose_msg += " current_time=" + to_string(current_time) + ", burst_time: " + to_string(burst_time);

		uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
		stream_cmd.num_samps = no_of_requested_samples;
		stream_cmd.stream_now = false;
		stream_cmd.time_spec = uhd::time_spec_t::from_ticks(tick, device_cfg->f_clk);

		rx_stream->issue_stream_cmd(stream_cmd);
		uhd::rx_metadata_t rx_md;

		int no_of_received_samples = rx_stream->recv(samples, no_of_requested_samples, rx_md, device_cfg->T_timeout);

		rx_verbose_msg += ", read_code: ";
		switch (rx_md.error_code)
		{
			case uhd::rx_metadata_t::ERROR_CODE_NONE:
				rx_verbose_msg += "ERROR_CODE_NONE";
				break;

			case uhd::rx_metadata_t::ERROR_CODE_TIMEOUT:
				rx_verbose_msg += "ERROR_CODE_TIMEOUT";
				break;

			case uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND:
				rx_verbose_msg += "ERROR_CODE_LATE_COMMAND";
				break;

			case uhd::rx_metadata_t::ERROR_CODE_BROKEN_CHAIN:
				rx_verbose_msg += "ERROR_CODE_BROKEN_CHAIN ";
				break;

			case uhd::rx_metadata_t::ERROR_CODE_OVERFLOW:
				rx_verbose_msg += "ERROR_CODE_OVERFLOW";
				break;

			case uhd::rx_metadata_t::ERROR_CODE_ALIGNMENT:
				rx_verbose_msg += "ERROR_CODE_ALIGNMENT ";
				break;

			case uhd::rx_metadata_t::ERROR_CODE_BAD_PACKET:
				rx_verbose_msg += "ERROR_CODE_BAD_PACKET";
				break;

			default:
				rx_verbose_msg += (boost::format("UNEXPECTED ERROR CODE: 0x%i") % rx_md.error_code).str();
				break;
		}

		rx_verbose_msg += (boost::format(", no_of_received_samples: %u/%u") % no_of_received_samples % no_of_requested_samples).str();

		if ((rx_md.error_code == uhd::rx_metadata_t::ERROR_CODE_NONE) && (no_of_received_samples == no_of_requested_samples))
		{
			msg("sdr: " + rx_verbose_msg);
			res = true;
		}
		else
		{
			msg("sdr: " + rx_verbose_msg, WARNING);
			res = false;
		}

		return res;
	}

	void SDR_Device_Wrapper::receive_samples_void(bool& success,const int64_t tick, std::complex<float>* samples, int no_of_requested_samples)
	{
		success = receive_samples(tick, samples, no_of_requested_samples);
	}

	uhd::usrp::multi_usrp::sptr SDR_Device_Wrapper::get_device()
	{
		return device;
	}

	SDR_Device_Config::sptr_t SDR_Device_Wrapper::get_device_config()
	{
		return device_cfg;
	}
}
