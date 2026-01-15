// SPDX-License-Identifier: GPL-2.0
/*
 * Texas Instruments ADS131E0x 4-, 6- and 8-Channel ADCs
 *
 * Copyright (c) 2020 AVL DiTEST GmbH
 *   Tomislav Denis <tomislav.denis@avl.com>
 *
 * Datasheet: https://www.ti.com/lit/ds/symlink/ads131e08.pdf
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/unaligned.h>

/* Commands */
#define ADS131E08_CMD_RESET 0x06
#define ADS131E08_CMD_START 0x08
#define ADS131E08_CMD_STOP 0x0A
#define ADS131E08_CMD_OFFSETCAL 0x1A
#define ADS131E08_CMD_RDATAC 0x10
#define ADS131E08_CMD_SDATAC 0x11
#define ADS131E08_CMD_RDATA 0x12
#define ADS131E08_CMD_RREG(r) (BIT(5) | (r & GENMASK(4, 0)))
#define ADS131E08_CMD_WREG(r) (BIT(6) | (r & GENMASK(4, 0)))

/* Registers */
#define ADS131E08_ADR_CFG1R 0x01
#define ADS131E08_ADR_CFG3R 0x03
#define ADS131E08_ADR_CH0R 0x05

/* Configuration register 1 */
#define ADS131E08_CFG1R_DR_MASK GENMASK(2, 0)

/* Configuration register 3 */
#define ADS131E08_CFG3R_PDB_REFBUF_MASK BIT(7)
#define ADS131E08_CFG3R_VREF_4V_MASK BIT(5)

/* Channel settings register */
#define ADS131E08_CHR_GAIN_MASK GENMASK(6, 4)
#define ADS131E08_CHR_MUX_MASK GENMASK(2, 0)
#define ADS131E08_CHR_PWD_MASK BIT(7)

/* ADC  misc */
#define ADS131E08_DEFAULT_DATA_RATE 32
#define ADS131E08_DEFAULT_PGA_GAIN 1
#define ADS131E08_DEFAULT_MUX 0

#define ADS131E08_VREF_2V4_mV 2400
#define ADS131E08_VREF_4V_mV 4000

#define ADS131E08_WAIT_RESET_CYCLES 20
#define ADS131E08_WAIT_SDECODE_CYCLES 6
#define ADS131E08_WAIT_OFFSETCAL_MS 200
#define ADS131E08_MAX_SETTLING_TIME_MS 6

#define ADS131E08_NUM_STATUS_BYTES 3
#define ADS131E08_NUM_DATA_BYTES_MAX 24
#define ADS131E08_NUM_DATA_BYTES(dr) (((dr) >= 32) ? 2 : 3)
#define ADS131E08_NUM_DATA_BITS(dr) (ADS131E08_NUM_DATA_BYTES(dr) * 8)
#define ADS131E08_NUM_STORAGE_BYTES 4

enum ads131e08_ids {
	ads131e04,
	ads131e06,
	ads131e08,
};

struct ads131e08_info {
	unsigned int max_channels;
	const char *name;
};

struct ads131e08_channel_config {
	unsigned int pga_gain;
	unsigned int mux;
};

struct ads131e08_state {
	const struct ads131e08_info *info;
	struct spi_device *spi;
	struct clk *adc_clk;
	struct completion completion;
	unsigned int sdecode_delay_us;
	unsigned int reset_delay_us;
	struct regulator *vref_reg;
	unsigned int vref_mv;
	struct ads131e08_channel_config *channel_config;
	u8 data_rate;
	u8 readback_len;
	bool rdatac_enabled;
	struct spi_transfer xfer;
	struct spi_message msg;
	/*
	 * Add extra one padding byte to be able to access the last channel
	 * value using u32 pointer
	 */
	u8 rx_buf[ADS131E08_NUM_STATUS_BYTES + ADS131E08_NUM_DATA_BYTES_MAX +
		  1] __aligned(IIO_DMA_MINALIGN);
	u8 *channel_ptrs[8];
	u32 data[8] __aligned(IIO_DMA_MINALIGN);
};

static const struct ads131e08_info ads131e08_info_tbl[] = {
	[ads131e04] = {
		.max_channels = 4,
		.name = "ads131e04",
	},
	[ads131e06] = {
		.max_channels = 6,
		.name = "ads131e06",
	},
	[ads131e08] = {
		.max_channels = 8,
		.name = "ads131e08",
	},
};

struct ads131e08_data_rate_desc {
	unsigned int rate; /* data rate in kSPS */
	u8 reg; /* reg value */
};

static const struct ads131e08_data_rate_desc ads131e08_data_rate_tbl[] = {
	{ .rate = 64, .reg = 0x00 }, { .rate = 32, .reg = 0x01 },
	{ .rate = 16, .reg = 0x02 }, { .rate = 8, .reg = 0x03 },
	{ .rate = 4, .reg = 0x04 },  { .rate = 2, .reg = 0x05 },
	{ .rate = 1, .reg = 0x06 },
};

struct ads131e08_pga_gain_desc {
	unsigned int gain; /* PGA gain value */
	u8 reg; /* field value */
};

static const struct ads131e08_pga_gain_desc ads131e08_pga_gain_tbl[] = {
	{ .gain = 1, .reg = 0x01 },  { .gain = 2, .reg = 0x02 },
	{ .gain = 4, .reg = 0x04 },  { .gain = 8, .reg = 0x05 },
	{ .gain = 12, .reg = 0x06 },
};

static const u8 ads131e08_valid_channel_mux_values[] = { 0, 1, 3, 4 };

static int ads131e08_exec_cmd(struct ads131e08_state *st, u8 cmd,
			      unsigned long delay_us)
{
	int ret;
	u8 tx = cmd;

	struct spi_transfer transfer = {
		.tx_buf = &tx,
		.len = 1,
		.cs_change = 0,
		.delay = { .value = delay_us, .unit = SPI_DELAY_UNIT_USECS }
	};

	ret = spi_sync_transfer(st->spi, &transfer, 1);
	if (ret) {
		dev_err(&st->spi->dev, "Exec cmd 0x%02x failed: %d\n", cmd,
			ret);
		return ret;
	}

	dev_info(&st->spi->dev, "Exec cmd 0x%02x\n", cmd);
	return 0;
}

static int ads131e08_read_reg(struct ads131e08_state *st, u8 reg, u8 *val)
{
	int ret;
	u8 cmd0 = ADS131E08_CMD_RREG(reg);
	u8 cmd1 = 0x00;
	u8 rx;

	struct spi_transfer transfer[] = {
		{ .tx_buf = &cmd0,
		  .len = 1,
		  .cs_change = 0,
		  .delay = { .value = st->sdecode_delay_us,
			     .unit = SPI_DELAY_UNIT_USECS } },
		{ .tx_buf = &cmd1,
		  .len = 1,
		  .cs_change = 0,
		  .delay = { .value = st->sdecode_delay_us,
			     .unit = SPI_DELAY_UNIT_USECS } },
		{ .rx_buf = &rx, .len = 1, .cs_change = 0 }
	};

	ret = spi_sync_transfer(st->spi, transfer, 3);
	if (ret) {
		dev_err(&st->spi->dev, "Read reg 0x%02x failed: %d\n", reg,
			ret);
		return ret;
	}

	*val = rx;

	return 0;
}

static int ads131e08_write_reg(struct ads131e08_state *st, u8 reg, u8 value)
{
	int ret;
	u8 cmd0 = ADS131E08_CMD_WREG(reg);
	u8 cmd1 = 0x00;
	u8 cmd2 = value;

	struct spi_transfer transfer[] = {
		{ .tx_buf = &cmd0,
		  .len = 1,
		  .cs_change = 0,
		  .delay = { .value = st->sdecode_delay_us,
			     .unit = SPI_DELAY_UNIT_USECS } },
		{ .tx_buf = &cmd1,
		  .len = 1,
		  .cs_change = 0,
		  .delay = { .value = st->sdecode_delay_us,
			     .unit = SPI_DELAY_UNIT_USECS } },
		{ .tx_buf = &cmd2,
		  .len = 1,
		  .cs_change = 0,
		  .delay = { .value = st->sdecode_delay_us,
			     .unit = SPI_DELAY_UNIT_USECS } }
	};

	ret = spi_sync_transfer(st->spi, transfer, 3);
	if (ret) {
		dev_err(&st->spi->dev, "Write reg 0x%02x failed: %d\n", reg,
			ret);
		return ret;
	}

	dev_info(&st->spi->dev, "Written to 0x%02x: value=%02x\n", cmd0, cmd2);

	return 0;
}

static int ads131e08_read_data(struct ads131e08_state *st)
{
	int ret;

	u8 tx = ADS131E08_CMD_RDATA;

	struct spi_transfer transfer[] = {
		{ .tx_buf = &tx, .len = 1, .cs_change = 0 },
		{ .rx_buf = st->rx_buf, .len = st->readback_len, .cs_change = 0 }
	};

	ret = spi_sync_transfer(st->spi, transfer, 2);
	if (ret)
		dev_err(&st->spi->dev, "Read data failed\n");

	return ret;
}

static int ads131e08_stop_read_data_continuous(struct ads131e08_state *st)
{
	int ret;
	u8 nop = 0x00;

	ret = ads131e08_exec_cmd(st, ADS131E08_CMD_SDATAC,
				 st->sdecode_delay_us);
	if (ret)
		return ret;

	ret = spi_write(st->spi, &nop, 1);
	if (ret)
		return ret;

	ret = spi_read(st->spi, st->rx_buf, st->readback_len);
	if (ret)
		return ret;

	ret = spi_write(st->spi, &nop, 1);
	if (ret)
		return ret;

	return 0;
}

static int ads131e08_check_status(struct ads131e08_state *st)
{
	u8 *buf = st->rx_buf;
	int i;
	int ret = 0;

	u32 status = ((u32)buf[0] << 16) | ((u32)buf[1] << 8) | ((u32)buf[2]);

	/* Header check (bits 23:20) should be 0b1100 */
	if (((status >> 20) & 0xF) != 0xC) {
		dev_err_ratelimited(&st->spi->dev,
				    "Status word header invalid: 0x%06x\n",
				    status);
		ret = -EIO;
	}

	u8 p_fault = (status >> 12) & 0xFF;
	u8 n_fault = (status >> 4) & 0xFF;

	for (i = 0; i < st->info->max_channels; i++) {
		if (p_fault & BIT(i))
			dev_warn_ratelimited(
				&st->spi->dev,
				"Positive fault detected on channel %d\n", i);

		if (n_fault & BIT(i))
			dev_warn_ratelimited(
				&st->spi->dev,
				"Negative fault detected on channel %d\n", i);
	}

	return ret;
}

static int ads131e08_set_data_rate(struct ads131e08_state *st, int data_rate)
{
	int i, ret;
	u8 reg;

	for (i = 0; i < ARRAY_SIZE(ads131e08_data_rate_tbl); i++) {
		if (ads131e08_data_rate_tbl[i].rate == data_rate)
			break;
	}

	if (i == ARRAY_SIZE(ads131e08_data_rate_tbl)) {
		dev_err(&st->spi->dev, "invalid data rate value\n");
		return -EINVAL;
	}

	ret = ads131e08_read_reg(st, ADS131E08_ADR_CFG1R, &reg);
	if (ret)
		return ret;

	reg &= ~ADS131E08_CFG1R_DR_MASK;
	reg |= FIELD_PREP(ADS131E08_CFG1R_DR_MASK,
			  ads131e08_data_rate_tbl[i].reg);

	ret = ads131e08_write_reg(st, ADS131E08_ADR_CFG1R, reg);
	if (ret)
		return ret;

	/* Update state */
	st->data_rate = data_rate;
	st->readback_len = ADS131E08_NUM_STATUS_BYTES +
			   ADS131E08_NUM_DATA_BYTES(st->data_rate) *
				   st->info->max_channels;
	st->xfer.len = st->readback_len;
	spi_message_init(&st->msg);
	spi_message_add_tail(&st->xfer, &st->msg);

	return 0;
}

static int ads131e08_pga_gain_to_field_value(struct ads131e08_state *st,
					     unsigned int pga_gain)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ads131e08_pga_gain_tbl); i++) {
		if (ads131e08_pga_gain_tbl[i].gain == pga_gain)
			break;
	}

	if (i == ARRAY_SIZE(ads131e08_pga_gain_tbl)) {
		dev_err(&st->spi->dev, "invalid PGA gain value\n");
		return -EINVAL;
	}

	return ads131e08_pga_gain_tbl[i].reg;
}

static int ads131e08_validate_channel_mux(struct ads131e08_state *st,
					  unsigned int mux)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ads131e08_valid_channel_mux_values); i++) {
		if (ads131e08_valid_channel_mux_values[i] == mux)
			break;
	}

	if (i == ARRAY_SIZE(ads131e08_valid_channel_mux_values)) {
		dev_err(&st->spi->dev, "invalid channel mux value\n");
		return -EINVAL;
	}

	return 0;
}

static int ads131e08_set_channel_config(struct ads131e08_state *st,
					unsigned int channel,
					unsigned int pga_gain, unsigned int mux,
					bool power_down)
{
	int gain, ret;
	u8 reg;

	/* Convert gain to register field */
	gain = ads131e08_pga_gain_to_field_value(st, pga_gain);
	if (gain < 0)
		return gain;

	/* Read current channel register */
	ret = ads131e08_read_reg(st, ADS131E08_ADR_CH0R + channel, &reg);
	if (ret)
		return ret;

	/* Update gain */
	reg &= ~ADS131E08_CHR_GAIN_MASK;
	reg |= FIELD_PREP(ADS131E08_CHR_GAIN_MASK, gain);

	/* Update mux */
	reg &= ~ADS131E08_CHR_MUX_MASK;
	reg |= FIELD_PREP(ADS131E08_CHR_MUX_MASK, mux);

	/* Update power down */
	reg &= ~ADS131E08_CHR_PWD_MASK;
	reg |= FIELD_PREP(ADS131E08_CHR_PWD_MASK, power_down);

	return ads131e08_write_reg(st, ADS131E08_ADR_CH0R + channel, reg);
}

static int ads131e08_config_reference_voltage(struct ads131e08_state *st)
{
	int ret;
	u8 reg;

	ret = ads131e08_read_reg(st, ADS131E08_ADR_CFG3R, &reg);
	if (ret)
		return ret;

	reg &= ~ADS131E08_CFG3R_PDB_REFBUF_MASK;
	if (!st->vref_reg) {
		reg |= FIELD_PREP(ADS131E08_CFG3R_PDB_REFBUF_MASK, 1);
		reg &= ~ADS131E08_CFG3R_VREF_4V_MASK;
		reg |= FIELD_PREP(ADS131E08_CFG3R_VREF_4V_MASK,
				  st->vref_mv == ADS131E08_VREF_4V_mV);
	}

	return ads131e08_write_reg(st, ADS131E08_ADR_CFG3R, reg);
}

static int ads131e08_initial_config(struct iio_dev *indio_dev)
{
	const struct iio_chan_spec *channel = indio_dev->channels;
	struct ads131e08_state *st = iio_priv(indio_dev);
	unsigned long active_channels = 0;
	int ret, i;

	/* Disable read data in continuous mode (enabled by default) */
	ret = ads131e08_stop_read_data_continuous(st);
	if (ret)
		return ret;

	ret = ads131e08_exec_cmd(st, ADS131E08_CMD_RESET, st->reset_delay_us);
	if (ret)
		return ret;

	ret = ads131e08_stop_read_data_continuous(st);
	if (ret)
		return ret;

	ret = ads131e08_set_data_rate(st, ADS131E08_DEFAULT_DATA_RATE);
	if (ret)
		return ret;

	ret = ads131e08_config_reference_voltage(st);
	if (ret)
		return ret;

	for (i = 0; i < indio_dev->num_channels; i++) {
		ret = ads131e08_set_channel_config(
			st, channel->channel, st->channel_config[i].pga_gain,
			st->channel_config[i].mux, false);
		if (ret)
			return ret;

		active_channels |= BIT(channel->channel);
		channel++;
	}

	/* Power down unused channels */
	for_each_clear_bit(i, &active_channels, st->info->max_channels) {
		ret = ads131e08_set_channel_config(st, i,
						   ADS131E08_DEFAULT_PGA_GAIN,
						   ADS131E08_DEFAULT_MUX, true);
		if (ret)
			return ret;
	}

	/* Request channel offset calibration */
	ret = ads131e08_exec_cmd(st, ADS131E08_CMD_OFFSETCAL,
				 st->sdecode_delay_us);
	if (ret)
		return ret;

	/*
	 * Channel offset calibration is triggered with the first START
	 * command. Since calibration takes more time than settling operation,
	 * this causes timeout error when command START is sent first
	 * time (e.g. first call of the ads131e08_read_direct method).
	 * To avoid this problem offset calibration is triggered here.
	 */
	ret = ads131e08_exec_cmd(st, ADS131E08_CMD_START,
				 st->sdecode_delay_us +
					 ADS131E08_WAIT_OFFSETCAL_MS * 1000);
	if (ret)
		return ret;

	return ads131e08_exec_cmd(st, ADS131E08_CMD_STOP, st->sdecode_delay_us);
}

static int ads131e08_poll_data(struct ads131e08_state *st)
{
	int ret;

	reinit_completion(&st->completion);

	ret = ads131e08_exec_cmd(st, ADS131E08_CMD_START, st->sdecode_delay_us);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(
		&st->completion,
		msecs_to_jiffies(ADS131E08_MAX_SETTLING_TIME_MS));
	if (!ret)
		return -ETIMEDOUT;

	ret = ads131e08_read_data(st);
	if (ret)
		return ret;

	ret = ads131e08_check_status(st);
	if (ret)
		return ret;

	return ads131e08_exec_cmd(st, ADS131E08_CMD_STOP, st->sdecode_delay_us);
}

static int ads131e08_read_direct(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *channel,
				 int *value)
{
	struct ads131e08_state *st = iio_priv(indio_dev);
	u8 num_bits, *src;
	int ret;

	ret = ads131e08_poll_data(st);
	if (ret)
		return ret;

	src = st->rx_buf + ADS131E08_NUM_STATUS_BYTES +
	      channel->channel * ADS131E08_NUM_DATA_BYTES(st->data_rate);

	num_bits = ADS131E08_NUM_DATA_BITS(st->data_rate);
	*value = sign_extend32(get_unaligned_be32(src) >> (32 - num_bits),
			       num_bits - 1);

	return 0;
}

static int ads131e08_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *channel, int *value,
			      int *value2, long mask)
{
	struct ads131e08_state *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = ads131e08_read_direct(indio_dev, channel, value);
		iio_device_release_direct_mode(indio_dev);
		if (ret)
			return ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		if (st->vref_reg) {
			ret = regulator_get_voltage(st->vref_reg);
			if (ret < 0)
				return ret;

			*value = ret / 1000;
		} else {
			*value = st->vref_mv;
		}

		*value /= st->channel_config[channel->address].pga_gain;
		*value2 = ADS131E08_NUM_DATA_BITS(st->data_rate) - 1;

		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*value = st->data_rate;

		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int ads131e08_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *channel, int value,
			       int value2, long mask)
{
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = ads131e08_set_data_rate(st, value);
		iio_device_release_direct_mode(indio_dev);
		return ret;

	default:
		return -EINVAL;
	}
}

static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("1 2 4 8 16 32 64");

static struct attribute *ads131e08_attributes[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr, NULL
};

static const struct attribute_group ads131e08_attribute_group = {
	.attrs = ads131e08_attributes,
};

static int ads131e08_debugfs_reg_access(struct iio_dev *indio_dev,
					unsigned int reg, unsigned int writeval,
					unsigned int *readval)
{
	struct ads131e08_state *st = iio_priv(indio_dev);
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	if (readval) {
		u8 reg_value;
		ret = ads131e08_read_reg(st, reg, &reg_value);
		if (ret)
			return ret;
		*readval = reg_value;
		return 0;
	}

	ret = ads131e08_write_reg(st, reg, writeval);
	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static const struct iio_info ads131e08_iio_info = {
	.read_raw = ads131e08_read_raw,
	.write_raw = ads131e08_write_raw,
	.attrs = &ads131e08_attribute_group,
	.debugfs_reg_access = &ads131e08_debugfs_reg_access,
};

static int ads131e08_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ads131e08_state *st = iio_priv(indio_dev);
	int ret, i = 0;

	iio_for_each_active_channel(indio_dev, chn)
	{
		dev_info(&st->spi->dev, "idx %d channel %d\n", i, chn);
		st->channel_ptrs[i] =
			st->rx_buf + ADS131E08_NUM_STATUS_BYTES +
			chn * ADS131E08_NUM_DATA_BYTES(st->data_rate);

		i++;
	}
	ret = ads131e08_exec_cmd(st, ADS131E08_CMD_RDATAC,
				 st->sdecode_delay_us);
	if (ret)
		return ret;

	ret = ads131e08_exec_cmd(st, ADS131E08_CMD_START, st->sdecode_delay_us);
	if (ret)
		return ret;

	st->rdatac_enabled = true;
	return 0;
}

static int ads131e08_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ads131e08_state *st = iio_priv(indio_dev);
	int ret;

	ret = ads131e08_stop_read_data_continuous(st);
	if (ret)
		return ret;

	ret = ads131e08_exec_cmd(st, ADS131E08_CMD_STOP, st->sdecode_delay_us);
	if (ret)
		return ret;

	st->rdatac_enabled = false;

	for (i = 0; i < indio_dev->num_channels; i++) {
		st->channel_ptrs[i] = NULL;
	}
	return 0;
}

static const struct iio_buffer_setup_ops ads131e08_buffer_ops = {
	.preenable = ads131e08_buffer_preenable,
	.postdisable = ads131e08_buffer_postdisable
};

static irqreturn_t ads131e08_interrupt(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct ads131e08_state *st = iio_priv(indio_dev);

	if (!st->rdatac_enabled) {
		complete(&st->completion);
		return IRQ_HANDLED;
	}

	return IRQ_WAKE_THREAD;
}

static irqreturn_t ads131e08_data_ready_thread(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct ads131e08_state *st = iio_priv(indio_dev);
	int i;
	u8 *src;
	u32 *data = st->data;

	if (!st->rdatac_enabled)
		return IRQ_HANDLED;

	if (spi_sync(st->spi, &st->msg)) {
		dev_warn(&st->spi->dev, "SPI read in thread failed\n");
		return IRQ_HANDLED;
	}

	if (ads131e08_check_status(st))
		return IRQ_HANDLED;

	if (st->data_rate < 32) {
		for (i = 0; i < indio_dev->num_channels; i++) {
			src = st->channel_ptrs[i];
			*data++ = ((u32)src[0] << 24) | ((u32)src[1] << 16) |
				  ((u32)src[2] << 8);
		}
	} else {
		for (i = 0; i < indio_dev->num_channels; i++) {
			src = st->channel_ptrs[i];
			u8 sign = src[0] & BIT(7) ? 0xff : 0x00;
			*data++ = ((u32)sign << 24) | ((u32)src[0] << 16) |
				  ((u32)src[1] << 8);
		}
	}

	iio_push_to_buffers_with_timestamp(indio_dev, (u8 *)st->data,
					   iio_get_time_ns(indio_dev));

	return IRQ_HANDLED;
}

static int ads131e08_alloc_channels(struct iio_dev *indio_dev)
{
	struct ads131e08_state *st = iio_priv(indio_dev);
	struct ads131e08_channel_config *channel_config;
	struct device *dev = &st->spi->dev;
	struct iio_chan_spec *channels;
	unsigned int channel, tmp;
	int num_channels, i, ret;

	ret = device_property_read_u32(dev, "ti,vref-internal", &tmp);
	if (ret)
		tmp = 0;

	switch (tmp) {
	case 0:
		st->vref_mv = ADS131E08_VREF_2V4_mV;
		break;
	case 1:
		st->vref_mv = ADS131E08_VREF_4V_mV;
		break;
	default:
		dev_err(&st->spi->dev, "invalid internal voltage reference\n");
		return -EINVAL;
	}

	num_channels = device_get_child_node_count(dev);
	if (num_channels == 0) {
		dev_err(&st->spi->dev, "no channel children\n");
		return -ENODEV;
	}

	if (num_channels > st->info->max_channels) {
		dev_err(&st->spi->dev,
			"num of channel children out of range\n");
		return -EINVAL;
	}

	channels = devm_kcalloc(&st->spi->dev, num_channels, sizeof(*channels),
				GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	channel_config = devm_kcalloc(&st->spi->dev, num_channels,
				      sizeof(*channel_config), GFP_KERNEL);
	if (!channel_config)
		return -ENOMEM;

	i = 0;
	device_for_each_child_node_scoped(dev, node) {
		ret = fwnode_property_read_u32(node, "reg", &channel);
		if (ret)
			return ret;

		ret = fwnode_property_read_u32(node, "ti,gain", &tmp);
		if (ret) {
			channel_config[i].pga_gain = ADS131E08_DEFAULT_PGA_GAIN;
		} else {
			ret = ads131e08_pga_gain_to_field_value(st, tmp);
			if (ret < 0)
				return ret;

			channel_config[i].pga_gain = tmp;
		}

		ret = fwnode_property_read_u32(node, "ti,mux", &tmp);
		if (ret) {
			channel_config[i].mux = ADS131E08_DEFAULT_MUX;
		} else {
			ret = ads131e08_validate_channel_mux(st, tmp);
			if (ret)
				return ret;

			channel_config[i].mux = tmp;
		}

		channels[i].type = IIO_VOLTAGE;
		channels[i].indexed = 1;
		channels[i].channel = channel;
		channels[i].address = i;
		channels[i].info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
						 BIT(IIO_CHAN_INFO_SCALE);
		channels[i].info_mask_shared_by_type =
			BIT(IIO_CHAN_INFO_SAMP_FREQ);
		channels[i].scan_index = i;
		channels[i].scan_type.sign = 's';
		channels[i].scan_type.realbits = ADS131E08_NUM_DATA_BYTES_MAX;
		channels[i].scan_type.storagebits = 32;
		channels[i].scan_type.shift = 0;
		channels[i].scan_type.endianness = IIO_BE;
		i++;
	}

	indio_dev->channels = channels;
	indio_dev->num_channels = num_channels;
	st->channel_config = channel_config;

	return 0;
}

static void ads131e08_regulator_disable(void *data)
{
	struct ads131e08_state *st = data;

	regulator_disable(st->vref_reg);
}

static int ads131e08_probe(struct spi_device *spi)
{
	const struct ads131e08_info *info;
	struct iio_dev *indio_dev;
	struct ads131e08_state *st;
	unsigned long adc_clk_hz;
	unsigned long adc_clk_ns;
	int ret;

	info = spi_get_device_match_data(spi);
	if (!info) {
		dev_err(&spi->dev, "failed to get match data\n");
		return -ENODEV;
	}

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev) {
		dev_err(&spi->dev, "failed to allocate IIO device\n");
		return -ENOMEM;
	}

	st = iio_priv(indio_dev);
	st->info = info;
	st->spi = spi;

	st->rdatac_enabled = false;

	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	init_completion(&st->completion);
	memset(st->rx_buf, 0, sizeof(st->rx_buf));
	memset(&st->xfer, 0, sizeof(st->xfer));
	st->xfer.rx_buf = st->rx_buf;
	st->xfer.cs_change = 0;

	for (i = 0; i < ARRAY_SIZE(st->channel_ptrs); i++) {
		st->channel_ptrs[i] = NULL;
	}

	ret = ads131e08_alloc_channels(indio_dev);
	if (ret)
		return ret;

	indio_dev->name = st->info->name;
	indio_dev->info = &ads131e08_iio_info;

	ret = devm_iio_kfifo_buffer_setup(&spi->dev, indio_dev,
					  &ads131e08_buffer_ops);
	if (ret) {
		dev_err(&spi->dev, "failed to setup kfifo buffer\n");
		return ret;
	}

	if (spi->irq) {
		ret = devm_request_threaded_irq(
			&spi->dev, spi->irq, ads131e08_interrupt,
			ads131e08_data_ready_thread,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			dev_name(&spi->dev), indio_dev);
		if (ret)
			return dev_err_probe(&spi->dev, ret,
					     "request irq failed\n");
	} else {
		dev_err(&spi->dev, "data ready IRQ missing\n");
		return -ENODEV;
	}

	st->vref_reg = devm_regulator_get_optional(&spi->dev, "vref");
	if (!IS_ERR(st->vref_reg)) {
		ret = regulator_enable(st->vref_reg);
		if (ret) {
			dev_err(&spi->dev,
				"failed to enable external vref supply\n");
			return ret;
		}

		ret = devm_add_action_or_reset(&spi->dev,
					       ads131e08_regulator_disable, st);
		if (ret)
			return ret;
	} else {
		if (PTR_ERR(st->vref_reg) != -ENODEV)
			return PTR_ERR(st->vref_reg);

		st->vref_reg = NULL;
	}

	st->adc_clk = devm_clk_get_enabled(&spi->dev, "adc-clk");
	if (IS_ERR(st->adc_clk))
		return dev_err_probe(&spi->dev, PTR_ERR(st->adc_clk),
				     "failed to get the ADC clock\n");

	adc_clk_hz = clk_get_rate(st->adc_clk);
	if (!adc_clk_hz) {
		dev_err(&spi->dev, "ADC clock speed not set\n");
		return -EINVAL;
	}

	adc_clk_ns = NSEC_PER_SEC / adc_clk_hz;

	st->sdecode_delay_us = DIV_ROUND_UP(
		ADS131E08_WAIT_SDECODE_CYCLES * adc_clk_ns, NSEC_PER_USEC);

	st->reset_delay_us = DIV_ROUND_UP(
		ADS131E08_WAIT_RESET_CYCLES * adc_clk_ns, NSEC_PER_USEC);

	ret = ads131e08_initial_config(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "initial configuration failed\n");
		return ret;
	}

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ads131e08_of_match[] = {
	{
		.compatible = "ti,ads131e04",
		.data = &ads131e08_info_tbl[ads131e04],
	},
	{
		.compatible = "ti,ads131e06",
		.data = &ads131e08_info_tbl[ads131e06],
	},
	{
		.compatible = "ti,ads131e08",
		.data = &ads131e08_info_tbl[ads131e08],
	},
	{}
};
MODULE_DEVICE_TABLE(of, ads131e08_of_match);

static const struct spi_device_id ads131e08_ids[] = {
	{ "ads131e04", (kernel_ulong_t)&ads131e08_info_tbl[ads131e04] },
	{ "ads131e06", (kernel_ulong_t)&ads131e08_info_tbl[ads131e06] },
	{ "ads131e08", (kernel_ulong_t)&ads131e08_info_tbl[ads131e08] },
	{}
};
MODULE_DEVICE_TABLE(spi, ads131e08_ids);

static struct spi_driver ads131e08_driver = {
	.driver = {
		.name = "ads131e08",
		.of_match_table = ads131e08_of_match,
	},
	.probe = ads131e08_probe,
	.id_table = ads131e08_ids,
};
module_spi_driver(ads131e08_driver);

MODULE_AUTHOR("Tomislav Denis <tomislav.denis@avl.com>");
MODULE_AUTHOR("Viktor Karamanis <viktor.karamanis@outlook.com>");
MODULE_DESCRIPTION("Driver for ADS131E0x ADC family");
MODULE_LICENSE("GPL v2");
