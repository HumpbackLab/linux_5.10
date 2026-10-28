// SPDX-License-Identifier: GPL-2.0-only
/* Silan SC7U22: 200/400 Hz, INT1 DRDY, hard-IRQ CLOCK_MONOTONIC timestamps.
 * Register/IIO layout based on ncerzzk's rk3506 SC7U22 IIO driver.
 * No synthetic timestamps and no polling fallback.
 */
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include <asm/unaligned.h>

#define SC_WHO_AM_I       0x01
#define SC_COM_CONF       0x04
#define SC_INT1_SEL1      0x05
#define SC_INT1_SEL2      0x06
#define SC_STATUS         0x0b
#define SC_ACC_XH         0x0c
#define SC_FIFO_CFG0      0x1c
#define SC_FIFO_CFG1      0x1d
#define SC_ACC_CONF       0x40
#define SC_ACC_RANGE      0x41
#define SC_GYR_CONF       0x42
#define SC_GYR_RANGE      0x43
#define SC_RESET          0x4a
#define SC_PWR_CTRL       0x7d
#define SC_SEG_SEL        0x7f
#define SC_ID             0x6a
#define SC_READ           BIT(7)
#define SC_BDU            BIT(6)
#define SC_AUTOINC        BIT(4)
#define SC_COM_DEFAULT    (SC_BDU | SC_AUTOINC) /* push-pull, active high */
#define SC_INT_GYR        BIT(2)
#define SC_READY          (BIT(0) | BIT(1))
#define SC_CONF_ERROR     (BIT(4) | BIT(5))
#define SC_PERFORMANCE    BIT(7)
#define SC_BWP_NORMAL     (2 << 4)
#define SC_ODR_200        9
#define SC_ODR_400        10
#define SC_16G            3
#define SC_2000DPS        0
#define SC_POWER_AGT      (BIT(1) | BIT(2) | BIT(3))
#define SC_FSIN_DIVIDER   10
#define SC_FSIN_HIGH_NS   100000LL

struct sc7u22 {
	struct spi_device *spi;
	struct mutex lock, control_lock;
	u32 rate_hz;
	s64 period_ns;
	struct gpio_desc *fsin_gpio;
	struct hrtimer fsin_timer;
	raw_spinlock_t fsin_lock;
	bool buffered, fsin_enabled, fsin_high;
	u32 fsin_divider, fsin_limit;
	u64 fsin_pulses, fsin_last_irq, fsin_rise_ns, fsin_last_irq_ns;
	u64 fsin_max_rise_delay_ns, fsin_max_high_ns, fsin_overruns;
	s64 irq_ns, last_ns;
	bool enabled, discard_first;
	atomic64_t irqs, pushed, spi_errors, not_ready, late_reads, buffer_drops;
	atomic64_t max_latency_ns, max_read_ns, gaps;
};
struct sc7u22_scan {
	__le16 axis[6];
	s64 timestamp __aligned(8);
};

static void sc_max(atomic64_t *v, s64 n)
{
	s64 old = atomic64_read(v);

	while (n > old) {
		s64 prev = atomic64_cmpxchg(v, old, n);

		if (prev == old)
			break;
		old = prev;
	}
}

/* SG2002 SPI needs a harmless transaction after idle before the real access.
 * Use full-duplex transfers, matching the verified TideSense SPI protocol.
 */
static int sc_xfer(struct sc7u22 *st, u8 reg, void *data, unsigned int len, bool read)
{
	u8 tx[16] = { SC_WHO_AM_I | SC_READ }, rx[16] = {};
	struct spi_transfer x = { .tx_buf = tx, .rx_buf = rx, .len = 2 };
	int ret;

	ret = spi_sync_transfer(st->spi, &x, 1);
	if (ret)
		goto error;
	memset(tx, 0, sizeof(tx));
	tx[0] = read ? reg | SC_READ : reg;
	if (!read)
		memcpy(tx + 1, data, len);
	x.len = len + 1;
	ret = spi_sync_transfer(st->spi, &x, 1);
	if (ret)
		goto error;
	if (read)
		memcpy(data, rx + 1, len);
	return 0;
error:
	atomic64_inc(&st->spi_errors);
	return ret;
}
static int sc_read(struct sc7u22 *st, u8 reg, void *buf, unsigned int len)
{
	return sc_xfer(st, reg, buf, len, true);
}
static int sc_write(struct sc7u22 *st, u8 reg, u8 value)
{
	int ret = sc_xfer(st, reg, &value, 1, false);

	usleep_range(1000, 1500);
	return ret;
}
static int sc_verify(struct sc7u22 *st, u8 reg, u8 value)
{
	u8 got;
	int ret = sc_read(st, reg, &got, 1);

	if (ret)
		return ret;
	if (got != value) {
		dev_err(&st->spi->dev, "reg %02x: got %02x expected %02x\n", reg, got, value);
		return -EIO;
	}
	return 0;
}
static void sc_stop(struct sc7u22 *st)
{
	sc_write(st, SC_INT1_SEL1, 0);
	sc_write(st, SC_INT1_SEL2, 0);
	sc_write(st, SC_PWR_CTRL, 0);
}
static int sc_start(struct sc7u22 *st)
{
	/* Normal bandwidth minimizes averaging at 200 Hz; the main data path
	 * has no unfiltered mode. ACC averages 2 and GYR averages 4 samples.
	 */
	u8 conf = SC_PERFORMANCE | SC_BWP_NORMAL |
		(st->rate_hz == 400 ? SC_ODR_400 : SC_ODR_200);
	const struct { u8 reg, value; } config[] = {
		{ SC_SEG_SEL, 0 }, { SC_COM_CONF, SC_COM_DEFAULT },
		{ SC_INT1_SEL1, 0 }, { SC_INT1_SEL2, 0 }, { SC_PWR_CTRL, 0 },
		{ SC_ACC_RANGE, SC_16G }, { SC_GYR_RANGE, SC_2000DPS },
		{ SC_ACC_CONF, conf }, { SC_GYR_CONF, conf },
		{ SC_FIFO_CFG0, 0 }, { SC_FIFO_CFG1, 0 },
		{ SC_PWR_CTRL, SC_POWER_AGT },
	};
	u8 raw[12], status;
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(config); ++i) {
		ret = sc_write(st, config[i].reg, config[i].value);
		if (ret)
			return ret;
		ret = sc_verify(st, config[i].reg, config[i].value);
		if (ret)
			return ret;
	}
	msleep(60);
	ret = sc_read(st, SC_STATUS, &status, 1);
	if (ret || (status & SC_CONF_ERROR))
		return ret ?: -EIO;
	/* Clear held startup data before routing the next gyro DRDY to INT1. */
	ret = sc_read(st, SC_ACC_XH, raw, sizeof(raw));
	if (ret)
		return ret;
	return 0;
}
/* GPIO is a non-sleeping MMIO controller. Never do SPI, PWM configuration,
 * logging or a busy wait on the rising-edge path. */
static enum hrtimer_restart sc_fsin_fall(struct hrtimer *timer)
{
	struct sc7u22 *st = container_of(timer, struct sc7u22, fsin_timer);
	unsigned long flags;
	u64 now;

	raw_spin_lock_irqsave(&st->fsin_lock, flags);
	gpiod_set_value(st->fsin_gpio, 0);
	now = ktime_get_ns();
	if (st->fsin_high)
		st->fsin_max_high_ns = max(st->fsin_max_high_ns, now - st->fsin_rise_ns);
	st->fsin_high = false;
	raw_spin_unlock_irqrestore(&st->fsin_lock, flags);
	return HRTIMER_NORESTART;
}

static void sc_fsin_irq(struct sc7u22 *st, u64 stamp)
{
	unsigned long flags;
	u64 rise;

	if (!st->fsin_gpio)
		return;
	raw_spin_lock_irqsave(&st->fsin_lock, flags);
	if (!st->fsin_enabled || (st->fsin_limit && st->fsin_pulses >= st->fsin_limit))
		goto out;
	if (++st->fsin_divider != SC_FSIN_DIVIDER)
		goto out;
	st->fsin_divider = 0;
	if (st->fsin_high) {
		/* Do not disguise a late falling edge as a new trigger. */
		++st->fsin_overruns;
		goto out;
	}
	gpiod_set_value(st->fsin_gpio, 1);
	rise = ktime_get_ns(); /* software bound after the GPIO MMIO write */
	st->fsin_high = true;
	st->fsin_rise_ns = rise;
	st->fsin_last_irq_ns = stamp;
	st->fsin_last_irq = atomic64_read(&st->irqs);
	++st->fsin_pulses;
	st->fsin_max_rise_delay_ns = max(st->fsin_max_rise_delay_ns, rise - stamp);
	hrtimer_start(&st->fsin_timer, ns_to_ktime(SC_FSIN_HIGH_NS), HRTIMER_MODE_REL_HARD);
out:
	raw_spin_unlock_irqrestore(&st->fsin_lock, flags);
}

/* control_lock serializes this with IIO enable/disable. */
static void sc_fsin_disarm(struct sc7u22 *st)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&st->fsin_lock, flags);
	st->fsin_enabled = false;
	raw_spin_unlock_irqrestore(&st->fsin_lock, flags);
	hrtimer_cancel(&st->fsin_timer);
	if (st->fsin_gpio)
		gpiod_set_value(st->fsin_gpio, 0);
	st->fsin_high = false;
}

static irqreturn_t sc_irq(int irq, void *private)
{
	/* Capture immediately, before scheduling or any SPI transaction. */
	s64 now = ktime_get_ns();
	struct sc7u22 *st = iio_priv(private);

	WRITE_ONCE(st->irq_ns, now);
	atomic64_inc(&st->irqs);
	sc_fsin_irq(st, now);
	return IRQ_WAKE_THREAD;
}
static irqreturn_t sc_thread(int irq, void *private)
{
	struct iio_dev *indio = private;
	struct sc7u22 *st = iio_priv(indio);
	struct sc7u22_scan scan = {};
	s64 stamp = READ_ONCE(st->irq_ns), start = ktime_get_ns(), end;
	u8 raw[13], status;
	int ret, i;

	sc_max(&st->max_latency_ns, start - stamp);
	mutex_lock(&st->lock);
	if (!st->enabled)
		goto out;
	/* STATUS and the six axes are consecutive. One burst avoids two extra
	 * SPI pump round trips per sample, particularly important at 400 Hz. */
	ret = sc_read(st, SC_STATUS, raw, sizeof(raw));
	end = ktime_get_ns();
	sc_max(&st->max_read_ns, end - stamp);
	if (ret)
		goto out;
	status = raw[0];
	if (st->discard_first) {
		st->discard_first = false;
		goto out; /* The first routed DRDY may describe data held during startup. */
	}
	if ((status & SC_READY) != SC_READY || (status & SC_CONF_ERROR)) {
		atomic64_inc(&st->not_ready);
		goto out;
	}
	/* BDU protects words, not an unlimited history. Never label a possibly
	 * overwritten sample with an old IRQ timestamp after a missed deadline.
	 */
	if (end - stamp >= st->period_ns) {
		atomic64_inc(&st->late_reads);
		goto out;
	}
	if (st->last_ns && stamp - st->last_ns > st->period_ns * 3 / 2)
		atomic64_inc(&st->gaps);
	st->last_ns = stamp;
	if (!st->buffered)
		goto out; /* Camera-only mode still reads/acknowledges every DRDY. */
	for (i = 0; i < 6; ++i)
		scan.axis[i] = cpu_to_le16(get_unaligned_be16(raw + 1 + i * 2));
	ret = iio_push_to_buffers_with_timestamp(indio, &scan, stamp);
	if (ret)
		atomic64_inc(&st->buffer_drops);
	else
		atomic64_inc(&st->pushed);
out:
	mutex_unlock(&st->lock);
	return IRQ_HANDLED;
}
static int sc_run(struct sc7u22 *st)
{
	bool armed = false;
	int ret;

	if (st->enabled)
		return 0;
	mutex_lock(&st->lock);
	st->last_ns = 0;
	st->discard_first = true;
	ret = sc_start(st);
	if (!ret) {
		st->enabled = true;
		/* Arm the GPIO before routing DRDY. Otherwise a held-high DRDY
		 * can lose its only edge while the GPIO IRQ is still masked.
		 */
		enable_irq(st->spi->irq);
		armed = true;
		ret = sc_write(st, SC_INT1_SEL1, SC_INT_GYR);
		if (!ret)
			ret = sc_verify(st, SC_INT1_SEL1, SC_INT_GYR);
	}
	if (ret) {
		if (armed)
			disable_irq_nosync(st->spi->irq);
		st->enabled = false;
		sc_stop(st);
	}
	mutex_unlock(&st->lock);
	if (ret && armed)
		synchronize_irq(st->spi->irq);
	return ret;
}
static void sc_halt(struct sc7u22 *st)
{
	if (!st->enabled)
		return;
	disable_irq(st->spi->irq);
	mutex_lock(&st->lock);
	st->enabled = false;
	sc_stop(st);
	mutex_unlock(&st->lock);
}
static int sc_enable(struct iio_dev *indio)
{
	struct sc7u22 *st = iio_priv(indio);
	int ret;

	if (iio_device_get_clock(indio) != CLOCK_MONOTONIC)
		return -EINVAL;
	mutex_lock(&st->control_lock);
	ret = sc_run(st);
	if (!ret) {
		mutex_lock(&st->lock);
		st->buffered = true;
		mutex_unlock(&st->lock);
	}
	mutex_unlock(&st->control_lock);
	return ret;
}
static int sc_disable(struct iio_dev *indio)
{
	struct sc7u22 *st = iio_priv(indio);

	mutex_lock(&st->control_lock);
	mutex_lock(&st->lock);
	st->buffered = false;
	mutex_unlock(&st->lock);
	if (!st->fsin_enabled)
		sc_halt(st);
	mutex_unlock(&st->control_lock);
	return 0;
}
static const struct iio_buffer_setup_ops sc_buffer_ops = {
	.postenable = sc_enable, .predisable = sc_disable,
};
static int sc_read_raw(struct iio_dev *indio, const struct iio_chan_spec *ch,
		       int *val, int *val2, long mask)
{
	if (mask == IIO_CHAN_INFO_SAMP_FREQ) {
		struct sc7u22 *st = iio_priv(indio);

		*val = READ_ONCE(st->rate_hz);
		return IIO_VAL_INT;
	}
	if (mask == IIO_CHAN_INFO_SCALE) {
		*val = 0;
		*val2 = ch->type == IIO_ACCEL ? 4788403 : 1065264;
		return IIO_VAL_INT_PLUS_NANO;
	}
	return -EINVAL;
}
static int sc_write_raw(struct iio_dev *indio, const struct iio_chan_spec *ch,
			int val, int val2, long mask)
{
	struct sc7u22 *st = iio_priv(indio);
	int ret;

	if (mask != IIO_CHAN_INFO_SAMP_FREQ || val2 || (val != 200 && val != 400))
		return -EINVAL;
	ret = iio_device_claim_direct_mode(indio);
	if (ret)
		return ret;
	mutex_lock(&st->control_lock);
	if (st->enabled || st->fsin_enabled || st->buffered) {
		ret = -EBUSY;
	} else {
		WRITE_ONCE(st->rate_hz, val);
		st->period_ns = NSEC_PER_SEC / val;
		ret = 0;
	}
	mutex_unlock(&st->control_lock);
	iio_device_release_direct_mode(indio);
	return ret;
}
static ssize_t diagnostics_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc7u22 *st = iio_priv(dev_to_iio_dev(dev));

	return scnprintf(buf, PAGE_SIZE,
		"irq_count=%lld\nframes_pushed=%lld\nspi_errors=%lld\nnot_ready=%lld\n"
		"late_reads=%lld\nbuffer_drops=%lld\ngaps=%lld\nmax_irq_thread_latency_ns=%lld\nmax_irq_to_read_ns=%lld\n",
		atomic64_read(&st->irqs), atomic64_read(&st->pushed),
		atomic64_read(&st->spi_errors), atomic64_read(&st->not_ready),
		atomic64_read(&st->late_reads), atomic64_read(&st->buffer_drops),
		atomic64_read(&st->gaps), atomic64_read(&st->max_latency_ns), atomic64_read(&st->max_read_ns));
}
static ssize_t fsin_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc7u22 *st = iio_priv(dev_to_iio_dev(dev));

	return scnprintf(buf, PAGE_SIZE, "%u\n", READ_ONCE(st->fsin_enabled));
}
static ssize_t fsin_enable_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sc7u22 *st = iio_priv(dev_to_iio_dev(dev));
	unsigned long flags;
	bool enable;
	int ret = kstrtobool(buf, &enable);

	if (ret)
		return ret;
	if (!st->fsin_gpio)
		return -ENODEV;
	mutex_lock(&st->control_lock);
	if (enable && !st->fsin_enabled) {
		/* Start/read the IMU first; do not trigger on held startup data. */
		ret = sc_run(st);
		if (!ret) {
			raw_spin_lock_irqsave(&st->fsin_lock, flags);
			st->fsin_divider = 0;
			st->fsin_pulses = 0;
			st->fsin_last_irq = 0;
			st->fsin_last_irq_ns = 0;
			st->fsin_rise_ns = 0;
			st->fsin_max_rise_delay_ns = 0;
			st->fsin_max_high_ns = 0;
			st->fsin_overruns = 0;
			st->fsin_enabled = true;
			raw_spin_unlock_irqrestore(&st->fsin_lock, flags);
		}
	} else if (!enable) {
		sc_fsin_disarm(st);
		if (!st->buffered)
			sc_halt(st);
	}
	mutex_unlock(&st->control_lock);
	return ret ?: count;
}
static ssize_t fsin_pulse_limit_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc7u22 *st = iio_priv(dev_to_iio_dev(dev));

	return scnprintf(buf, PAGE_SIZE, "%u\n", READ_ONCE(st->fsin_limit));
}
static ssize_t fsin_pulse_limit_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sc7u22 *st = iio_priv(dev_to_iio_dev(dev));
	u32 n;
	int ret = kstrtou32(buf, 10, &n);

	if (ret || n > 10000000)
		return ret ?: -EINVAL;
	mutex_lock(&st->control_lock);
	if (st->fsin_enabled)
		ret = -EBUSY;
	else
		st->fsin_limit = n;
	mutex_unlock(&st->control_lock);
	return ret ?: count;
}
static ssize_t fsin_diagnostics_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc7u22 *st = iio_priv(dev_to_iio_dev(dev));
	unsigned long flags;
	u32 limit;
	bool enabled;
	u64 pulses, irq, stamp, rise, delay, high, overruns;

	/* Only snapshot under the IRQ lock; formatting must not delay FSIN. */
	raw_spin_lock_irqsave(&st->fsin_lock, flags);
	enabled = st->fsin_enabled;
	limit = st->fsin_limit;
	pulses = st->fsin_pulses;
	irq = st->fsin_last_irq;
	stamp = st->fsin_last_irq_ns;
	rise = st->fsin_rise_ns;
	delay = st->fsin_max_rise_delay_ns;
	high = st->fsin_max_high_ns;
	overruns = st->fsin_overruns;
	raw_spin_unlock_irqrestore(&st->fsin_lock, flags);
	return scnprintf(buf, PAGE_SIZE,
	 "available=%u\nenable=%u\ndivider=10\npulse_width_ns=100000\npulse_limit=%u\n"
	 "pulses=%llu\nlast_irq_count=%llu\nlast_irq_ns=%llu\nlast_gpio_write_ns=%llu\n"
	 "max_irq_to_gpio_write_ns=%llu\nmax_gpio_high_ns=%llu\noverruns=%llu\n",
	 !!st->fsin_gpio, enabled, limit, pulses, irq, stamp, rise, delay, high, overruns);
}
static IIO_DEVICE_ATTR_RW(fsin_enable, 0);
static IIO_DEVICE_ATTR_RW(fsin_pulse_limit, 0);
static IIO_DEVICE_ATTR_RO(fsin_diagnostics, 0);
static IIO_DEVICE_ATTR_RO(diagnostics, 0);
static IIO_CONST_ATTR(in_accel_sampling_frequency_available, "200 400");
static IIO_CONST_ATTR(in_anglvel_sampling_frequency_available, "200 400");
static struct attribute *sc_attrs[] = {
	&iio_const_attr_in_accel_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_anglvel_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_diagnostics.dev_attr.attr,
	&iio_dev_attr_fsin_enable.dev_attr.attr,
	&iio_dev_attr_fsin_pulse_limit.dev_attr.attr,
	&iio_dev_attr_fsin_diagnostics.dev_attr.attr, NULL
};
static const struct attribute_group sc_group = { .attrs = sc_attrs };
static const struct iio_info sc_info = {
	.read_raw = sc_read_raw, .write_raw = sc_write_raw, .attrs = &sc_group,
};
#define SC_CHAN(type_, axis_, index_) { .type = type_, .modified = 1, \
	.channel2 = IIO_MOD_##axis_, .scan_index = index_, \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_type = { .sign = 's', .realbits = 16, .storagebits = 16, .endianness = IIO_LE } }
static const struct iio_chan_spec sc_channels[] = {
	SC_CHAN(IIO_ACCEL, X, 0), SC_CHAN(IIO_ACCEL, Y, 1), SC_CHAN(IIO_ACCEL, Z, 2),
	SC_CHAN(IIO_ANGL_VEL, X, 3), SC_CHAN(IIO_ANGL_VEL, Y, 4), SC_CHAN(IIO_ANGL_VEL, Z, 5),
	IIO_CHAN_SOFT_TIMESTAMP(6),
};
static const unsigned long sc_masks[] = { GENMASK(5, 0), 0 };
static void sc_shutdown(void *private)
{
	struct sc7u22 *st = private;

	sc_fsin_disarm(st);
	sc_halt(st);
}
static int sc_probe(struct spi_device *spi)
{
	struct iio_dev *indio;
	struct sc7u22 *st;
	struct iio_buffer *buffer;
	int ret;

	if (spi->irq <= 0)
		return dev_err_probe(&spi->dev, -EINVAL, "INT1 IRQ required\n");
	indio = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio)
		return -ENOMEM;
	st = iio_priv(indio);
	st->rate_hz = 200;
	st->period_ns = NSEC_PER_SEC / 200;
	st->spi = spi;
	mutex_init(&st->lock);
	mutex_init(&st->control_lock);
	raw_spin_lock_init(&st->fsin_lock);
	hrtimer_init(&st->fsin_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_HARD);
	st->fsin_timer.function = sc_fsin_fall;
	st->fsin_gpio = devm_gpiod_get_optional(&spi->dev, "fsin", GPIOD_OUT_LOW);
	if (IS_ERR(st->fsin_gpio))
		return PTR_ERR(st->fsin_gpio);
	if (st->fsin_gpio) {
		u32 mux[3], v;
		void __iomem *reg;

		if (gpiod_cansleep(st->fsin_gpio))
			return dev_err_probe(&spi->dev, -EINVAL, "FSIN requires non-sleeping GPIO\n");
		/* This SDK lacks a pinctrl mapping for the PWR pad. DT supplies
		 * register/mask/value; never switch to independently clocked PWM. */
		ret = of_property_read_u32_array(spi->dev.of_node, "silan,fsin-pinmux", mux, 3);
		if (ret)
			return ret;
		reg = devm_ioremap(&spi->dev, mux[0], 4);
		if (!reg)
			return -ENOMEM;
		v = readl(reg);
		writel((v & ~mux[1]) | (mux[2] & mux[1]), reg);
	}
	spi->mode = SPI_MODE_3;
	spi->max_speed_hz = min(spi->max_speed_hz, 1000000U);
	spi->bits_per_word = 8;
	spi->rt = true; /* Keep the SPI pump schedulable alongside the IRQ thread. */
	ret = spi_setup(spi);
	if (ret)
		return ret;
	ret = sc_write(st, SC_SEG_SEL, 0);
	if (ret)
		return ret;
	ret = sc_verify(st, SC_WHO_AM_I, SC_ID);
	if (ret)
		return ret;
	ret = sc_write(st, SC_COM_CONF, SC_COM_DEFAULT);
	if (ret)
		return ret;
	ret = sc_write(st, SC_RESET, 0xa5);
	if (ret)
		return ret;
	ret = sc_write(st, SC_RESET, 0xa5);
	if (ret)
		return ret;
	msleep(200);
	sc_stop(st);
	indio->name = "sc7u22";
	indio->info = &sc_info;
	indio->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio->channels = sc_channels;
	indio->num_channels = ARRAY_SIZE(sc_channels);
	indio->available_scan_masks = sc_masks;
	indio->setup_ops = &sc_buffer_ops;
	ret = iio_device_set_clock(indio, CLOCK_MONOTONIC);
	if (ret)
		return ret;
	buffer = devm_iio_kfifo_allocate(&spi->dev);
	if (!buffer)
		return -ENOMEM;
	iio_device_attach_buffer(indio, buffer);
	irq_set_status_flags(spi->irq, IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(&spi->dev, spi->irq, sc_irq, sc_thread,
				IRQF_ONESHOT, "sc7u22", indio);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&spi->dev, sc_shutdown, st);
	if (ret)
		return ret;
	ret = devm_iio_device_register(&spi->dev, indio);
	if (!ret)
		dev_info(&spi->dev, "200 Hz INT1 IRQ %d, hard-IRQ monotonic timestamps\n", spi->irq);
	return ret;
}
static const struct of_device_id sc_of[] = { { .compatible = "silan,sc7u22" }, {} };
MODULE_DEVICE_TABLE(of, sc_of);
static const struct spi_device_id sc_ids[] = { { "sc7u22", 0 }, {} };
MODULE_DEVICE_TABLE(spi, sc_ids);
static struct spi_driver sc_driver = {
	.driver = { .name = "sc7u22", .of_match_table = sc_of },
	.probe = sc_probe, .id_table = sc_ids,
};
module_spi_driver(sc_driver);
MODULE_DESCRIPTION("SC7U22 200/400 Hz INT1 timestamped IIO IMU");
MODULE_LICENSE("GPL");
