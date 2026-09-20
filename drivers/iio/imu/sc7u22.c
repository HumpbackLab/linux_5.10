// SPDX-License-Identifier: GPL-2.0-only
/* Silan SC7U22: 200 Hz, INT1 DRDY, hard-IRQ CLOCK_MONOTONIC timestamps.
 * Register/IIO layout based on ncerzzk's rk3506 SC7U22 IIO driver.
 * No synthetic timestamps and no polling fallback.
 */
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
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
#define SC_ODR_200        9
#define SC_CONF_200       (SC_PERFORMANCE | SC_ODR_200) /* BWP=0: OSR4 */
#define SC_16G            3
#define SC_2000DPS        0
#define SC_POWER_AGT      (BIT(1) | BIT(2) | BIT(3))
#define SC_PERIOD_NS      5000000LL

struct sc7u22 {
	struct spi_device *spi;
	struct mutex lock;
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
	static const struct { u8 reg, value; } config[] = {
		{ SC_SEG_SEL, 0 }, { SC_COM_CONF, SC_COM_DEFAULT },
		{ SC_INT1_SEL1, 0 }, { SC_INT1_SEL2, 0 }, { SC_PWR_CTRL, 0 },
		{ SC_ACC_RANGE, SC_16G }, { SC_GYR_RANGE, SC_2000DPS },
		{ SC_ACC_CONF, SC_CONF_200 }, { SC_GYR_CONF, SC_CONF_200 },
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
static irqreturn_t sc_irq(int irq, void *private)
{
	/* Capture immediately, before scheduling or any SPI transaction. */
	s64 now = ktime_get_ns();
	struct sc7u22 *st = iio_priv(private);

	WRITE_ONCE(st->irq_ns, now);
	atomic64_inc(&st->irqs);
	return IRQ_WAKE_THREAD;
}
static irqreturn_t sc_thread(int irq, void *private)
{
	struct iio_dev *indio = private;
	struct sc7u22 *st = iio_priv(indio);
	struct sc7u22_scan scan = {};
	s64 stamp = READ_ONCE(st->irq_ns), start = ktime_get_ns(), end;
	u8 raw[12], status;
	int ret, i;

	sc_max(&st->max_latency_ns, start - stamp);
	mutex_lock(&st->lock);
	if (!st->enabled)
		goto out;
	ret = sc_read(st, SC_STATUS, &status, 1);
	if (ret)
		goto out;
	ret = sc_read(st, SC_ACC_XH, raw, sizeof(raw));
	end = ktime_get_ns();
	sc_max(&st->max_read_ns, end - stamp);
	if (ret)
		goto out;
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
	if (end - stamp >= SC_PERIOD_NS) {
		atomic64_inc(&st->late_reads);
		goto out;
	}
	if (st->last_ns && stamp - st->last_ns > SC_PERIOD_NS * 3 / 2)
		atomic64_inc(&st->gaps);
	st->last_ns = stamp;
	for (i = 0; i < 6; ++i)
		scan.axis[i] = cpu_to_le16(get_unaligned_be16(raw + i * 2));
	ret = iio_push_to_buffers_with_timestamp(indio, &scan, stamp);
	if (ret)
		atomic64_inc(&st->buffer_drops);
	else
		atomic64_inc(&st->pushed);
out:
	mutex_unlock(&st->lock);
	return IRQ_HANDLED;
}
static int sc_enable(struct iio_dev *indio)
{
	struct sc7u22 *st = iio_priv(indio);
	bool armed = false;
	int ret;

	/* This driver records CLOCK_MONOTONIC in the hard IRQ unconditionally. */
	if (iio_device_get_clock(indio) != CLOCK_MONOTONIC)
		return -EINVAL;
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
static int sc_disable(struct iio_dev *indio)
{
	struct sc7u22 *st = iio_priv(indio);

	disable_irq(st->spi->irq);
	mutex_lock(&st->lock);
	st->enabled = false;
	sc_stop(st);
	mutex_unlock(&st->lock);
	return 0;
}
static const struct iio_buffer_setup_ops sc_buffer_ops = {
	.postenable = sc_enable, .predisable = sc_disable,
};
static int sc_read_raw(struct iio_dev *indio, const struct iio_chan_spec *ch,
		       int *val, int *val2, long mask)
{
	if (mask == IIO_CHAN_INFO_SAMP_FREQ) {
		*val = 200;
		return IIO_VAL_INT;
	}
	if (mask == IIO_CHAN_INFO_SCALE) {
		*val = 0;
		*val2 = ch->type == IIO_ACCEL ? 4788403 : 1065264;
		return IIO_VAL_INT_PLUS_NANO;
	}
	return -EINVAL;
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
static IIO_DEVICE_ATTR_RO(diagnostics, 0);
static struct attribute *sc_attrs[] = { &iio_dev_attr_diagnostics.dev_attr.attr, NULL };
static const struct attribute_group sc_group = { .attrs = sc_attrs };
static const struct iio_info sc_info = { .read_raw = sc_read_raw, .attrs = &sc_group };
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

	sc_stop(st);
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
	st->spi = spi;
	mutex_init(&st->lock);
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
	ret = devm_add_action_or_reset(&spi->dev, sc_shutdown, st);
	if (ret)
		return ret;
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
MODULE_DESCRIPTION("SC7U22 200 Hz INT1 timestamped IIO IMU");
MODULE_LICENSE("GPL");
