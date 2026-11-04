/* A stand-in for librtlsdr that plays an IQ recording through the real API.
 *
 * It compiles against the real rtl-sdr.h, so the compiler checks every prototype,
 * and lets the receiver's whole radio path run without a dongle: library loading,
 * configuration, the asynchronous callback, retuning and cancelling.
 *
 *   cc -shared -fPIC -o libfake_rtlsdr.so tools/fake_rtlsdr.c -lpthread
 *   LIBRTLSDR_PATH=./libfake_rtlsdr.so FAKE_RTLSDR_IQ=capture.cu8 lora-rtl-rx
 *
 * FAKE_RTLSDR_LOG names a file that receives one line per API call.
 */
#include <rtl-sdr.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

struct rtlsdr_dev {
	uint32_t rate;
	uint32_t freq;
	int ppm;
	int gain;
	atomic_int cancel;
	atomic_int streaming;
};

static const int gains[] = {0, 9, 14, 27, 37, 77, 87, 125, 144, 157, 166, 197, 207, 229, 254, 280, 297, 328,
			    338, 364, 372, 386, 402, 421, 434, 439, 445, 480, 496};

static void note(const char *format, ...)
{
	const char *path = getenv("FAKE_RTLSDR_LOG");
	FILE *f = path ? fopen(path, "a") : NULL;
	va_list args;
	if (!f)
		return;
	va_start(args, format);
	vfprintf(f, format, args);
	va_end(args);
	fputc('\n', f);
	fclose(f);
}

uint32_t rtlsdr_get_device_count(void) { return 1; }
const char *rtlsdr_get_device_name(uint32_t index) { return index == 0 ? "Fake RTL2832U" : ""; }

int rtlsdr_get_device_usb_strings(uint32_t index, char *manufact, char *product, char *serial)
{
	if (index != 0)
		return -1;
	strcpy(manufact, "Test");
	strcpy(product, "Fake");
	strcpy(serial, "00000001");
	return 0;
}

int rtlsdr_open(rtlsdr_dev_t **dev, uint32_t index)
{
	if (index != 0)
		return -1;
	*dev = calloc(1, sizeof(**dev));
	note("open %u", index);
	return 0;
}

int rtlsdr_close(rtlsdr_dev_t *dev)
{
	note("close streaming=%d", atomic_load(&dev->streaming));
	free(dev);
	return 0;
}

int rtlsdr_set_center_freq(rtlsdr_dev_t *dev, uint32_t freq) { dev->freq = freq; note("freq %u", freq); return 0; }
int rtlsdr_set_freq_correction(rtlsdr_dev_t *dev, int ppm) { dev->ppm = ppm; note("ppm %d", ppm); return 0; }
enum rtlsdr_tuner rtlsdr_get_tuner_type(rtlsdr_dev_t *dev) { (void)dev; return RTLSDR_TUNER_R820T; }

int rtlsdr_get_tuner_gains(rtlsdr_dev_t *dev, int *out)
{
	(void)dev;
	if (out)
		memcpy(out, gains, sizeof(gains));
	return sizeof(gains) / sizeof(gains[0]);
}

int rtlsdr_set_tuner_gain(rtlsdr_dev_t *dev, int gain) { dev->gain = gain; note("gain %d", gain); return 0; }
int rtlsdr_set_tuner_gain_mode(rtlsdr_dev_t *dev, int manual) { (void)dev; note("gain_mode %d", manual); return 0; }
int rtlsdr_set_sample_rate(rtlsdr_dev_t *dev, uint32_t rate) { dev->rate = rate; note("rate %u", rate); return 0; }
uint32_t rtlsdr_get_sample_rate(rtlsdr_dev_t *dev) { return dev->rate; }
int rtlsdr_set_agc_mode(rtlsdr_dev_t *dev, int on) { (void)dev; note("agc %d", on); return 0; }
int rtlsdr_set_bias_tee(rtlsdr_dev_t *dev, int on) { (void)dev; note("bias_tee %d", on); return 0; }
int rtlsdr_reset_buffer(rtlsdr_dev_t *dev) { (void)dev; note("reset_buffer"); return 0; }

int rtlsdr_read_async(rtlsdr_dev_t *dev, rtlsdr_read_async_cb_t cb, void *ctx, uint32_t buf_num, uint32_t buf_len)
{
	const char *path = getenv("FAKE_RTLSDR_IQ");
	FILE *f = path ? fopen(path, "rb") : NULL;
	unsigned char *buffer = malloc(buf_len);
	struct timespec wait;
	long ns;

	note("read_async buf_num=%u buf_len=%u", buf_num, buf_len);
	if (buf_len % 512 != 0)
		return -8;
	ns = (long)((double)buf_len / 2.0 / dev->rate * 1e9);
	wait.tv_sec = ns / 1000000000L;
	wait.tv_nsec = ns % 1000000000L;
	atomic_store(&dev->streaming, 1);
	while (!atomic_load(&dev->cancel)) {
		size_t got = f ? fread(buffer, 1, buf_len, f) : 0;
		/* After the recording ends, send a quiet carrier-free signal. */
		for (size_t i = got; i < buf_len; i++)
			buffer[i] = 127 + (rand() & 3);
		cb(buffer, buf_len, ctx);
		nanosleep(&wait, NULL);
	}
	atomic_store(&dev->streaming, 0);
	free(buffer);
	if (f)
		fclose(f);
	note("read_async returned");
	return 0;
}

int rtlsdr_cancel_async(rtlsdr_dev_t *dev) { note("cancel_async"); atomic_store(&dev->cancel, 1); return 0; }
