#include <math.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define SAMPLE_RATE 11025
#define SAMPLE_BIT_WIDTH 16
#define NUM_CHANNELS 2
#define BYTES_PER_SAMPLE (SAMPLE_BIT_WIDTH / 8)

#define SAMPLES_PER_BLOCK 512
#define SAMPLES_PER_BUFFER (SAMPLES_PER_BLOCK * NUM_CHANNELS)
#define BUFFER_SIZE_BYTES (SAMPLES_PER_BUFFER * BYTES_PER_SAMPLE)

#define I2S_DEV DT_NODELABEL(i2s0)

K_MEM_SLAB_DEFINE(tx_mem_slab, BUFFER_SIZE_BYTES, 2, 4);

#define SINE_TABLE_SIZE 256
static const int16_t sine_lookup_table[SINE_TABLE_SIZE] = {
    0,      804,    1608,   2410,   3212,   4011,   4808,   5602,   6393,
    7179,   7962,   8739,   9512,   10278,  11039,  11794,  12542,  13283,
    14017,  14744,  15464,  16175,  16878,  17572,  18257,  18933,  19599,
    20256,  20902,  21538,  22164,  22778,  23381,  23972,  24551,  25117,
    25671,  26211,  26739,  27253,  27753,  28239,  28711,  29168,  29611,
    30039,  30451,  30849,  31231,  31598,  31950,  32286,  32606,  32911,
    33199,  33472,  33728,  33968,  34192,  34400,  34591,  34766,  34925,
    35067,  35192,  35301,  35393,  35468,  35526,  35568,  35593,  35601,
    35593,  35568,  35526,  35468,  35393,  35301,  35192,  35067,  34925,
    34766,  34591,  34400,  34192,  33968,  33728,  33472,  33199,  32911,
    32606,  32286,  31950,  31598,  31231,  30849,  30451,  30039,  29611,
    29168,  28711,  28239,  27753,  27253,  26739,  26211,  25671,  25117,
    24551,  23972,  23381,  22778,  22164,  21538,  20902,  20256,  19599,
    18933,  18257,  17572,  16878,  16175,  15464,  14744,  14017,  13283,
    12542,  11794,  11039,  10278,  9512,   8739,   7962,   7179,   6393,
    5602,   4808,   4011,   3212,   2410,   1608,   804,    0,      -804,
    -1608,  -2410,  -3212,  -4011,  -4808,  -5602,  -6393,  -7179,  -7962,
    -8739,  -9512,  -10278, -11039, -11794, -12542, -13283, -14017, -14744,
    -15464, -16175, -16878, -17572, -18257, -18933, -19599, -20256, -20902,
    -21538, -22164, -22778, -23381, -23972, -24551, -25117, -25671, -26211,
    -26739, -27253, -27753, -28239, -28711, -29168, -29611, -30039, -30451,
    -30849, -31231, -31598, -31950, -32286, -32606, -32911, -33199, -33472,
    -33728, -33968, -34192, -34400, -34591, -34766, -34925, -35067, -35192,
    -35301, -35393, -35468, -35526, -35568, -35593, -35601, -35593, -35568,
    -35526};

#define TONE_FREQUENCY 440.0f
static uint32_t phase_accumulator = 0;
static const uint32_t phase_step =
    (uint32_t)((TONE_FREQUENCY / SAMPLE_RATE) * SINE_TABLE_SIZE * 65536);

static int16_t tx_buffer[2][SAMPLES_PER_BUFFER];

void generate_sine_wave(int16_t *buffer, uint32_t num_samples_total) {
    for (uint32_t i = 0; i < num_samples_total; i += NUM_CHANNELS) {
        uint32_t table_index = phase_accumulator >> 16;
        uint32_t wrapped_index = table_index & (SINE_TABLE_SIZE - 1);

        int16_t sample = sine_lookup_table[wrapped_index];

        buffer[i] = sample;
        buffer[i + 1] = sample;

        phase_accumulator += phase_step;
    }
}

int main(void) {
    printk("Starting I2S test...\n");

    const struct device *i2s_tx_dev = DEVICE_DT_GET(I2S_DEV);
    struct i2s_config cfg;
    int ret;

    if (!device_is_ready(i2s_tx_dev)) {
        printk("I2S device %s not ready.\n", i2s_tx_dev->name);
        return -1;
    }

    printk("I2S device %s ready.\n", i2s_tx_dev->name);

    cfg.word_size = SAMPLE_BIT_WIDTH;
    cfg.channels = NUM_CHANNELS;
    cfg.format = I2S_FMT_DATA_FORMAT_I2S;
    cfg.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
    cfg.frame_clk_freq = SAMPLE_RATE;
    cfg.mem_slab = &tx_mem_slab;
    cfg.block_size = BUFFER_SIZE_BYTES;
    cfg.timeout = 1000;

    ret = i2s_configure(i2s_tx_dev, I2S_DIR_TX, &cfg);
    if (ret < 0) {
        printk("Failed to configure I2S TX: %d\n", ret);
        return -1;
    }

    printk("I2S TX configured.\n");

    for (int i = 0; i < 2; i++) {
        generate_sine_wave(tx_buffer[i], SAMPLES_PER_BUFFER);
        ret = i2s_write(i2s_tx_dev, tx_buffer[i], BUFFER_SIZE_BYTES);
        if (ret < 0) {
            printk("Initial i2s_write failed with error: %d\n", ret);
            return -1;
        }
    }

    ret = i2s_trigger(i2s_tx_dev, I2S_DIR_TX, I2S_TRIGGER_START);
    if (ret < 0) {
        printk("Failed to start I2S TX trigger: %d\n", ret);
        return -1;
    }

    printk("Playing 440 Hz tone at %u Hz sample rate...\n", SAMPLE_RATE);

    int current_buf = 0;
    while (1) {
        generate_sine_wave(tx_buffer[current_buf], SAMPLES_PER_BUFFER);

        ret = i2s_write(i2s_tx_dev, tx_buffer[current_buf], BUFFER_SIZE_BYTES);
        if (ret < 0) {
            printk("i2s_write failed with error: %d\n", ret);
            i2s_trigger(i2s_tx_dev, I2S_DIR_TX, I2S_TRIGGER_STOP);
            return -1;
        }

        current_buf = (current_buf + 1) % 2;
    }

    return 0;
}
