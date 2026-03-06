#include <Arduino.h>


// Pin and I2C address definitions
#define SIG_PIN      8    // Sampling Signal(回路図参照)
#define MCP3425_SCL  7    // MCP3425
#define MCP3425_SDA  9
#define MCP3425_ADDR 0x68
#define MODE_SW      11   // モード(OSC/FFT)切り替えスイッチ(回路図参照)
#define ALT_SW       1    // FFTモードでのLIN/dB表示切り替え
#define OLED_CLK     10   // SSD1306
#define OLED_MOSI    6
#define OLED_RESET   5
#define OLED_DC      4
#define OLED_CS      3


// Built-in RGB LED (debug用なので消しても動作には問題なし、Tenstar ROBOT製のESP32S3ボードに搭載されているRGB LEDが前提)
#include <FastLED.h>
#define NUM_LEDS 1
#define LED_DATA_PIN 48
CRGB leds[NUM_LEDS];


// SSD1306 OLED Display (SPI)
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
inline void setContrast(uint8_t contrast) {
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(contrast);
}

// For Drawing
#define TOP_MARGIN    8
#define BOTTOM_MARGIN 8
#define DRAW_Y_MIN  (TOP_MARGIN + 1)                    // 9
#define DRAW_Y_MAX  (SCREEN_HEIGHT - BOTTOM_MARGIN - 1) // 55
#define DRAW_HEIGHT (DRAW_Y_MAX - DRAW_Y_MIN + 1)


// POT Read (Using MCP3425)
#include <Wire.h>
void mcp3425_init() {
    Wire.begin(MCP3425_SDA, MCP3425_SCL);
    Wire.beginTransmission(MCP3425_ADDR);
    Wire.write(0b10010000);
    //           ||||||||
    //           ||||||++---- 00 : PGA x1
    //           ||||++------ 00 : 12bit, 240SPS
    //           |||+--------  1 : Continuous Conversion Mode
    //           |++--------- 00 : Channel 0 (No effect for MCP3425)
    //           +-----------  1 : Ready Bit (No effect for MCP3425 on Continuous Mode)
    Wire.endTransmission();
}
uint32_t read_pot() {
Wire.requestFrom(MCP3425_ADDR, 3);

if (Wire.available() < 2) return 0;

uint8_t msb = Wire.read();
uint8_t lsb = Wire.read();

while (Wire.available() != 0) {
    Wire.read(); //configデータ捨て
}

// 上位12bitが有効データ
uint32_t raw = ((msb << 8) | lsb);

// 12bit符号つき整数を32bit符号なし整数に変換(MCP3425は差動入力のため負の値もありえる)
if (raw & 0x00008000) raw = 0;

return raw;
}


// ESP32S3 ADC Definitions
extern "C" {
#include "soc/adc_channel.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_continuous.h"
}
#define ADC_ATTEN     ADC_ATTEN_DB_12
#define ADC_BITWIDTH  ADC_BITWIDTH_12
#define SAMPLE_RATE   64000           // サンプリング周波数
#define ADC_MAX       3745            // ADC結果(0~4095)のうちの表示範囲の上限
#define ADC_MIN       200             //             同                下限
#define ADC_RING_SIZE 65535           // リングバッファのサイズ

// 下2つによりADCユニットとチャネルはsetup()内でSIG_PINのピン番号によって自動的に割り振られます。
// なお、Continuousモードで使用する場合はADCユニット1のほうがよいらしいです。
// また、ADCユニットごとにチャンネルは10割り振られていますが、モード(Continuous/Oneshot)は一つしか選択できません。
// MCP3425の代わりに、余ったADCユニット2をOneshotモードで用いて可変抵抗の値を見ることも可能です。
adc_unit_t    SIG_PIN_ADC_UNIT;
adc_channel_t SIG_PIN_ADC_CH;

// ADC Continuous Handler
adc_continuous_handle_t adc_c_handle = nullptr;

// ADC Continuous Ring Buffer
uint16_t adc_ring[ADC_RING_SIZE + 1] = {0x00FF};
volatile uint16_t adc_wp = 0;

// ADC Continuous Setup
void setup_adc_continuous(adc_unit_t adc_unit, adc_channel_t adc_ch) {
    adc_continuous_handle_cfg_t adc_c_handle_cfg = {
        .max_store_buf_size = 2048,
        .conv_frame_size = 1024,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_c_handle_cfg, &adc_c_handle));
    static adc_digi_pattern_config_t adc_c_pattern[1] = {
        {
        .atten = ADC_ATTEN,
        .channel = adc_unit,
        .unit = adc_ch,
        .bit_width = ADC_BITWIDTH,
        }
    };
    adc_continuous_config_t adc_c_config = {};
    adc_c_config.pattern_num = 1;
    adc_c_config.adc_pattern = adc_c_pattern;
    adc_c_config.sample_freq_hz = SAMPLE_RATE;
    adc_c_config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    adc_c_config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    ESP_ERROR_CHECK(adc_continuous_config(adc_c_handle, &adc_c_config));
}

// ADC Continuous Task
void adc_task(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(adc_continuous_start(adc_c_handle));

    static uint8_t raw_buf[1024];
    uint32_t out_len = 0;

    while (true) {
        esp_err_t ret = adc_continuous_read(adc_c_handle, raw_buf, sizeof(raw_buf), &out_len, 100);
        if (ret == ESP_OK && out_len > 0) {
            uint32_t samples = out_len / sizeof(adc_digi_output_data_t);
            for (uint32_t i = 0; i < samples; i++) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t *)&raw_buf[i * sizeof(adc_digi_output_data_t)];
                adc_ring[adc_wp++ & (ADC_RING_SIZE - 1)] = p->type2.data;
            }
        }
        vTaskDelay(1);
    }
}


// ArduinoFFT
#include <arduinoFFT.h>
#define FFT_SIZE (SCREEN_WIDTH * 32)   // SCREEN_WIDTH(128)の2のべき乗倍
float vReal[FFT_SIZE];
float vImag[FFT_SIZE];
ArduinoFFT<float> fft = ArduinoFFT<float>(vReal, vImag, FFT_SIZE, SAMPLE_RATE);

// For FFT Visualization (LOG表示)
// Definitions
#define BAR_STEP 2 // バー(幅1px)の表示間隔
#define NUM_BARS ((int)(SCREEN_WIDTH / BAR_STEP)) // バーの本数
#define DB_MIN   -18   //
#define DB_MAX   -5
#define F_MIN    20
#define F_MAX    16000

typedef struct {
    uint16_t i0;
    uint16_t i1;
} BandMap;
BandMap bandMap[NUM_BARS];

// Pseudo Log Calculation (数学的に正しいかは知らないがlogっぽくはなる)
static inline uint32_t fast_log2_q4(uint32_t val) {
    if (val == 0) return 0;
    int e = 31 - __builtin_clz(val);
    uint32_t m = val >> (e-4);
    return (uint32_t)((e << 4) + (m & 0x0F));
}

// Generate Band Map
void generateBandMap() {
    for (int n = 0; n < NUM_BARS; n++) {
        double f0 = F_MIN * pow(F_MAX / F_MIN,
                                (double)n / NUM_BARS);
        double f1 = F_MIN * pow(F_MAX / F_MIN,
                                (double)(n + 1) / NUM_BARS);
        int i0 = ceil(f0 * FFT_SIZE / SAMPLE_RATE);
        int i1 = floor(f1 * FFT_SIZE / SAMPLE_RATE);
        if (i0 < 1) i0 = 1;
        if (i1 >= FFT_SIZE / 2) i1 = FFT_SIZE / 2 - 1;
        if (i1 < i0) i1 = i0;
        bandMap[n].i0 = i0;
        bandMap[n].i1 = i1;
    }
}


// State
enum Mode {
    MODE_OSC,
    MODE_FFT
};
Mode mode = MODE_OSC;


void setup() {
    delay(100);

    // Serial Setup
    Serial.begin(115200);
    Serial.println("Starting ...");
    pinMode(MODE_SW, INPUT_PULLUP);
    pinMode(ALT_SW, INPUT_PULLUP);

    // LED Setup
    FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS);
    FastLED.setBrightness(255);
    FastLED.setMaxPowerInVoltsAndMilliamps(5, 100);
    leds[0] = CRGB::Yellow; FastLED.show();

    // Generate Band Map
    generateBandMap();

    // ADC Setup
    mcp3425_init();
    adc_continuous_io_to_channel(SIG_PIN, &SIG_PIN_ADC_UNIT, &SIG_PIN_ADC_CH);
    setup_adc_continuous(SIG_PIN_ADC_UNIT, SIG_PIN_ADC_CH);
    xTaskCreateUniversal(
        adc_task,
        "adc_task",
        4096,
        nullptr,
        2,
        nullptr,
        1
    );

    // Display Setup
    if(!display.begin(SSD1306_SWITCHCAPVCC)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;) {
        leds[0] = CRGB::Purple; FastLED.show();
        delay(500);
        leds[0] = CRGB::Black; FastLED.show();
        delay(500);
        }
    }

    setContrast(0xFF);
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.println("OSC/FFT Wave Monitor");
    display.println("         by ESP32-S3");
    display.println("");
    display.print("Starting . . . ");
    display.display();
    delay(2000);

    leds[0] = CRGB::DarkRed; FastLED.show();
}

void loop() {
    display.clearDisplay();

    mode = (digitalRead(MODE_SW) == HIGH) ? MODE_OSC : MODE_FFT;

    if (mode == MODE_OSC) osc();
    else fft();

    display.display();
}

void osc() {
    // 表示用配列
    static uint16_t samples[SCREEN_WIDTH] = {0x00FF};

    // サンプル取得
    // 表示幅決定
    uint32_t pot = read_pot(); // 抵抗値(12bit signedの正の部分 0~2048)
    float t_ms = constrain(map(pot,0, 1940/*←お好みで調整*/, 10, 10000) / 10.0,
                           1.0,
                           1000.0); // 可変抵抗の値を1ms〜1000msへ変換

    // 表示範囲に入っているサンプル数
    int32_t samples_num = (int)(SAMPLE_RATE * t_ms / 1000.0);
    if (samples_num > ADC_RING_SIZE) samples_num = ADC_RING_SIZE;

    int32_t rp = adc_wp - samples_num;
    while (rp < 0) rp += ADC_RING_SIZE;
    if (rp >= ADC_RING_SIZE) return;

    uint32_t phase = 0;
    uint32_t phase_step = ((uint32_t)total_samples << 16) / SCREEN_WIDTH;

    // 描画
    if (t_ms < 100.0) {
        // 通常のライン描画
        for (int i = 0; i < SCREEN_WIDTH; i++) {
            uint32_t idx = rp + (phase >> 16);
            samples[i] = adc_ring[idx & (ADC_RING_SIZE - 1)];
            phase += phase_step;
        }
        int prevY = -1;
        for (int x = 0; x < SCREEN_WIDTH; x++) {
            int y = map(samples[x], ADC_MIN, ADC_MAX, DRAW_Y_MAX, DRAW_Y_MIN);

            if (prevY >= 0) {
                display.drawLine(x - 1, prevY, x, y, SSD1306_WHITE);
            }
            prevY = y;
        }
    } else {
        // Vmin-Vmax縦線(エイリアシング対策)
        for (int i = 0; i < SCREEN_WIDTH; i++) {
            uint32_t start = phase >> 16;
            uint32_t end   = (phase + phase_step) >> 16;
            if (end <= start) end = start + 1;

            uint16_t vmin = 0x00FF, vmax = 0x00FF;

            for (uint32_t j = start; j < end; j++) {
                uint16_t v = adc_ring[(rp + j) & (ADC_RING_SIZE - 1)];
                if (v < vmin) vmin = v;
                if (v > vmax) vmax = v;
            }

            phase += phase_step;

            int y1 = map(vmin, ADC_MIN, ADC_MAX, DRAW_Y_MAX, DRAW_Y_MIN);
            int y2 = map(vmax, ADC_MIN, ADC_MAX, DRAW_Y_MAX, DRAW_Y_MIN);
            display.drawLine(i, y1, i, y2, SSD1306_WHITE);
        }
    }

    display.setCursor(0,0);
    display.printf("OSC        Fs=%dHz", SAMPLE_RATE);
    display.setCursor(0,57);
    display.printf("%6.1fms", t_ms);
}

void fft() {
    // Fetch Samples from Ring Buffer
    bool isdBScale = (digitalRead(ALT_SW) == LOW);
    uint32_t rp = adc_wp - FFT_SIZE;
    for (int i = 0; i < FFT_SIZE; i++) {
        vReal[i] = adc_ring[(rp + i) & (ADC_RING_SIZE - 1)];
        vImag[i] = 0.0;
    }

    // FFT Computation
    fft.windowing(FFTWindow::Hann, FFTDirection::Forward);
    fft.compute(FFTDirection::Forward);
    fft.complexToMagnitude();

    // Drawing
    for (int b = 0; b < NUM_BARS; b++) {
        uint16_t i0 = bandMap[b].i0;
        uint16_t i1 = bandMap[b].i1;

        // Find Peak in the Band
        float mag = vReal[i0];
            for (uint16_t i = i0; i < i1; i++) {
            if (vReal[i] > mag) mag = vReal[i];
        }

        mag /= (FFT_SIZE / 2); // Normalize

        int16_t h = 0;
        if (isdBScale) {
            // dB表示
            mag /= 2048.0;    // ADCレンジ補正 (12bit + 12dB)
            if (mag < 0.00001) mag = 0.001; // To avoid log(0)
            // mag = sqrt(mag); // Power to RMS
            // double db = 20.0 * log10(mag);
            // ↓ これはおなじこと
            // double db = 10.0 * log10(mag); // Power dB
            // ↓ Pseudo-log
            uint32_t mag_q4 = (uint32_t)(mag * (1 << 16) * 16); // Q4 format
            uint32_t log_q4 = fast_log2_q4(mag_q4);
            uint32_t db_q4 = log_q4 - REF_DB_Q4; // -24dBを基準にする
            h = constrain(
                map((int32_t)db_q4,
                    DB_MIN * 16,
                    DB_MAX * 16,
                    0,
                    DRAW_HEIGHT + BOTTOM_MARGIN),
                0,
                DRAW_HEIGHT + BOTTOM_MARGIN
            );
        } else {
            // リニア表示
            mag /= 128.0; // ADCレンジ補正 (12bit)
            h = constrain(
                map((int32_t)(mag * 10000.0),
                    0,
                    10000,
                    0,
                    DRAW_HEIGHT + BOTTOM_MARGIN),
                0,
                DRAW_HEIGHT + BOTTOM_MARGIN
            );
        }

        int x = b * BAR_STEP;
        display.drawLine(x, SCREEN_HEIGHT - 1,
                         x, SCREEN_HEIGHT - 1 - h,
                         SSD1306_WHITE);
    }

    display.setCursor(0,0);
    display.printf("FFT      %dHz-%dHz", (int)F_MIN, (int)F_MAX);
    display.setCursor(108,8);
    display.printf(isdBScale ? "LOG" : "LIN");
}
