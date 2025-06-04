/*******************************************************************************************
 *  ICM-45686 – Aquisição a 1 kHz + Disparo de Laser                                         *
 *  --------------------------------------------------------------------------------------- *
 *  Este sketch realiza três tarefas principais:                                            *
 *    1. Lê acelerômetro e giroscópio do IMU ICM-45686 a 1 kHz.                             *
 *    2. Envia as amostras (botão, ax, ay, az, gx, gy, gz) pela porta serial (1 MBd).        *
 *    3. Quando o botão (GPIO 17) é pressionado, ativa-se o pino do laser (GPIO 33)          *
 *       com a latência mínima possível, usando escrita direta em registrador.              *
 *                                                                                          *
 *  Recursos de depuração adicionados:                                                      *
 *    • O LED da placa (GPIO 2) alterna a cada amostra → fácil verificar 1 kHz num osciloscópio. *
 *    • (Comentado) cálculo opcional da taxa de amostragem, imprimindo  “# Hz=…” a cada 1 s. *
 *                                                                                          *
 *  Autor: Lucian Ribeiro da Silva – Jun/2025                                               *
 *******************************************************************************************/

/* ────────────────────────────────────
 *  Bibliotecas e cabeçalhos necessários
 * ──────────────────────────────────── */
#include <Arduino.h>         // Núcleo Arduino para ESP32
#include <SPI.h>             // Comunicação SPI
#include "ICM45686.h"        // Driver oficial da TDK-InvenSense
#include "soc/gpio_reg.h"    // Endereços dos registradores de GPIO (esp-idf)

/* ────────────────────────────────────
 *  Definição dos pinos utilizados
 * ──────────────────────────────────── */
constexpr int CS_PIN        = 5;   // Chip-Select do IMU (VSPI SS)
constexpr int INT1_PIN      = 4;   // Pino de interrupção do IMU (não usado aqui)
constexpr int PINO_BOTAO    = 33;  // Botão de acionamento (input)
constexpr int PINO_TRIGGER  = 17;  // Saída que aciona o laser
constexpr int DEBUG_PIN     = 2;   // LED on-board da maioria dos kits ESP32

/* ────────────────────────────────────
 *  Variáveis globais
 * ──────────────────────────────────── */
// Offsets DC calculados na calibração (subtraídos de cada leitura)
int32_t off_ax, off_ay, off_az;
int32_t off_gx, off_gy, off_gz;

// Objeto que representa o IMU (ligado ao barramento VSPI)
ICM456xx IMU(SPI, CS_PIN);

// Flag que indica se o botão já foi pressionado alguma vez
volatile bool botaoPressionado = false;

// Buffer estático para formatar a linha de saída serial (evita alocação dinâmica)
char buffer[64];

/* ────────────────────────────────────
 *  Helpers para acesso RÁPIDO a registradores
 * ──────────────────────────────────── */
// Converte endereço em ponteiro volátil de 32 bits
#define REG32(addr) (*reinterpret_cast<volatile uint32_t*>(addr))

// Os GPIOs 0-31 ficam no bloco OUT/ENABLE, 32-39 no bloco OUT1/ENABLE1
/*#if PINO_TRIGGER < 32
  #define TRIGGER_GPIO_OUT_REG     REG32(GPIO_OUT_REG)
  #define TRIGGER_GPIO_ENABLE_REG  REG32(GPIO_ENABLE_REG)
  #define TRIGGER_BIT_MASK         (1U << PINO_TRIGGER)
#else
  #define TRIGGER_GPIO_OUT_REG     REG32(GPIO_OUT1_REG)
  #define TRIGGER_GPIO_ENABLE_REG  REG32(GPIO_ENABLE1_REG)
  #define TRIGGER_BIT_MASK         (1U << (PINO_TRIGGER - 32))
#endif
*/
/* ────────────────────────────────────
 *  Rotina de Interrupção (ISR) do botão
 *  Executa em contexto crítico → mantenha curto!
 * ──────────────────────────────────── */
void handleBotao()
{
    // Seta o bit correspondente → laser ON imediato (<100 ns)
    //TRIGGER_GPIO_OUT_REG |= TRIGGER_BIT_MASK;
    

    // Lembra que o botão foi acionado (não volta a false)
    botaoPressionado = true;
    //digitalWrite(PINO_TRIGGER,HIGH);
}

/* ────────────────────────────────────
 *  Variáveis para monitorar taxa de loop
 * ──────────────────────────────────── */
uint32_t lastSecond  = 0;
uint32_t sampleCount = 0;
float    loopRateHz  = 0.0f;

/* ────────────────────────────────────
 *  Configuração (executa uma única vez)
 * ──────────────────────────────────── */
void setup()
{
    /* ---- Serial ---- */
    Serial.begin(1'000'000);          // Porta serial em 1 Mb/s
    while (!Serial) {}                // Aguarda terminal abrir (USB-CDC)

    /* ---- Pino do laser ---- */
   /* TRIGGER_GPIO_ENABLE_REG |= TRIGGER_BIT_MASK;  // Configura como saída
    TRIGGER_GPIO_OUT_REG    &= ~TRIGGER_BIT_MASK; // Garante nível LOW inicial
*/
    /* ---- Botão com interrupção ---- */
    pinMode(PINO_BOTAO, INPUT_PULLDOWN);          // Pull-down interno
    attachInterrupt(digitalPinToInterrupt(PINO_BOTAO),
                    handleBotao, RISING);         // Chama ISR na borda de subida

    /* ---- LED de depuração ---- */
    pinMode(DEBUG_PIN, OUTPUT);
    digitalWrite(DEBUG_PIN, LOW);

    /* ---- Inicializa SPI e IMU ---- */
    SPI.begin(18, 19, 23, CS_PIN);     // SCLK=18, MISO=19, MOSI=23
    pinMode(CS_PIN, OUTPUT);           // CS manual
    digitalWrite(CS_PIN, HIGH);

    pinMode(INT1_PIN, INPUT_PULLUP);   // INT1 não usado (pode ficar desconectado)
    pinMode(PINO_TRIGGER,OUTPUT);

    if (IMU.begin() != 0) {            // Conecta e verifica WHO_AM_I
        Serial.println("# ERRO: IMU não inicializou");
        while (true) delay(1000);      // Travamos aqui em caso de falha
    }

    /* ---- Configuração de faixa e ODR ----
     * startAccel(odr_hz, g_fs)  → odr_hz=1600, faixa=±2 g
     * startGyro (odr_hz, dps_fs)→ odr_hz=1600, faixa=±2000 °/s (16 = enum da lib)
     */
    IMU.startAccel(1600, 2);
    IMU.startGyro (1600, 15);
    delay(50);                         // Espera estabilizar

    /* ---- Calibração simples de offsets ----
     * Média de 500 amostras em ~0,5 s com espera de 1 ms entre leituras.
     */
    int64_t sum[6] = {0};
    inv_imu_sensor_data_t d;
    for (int i = 0; i < 500; ++i) {
        IMU.getDataFromRegisters(d);
        sum[0] += d.accel_data[0]; sum[1] += d.accel_data[1]; sum[2] += d.accel_data[2];
        sum[3] += d.gyro_data[0];  sum[4] += d.gyro_data[1];  sum[5] += d.gyro_data[2];
        delayMicroseconds(1000);
    }
    off_ax = sum[0] / 500;  off_ay = sum[1] / 500;  off_az = sum[2] / 500;
    off_gx = sum[3] / 500;  off_gy = sum[4] / 500;  off_gz = sum[5] / 500;

    /* ---- Cabeçalho CSV ---- */
    Serial.println("botao\tax\tay\taz\tgx\tgy\tgz");

    lastSecond = millis();             // Marca t0 para contagem de amostras
}

/* ────────────────────────────────────
 *  Loop principal — executa exatamente a cada 1 ms
 * ──────────────────────────────────── */
void loop()
{
    /* ---------- Controle de tempo ---------- */
    static uint32_t nextSample = micros() + 1000;  // alvo da próxima amostra
    while (micros() < nextSample) {}               // espera ocupada (~1 µs de jitter)
    nextSample += 1000;                            // +1 ms → 1 kHz

    /* ---------- Depuração: LED toggle ---------- */
    digitalWrite(DEBUG_PIN, !digitalRead(DEBUG_PIN)); // Gera onda quadrada 1 kHz

    /* ---------- Leitura do IMU ---------- */
    
    if(botaoPressionado == 1) digitalWrite(PINO_TRIGGER,HIGH); //can be improved
    inv_imu_sensor_data_t d;
    IMU.getDataFromRegisters(d);      // Lê registradores RAW (mais rápido)

    // Aplica offsets de calibração
    int32_t ax = d.accel_data[0] - off_ax;
    int32_t ay = d.accel_data[1] - off_ay;
    int32_t az = d.accel_data[2] - off_az;
    int32_t gx = d.gyro_data[0]  - off_gx;
    int32_t gy = d.gyro_data[1]  - off_gy;
    int32_t gz = d.gyro_data[2]  - off_gz;

    /* ---------- Envio serial ----------
     * Formato tab-separado para fácil importação em Excel/MATLAB.
     * Primeiro campo = 1 se botão já foi pressionado, 0 caso contrário.
     */



    snprintf(buffer, sizeof(buffer),
             "%d\t%ld\t%ld\t%ld\t%ld\t%ld\t%ld",
             botaoPressionado ? 1 : 0, ax, ay, az, gx, gy, gz);
    Serial.println(buffer);

    /* ---------- Medidor de taxa de loop (opcional) ----------
     * Descomente para imprimir “# Hz=xxx.x” a cada 1 s.
     *
     * sampleCount++;
     * uint32_t now = millis();
     * if (now - lastSecond >= 1000) {
     *     loopRateHz = sampleCount * 1000.0f / (now - lastSecond);
     *     Serial.printf("# Hz=%.1f\n", loopRateHz);
     *     sampleCount = 0;
     *     lastSecond  = now;
     * }
     */
}
