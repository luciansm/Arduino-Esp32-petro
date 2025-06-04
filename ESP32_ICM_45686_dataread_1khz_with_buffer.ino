/*******************************************************************************************
 *  ICM-45686 – Aquisição a 1 kHz + Disparo de Laser                                       *
 *                                                                                        *
 *  FUNCIONALIDADES:                                                                      *
 *   • Lê sinais de acelerômetro e giroscópio do IMU ICM-45686 a 1 kHz                    *
 *   • Envia os dados por serial em formato tabulado                                     *
 *   • Aciona o laser (GPIO 17) imediatamente após detecção de botão (GPIO 33)            *
 *                                                                                        *
 *  Autor: Lucian Ribeiro da Silva – Jun/2025                                             *
 *******************************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "ICM45686.h"
#include "soc/gpio_reg.h"  // Para manipulação direta de registradores (se necessário)

/* ─────────────────────────────────────
 *  Definições de pinos da aplicação
 * ───────────────────────────────────── */
constexpr int CS_PIN        = 5;    // Chip Select do IMU no barramento SPI
constexpr int INT1_PIN      = 4;    // Pino de interrupção da IMU (não usado)
constexpr int PINO_BOTAO    = 33;   // Entrada digital para botão de disparo
constexpr int PINO_TRIGGER  = 17;   // Saída digital que ativa o laser
constexpr int DEBUG_PIN     = 2;    // LED on-board para depuração (opcional)

/* ─────────────────────────────────────
 *  Variáveis globais
 * ───────────────────────────────────── */
int32_t off_ax, off_ay, off_az;     // Offsets de aceleração (calculados na calibração)
int32_t off_gx, off_gy, off_gz;     // Offsets de giroscópio

ICM456xx IMU(SPI, CS_PIN);          // Objeto do IMU conectado via SPI

volatile int botaoPressionado = 0;  // Flag de disparo (1 = botão já foi pressionado)

char buffer[64];                    // Buffer para formatação da saída serial

/* ─────────────────────────────────────
 *  Interrupção do botão
 *  Aciona a flag assim que botão for pressionado (borda de subida)
 * ───────────────────────────────────── */
void handleBotao() {
    botaoPressionado = 1;
}

/* ─────────────────────────────────────
 *  Configuração inicial do sistema
 * ───────────────────────────────────── */
void setup() {
    // Inicialização da porta serial a 1 Mbaud
    Serial.begin(1'000'000);
    while (!Serial) {}  // Espera até conexão com o terminal serial (USB-CDC)

    // Configura botão com resistor de pull-down interno e interrupção
    pinMode(PINO_BOTAO, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PINO_BOTAO), handleBotao, RISING);

    // Configura pino de trigger (laser) como saída, inicialmente desligado
    pinMode(PINO_TRIGGER, OUTPUT);
    digitalWrite(PINO_TRIGGER, LOW);

    // Configura LED onboard como saída
    pinMode(DEBUG_PIN, OUTPUT);
    digitalWrite(DEBUG_PIN, LOW);

    // Inicializa barramento SPI com pinos padrão da ESP32
    SPI.begin(18, 19, 23, CS_PIN);
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);

    pinMode(INT1_PIN, INPUT_PULLUP);  // Pino INT1 não é utilizado

    // Inicializa IMU e verifica comunicação
    if (IMU.begin() != 0) {
        Serial.println("# ERRO: IMU não inicializou");
        while (true) delay(1000);  // Fica travado se falhar
    }

    // Configura IMU com ODR = 1600 Hz e faixas apropriadas
    IMU.startAccel(1600, 2);    // ±2g
    IMU.startGyro (1600, 15);   // ±15,625°/s (15 = enum da biblioteca)
    delay(50);                  // Espera estabilizar

    // Calibração dos sensores (offsets): média de 500 amostras
    int64_t sum[6] = {0};
    inv_imu_sensor_data_t d;
    for (int i = 0; i < 500; ++i) {
        IMU.getDataFromRegisters(d);
        sum[0] += d.accel_data[0];
        sum[1] += d.accel_data[1];
        sum[2] += d.accel_data[2];
        sum[3] += d.gyro_data[0];
        sum[4] += d.gyro_data[1];
        sum[5] += d.gyro_data[2];
        delayMicroseconds(1000);  // Aprox. 1 ms entre leituras
    }
    off_ax = sum[0] / 500;
    off_ay = sum[1] / 500;
    off_az = sum[2] / 500;
    off_gx = sum[3] / 500;
    off_gy = sum[4] / 500;
    off_gz = sum[5] / 500;

    // Cabeçalho CSV para leitura dos dados no terminal
    Serial.println("botao\tax\tay\taz\tgx\tgy\tgz");
}

/* ─────────────────────────────────────
 *  Loop principal – executa a 1 kHz exato
 * ───────────────────────────────────── */
void loop() {
    static uint32_t nextSample = micros() + 1000;  // tempo-alvo da próxima amostra
    while (micros() < nextSample) {}               // espera ocupada
    nextSample += 1000;                            // próximo ciclo (1 ms adiante)

    // Aciona o laser se botão tiver sido pressionado (apenas uma vez)
    if (botaoPressionado == 1)
        digitalWrite(PINO_TRIGGER, HIGH);

    // Leitura bruta da IMU (sem uso de FIFO, mais rápido)
    inv_imu_sensor_data_t d;
    IMU.getDataFromRegisters(d);

    // Subtrai os offsets
    int32_t ax = d.accel_data[0] - off_ax;
    int32_t ay = d.accel_data[1] - off_ay;
    int32_t az = d.accel_data[2] - off_az;
    int32_t gx = d.gyro_data[0]  - off_gx;
    int32_t gy = d.gyro_data[1]  - off_gy;
    int32_t gz = d.gyro_data[2]  - off_gz;

    // Monta linha de saída (formato tabulado)
    snprintf(buffer, sizeof(buffer),
             "%d\t%ld\t%ld\t%ld\t%ld\t%ld\t%ld",
             botaoPressionado, ax, ay, az, gx, gy, gz);
    Serial.println(buffer);

    // (Opcional: descomente para laser com pulso único)
    // botaoPressionado = 0;  // reinicia flag para evitar manter o laser ligado
}
