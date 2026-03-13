#include "unity.h"
#include "ld2410c.h"
#include <string.h>

/*
 * LD2410C reporting frame layout (standard mode, no engineering data):
 *
 * [0..3]   Header:  F4 F3 F2 F1
 * [4..5]   Data length (LE uint16) — includes bytes 6..N before footer
 * [6]      Data type: 0x02 (reporting)
 * [7]      Head: 0xAA
 * [8]      Target state: 0x00=none, 0x01=moving, 0x02=stationary, 0x03=both
 * [9..10]  Moving distance (LE uint16, cm)
 * [11]     Moving energy (0-100)
 * [12..13] Stationary distance (LE uint16, cm)
 * [14]     Stationary energy (0-100)
 * [15..16] Detection distance (LE uint16, cm)
 * [17]     Tail: 0x55
 * [18]     Check: 0x00
 * [19..22] Footer: F8 F7 F6 F5
 *
 * Total: 23 bytes for a standard reporting frame with data_len = 13
 */

/* Helper: build a valid standard reporting frame */
static void build_frame(uint8_t *buf, size_t *len,
                        uint8_t target_state,
                        uint16_t move_dist, uint8_t move_energy,
                        uint16_t still_dist, uint8_t still_energy,
                        uint16_t det_dist)
{
    size_t i = 0;
    /* Header */
    buf[i++] = 0xF4; buf[i++] = 0xF3; buf[i++] = 0xF2; buf[i++] = 0xF1;
    /* Data length = 13 (bytes 6..18 inclusive) */
    buf[i++] = 13; buf[i++] = 0;
    /* Data type */
    buf[i++] = 0x02;
    /* Head */
    buf[i++] = 0xAA;
    /* Target state */
    buf[i++] = target_state;
    /* Moving distance (LE) */
    buf[i++] = move_dist & 0xFF; buf[i++] = (move_dist >> 8) & 0xFF;
    /* Moving energy */
    buf[i++] = move_energy;
    /* Stationary distance (LE) */
    buf[i++] = still_dist & 0xFF; buf[i++] = (still_dist >> 8) & 0xFF;
    /* Stationary energy */
    buf[i++] = still_energy;
    /* Detection distance (LE) */
    buf[i++] = det_dist & 0xFF; buf[i++] = (det_dist >> 8) & 0xFF;
    /* Tail */
    buf[i++] = 0x55;
    /* Check */
    buf[i++] = 0x00;
    /* Footer */
    buf[i++] = 0xF8; buf[i++] = 0xF7; buf[i++] = 0xF6; buf[i++] = 0xF5;
    *len = i;
}

/* ── Valid frame parsing ─────────────────────────────────────── */

TEST_CASE("Valid frame with moving target parses correctly", "[ld2410c]")
{
    uint8_t buf[32];
    size_t len;
    build_frame(buf, &len, 0x01, 150, 75, 0, 0, 150);

    ld2410c_data_t data;
    TEST_ASSERT_EQUAL(ESP_OK, ld2410c_parse_frame(buf, len, &data));
    TEST_ASSERT_TRUE(data.moving_target);
    TEST_ASSERT_FALSE(data.stationary_target);
    TEST_ASSERT_EQUAL_UINT8(75, data.move_energy);
    TEST_ASSERT_EQUAL_UINT8(0, data.static_energy);
    TEST_ASSERT_EQUAL_UINT16(150, data.target_distance_cm);
}

TEST_CASE("Valid frame with stationary target parses correctly", "[ld2410c]")
{
    uint8_t buf[32];
    size_t len;
    build_frame(buf, &len, 0x02, 0, 0, 200, 60, 200);

    ld2410c_data_t data;
    TEST_ASSERT_EQUAL(ESP_OK, ld2410c_parse_frame(buf, len, &data));
    TEST_ASSERT_FALSE(data.moving_target);
    TEST_ASSERT_TRUE(data.stationary_target);
    TEST_ASSERT_EQUAL_UINT8(0, data.move_energy);
    TEST_ASSERT_EQUAL_UINT8(60, data.static_energy);
    TEST_ASSERT_EQUAL_UINT16(200, data.target_distance_cm);
}

TEST_CASE("Valid frame with both targets parses correctly", "[ld2410c]")
{
    uint8_t buf[32];
    size_t len;
    build_frame(buf, &len, 0x03, 100, 50, 200, 40, 100);

    ld2410c_data_t data;
    TEST_ASSERT_EQUAL(ESP_OK, ld2410c_parse_frame(buf, len, &data));
    TEST_ASSERT_TRUE(data.moving_target);
    TEST_ASSERT_TRUE(data.stationary_target);
    TEST_ASSERT_EQUAL_UINT8(50, data.move_energy);
    TEST_ASSERT_EQUAL_UINT8(40, data.static_energy);
    TEST_ASSERT_EQUAL_UINT16(100, data.target_distance_cm);
}

TEST_CASE("Valid frame with no target parses correctly", "[ld2410c]")
{
    uint8_t buf[32];
    size_t len;
    build_frame(buf, &len, 0x00, 0, 0, 0, 0, 0);

    ld2410c_data_t data;
    TEST_ASSERT_EQUAL(ESP_OK, ld2410c_parse_frame(buf, len, &data));
    TEST_ASSERT_FALSE(data.moving_target);
    TEST_ASSERT_FALSE(data.stationary_target);
    TEST_ASSERT_EQUAL_UINT8(0, data.move_energy);
    TEST_ASSERT_EQUAL_UINT8(0, data.static_energy);
    TEST_ASSERT_EQUAL_UINT16(0, data.target_distance_cm);
}

/* ── Invalid frame rejection ─────────────────────────────────── */

TEST_CASE("Frame with bad header is rejected", "[ld2410c]")
{
    uint8_t buf[32];
    size_t len;
    build_frame(buf, &len, 0x01, 100, 50, 0, 0, 100);
    buf[0] = 0x00; /* corrupt header */

    ld2410c_data_t data;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ld2410c_parse_frame(buf, len, &data));
}

TEST_CASE("Frame with bad footer is rejected", "[ld2410c]")
{
    uint8_t buf[32];
    size_t len;
    build_frame(buf, &len, 0x01, 100, 50, 0, 0, 100);
    buf[len - 1] = 0x00; /* corrupt footer */

    ld2410c_data_t data;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ld2410c_parse_frame(buf, len, &data));
}

TEST_CASE("Frame with bad check byte is rejected", "[ld2410c]")
{
    uint8_t buf[32];
    size_t len;
    build_frame(buf, &len, 0x01, 100, 50, 0, 0, 100);
    buf[18] = 0xFF; /* corrupt check byte (should be 0x00) */

    ld2410c_data_t data;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ld2410c_parse_frame(buf, len, &data));
}

TEST_CASE("Partial frame (too short) does not crash", "[ld2410c]")
{
    uint8_t buf[] = {0xF4, 0xF3, 0xF2, 0xF1, 0x02, 0x00};
    ld2410c_data_t data;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ld2410c_parse_frame(buf, sizeof(buf), &data));
}

TEST_CASE("NULL buffer returns error", "[ld2410c]")
{
    ld2410c_data_t data;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ld2410c_parse_frame(NULL, 23, &data));
}

TEST_CASE("NULL output returns error", "[ld2410c]")
{
    uint8_t buf[32];
    size_t len;
    build_frame(buf, &len, 0x00, 0, 0, 0, 0, 0);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ld2410c_parse_frame(buf, len, NULL));
}

/* ── Command frame building tests ──────────────────────────── */

TEST_CASE("Build enable config command has correct structure", "[ld2410c]")
{
    uint8_t buf[32];
    uint8_t payload[] = {0x01, 0x00};
    size_t len = ld2410c_build_cmd(buf, LD2410C_CMD_ENABLE_CONFIG, payload, 2);

    /* Total: header(4) + len(2) + cmd(2) + payload(2) + footer(4) = 14 */
    TEST_ASSERT_EQUAL(14, len);

    /* Header */
    TEST_ASSERT_EQUAL_HEX8(0xFD, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xFC, buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFB, buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFA, buf[3]);

    /* Length = 4 (2 cmd + 2 payload) */
    TEST_ASSERT_EQUAL_HEX8(0x04, buf[4]);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[5]);

    /* Command 0x00FF little-endian */
    TEST_ASSERT_EQUAL_HEX8(0xFF, buf[6]);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[7]);

    /* Payload */
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[8]);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[9]);

    /* Footer */
    TEST_ASSERT_EQUAL_HEX8(0x04, buf[10]);
    TEST_ASSERT_EQUAL_HEX8(0x03, buf[11]);
    TEST_ASSERT_EQUAL_HEX8(0x02, buf[12]);
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[13]);
}

TEST_CASE("Build end config command (no payload)", "[ld2410c]")
{
    uint8_t buf[32];
    size_t len = ld2410c_build_cmd(buf, LD2410C_CMD_END_CONFIG, NULL, 0);

    /* Total: header(4) + len(2) + cmd(2) + footer(4) = 12 */
    TEST_ASSERT_EQUAL(12, len);

    /* Length = 2 (cmd only) */
    TEST_ASSERT_EQUAL_HEX8(0x02, buf[4]);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[5]);

    /* Command 0x00FE little-endian */
    TEST_ASSERT_EQUAL_HEX8(0xFE, buf[6]);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[7]);
}

/* ── ACK parsing tests ─────────────────────────────────────── */

/* Helper: build a valid ACK frame */
static void build_ack(uint8_t *buf, size_t *len, uint16_t cmd, uint16_t status)
{
    size_t i = 0;
    /* Header */
    buf[i++] = 0xFD; buf[i++] = 0xFC; buf[i++] = 0xFB; buf[i++] = 0xFA;
    /* Length = 4 (cmd_echo(2) + status(2)) */
    buf[i++] = 0x04; buf[i++] = 0x00;
    /* ACK echoes cmd | 0x0100 */
    uint16_t ack_cmd = cmd | 0x0100;
    buf[i++] = ack_cmd & 0xFF;
    buf[i++] = (ack_cmd >> 8) & 0xFF;
    /* Status */
    buf[i++] = status & 0xFF;
    buf[i++] = (status >> 8) & 0xFF;
    /* Footer */
    buf[i++] = 0x04; buf[i++] = 0x03; buf[i++] = 0x02; buf[i++] = 0x01;
    *len = i;
}

TEST_CASE("Valid ACK for enable config parses successfully", "[ld2410c]")
{
    uint8_t buf[32];
    size_t len;
    build_ack(buf, &len, LD2410C_CMD_ENABLE_CONFIG, 0x0000);
    TEST_ASSERT_EQUAL(ESP_OK, ld2410c_parse_ack(buf, len, LD2410C_CMD_ENABLE_CONFIG));
}

TEST_CASE("Valid ACK for set max gate parses successfully", "[ld2410c]")
{
    uint8_t buf[32];
    size_t len;
    build_ack(buf, &len, LD2410C_CMD_SET_MAX_GATE, 0x0000);
    TEST_ASSERT_EQUAL(ESP_OK, ld2410c_parse_ack(buf, len, LD2410C_CMD_SET_MAX_GATE));
}

TEST_CASE("ACK with failure status returns ESP_FAIL", "[ld2410c]")
{
    uint8_t buf[32];
    size_t len;
    build_ack(buf, &len, LD2410C_CMD_ENABLE_CONFIG, 0x0001);
    TEST_ASSERT_EQUAL(ESP_FAIL, ld2410c_parse_ack(buf, len, LD2410C_CMD_ENABLE_CONFIG));
}

TEST_CASE("ACK with wrong command returns ESP_ERR_INVALID_RESPONSE", "[ld2410c]")
{
    uint8_t buf[32];
    size_t len;
    build_ack(buf, &len, LD2410C_CMD_ENABLE_CONFIG, 0x0000);
    /* Check against a different command */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_RESPONSE,
                      ld2410c_parse_ack(buf, len, LD2410C_CMD_END_CONFIG));
}

TEST_CASE("ACK with bad header returns error", "[ld2410c]")
{
    uint8_t buf[32];
    size_t len;
    build_ack(buf, &len, LD2410C_CMD_ENABLE_CONFIG, 0x0000);
    buf[0] = 0x00; /* corrupt header */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ld2410c_parse_ack(buf, len, LD2410C_CMD_ENABLE_CONFIG));
}

TEST_CASE("ACK with bad footer returns error", "[ld2410c]")
{
    uint8_t buf[32];
    size_t len;
    build_ack(buf, &len, LD2410C_CMD_ENABLE_CONFIG, 0x0000);
    buf[len - 1] = 0xFF; /* corrupt footer */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ld2410c_parse_ack(buf, len, LD2410C_CMD_ENABLE_CONFIG));
}

TEST_CASE("ACK too short returns error", "[ld2410c]")
{
    uint8_t buf[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ld2410c_parse_ack(buf, sizeof(buf), LD2410C_CMD_ENABLE_CONFIG));
}

TEST_CASE("NULL ACK buffer returns error", "[ld2410c]")
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ld2410c_parse_ack(NULL, 14, LD2410C_CMD_ENABLE_CONFIG));
}
