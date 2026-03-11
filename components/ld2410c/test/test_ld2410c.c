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
