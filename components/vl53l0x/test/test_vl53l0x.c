#include "unity.h"
#include "vl53l0x.h"
#include <stddef.h>

/*
 * These tests validate the status mapping and sentinel logic without hardware.
 * Tests requiring an actual VL53L0X sensor are marked and should be skipped in CI.
 */

/* ── Status mapping tests ────────────────────────────────────── */

TEST_CASE("Status 0 (valid range) maps to 0", "[vl53l0x]")
{
    TEST_ASSERT_EQUAL_UINT8(0, vl53l0x_map_status(0));
}

TEST_CASE("Status 3 (min range) maps to 0 (valid)", "[vl53l0x]")
{
    /* Min range is still a valid measurement */
    TEST_ASSERT_EQUAL_UINT8(0, vl53l0x_map_status(3));
}

TEST_CASE("Status 1 (sigma fail) maps to 1", "[vl53l0x]")
{
    TEST_ASSERT_EQUAL_UINT8(1, vl53l0x_map_status(1));
}

TEST_CASE("Status 2 (signal fail) maps to 2", "[vl53l0x]")
{
    TEST_ASSERT_EQUAL_UINT8(2, vl53l0x_map_status(2));
}

TEST_CASE("Status 4 (phase fail) maps to 255 (no target)", "[vl53l0x]")
{
    TEST_ASSERT_EQUAL_UINT8(255, vl53l0x_map_status(4));
}

TEST_CASE("Status 5 (hw fail) maps to 255 (no target)", "[vl53l0x]")
{
    TEST_ASSERT_EQUAL_UINT8(255, vl53l0x_map_status(5));
}

TEST_CASE("Unknown status maps to 255 (no target)", "[vl53l0x]")
{
    TEST_ASSERT_EQUAL_UINT8(255, vl53l0x_map_status(99));
    TEST_ASSERT_EQUAL_UINT8(255, vl53l0x_map_status(255));
}

/* ── Struct size test ────────────────────────────────────────── */

TEST_CASE("vl53l0x_data_t packs to expected size", "[vl53l0x]")
{
    /* uint16_t range_mm + uint8_t status = minimum 3 bytes, but struct padding
     * may increase this to 4 bytes. Accept either. */
    TEST_ASSERT(sizeof(vl53l0x_data_t) >= 3);
    TEST_ASSERT(sizeof(vl53l0x_data_t) <= 4);
}

/* ── Sentinel value test ─────────────────────────────────────── */

TEST_CASE("Out-of-range sentinel is 2000mm", "[vl53l0x]")
{
    TEST_ASSERT_EQUAL_UINT16(2000, VL53L0X_OUT_OF_RANGE_MM);
}

/*
 * NOTE: The following tests require hardware (VL53L0X connected via I2C)
 * and should be skipped in CI:
 *
 * - vl53l0x_init succeeds with sensor connected
 * - vl53l0x_read returns valid range when target in range
 * - vl53l0x_read returns sentinel for out-of-range target
 * - vl53l0x_deinit cleans up without error
 */
