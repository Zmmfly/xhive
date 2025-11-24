#include "main.hpp"
#include <xhive/bitmap.h>

// Test fixture for bitmap tests
class BitmapTest : public ::testing::Test {
protected:
    void SetUp() override {
        bitmap = nullptr;
    }

    void TearDown() override {
        if (bitmap) {
            xh_bitmap_destroy(bitmap);
            bitmap = nullptr;
        }
    }

    xh_bitmap_p bitmap;
};

// Test bitmap creation and destruction
TEST_F(BitmapTest, CreateDestroy) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);
    EXPECT_NE(bitmap->data, nullptr);
    EXPECT_EQ(bitmap->bits, 128);

    // Verify memory is zero-initialized
    size_t len = (128 + BITDAT_BITS - 1) / BITDAT_BITS;
    for (size_t i = 0; i < len; i++) {
        EXPECT_EQ(bitmap->data[i], 0);
    }
}

TEST_F(BitmapTest, CreateDestroyZeroBits) {
    bitmap = xh_bitmap_create(0);
    ASSERT_NE(bitmap, nullptr);
    EXPECT_NE(bitmap->data, nullptr);
    EXPECT_EQ(bitmap->bits, 0);
}

TEST_F(BitmapTest, CreateDestroySmallBitmap) {
    bitmap = xh_bitmap_create(1);
    ASSERT_NE(bitmap, nullptr);
    EXPECT_NE(bitmap->data, nullptr);
    EXPECT_EQ(bitmap->bits, 1);
}

TEST_F(BitmapTest, CreateDestroyBoundary) {
    // Test boundary: 32 bits (exactly one uint32_t)
    bitmap = xh_bitmap_create(32);
    ASSERT_NE(bitmap, nullptr);
    EXPECT_EQ(bitmap->bits, 32);
    xh_bitmap_destroy(bitmap);
    bitmap = nullptr;

    // Test boundary: 33 bits (requires two uint32_t)
    bitmap = xh_bitmap_create(33);
    ASSERT_NE(bitmap, nullptr);
    EXPECT_EQ(bitmap->bits, 33);
}

TEST_F(BitmapTest, CreateLargeBitmap) {
    bitmap = xh_bitmap_create(10000);
    ASSERT_NE(bitmap, nullptr);
    EXPECT_EQ(bitmap->bits, 10000);
}

// Test xh_bitmap_all_to function
TEST_F(BitmapTest, AllToTrue) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_all_to(bitmap, true);

    // Check first 4 uint32_t (128 bits)
    for (size_t i = 0; i < 128/BITDAT_BITS; i++) {
        EXPECT_EQ(bitmap->data[i], BITDAT_MAX);
    }
}

TEST_F(BitmapTest, AllToFalse) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    // First set all bits to true
    xh_bitmap_all_to(bitmap, true);
    // Then set all to false
    xh_bitmap_all_to(bitmap, false);

    // Check all allocated words are zero
    size_t len = (128 + 63) / 64;
    for (size_t i = 0; i < len; i++) {
        EXPECT_EQ(bitmap->data[i], 0);
    }
}

TEST_F(BitmapTest, AllToPartialBits) {
    bitmap = xh_bitmap_create(50);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_all_to(bitmap, true);

    // First uint32_t should be all 1s
    EXPECT_EQ(bitmap->data[0], 0x3FFFFFFFFFFFF); // 50 bits set
}

TEST_F(BitmapTest, AllToWithSetAndClear) {
    bitmap = xh_bitmap_create(64);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_all_to(bitmap, true);
    EXPECT_EQ(bitmap->data[0], UINT64_MAX);

    xh_bitmap_all_to(bitmap, false);
    EXPECT_EQ(bitmap->data[0], 0);
}

// Test xh_bitmap_set, xh_bitmap_clear, and xh_bitmap_test
TEST_F(BitmapTest, SetClearTestSingleBit) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    // Set bit 0
    xh_bitmap_set(bitmap, 0);
    EXPECT_TRUE(xh_bitmap_test(bitmap, 0));
    EXPECT_FALSE(xh_bitmap_test(bitmap, 1));

    // Set bit 1
    xh_bitmap_set(bitmap, 1);
    EXPECT_TRUE(xh_bitmap_test(bitmap, 0));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 1));

    // Clear bit 0
    xh_bitmap_clear(bitmap, 0);
    EXPECT_FALSE(xh_bitmap_test(bitmap, 0));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 1));
}

TEST_F(BitmapTest, SetClearTestMultipleBits) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    // Set bits at various positions
    xh_bitmap_set(bitmap, 0);
    xh_bitmap_set(bitmap, 31);
    xh_bitmap_set(bitmap, 32);
    xh_bitmap_set(bitmap, 63);
    xh_bitmap_set(bitmap, 64);
    xh_bitmap_set(bitmap, 127);

    EXPECT_TRUE(xh_bitmap_test(bitmap, 0));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 31));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 32));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 63));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 64));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 127));

    // Clear some bits
    xh_bitmap_clear(bitmap, 31);
    xh_bitmap_clear(bitmap, 63);
    xh_bitmap_clear(bitmap, 127);

    EXPECT_FALSE(xh_bitmap_test(bitmap, 31));
    EXPECT_FALSE(xh_bitmap_test(bitmap, 63));
    EXPECT_FALSE(xh_bitmap_test(bitmap, 127));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 0));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 32));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 64));
}

TEST_F(BitmapTest, SetClearTestBoundaryBits) {
    bitmap = xh_bitmap_create(64);
    ASSERT_NE(bitmap, nullptr);

    // Test boundary bits (31, 32)
    xh_bitmap_set(bitmap, 31);
    xh_bitmap_set(bitmap, 32);

    EXPECT_TRUE(xh_bitmap_test(bitmap, 31));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 32));

    xh_bitmap_clear(bitmap, 31);
    EXPECT_FALSE(xh_bitmap_test(bitmap, 31));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 32));
}

TEST_F(BitmapTest, SetTestAllBits) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    // Set all bits individually and verify
    for (size_t i = 0; i < 128; i++) {
        xh_bitmap_set(bitmap, i);
    }

    for (size_t i = 0; i < 128; i++) {
        EXPECT_TRUE(xh_bitmap_test(bitmap, i));
    }
}

TEST_F(BitmapTest, ClearTestAllBits) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    // First set all bits
    xh_bitmap_all_to(bitmap, true);

    // Then clear all bits individually
    for (size_t i = 0; i < 128; i++) {
        xh_bitmap_clear(bitmap, i);
    }

    for (size_t i = 0; i < 128; i++) {
        EXPECT_FALSE(xh_bitmap_test(bitmap, i));
    }
}

// Test xh_bitmap_count_set_bits
TEST_F(BitmapTest, CountSetBitsEmpty) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    EXPECT_EQ(xh_bitmap_count_set_bits(bitmap), 0);
}

TEST_F(BitmapTest, CountSetBitsAll) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_all_to(bitmap, true);
    EXPECT_EQ(xh_bitmap_count_set_bits(bitmap), 128);
}

TEST_F(BitmapTest, CountSetBitsPartial) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_set(bitmap, 0);
    xh_bitmap_set(bitmap, 31);
    xh_bitmap_set(bitmap, 32);
    xh_bitmap_set(bitmap, 63);
    xh_bitmap_set(bitmap, 64);

    EXPECT_EQ(xh_bitmap_count_set_bits(bitmap), 5);
}

TEST_F(BitmapTest, CountSetBitsWithAlternatingPattern) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    // Set even bits
    for (size_t i = 0; i < 128; i += 2) {
        xh_bitmap_set(bitmap, i);
    }

    EXPECT_EQ(xh_bitmap_count_set_bits(bitmap), 64);
}

TEST_F(BitmapTest, CountSetBitsSmallBitmap) {
    bitmap = xh_bitmap_create(5);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_set(bitmap, 0);
    xh_bitmap_set(bitmap, 2);
    xh_bitmap_set(bitmap, 4);

    EXPECT_EQ(xh_bitmap_count_set_bits(bitmap), 3);
}

TEST_F(BitmapTest, CountSetBitsLarge) {
    bitmap = xh_bitmap_create(1000);
    ASSERT_NE(bitmap, nullptr);

    // Set 100 bits
    for (size_t i = 0; i < 100; i++) {
        xh_bitmap_set(bitmap, i * 10);
    }

    EXPECT_EQ(xh_bitmap_count_set_bits(bitmap), 100);
}

// Test xh_bitmap_index_of_left_set
TEST_F(BitmapTest, IndexOfLeftSetBasic) {
    bitmap = xh_bitmap_create(48);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_set(bitmap, 10);
    xh_bitmap_set(bitmap, 20);
    xh_bitmap_set(bitmap, 30);

    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 30), 30);
    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 25), 20);
    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 20), 20);
    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 15), 10);
    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 10), 10);
}

TEST_F(BitmapTest, IndexOfLeftSetNotFound) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_set(bitmap, 10);
    xh_bitmap_set(bitmap, 20);

    // Search from index 9, should find bit 0
    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 5), SIZE_MAX);
}

TEST_F(BitmapTest, IndexOfLeftSetBoundary) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_set(bitmap, 0);
    xh_bitmap_set(bitmap, 31);
    xh_bitmap_set(bitmap, 32);
    xh_bitmap_set(bitmap, 63);

    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 63), 63);
    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 62), 32);
    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 32), 32);
    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 31), 31);
    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 30), 0);
}

// Test xh_bitmap_index_of_right_set
TEST_F(BitmapTest, IndexOfRightSetBasic) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_set(bitmap, 10);
    xh_bitmap_set(bitmap, 20);
    xh_bitmap_set(bitmap, 30);

    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 10), 10);
    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 15), 20);
    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 20), 20);
    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 25), 30);
    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 30), 30);
}

TEST_F(BitmapTest, IndexOfRightSetNotFound) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_set(bitmap, 10);
    xh_bitmap_set(bitmap, 20);

    // Search from index 21 to end
    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 21), SIZE_MAX);
}

TEST_F(BitmapTest, IndexOfRightSetBoundary) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_set(bitmap, 0);
    xh_bitmap_set(bitmap, 31);
    xh_bitmap_set(bitmap, 32);
    xh_bitmap_set(bitmap, 63);

    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 0), 0);
    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 1), 31);
    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 31), 31);
    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 32), 32);
    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 33), 63);
}

// Additional edge case tests
TEST_F(BitmapTest, SizeMaxBits) {
    bitmap = xh_bitmap_create(SIZE_MAX / 2);  // Large but should work
    if (bitmap) {
        EXPECT_EQ(bitmap->bits, SIZE_MAX / 2);
        xh_bitmap_destroy(bitmap);
        bitmap = nullptr;
    }
}

TEST_F(BitmapTest, SetAllBitsAndCount) {
    bitmap = xh_bitmap_create(100);
    ASSERT_NE(bitmap, nullptr);

    for (size_t i = 0; i < 100; i++) {
        xh_bitmap_set(bitmap, i);
    }

    EXPECT_EQ(xh_bitmap_count_set_bits(bitmap), 100);

    // Now clear half
    for (size_t i = 0; i < 100; i += 2) {
        xh_bitmap_clear(bitmap, i);
    }

    EXPECT_EQ(xh_bitmap_count_set_bits(bitmap), 50);
}

TEST_F(BitmapTest, AlternatingPattern) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    // Set alternating bits
    for (size_t i = 0; i < 128; i += 2) {
        xh_bitmap_set(bitmap, i);
    }

    for (size_t i = 0; i < 128; i++) {
        if (i % 2 == 0) {
            EXPECT_TRUE(xh_bitmap_test(bitmap, i));
        } else {
            EXPECT_FALSE(xh_bitmap_test(bitmap, i));
        }
    }
}

TEST_F(BitmapTest, FindConsecutiveSetBits) {
    bitmap = xh_bitmap_create(128);
    ASSERT_NE(bitmap, nullptr);

    // Set consecutive bits 10-19
    for (size_t i = 10; i < 20; i++) {
        xh_bitmap_set(bitmap, i);
    }

    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 19), 19);
    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 10), 10);
    EXPECT_EQ(xh_bitmap_count_set_bits(bitmap), 10);
}

// Boundary tests for non-32-multiple bitmap sizes
TEST_F(BitmapTest, AllToBoundary33Bits) {
    bitmap = xh_bitmap_create(33);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_all_to(bitmap, true);
    // First 32 bits should be all 1s
    EXPECT_EQ(bitmap->data[0], 0x1FFFFFFFF);

    xh_bitmap_all_to(bitmap, false);
    EXPECT_EQ(bitmap->data[0], 0);
}

TEST_F(BitmapTest, AllToBoundary63Bits) {
    bitmap = xh_bitmap_create(63);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_all_to(bitmap, true);
    EXPECT_EQ(bitmap->data[0], 0x7FFFFFFFFFFFFFFF);
}

TEST_F(BitmapTest, SetClearTestBoundaryAtWordEdge) {
    bitmap = xh_bitmap_create(65);
    ASSERT_NE(bitmap, nullptr);

    // Test bits at word boundary (31, 32, 63, 64)
    xh_bitmap_set(bitmap, 31);  // Last bit of first word
    xh_bitmap_set(bitmap, 32);  // First bit of second word
    xh_bitmap_set(bitmap, 63);  // Last bit of second word
    xh_bitmap_set(bitmap, 64);  // First bit of third word (for 65 bits)

    EXPECT_TRUE(xh_bitmap_test(bitmap, 31));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 32));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 63));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 64));

    // Clear and verify
    xh_bitmap_clear(bitmap, 31);
    xh_bitmap_clear(bitmap, 32);
    EXPECT_FALSE(xh_bitmap_test(bitmap, 31));
    EXPECT_FALSE(xh_bitmap_test(bitmap, 32));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 63));
    EXPECT_TRUE(xh_bitmap_test(bitmap, 64));
}

TEST_F(BitmapTest, CountSetBitsAtBoundary) {
    bitmap = xh_bitmap_create(65);
    ASSERT_NE(bitmap, nullptr);

    // Set bits that cross word boundary
    xh_bitmap_set(bitmap, 30);
    xh_bitmap_set(bitmap, 31);
    xh_bitmap_set(bitmap, 32);
    xh_bitmap_set(bitmap, 33);

    EXPECT_EQ(xh_bitmap_count_set_bits(bitmap), 4);

    // Add more bits in third word
    xh_bitmap_set(bitmap, 64);
    EXPECT_EQ(xh_bitmap_count_set_bits(bitmap), 5);
}

TEST_F(BitmapTest, IndexOfLeftSetBoundary65Bits) {
    bitmap = xh_bitmap_create(65);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_set(bitmap, 0);
    xh_bitmap_set(bitmap, 31);
    xh_bitmap_set(bitmap, 32);
    xh_bitmap_set(bitmap, 64);

    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 64), 64);
    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 63), 63);
    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 32), 32);
    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 31), 31);
    EXPECT_EQ(xh_bitmap_index_of_left_set(bitmap, 30), 0);
}

TEST_F(BitmapTest, IndexOfRightSetBoundary65Bits) {
    bitmap = xh_bitmap_create(65);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_set(bitmap, 0);
    xh_bitmap_set(bitmap, 31);
    xh_bitmap_set(bitmap, 32);
    xh_bitmap_set(bitmap, 64);

    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 0), 0);
    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 1), 31);
    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 31), 31);
    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 32), 32);
    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 33), 64);
    EXPECT_EQ(xh_bitmap_index_of_right_set(bitmap, 64), 64);
}

TEST_F(BitmapTest, AllBitsInNon32MultipleBitmap) {
    bitmap = xh_bitmap_create(47);
    ASSERT_NE(bitmap, nullptr);

    // Set all bits
    for (size_t i = 0; i < 47; i++) {
        xh_bitmap_set(bitmap, i);
    }

    EXPECT_EQ(xh_bitmap_count_set_bits(bitmap), 47);

    // Verify no bits beyond 47 are set
    EXPECT_EQ(bitmap->data[0], 0x7FFFFFFFFFFF);

    // Clear a bit in the middle and at boundary
    xh_bitmap_clear(bitmap, 20);
    xh_bitmap_clear(bitmap, 46);

    EXPECT_FALSE(xh_bitmap_test(bitmap, 20));
    EXPECT_FALSE(xh_bitmap_test(bitmap, 46));
    EXPECT_EQ(xh_bitmap_count_set_bits(bitmap), 45);
}

TEST_F(BitmapTest, ExtremeBoundaries) {
    // Test bitmap with just 1 bit
    bitmap = xh_bitmap_create(1);
    ASSERT_NE(bitmap, nullptr);

    EXPECT_FALSE(xh_bitmap_test(bitmap, 0));
    xh_bitmap_set(bitmap, 0);
    EXPECT_TRUE(xh_bitmap_test(bitmap, 0));
    EXPECT_EQ(xh_bitmap_count_set_bits(bitmap), 1);

    xh_bitmap_destroy(bitmap);
    bitmap = nullptr;

    // Test bitmap with 31 bits (just before word boundary)
    bitmap = xh_bitmap_create(31);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_set(bitmap, 30);
    EXPECT_TRUE(xh_bitmap_test(bitmap, 30));
    EXPECT_EQ(bitmap->data[0], 0x40000000);

    xh_bitmap_destroy(bitmap);
    bitmap = nullptr;

    // Test bitmap with 32 bits (exact word boundary)
    bitmap = xh_bitmap_create(32);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_set(bitmap, 31);
    EXPECT_TRUE(xh_bitmap_test(bitmap, 31));
    EXPECT_EQ(bitmap->data[0], 0x80000000);

    xh_bitmap_destroy(bitmap);
    bitmap = nullptr;

    // Test bitmap with 33 bits (just after word boundary)
    bitmap = xh_bitmap_create(33);
    ASSERT_NE(bitmap, nullptr);

    xh_bitmap_set(bitmap, 32);
    EXPECT_TRUE(xh_bitmap_test(bitmap, 32));
    EXPECT_EQ(bitmap->data[0], 0x100000000);
}
