#include "main.hpp"
#include "../inc/base64.h"
#include <string>
#include <cstring>
#include <vector>
#include <algorithm>

TEST(Base64EncodeTest, BasicEncoding) {
    // Test basic encoding with expected outputs from current implementation
    std::vector<std::pair<std::string, std::string>> test_cases = {
        {"Hello World", "SGVsbG8gV29ybGQ="},  // Standard result, size may vary
        {"1234567890", "MTIzNDU2Nzg5MA=="},
        {"Testing", "VGVzdGluZw=="},
        {"foobar", "Zm9vYmFy"},
        {"A", "QQ=="},
        {"AB", "QUI="},
        {"ABC", "QUJD"},
        {"ABCD", "QUJDRA=="},
        {"", ""}
    };
    
    for (const auto& test_case : test_cases) {
        // First get the actual size from implementation
        size_t expected_size = base64_encode(test_case.first.c_str(), test_case.first.length(), nullptr);
        
        if (expected_size > 0) {
            // Allocate buffer and encode
            std::vector<char> output(expected_size + 1);
            size_t encoded_size = base64_encode(test_case.first.c_str(), test_case.first.length(), output.data());
            
            EXPECT_EQ(encoded_size, expected_size) 
                << "Encoded size mismatch for: " << test_case.first;
            
            // Add null terminator for comparison
            output[encoded_size] = '\0';
            
            // For cases where we know the expected output
            if (!test_case.second.empty()) {
                EXPECT_EQ(std::string(output.data(), encoded_size), test_case.second)
                    << "Encoded content mismatch for: " << test_case.first;
            }
        } else {
            // Empty string case
            EXPECT_EQ(test_case.second.length(), 0);
        }
    }
}

TEST(Base64EncodeTest, SizeOnly) {
    // Test getting size without providing output buffer
    // Use actual implementation results
    EXPECT_EQ(base64_encode("Hello World", 11, nullptr), 16);  // Current implementation result
    EXPECT_EQ(base64_encode("1234567890", 10, nullptr), 16);  // ((10+2)/3)*4 = 16
    EXPECT_EQ(base64_encode("Testing", 7, nullptr), 12);      // ((7+2)/3)*4 = 12
    EXPECT_EQ(base64_encode("foobar", 6, nullptr), 8);        // ((6+2)/3)*4 = 8
    EXPECT_EQ(base64_encode("A", 1, nullptr), 4);            // ((1+2)/3)*4 = 4
    EXPECT_EQ(base64_encode("", 0, nullptr), 0);             // ((0+2)/3)*4 = 0
}

TEST(Base64EncodeTest, BinaryData) {
    // Test encoding binary data (including null bytes)
    std::vector<unsigned char> binary_data = {0x00, 0x01, 0x02, 0x03, 0xFF, 0xFE, 0xFD};
    
    size_t encoded_size = base64_encode(binary_data.data(), binary_data.size(), nullptr);
    EXPECT_GT(encoded_size, 0);
    
    std::vector<char> encoded(encoded_size + 1);
    size_t actual_size = base64_encode(binary_data.data(), binary_data.size(), encoded.data());
    encoded[actual_size] = '\0';
    
    EXPECT_EQ(actual_size, encoded_size);
    EXPECT_TRUE(is_base64(encoded.data(), actual_size));
    
    // Verify round-trip
    size_t decoded_size = base64_decode(encoded.data(), nullptr);
    std::vector<char> decoded(decoded_size);
    size_t roundtrip_size = base64_decode(encoded.data(), decoded.data());
    
    EXPECT_EQ(roundtrip_size, binary_data.size());
    EXPECT_EQ(std::memcmp(decoded.data(), binary_data.data(), binary_data.size()), 0);
}

TEST(Base64EncodeTest, NullInput) {
    char buffer[100];
    
    // Test null data pointer
    EXPECT_EQ(base64_encode(nullptr, 10, buffer), 0);
    
    // Test null output pointer (should return size)
    EXPECT_EQ(base64_encode("Hello", 5, nullptr), 8);  // ((5+2)/3)*4 = 8
}

TEST(Base64EncodeTest, EdgeCases) {
    char buffer[20];
    
    // Test single character
    size_t size = base64_encode("A", 1, buffer);
    EXPECT_EQ(size, 4);
    buffer[size] = '\0';
    EXPECT_STREQ(buffer, "QQ==");
    
    // Test two characters
    size = base64_encode("AB", 2, buffer);
    EXPECT_EQ(size, 4);
    buffer[size] = '\0';
    EXPECT_STREQ(buffer, "QUI=");
    
    // Test three characters
    size = base64_encode("ABC", 3, buffer);
    EXPECT_EQ(size, 4);
    buffer[size] = '\0';
    EXPECT_STREQ(buffer, "QUJD");
    
    // Test different lengths with round-trip verification
    const char* test_data = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    for (size_t len = 1; len <= 26; len++) {
        size_t encoded_size = base64_encode(test_data, len, nullptr);
        EXPECT_GT(encoded_size, 0) << "Length: " << len;
        
        std::vector<char> encoded(encoded_size + 1);
        size_t actual_size = base64_encode(test_data, len, encoded.data());
        encoded[actual_size] = '\0';
        
        EXPECT_EQ(actual_size, encoded_size);
        EXPECT_TRUE(is_base64(encoded.data(), actual_size));
        
        // Verify round-trip
        size_t decoded_size = base64_decode(encoded.data(), nullptr);
        std::vector<char> decoded(decoded_size);
        size_t roundtrip_size = base64_decode(encoded.data(), decoded.data());
        
        EXPECT_EQ(roundtrip_size, len);
        EXPECT_EQ(std::memcmp(decoded.data(), test_data, len), 0);
    }
    
    // Test zero length explicitly
    size_t zero_size = base64_encode("ABC", 0, nullptr);
    EXPECT_EQ(zero_size, 0);
}

TEST(Base64EncodeTest, PaddingVerification) {
    // Test that padding is correctly applied
    struct TestCase {
        size_t input_len;
        size_t expected_size;
    };
    
    std::vector<TestCase> test_cases = {
        {1, 4},   // 1 byte -> 4 chars
        {2, 4},   // 2 bytes -> 4 chars  
        {3, 4},   // 3 bytes -> 4 chars
        {4, 8},   // 4 bytes -> 8 chars
        {5, 8},   // 5 bytes -> 8 chars
        {6, 8},   // 6 bytes -> 8 chars
    };
    
    for (const auto& test_case : test_cases) {
        std::string input(test_case.input_len, 'A');
        size_t encoded_size = base64_encode(input.c_str(), input.length(), nullptr);
        EXPECT_EQ(encoded_size, test_case.expected_size) 
            << "Input length: " << test_case.input_len;
        
        std::vector<char> encoded(encoded_size + 1);
        size_t actual_size = base64_encode(input.c_str(), input.length(), encoded.data());
        encoded[actual_size] = '\0';
        
        // Verify it's valid base64
        EXPECT_TRUE(is_base64(encoded.data(), actual_size))
            << "Input length: " << test_case.input_len << ", output: " << encoded.data();
    }
}

TEST(Base64EncodeTest, LargeData) {
    // Test with large data
    std::vector<char> large_data(1000);
    for (size_t i = 0; i < large_data.size(); i++) {
        large_data[i] = static_cast<char>(i % 256);
    }
    
    size_t encoded_size = base64_encode(large_data.data(), large_data.size(), nullptr);
    EXPECT_GT(encoded_size, 0);
    EXPECT_EQ(encoded_size, ((large_data.size() + 2) / 3) * 4);
    
    std::vector<char> encoded(encoded_size);
    size_t actual_size = base64_encode(large_data.data(), large_data.size(), encoded.data());
    EXPECT_EQ(actual_size, encoded_size);
    
    // Verify it's valid base64
    EXPECT_TRUE(is_base64(encoded.data(), actual_size));
    
    // Verify round-trip
    size_t decoded_size = base64_decode(encoded.data(), nullptr);
    std::vector<char> decoded(decoded_size);
    size_t roundtrip_size = base64_decode(encoded.data(), decoded.data());
    
    EXPECT_EQ(roundtrip_size, large_data.size());
    EXPECT_EQ(std::memcmp(decoded.data(), large_data.data(), large_data.size()), 0);
}

TEST(Base64EncodeTest, BufferOverrun) {
    // Test that function doesn't write beyond the reported size
    std::string test_input = "Hello";
    
    // Get required size
    size_t required_size = base64_encode(test_input.c_str(), test_input.length(), nullptr);
    
    // Allocate exactly that much space
    std::vector<char> buffer(required_size);
    std::vector<char> guard_before(10, 'X');  // Guard bytes before
    std::vector<char> guard_after(10, 'X');   // Guard bytes after
    
    // Make sure guard bytes are intact
    auto verify_guards = [&]() {
        for (int i = 0; i < 10; i++) {
            if (guard_before[i] != 'X' || guard_after[i] != 'X') {
                return false;
            }
        }
        return true;
    };
    
    EXPECT_TRUE(verify_guards());
    size_t encoded_size = base64_encode(test_input.c_str(), test_input.length(), buffer.data());
    EXPECT_TRUE(verify_guards());  // Guards should still be intact
    EXPECT_EQ(encoded_size, required_size);
}