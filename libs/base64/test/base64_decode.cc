#include "main.hpp"
#include "../inc/base64.h"
#include <string>
#include <cstring>
#include <vector>
#include <algorithm>

TEST(Base64DecodeTest, BasicDecoding) {
    // Test basic decoding with known inputs/outputs
    std::vector<std::pair<std::string, std::string>> test_cases = {
        {"SGVsbG8gV29ybGQ=", "Hello World"},
        {"YWJjZGVmZ2hpams=", "abcdefghijk"},
        {"MTIzNDU2Nzg5MA==", "1234567890"},
        {"VGVzdGluZw==", "Testing"},
        {"Zm9vYmFy", "foobar"},
        {"QQ==", "A"},
        {"QUI=", "AB"},
        {"QUJD", "ABC"},
        {"QUJDRA==", "ABCD"},
        {"", ""}
    };
    
    for (const auto& test_case : test_cases) {
        // First get the expected size
        size_t expected_size = base64_decode(test_case.first.c_str(), nullptr);
        EXPECT_EQ(expected_size, test_case.second.length()) 
            << "Size mismatch for: " << test_case.first;
        
        if (expected_size > 0) {
            // Allocate buffer and decode
            std::vector<char> output(expected_size);
            size_t decoded_size = base64_decode(test_case.first.c_str(), output.data());
            
            EXPECT_EQ(decoded_size, expected_size) 
                << "Decoded size mismatch for: " << test_case.first;
            EXPECT_EQ(std::string(output.data(), decoded_size), test_case.second)
                << "Decoded content mismatch for: " << test_case.first;
        }
    }
}

TEST(Base64DecodeTest, SizeOnly) {
    // Test getting size without providing output buffer
    EXPECT_EQ(base64_decode("SGVsbG8gV29ybGQ=", nullptr), 11);
    EXPECT_EQ(base64_decode("YWJjZGVmZ2hpams=", nullptr), 11);
    EXPECT_EQ(base64_decode("MTIzNDU2Nzg5MA==", nullptr), 10);
    EXPECT_EQ(base64_decode("VGVzdGluZw==", nullptr), 7);
    EXPECT_EQ(base64_decode("Zm9vYmFy", nullptr), 6);
    EXPECT_EQ(base64_decode("QQ==", nullptr), 1);
    EXPECT_EQ(base64_decode("", nullptr), 0);
}

TEST(Base64DecodeTest, BinaryData) {
    // Test decoding binary data (including null bytes)
    std::vector<unsigned char> binary_data = {0x00, 0x01, 0x02, 0x03, 0xFF, 0xFE, 0xFD};
    
    // First encode this binary data using our encode function
    size_t encoded_size = base64_encode(binary_data.data(), binary_data.size(), nullptr);
    std::vector<char> encoded(encoded_size + 1);
    base64_encode(binary_data.data(), binary_data.size(), encoded.data());
    encoded[encoded_size] = '\0';
    
    // Now decode it back
    size_t decoded_size = base64_decode(encoded.data(), nullptr);
    std::vector<char> decoded(decoded_size);
    size_t actual_size = base64_decode(encoded.data(), decoded.data());
    
    EXPECT_EQ(decoded_size, binary_data.size());
    EXPECT_EQ(actual_size, binary_data.size());
    EXPECT_EQ(std::memcmp(decoded.data(), binary_data.data(), binary_data.size()), 0);
}

TEST(Base64DecodeTest, InvalidInputHandling) {
    char buffer[100];
    
    // Test with invalid characters - function should skip them and still decode valid parts
    // "SGVsbG8gV29ybGQ!" - '!' will be skipped
    size_t result1 = base64_decode("SGVsbG8gV29ybGQ!", buffer);
    EXPECT_GT(result1, 0) << "Should still decode valid part of string with invalid character";
    
    // "YWJjZGVmZ2hpams@" - '@' will be skipped  
    size_t result2 = base64_decode("YWJjZGVmZ2hpams@", buffer);
    EXPECT_GT(result2, 0) << "Should still decode valid part of string with invalid character";
    
    // Test with extra padding - function stops at first '='
    size_t result3 = base64_decode("SGVsbG8gV29ybGQ===", buffer);
    EXPECT_EQ(result3, 11) << "Should stop at first padding character";
    
    // Test with padding in wrong position
    size_t result4 = base64_decode("QQ=Q", buffer);
    EXPECT_EQ(result4, 1) << "Should stop at first '=' and decode what it can";
    
    // Test with short length (not valid base64 but function will try)
    size_t result5 = base64_decode("SGVs", buffer);
    EXPECT_EQ(result5, 3) << "Should attempt to decode what it can";
    
    // Test null input
    EXPECT_EQ(base64_decode(nullptr, buffer), 0);
}

TEST(Base64DecodeTest, NullInput) {
    // Test with null input pointer
    EXPECT_EQ(base64_decode(nullptr, nullptr), 0);
    EXPECT_EQ(base64_decode(nullptr, nullptr), 0);
}

TEST(Base64DecodeTest, EdgeCases) {
    char buffer[100];
    
    // Single character
    EXPECT_EQ(base64_decode("QQ==", buffer), 1);
    EXPECT_EQ(buffer[0], 'A');
    
    // Two characters
    EXPECT_EQ(base64_decode("QUI=", buffer), 2);
    EXPECT_EQ(buffer[0], 'A');
    EXPECT_EQ(buffer[1], 'B');
    
    // Three characters
    EXPECT_EQ(base64_decode("QUJD", buffer), 3);
    EXPECT_EQ(buffer[0], 'A');
    EXPECT_EQ(buffer[1], 'B');
    EXPECT_EQ(buffer[2], 'C');
    
    // All base64 characters
    const char* all_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t decoded_size = base64_decode(all_chars, nullptr);
    EXPECT_GT(decoded_size, 0);
    
    std::vector<char> decoded(decoded_size);
    size_t actual_size = base64_decode(all_chars, decoded.data());
    EXPECT_EQ(decoded_size, actual_size);
    
    // Test with whitespace - should be skipped
    std::string with_whitespace = "SGVs bG8g V29y bGQ="; // with spaces
    size_t ws_size = base64_decode(with_whitespace.c_str(), nullptr);
    EXPECT_GT(ws_size, 0);
}

TEST(Base64DecodeTest, BufferOverrun) {
    // Test that function doesn't write beyond the reported size
    std::string test_input = "SGVsbG8gV29ybGQ=";
    
    // Get required size
    size_t required_size = base64_decode(test_input.c_str(), nullptr);
    
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
    size_t decoded_size = base64_decode(test_input.c_str(), buffer.data());
    EXPECT_TRUE(verify_guards());  // Guards should still be intact
    EXPECT_EQ(decoded_size, required_size);
}