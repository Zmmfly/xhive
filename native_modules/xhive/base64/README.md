# base64 - Base64 Encoding and Decoding Module

This module provides a simple and efficient interface for Base64 encoding and decoding operations. It includes strict input validation to ensure data integrity and proper error handling.

## Features

- **String Encoding**: Encode strings to Base64 format with strict input validation
- **Base64 Decoding**: Decode Base64 strings back to original format with validation
- **Input Validation**: Comprehensive validation for both string inputs and Base64 format
- **Error Handling**: Clear error messages for invalid inputs
- **Memory Safety**: Proper memory management and null-termination
- **Standard Compliance**: Supports standard Base64 encoding with padding

## API Reference

### encode(input_str)

Encodes a string to Base64 format.

```lua
local base64 = import("xhive.base64")
local encoded = base64.encode("Hello, World!")
print(encoded) -- "SGVsbG8sIFdvcmxkIQ=="
```

**Parameters:**
- `input_str` (string): The input string to encode (must be a non-empty string)

**Returns:**
- `encoded_str` (string): The Base64 encoded string

**Errors:**
- Raises error if input is not a string
- Raises error if input is an empty string
- Raises error if memory allocation fails
- Raises error if encoding fails

---

### decode(base64_str)

Decodes a Base64 encoded string back to the original string.

```lua
local base64 = import("xhive.base64")
local decoded = base64.decode("SGVsbG8sIFdvcmxkIQ==")
print(decoded) -- "Hello, World!"
```

**Parameters:**
- `base64_str` (string): The Base64 encoded string to decode

**Returns:**
- `decoded_str` (string): The decoded string

**Errors:**
- Raises error if input is not a valid Base64 string
- Raises error if the string contains invalid characters
- Raises error if padding is incorrect
- Raises error if memory allocation fails
- Raises error if decoding fails

**Validation Rules:**
- Only allows characters: A-Z, a-z, 0-9, +, /, -, _
- Padding character '=' is only allowed at the end (max 2 characters)
- Minimum length is 4 characters
- Proper Base64 structure validation

---

### is_valid(input_str)

Validates if a string is in valid Base64 format.

```lua
local base64 = import("xhive.base64")
local valid = base64.is_valid("SGVsbG8sIFdvcmxkIQ==")
print(valid) -- true

local invalid = base64.is_valid("Invalid@String")
print(invalid) -- false
```

**Parameters:**
- `input_str` (string): The string to validate

**Returns:**
- `is_valid` (boolean): True if valid Base64 format, false otherwise

## Usage Examples

### Basic Encoding and Decoding

```lua
local base64 = import("xhive.base64")

-- Encode a string
local original = "Hello, Base64!"
local encoded = base64.encode(original)
print("Encoded:", encoded) -- "SGVsbG8sIEJhc2U2NCE="

-- Decode back
local decoded = base64.decode(encoded)
print("Decoded:", decoded) -- "Hello, Base64!"

-- Verify round-trip
assert(original == decoded)
print("Round-trip successful!")
```

### Validation

```lua
local base64 = require("base64")

-- Valid Base64 strings
local valid_strings = {
    "SGVsbG8=",
    "SGVsbG8sIFdvcmxkIQ==",
    "VGVzdGluZyAxMjM=",
    "YWJjZGVmZ2hpams="
}

for _, str in ipairs(valid_strings) do
    if base64.is_valid(str) then
        print("✓ Valid:", str)
    else
        print("✗ Invalid:", str)
    end
end

-- Invalid Base64 strings
local invalid_strings = {
    "Invalid@String",
    "Short",
    "Too=Many==Padds==",
    "",
    "ABC!"  -- Invalid character
}

for _, str in ipairs(invalid_strings) do
    if base64.is_valid(str) then
        print("✗ Should be invalid:", str)
    else
        print("✓ Correctly identified as invalid:", str)
    end
end
```

### Error Handling

```lua
local base64 = require("base64")

-- Safe encoding with error handling
local function safe_encode(input)
    local ok, result = pcall(base64.encode, input)
    if ok then
        return result
    else
        print("Encode error:", result)
        return nil
    end
end

-- Safe decoding with error handling
local function safe_decode(input)
    local ok, result = pcall(base64.decode, input)
    if ok then
        return result
    else
        print("Decode error:", result)
        return nil
    end
end

-- Test with various inputs
print("Empty string:", safe_encode(""))        -- Will error
print("Nil input:", safe_encode(nil))          -- Will error
print("Valid input:", safe_encode("test"))     -- Success
print("Invalid Base64:", safe_decode("@@@"))   -- Will error
print("Valid Base64:", safe_decode("dGVzdA==")) -- Success
```

### Working with Binary Data

```lua
local base64 = require("base64")

-- Encode binary data (as Lua string)
local binary_data = string.char(0x48, 0x65, 0x6C, 0x6C, 0x6F) -- "Hello" in bytes
local encoded_binary = base64.encode(binary_data)
print("Binary encoded:", encoded_binary) -- "SGVsbG8="

-- Decode back to binary data
local decoded_binary = base64.decode(encoded_binary)
print("Binary decoded:", decoded_binary) -- "Hello"

-- Verify binary data integrity
assert(binary_data == decoded_binary)
```

### Common Use Cases

```lua
local base64 = require("base64")

-- URL-safe encoding preparation
local function prepare_for_url(text)
    local encoded = base64.encode(text)
    -- Replace URL-unsafe characters (if needed for specific use cases)
    return encoded:gsub("+", "-"):gsub("/", "_"):gsub("=", "")
end

-- Simple data obfuscation (not encryption!)
local function obfuscate(data)
    return base64.encode(data)
end

local function deobfuscate(obfuscated_data)
    return base64.decode(obfuscated_data)
end

-- Test obfuscation
local sensitive = "user:password"
local obfuscated = obfuscate(sensitive)
local revealed = deobfuscate(obfuscated)

print("Original:", sensitive)
print("Obfuscated:", obfuscated)
print("Revealed:", revealed)
assert(sensitive == revealed)
```

## Performance Considerations

- **Memory Efficiency**: The module allocates exact buffer sizes needed for encoding/decoding
- **Validation Speed**: Input validation is optimized for common cases
- **Error Handling**: Early validation prevents unnecessary processing

## Security Notes

- **Input Validation**: All inputs are strictly validated to prevent injection attacks
- **Memory Management**: Proper memory allocation and cleanup prevents leaks
- **Error Handling**: Clear error messages help with debugging but don't expose sensitive data
- **Not for Encryption**: Base64 is encoding, not encryption. Don't use it for security-sensitive data

## Dependencies

- [Base64 library](../../../libs/base64/) - Core Base64 encoding/decoding functions
- Lua C API

## License

This module is licensed under GPLv3. Please see the LICENSE file for details.

## Troubleshooting

### Common Issues

1. **"Input must be a non-empty string"**
   - Ensure you're passing a valid string, not nil or empty string
   - Check that your variable actually contains string data

2. **"Input must be a valid Base64 encoded string"**
   - Verify your Base64 string uses only valid characters
   - Check that padding (if present) is correct
   - Ensure minimum length requirement (4 characters)

3. **"Memory allocation failed"**
   - Check available system memory
   - Ensure input size is reasonable for your system

### Debug Tips

```lua
local base64 = require("base64")

-- Debug Base64 validation
local function debug_validation(input)
    print("Input:", input)
    print("Length:", #input)
    print("Valid:", base64.is_valid(input))
    
    -- Try to decode and show result
    local ok, result = pcall(base64.decode, input)
    if ok then
        print("Decoded:", result)
        print("Decoded length:", #result)
    else
        print("Decode error:", result)
    end
    print("---")
end

-- Test various inputs
debug_validation("Valid==")
debug_validation("Invalid@")
debug_validation("TooShort")
```