# mecc - Elliptic Curve Cryptography Module

This module provides an interface for performing elliptic curve cryptography (ECC) operations with [uECC](https://github.com/kmackay/micro-ecc), including key generation, signing, and verification.

## Features

- **Key Generation**: Generate ECC keypairs for various curves
- **Public Key Derivation**: Derive public keys from private keys
- **Hash Signing**: Sign pre-computed message hashes with both regular and deterministic (RFC 6979) methods
- **Signature Verification**: Verify signatures against public keys using the original message hashes
- **Public Key Validation**: Validate public keys for specific curves
- **Curve Discovery**: List all supported elliptic curves
- **Encoding Support**: Automatic detection and support for both binary and Base64 encoding
- **Multiple Curves**: Support for secp160r1, secp192r1, secp224r1, secp256r1, secp256k1

## Supported Curves

The module supports the following elliptic curves based on uECC configuration:

| Curve Name | Description | Key Size |
|------------|-------------|----------|
| `secp160r1` | NIST B-163 | 20 bytes (private), 40 bytes (public) |
| `secp192r1` | NIST P-192 | 24 bytes (private), 48 bytes (public) |
| `secp224r1` | NIST P-224 | 28 bytes (private), 56 bytes (public) |
| `secp256r1` | NIST P-256 | 32 bytes (private), 64 bytes (public) |
| `prime256v1` | Alias for secp256r1 | 32 bytes (private), 64 bytes (public) |
| `secp256k1` | Bitcoin curve | 32 bytes (private), 64 bytes (public) |

## API Reference

### list_curves()

Lists all supported elliptic curves.

```lua
local mecc = import("xhive.mecc")
local curves = mecc.list_curves()
for i, curve in ipairs(curves) do
    print(curve)
end
```

**Returns:** `table` - Array of supported curve names

---

### mk_keypair(curve_name, use_base64?)

Generates a new ECC keypair.

```lua
local mecc = import("xhive.mecc")
local private_key, public_key = mecc.mk_keypair("secp256r1", false)
```

**Parameters:**
- `curve_name` (string): Name of the elliptic curve to use
- `use_base64` (boolean, optional): If true, returns keys in Base64 encoding. Default is false

**Returns:**
- `private_key` (string): Generated private key
- `public_key` (string): Generated public key

---

### mk_pubkey(curve_name, private_key, use_base64?)

Derives the public key from a given private key.

```lua
local mecc = import("xhive.mecc")
local private_key = "..." -- binary or Base64 encoded private key
local public_key = mecc.mk_pubkey("secp256r1", private_key, true)
```

**Parameters:**
- `curve_name` (string): Name of the elliptic curve
- `private_key` (string): Private key in binary string or Base64 encoding (auto-detected)
- `use_base64` (boolean, optional): If true, returns public key in Base64 encoding. Default is false

**Returns:**
- `public_key` (string): Derived public key

---

### hash_sign(curve_name, private_key, message_hash, use_base64?)

Signs a message hash using the provided private key with regular random k generation.

```lua
local mecc = import("xhive.mecc")
local message_hash = sha256("Hello, World!") -- Pre-computed hash
local signature = mecc.hash_sign("secp256r1", private_key, message_hash)
```

**Parameters:**
- `curve_name` (string): Name of the elliptic curve
- `private_key` (string): Private key in binary string or Base64 encoding
- `message_hash` (string): Message hash to be signed (arbitrary length supported)
- `use_base64` (boolean, optional): If true, returns signature in Base64 encoding. Default is false

**Returns:**
- `signature` (string): Generated signature

**Note:** This function accepts hash values of any length and uses the library's random number generator for k generation.

---

### det_sign(curve_name, private_key, message_hash, use_base64?)

Signs a message hash with RFC 6979 deterministic k generation using SHA256-HMAC internally.

```lua
local mecc = import("xhive.mecc")
local message_hash = sha256("Hello, World!") -- Can be any hash algorithm output
local signature = mecc.det_sign("secp256r1", private_key, message_hash)
```

**Parameters:**
- `curve_name` (string): Name of the elliptic curve
- `private_key` (string): Private key in binary string or Base64 encoding
- `message_hash` (string): Message hash to be signed (arbitrary length supported)
- `use_base64` (boolean, optional): If true, returns signature in Base64 encoding. Default is false

**Returns:**
- `signature` (string): Generated signature

**Note:** This function accepts hash values of any length and uses SHA256-HMAC internally for deterministic k generation according to RFC 6979.

---

### verify_sign(curve_name, public_key, message, signature)

Verifies a signature using the provided public key.

```lua
local mecc = import("xhive.mecc")
local is_valid = mecc.verify_sign("secp256r1", public_key, message, signature)
if is_valid then
    print("Signature is valid!")
else
    print("Signature is invalid!")
end
```

**Parameters:**
- `curve_name` (string): Name of the elliptic curve
- `public_key` (string): Public key in binary string or Base64 encoding
- `message` (string): Original message that was signed
- `signature` (string): Signature to verify

**Returns:**
- `is_valid` (boolean): True if the signature is valid, false otherwise

---

### verify_pubkey(curve_name, public_key)

Verifies if a public key is valid for the specified curve.

```lua
local mecc = import("xhive.mecc")
local is_valid = mecc.verify_pubkey("secp256r1", public_key)
```

**Parameters:**
- `curve_name` (string): Name of the elliptic curve
- `public_key` (string): Public key in binary string or Base64 encoding

**Returns:**
- `is_valid` (boolean): True if the public key is valid, false otherwise

## Usage Examples

### Basic Key Generation and Signing

```lua
local mecc = import("xhive.mecc")
local sha256 = import("xhive.sha256") -- or any SHA256 implementation

-- Generate keypair
local curve = "secp256r1"
local private_key, public_key = mecc.mk_keypair(curve, false)

print("Private key:", private_key)
print("Public key:", public_key)

-- Verify public key
if mecc.verify_pubkey(curve, public_key) then
    print("Public key is valid")
end

-- Sign a message (requires pre-computed hash)
local message = "Hello, ECC!"
local message_hash = sha256(message)
local signature = mecc.hash_sign(curve, private_key, message_hash)

-- Verify signature
local is_valid = mecc.verify_sign(curve, public_key, message_hash, signature)
print("Signature valid:", is_valid)
```

### Deterministic Signing (RFC 6979)

```lua
local mecc = import("xhive.mecc")
local sha256 = import("xhive.sha256") -- or any SHA256 implementation

-- Generate keypair
local private_key, public_key = mecc.mk_keypair("secp256r1")

-- Create message hash using any hash algorithm
local message = "Important message"
local message_hash = sha256(message)

-- Sign with deterministic k (RFC 6979 compliant)
local signature = mecc.det_sign("secp256r1", private_key, message_hash)

-- Verify signature
local is_valid = mecc.verify_sign("secp256r1", public_key, message_hash, signature)
print("Deterministic signature valid:", is_valid)

-- Example with different hash algorithms
-- local message_hash_sha384 = sha384(message) -- 48 bytes
-- local signature_sha384 = mecc.det_sign("secp256r1", private_key, message_hash_sha384)
```

### hash_sign vs det_sign - Understanding the Difference

The module provides two signing functions with different security properties:

```lua
local message = "Important data"
local message_hash = sha256(message)

-- Regular signing - uses random k generation
local signature1 = mecc.hash_sign("secp256r1", private_key, message_hash)

-- Deterministic signing - uses RFC 6979 for k generation
local signature2 = mecc.det_sign("secp256r1", private_key, message_hash)

-- Both signatures are valid but different
print("Random signature valid:", mecc.verify_sign("secp256r1", public_key, message_hash, signature1))
print("Deterministic signature valid:", mecc.verify_sign("secp256r1", public_key, message_hash, signature2))
```

**Key Differences:**
- **hash_sign**: Uses random number generation for k parameter (non-deterministic)
- **det_sign**: Uses RFC 6979 deterministic k generation based on private key and message hash
- **Security**: `det_sign` is preferred for production use as it eliminates randomness-related attack vectors
- **Consistency**: `det_sign` always produces the same signature for the same message hash and private key

### Using Base64 Encoding

```lua
local mecc = import("xhive.mecc")
local sha256 = import("xhive.sha256") -- or any SHA256 implementation

-- Generate keys with Base64 encoding
local private_key_b64, public_key_b64 = mecc.mk_keypair("secp256k1", true)

print("Private key (Base64):", private_key_b64)
print("Public key (Base64):", public_key_b64)

-- Use Base64 encoded keys directly (requires pre-computed hash)
local message = "Base64 test"
local message_hash = sha256(message)
local signature_b64 = mecc.hash_sign("secp256k1", private_key_b64, message_hash, true)

-- Verify with Base64 encoded data
local is_valid = mecc.verify_sign("secp256k1", public_key_b64, message_hash, signature_b64)
print("Base64 signature valid:", is_valid)
```

### Listing Supported Curves

```lua
local mecc = import("xhive.mecc")

local curves = mecc.list_curves()
print("Supported curves:")
for i, curve in ipairs(curves) do
    print(string.format("%d. %s", i, curve))
end
```

## Error Handling

All functions will raise errors with descriptive messages when:
- Invalid curve names are provided
- Key sizes don't match the expected curve requirements
- Memory allocation fails
- Invalid data formats are provided
- Cryptographic operations fail

Always wrap calls in appropriate error handling for production use:

```lua
local ok, result = pcall(function()
    return mecc.mk_keypair("secp256r1")
end)

if ok then
    local private_key, public_key = result
    -- Use keys...
else
    print("Error:", result)
end
```

## Security Notes

- For production use, ensure proper random number generation is configured in uECC
- Prefer deterministic signing (`det_sign`) when possible for better security
- Always validate public keys before using them for signature verification
- Use appropriate key sizes for your security requirements
- Consider the security implications of your chosen curve

## Dependencies

- [uECC](https://github.com/kmackay/micro-ecc) - Micro-ECC library for elliptic curve operations
- SHA256 implementation (for deterministic signing)
- Base64 implementation (for Base64 encoding support)
- Lua C API

## License

This module is licensed under GPLv3. Please see the LICENSE file for details.

