#ifdef _UECC_H_

#include <xhive_config.h>

#ifdef CONFIG_MICRO_ECC_ENABLE_SECP160R1
#define uECC_SUPPORTS_secp160r1 1
#else
#define uECC_SUPPORTS_secp160r1 0
#endif

#ifdef CONFIG_MICRO_ECC_ENABLE_SECP192R1
#define uECC_SUPPORTS_secp192r1 1
#else
#define uECC_SUPPORTS_secp192r1 0
#endif

#ifdef CONFIG_MICRO_ECC_ENABLE_SECP224R1
#define uECC_SUPPORTS_secp224r1 1
#else
#define uECC_SUPPORTS_secp224r1 0
#endif

#ifdef CONFIG_MICRO_ECC_ENABLE_SECP256R1
#define uECC_SUPPORTS_secp256r1 1
#else
#define uECC_SUPPORTS_secp256r1 0
#endif

#ifdef CONFIG_MICRO_ECC_ENABLE_SECP256K1
#define uECC_SUPPORTS_secp256k1 1
#else
#define uECC_SUPPORTS_secp256k1 0
#endif

#ifdef CONFIG_MICRO_ECC_ENABLE_COMPRESSED_POINT
#define uECC_SUPPORT_COMPRESSED_POINT 1
#else
#define uECC_SUPPORT_COMPRESSED_POINT 0
#endif

#endif /* _UECC_H_ */
