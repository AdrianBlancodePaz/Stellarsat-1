#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <correct.h>

// To compile, do not forget to add `-lcorrect` to gcc call

#define NUM_ROOTS 16
#define MSG_LEN (255 - NUM_ROOTS)  // 239 bytes

int main() {
    uint8_t message[MSG_LEN];
    uint8_t encoded[256];  // 255 bytes will be written
    uint8_t decoded[MSG_LEN];

    // Initialize the message (example: fill with a sequence)
    for (size_t i = 0; i < MSG_LEN; i++) {
        message[i] = i & 0xFF;
    }

    // Create a Reed-Solomon encoder/decoder.
    // Make sure you are using the correct constant for the polynomial.
    // (Check your header for the exact name. It might be CORRECT_RS_PRIMITIVE_POLYNOMIAL_CCSDS or similar.)
    correct_reed_solomon *rs = correct_reed_solomon_create(
        correct_rs_primitive_polynomial_ccsds, // Use the proper constant from your header.
        1,                                     // First consecutive root
        1,                                     // Generator root gap
        NUM_ROOTS                              // Number of parity bytes
    );

    if (!rs) {
        printf("Failed to create Reed-Solomon encoder/decoder.\n");
        return 1;
    }

    // Encode. The encoded block will be 255 bytes in length.
    ssize_t encoded_len = correct_reed_solomon_encode(rs, message, sizeof(message), encoded);
    if (encoded_len <= 0) {
        printf("Encoding failed!\n");
        return 1;
    }

    // Optionally, you can print the encoded block for inspection.
    printf("Encoded block length: %zd\n", encoded_len);

    // Introduce errors. For instance, corrupt 4 bytes (well within the correctable limit of 8 errors).
    encoded[50]  ^= 0xFF;  // Flip bits in byte at index 50
    encoded[100] ^= 0xFF;  // Flip bits in byte at index 100
    encoded[150] ^= 0xFF;  // Flip bits in byte at index 150
    encoded[200] ^= 0xFF;  // Flip bits in byte at index 200

    // Decode the encoded block.
    ssize_t decoded_len = correct_reed_solomon_decode(rs, encoded, encoded_len, decoded);

    if (decoded_len > 0) {
        printf("Decoded successfully! %zd bytes\n", decoded_len);
        // You can compare the decoded data with the original message.
        if (decoded_len != sizeof(message) || memcmp(message, decoded, sizeof(message)) != 0) {
            printf("Decoded message does not match the original.\n");
        } else {
            printf("Decoded message matches the original.\n");
        }
    } else {
        printf("Decoding failed.\n");
    }

    // Re-encode the decoded message
    uint8_t re_encoded[256] = {0};
    correct_reed_solomon_encode(rs, decoded, decoded_len, re_encoded);

    // Check if the re-encoded message matches the corrupted one
    int error_detected = memcmp(encoded, re_encoded, encoded_len) != 0;

    if (error_detected) {
	printf("Error correction was applied.\n");
    } else {
	printf("No errors were detected.\n");
    }

    // Cleanup
    correct_reed_solomon_destroy(rs);

    return 0;
}