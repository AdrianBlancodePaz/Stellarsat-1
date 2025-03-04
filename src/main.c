#include <stdio.h>
#include <stdlib.h>
#include <correct.h>

void print_bits(uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        for (int j = 7; j >= 0; j--) {
            printf("%d", (data[i] >> j) & 1);
        }
        printf(" ");
    }
    printf("\n");
}

int main() {
    // Polinomios generadores: (7,5) en octal -> (111,101) en binario
    const correct_convolutional_polynomial_t poly[] = {0b111, 0b101};

    // Crear codificador convolucional 1/2, constraint length 3 (order = 2)
    correct_convolutional *conv = correct_convolutional_create(2, 9, poly);
    if (!conv) {
        fprintf(stderr, "Error: No se pudo crear el codificador convolucional\n");
        return 1;
    }

    // Datos de entrada (ejemplo: 4 bytes)
    uint8_t input[] = {0xFF, 0xA3, 0x55, 0XB0, 0XCC, 0X10, 0X11};
    size_t input_len = sizeof(input)/sizeof(input[0]);
    printf("Input lenght: %d\n", input_len);


    // Calcular el tamaño de salida esperado
    size_t encoded_max_len = correct_convolutional_encode_len(conv, input_len);
    uint8_t encoded[encoded_max_len];

    // Codificar datos
    size_t encoded_len = correct_convolutional_encode(conv, input, input_len, encoded);
    if (encoded_len == 0) {
        fprintf(stderr, "Error: La codificación falló\n");
        correct_convolutional_destroy(conv);
        return 1;
    }

    // Mostrar datos codificados
    printf("Datos codificados: ");
    for (size_t i = 0; i < encoded_len/8; i++) {
        printf("%02X ", encoded[i]);
    }
    printf("\n");
    printf("Encode lenght: %d\n", encoded_len/8);


    // Simulación de canal ruidoso (introducimos un error)
    //encoded[3] ^= 0x01;  // Modificamos un bit para simular ruido

    // Espacio para los datos decodificados
    size_t decoded_max_len = input_len; // El tamaño correcto para decodificación
    uint8_t decoded[decoded_max_len];

    // Decodificar usando Viterbi (versión HARD)
    size_t decoded_len = correct_convolutional_decode(conv, encoded, encoded_len, decoded);
    if (decoded_len == 0) {
        fprintf(stderr, "Error: La decodificación falló\n");
        correct_convolutional_destroy(conv);
        return 1;
    }

    // Mostrar datos decodificados
    printf("Datos decodificados: ");
    for (size_t i = 0; i < decoded_len; i++) {
        printf("%02X ", decoded[i]);
    }
    printf("\n");
    printf("Decode lenght: %d\n", decoded_len);

    
    printf("Datos codificados en bits: ");
    print_bits(encoded, encoded_len / 8);
    // Liberar memoria
    correct_convolutional_destroy(conv);

    return 0;
}
