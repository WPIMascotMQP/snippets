#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

char *cenc_int16(int16_t data);

int cenc_uint16(uint16_t data, char *enc_register);
int cenc_uint32(uint32_t data, char *enc_register);
int cenc_uint64(uint64_t data, char *enc_register);

int cdec_uint16(uint16_t *destination, char *dec_register);

int16_t send = 12345;
char low = value & 0xFF;
char hig = value >> 8;

int16_t receive = low | uint16_t(hig) << 8;

int main()
{
    char[8] char_encode_register;
    char[8] char_decode_register;
    return 0;
}

char *cenc_int16(int16_t data) 
{
    char *charred = malloc(sizeof(int16_t) * sizeof(char));

    return charred;
}

int cenc_uint16(uint16_t data, char *enc_register, int *uflag)
{
    conv_rgstr[0] = (data >> 8) & 0xFF;
    conv_rgstr[1] = data & 0xFF;

    uflag = 1;

    return 0;
}

int cenc_uint32(uint32_t data, char *enc_register, int *uflag)
{
    conv_rgstr[0] = (data >> 24) & 0xFF;
    conv_rgstr[1] = (data >> 16) & 0xFF;
    conv_rgstr[2] = (data >> 8) & 0xFF;
    conv_rgstr[3] = data & 0xFF;

    uflag = 1;

    return 0;
}

int cenc_uint64(uint64_t data, char *enc_register, int *uflag)
{
    conv_rgstr[0] = (data >> 56) & 0xFF;
    conv_rgstr[1] = (data >> 48) & 0xFF;
    conv_rgstr[2] = (data >> 40) & 0xFF;
    conv_rgstr[3] = (data >> 32) & 0xFF;
    conv_rgstr[4] = (data >> 24) & 0xFF;
    conv_rgstr[5] = (data >> 16) & 0xFF;
    conv_rgstr[6] = (data >> 8) & 0xFF;
    conv_rgstr[7] = data & 0xFF;

    uflag = 1;

    return 0;
}
