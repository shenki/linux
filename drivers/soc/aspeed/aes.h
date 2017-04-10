#ifndef _AES_H
#define _AES_H

struct ast_aes_context
{
    u32 erk[64];     /* encryption round keys */
    u32 drk[64];     /* decryption round keys */
    int nr;             /* number of rounds */
};

int  aes_set_key( struct ast_aes_context *ctx, u8 *key, int nbits );
void aes_encrypt( struct ast_aes_context *ctx, u8 input[16], u8 output[16] );
void aes_decrypt( struct ast_aes_context *ctx, u8 input[16], u8 output[16] );

#endif /* aes.h */
